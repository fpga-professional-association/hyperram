// hyperram_bw_test — synthesizable read/write BANDWIDTH-TEST engine for the HyperBus master IP.
//
// A self-contained traffic generator + scoreboard that drives an Avalon-MM MASTER port (wired to
// the hyperram_avalon slave) and is controlled/read back through an Avalon-MM CSR SLAVE port
// (driven on hardware by a JTAG-to-Avalon master, in sim by a testbench). It measures the raw
// throughput of the HyperBus datapath by streaming a WRITE phase then a READ phase over LEN words
// and counting the clk cycles each phase occupies, so the host can compute MB/s off-chip.
//
// Data integrity: every word carries a deterministic, address-seeded pattern (xorshift of the word
// address). The READ phase recomputes the same pattern for each returned word and counts
// mismatches; any mismatch latches STATUS.error and increments ERR_COUNT.
//
// Timing model of the counters (matches "first command asserted -> last beat accepted/returned"):
//   * WR_CYCLES counts every clk from the cycle the first write command is asserted on the Avalon
//     master through the cycle the final write beat is accepted (inter-burst bubbles included; the
//     leading command-setup cycle excluded).
//   * RD_CYCLES counts every clk from the cycle the first read command is asserted through the cycle
//     the final read word is returned (read-latency and inter-burst bubbles included).
// The host bandwidth is therefore  MB/s = (LEN * DATA_BYTES_PER_WORD) / (cycles / f_clk).
//
// Design rules: single clk, synchronous active-high rst, clean FSM, NO vendor primitives — it
// simulates cleanly under Verilator (verilator --binary --timing). All bus geometry comes from
// hyperbus_pkg; no magic numbers. The Avalon MASTER widths below match the hyperram_avalon slave
// EXACTLY (DATA_WIDTH=16, ADDR_WIDTH=32 word address, LEN_WIDTH=16 burstcount).
//
// =====================================================================================
// CSR MAP (Avalon-MM CSR slave; 32-bit registers). csr_address is a WORD address: register k is at
// host byte offset 4*k. csr_waitrequest is tied low (0 wait states); reads are combinational.
// =====================================================================================
//   byte off | word | name                 | access | bits / meaning
//   ---------+------+----------------------+--------+------------------------------------------------
//    0x00    |  0   | CTRL  (on write)     |  W     | bit0 = start (self-clearing strobe; ignored while busy)
//            |      |                      |        | bit1 = READ-ONLY run (skip the write phase; score the
//            |      |                      |        |         readback against gen_pattern — pending-buffer probe)
//            |      | STATUS(on read)      |  R     | bit0 = busy, bit1 = done, bit2 = error
//    0x04    |  1   | LEN                  |  R/W   | number of words to test (per phase)
//    0x08    |  2   | BASE_ADDR            |  R/W   | starting WORD address (keep MSB=0 => memory space)
//    0x0C    |  3   | WR_CYCLES            |  R     | clk cycles of the WRITE phase (see above)
//    0x10    |  4   | RD_CYCLES            |  R     | clk cycles of the READ phase  (see above)
//    0x14    |  5   | ERR_COUNT            |  R     | number of read words that mismatched
//    0x18    |  6   | DATA_BYTES_PER_WORD  |  R     | constant = 2 (bytes per HyperBus word)
//    0x1C    |  7   | VERSION / MAGIC      |  R     | constant identifier (default 0x48425755 = "HBWU",
//            |      |                      |        |   instrumented issue-#13 build; was 0x48425754 "HBWT")
//    0x20    |  8   | ERR_ADDR             |  R     | WORD address of the first read mismatch
//    0x24    |  9   | ERR_GOT              |  R     | read data returned at the first mismatch
//    0x28    | 10   | ERR_EXP              |  R     | expected pattern at the first mismatch
//    0x2C    | 11   | BURSTW               |  R/W   | WRITE-phase HyperBus burst length (words); 0 => default
//    0x30    | 12   | RBURSTW              |  R/W   | READ-phase  HyperBus burst length (words); 0 => default
//    0x34    | 13   | REG_CAL              |  R/W   | live PHY read-eye calibration image (drives cal_*):
//            |      |                      |        |   [0]=cal_capture_phase  [3:1]=cal_preamble_skip
//            |      |                      |        |   [8:4]=cal_rx_tap       [9]=cal_pair_skew
//            |      |                      |        | plain R/W (NO 0=>default carve-out; 0 is a valid
//            |      |                      |        | cal value); reset image = CAL_RESET parameter
//   ---------+------+----------------------+--------+------------------------------------------------
//   issue #13 instrumented-build knobs (words 14..20; wound/end-garble experiment ladder). All new
//   knobs are POR-default-legacy: with reset values the datapath is bit-identical to the shipped build.
//    0x38    | 14   | REG_DBG              |  R/W   | live ctrl/PHY debug knobs (drive dbg_* bundle):
//            |      |                      |        |   [3:0]=dbg_wr_lat_trim  [7:4]=dbg_lat_clocks (6/7)
//            |      |                      |        |   [8]=cr0_reprog (W1 strobe, self-clearing, reads 0)
//            |      |                      |        |   [9]=dbg_prewin_drive  [12:10]=dbg_prewin_n(0..7)
//            |      |                      |        |   [13]=dbg_prewin_marker [14]=dbg_postwin_hold
//            |      |                      |        |   [15]=dbg_ck_stretch_off
//            |      |                      |        |   [16]=dbg_prewin_contig (round 2 A: heal contiguous
//            |      |                      |        |        command-edge ST_IDLE write reopens)
//            |      |                      |        |   [17]=dbg_end_cwrite (round 3 B: end-of-row commit-WRITE
//            |      |                      |        |        — masked 4-word write at the row-aligned end; the
//            |      |                      |        |        prewin tail heals the orphan home. Was end-READ
//            |      |                      |        |        (round 2), falsified: a read sprays it one row low)
//            |      |                      |        |   reset = DBG_RESET
//    0x3C    | 15   | REG_EMAP_STAT        |  R     | wound-map FIFO status: [6:0]=count(0..64),
//            |      |                      |        |   [7]=valid(count>0), [8]=overflow(sticky/run)
//    0x40    | 16   | REG_PAT              |  R/W   | pattern select [1:0]: 0=gen_pattern 1=0xFFFF
//            |      |                      |        |   2=0x0000 3=addr-echo (applied to write AND read)
//    0x44    | 17   | REG_WRAP             |  R/W   | nonzero V arms ONE wrapped write at word addr V
//            |      |                      |        |   (burst=WRAP_PROBE_WORDS, cmd_wrap=1); 0 = off
//    0x48    | 18   | REG_EMAP_IDX         |  R/W   | wound-map read index [5:0] (host-set, 0..63)
//    0x4C    | 19   | REG_EMAP_ADDR        |  R     | emap[IDX] word address (REGISTERED read, A5)
//    0x50    | 20   | REG_EMAP_DATA        |  R     | emap[IDX] {got[31:16], exp[15:0]} (REGISTERED, A5)
// =====================================================================================
//
// See docs/BW_TEST.md for the full narrative, the MB/s formula and the sim/hardware run notes.

module hyperram_bw_test
    import hyperbus_pkg::*;
#(
    parameter int unsigned DATA_WIDTH     = HB_DATA_WIDTH_DEFAULT,  // 16 — native HyperBus word
    parameter int unsigned ADDR_WIDTH     = HB_ADDR_WIDTH,          // 32 — word-address width
    parameter int unsigned LEN_WIDTH      = HB_LEN_WIDTH_DEFAULT,   // 16 — Avalon burstcount width
    parameter int unsigned BURST_WORDS    = HB_BURST_WORDS_DEFAULT, // 16 — words per Avalon burst
    parameter int unsigned CSR_ADDR_WIDTH = 5,                      // 32 word-registers (0x00..0x7C)
    parameter logic [31:0] VERSION_MAGIC  = 32'h4842_5755,          // "HBWU" (instrumented; was 0x..54 "HBWT")
    parameter logic [31:0] CAL_RESET      = 32'h0000_0000,          // POR image of REG_CAL (cal_* seed)
    // ---- issue #13 instrumented-build seeds ---------------------------------
    // DBG_RESET is per-INSTANCE-derived (A2): default 0x60 = dbg_lat_clocks=6, dbg_wr_lat_trim=0 — the
    // sim/GENERIC leg (its ctrl has WR_LAT_TRIM=0). The BOARD instance overrides to 0x63 (trim=3) to
    // match its ctrl WR_LAT_TRIM(3). With the reset value the ctrl seeds are bit-identical to the
    // WR_LAT_TRIM/LATENCY_CLOCKS parameters, so the datapath is legacy out of reset (§8 invariance).
    parameter logic [31:0] DBG_RESET       = 32'h0000_0060,         // [7:4]=lat=6, [3:0]=trim=0
    parameter int unsigned WRAP_PROBE_WORDS = 16                    // REG_WRAP burst = CR0 wrap group (0x8F1F => 16)
) (
    input  logic                        clk,
    input  logic                        rst,          // synchronous, active-high

    // ---- Avalon-MM CSR slave (host / JTAG control + readback) ---------------
    input  logic [CSR_ADDR_WIDTH-1:0]   csr_address,  // WORD address (byte offset = 4*addr)
    input  logic                        csr_read,
    output logic [31:0]                 csr_readdata,
    input  logic                        csr_write,
    input  logic [31:0]                 csr_writedata,
    output logic                        csr_waitrequest,

    // ---- Avalon-MM master (to the hyperram_avalon slave) --------------------
    output logic [ADDR_WIDTH-1:0]       m_address,    // WORD address
    output logic [LEN_WIDTH-1:0]        m_burstcount, // words in burst
    output logic                        m_read,
    output logic                        m_write,
    output logic [DATA_WIDTH-1:0]       m_writedata,
    input  logic [DATA_WIDTH-1:0]       m_readdata,
    input  logic                        m_readdatavalid,
    input  logic                        m_waitrequest,

    // ---- runtime PHY read-eye calibration (to hyperram_avalon's cal_* inputs) ----------------------
    // Decoded from REG_CAL so a host CSR write retunes the read eye with NO recompile. Bit map (REG_CAL):
    //   [0]=cal_capture_phase  [3:1]=cal_preamble_skip  [8:4]=cal_rx_tap  [9]=cal_pair_skew.
    output logic                                  cal_capture_phase,
    output logic [HB_CAL_PREAMBLE_SKIP_WIDTH-1:0] cal_preamble_skip,
    output logic [HB_CAL_RX_TAP_WIDTH-1:0]        cal_rx_tap,
    output logic                                  cal_pair_skew,

    // ---- issue #13 debug bundle (frozen contract §0) -------------------------
    // Decoded live from REG_DBG; a host CSR write retunes the controller/PHY with NO recompile. Same
    // single `clk` domain shared with ctrl/gpio_io, quasi-static (host pokes only while STATUS.busy=0)
    // => NO synchronizer (contrast cal_*'s clk->clk90 crossing). Drives hyperbus_ctrl (sim: through
    // hyperram_avalon; board: direct top wire) and — for dbg_ck_stretch_off — hyperbus_gpio_io.
    output logic [3:0] dbg_wr_lat_trim,    // overrides ctrl WR_LAT_TRIM     (POR = DBG_RESET[3:0])
    output logic [3:0] dbg_lat_clocks,     // overrides ctrl LATENCY_CLOCKS  (POR = DBG_RESET[7:4]; legal 6/7)
    output logic       dbg_cr0_reprog,     // 1-clk pulse: relaunch init CR0 write with the new latency code
    output logic       dbg_prewin_drive,   // heal probe: drive shadow words in the pre-data window
    output logic [2:0] dbg_prewin_n,       // # trailing latency CK to drive (0..7; sweep 3/4/5)
    output logic       dbg_prewin_marker,  // 1 = drive 0xA500|k marker instead of shadow (attribution)
    output logic       dbg_postwin_hold,   // hold last data word 4 CK into the tail (law-3 analog)
    output logic       dbg_ck_stretch_off, // board-only: disable gpio_io ck_stretch trailing masked cycle
    output logic       dbg_prewin_contig,  // round 2 (A): keep shadow at a contiguous command-edge reopen
    output logic       dbg_end_cwrite,      // round 3 (B): end-of-row (BURST_BOUNDARY-aligned) commit-WRITE
    output logic       wrap_en             // 1 = drive front-end cmd_wrap for the REG_WRAP probe burst
);

    // ---- derived constants --------------------------------------------------
    localparam int unsigned BYTES_PER_WORD = DATA_WIDTH / 8;   // = 2

    // CSR word-register indices (byte offset >> 2). Sized with an explicit cast so they are
    // width-clean at any CSR_ADDR_WIDTH (indices >= 8 alias low registers when CSR_ADDR_WIDTH < 4 —
    // harmless, those hosts only use the low 8 regs).
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_CTRL    = CSR_ADDR_WIDTH'(0);   // W: CTRL / R: STATUS
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_LEN     = CSR_ADDR_WIDTH'(1);
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_BASE    = CSR_ADDR_WIDTH'(2);
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_WRCYC   = CSR_ADDR_WIDTH'(3);
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_RDCYC   = CSR_ADDR_WIDTH'(4);
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRCNT  = CSR_ADDR_WIDTH'(5);
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_BYTES   = CSR_ADDR_WIDTH'(6);
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_MAGIC   = CSR_ADDR_WIDTH'(7);
    // First-mismatch diagnostics (latched on the FIRST read mismatch of a run; cleared at start).
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRADDR = CSR_ADDR_WIDTH'(8);   // WORD address of first mismatch
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRGOT  = CSR_ADDR_WIDTH'(9);   // read data at that address
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERREXP  = CSR_ADDR_WIDTH'(10);  // expected pattern there
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_BURSTW  = CSR_ADDR_WIDTH'(11);  // R/W: WRITE burst length (words)
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_RBURSTW = CSR_ADDR_WIDTH'(12);  // R/W: READ  burst length (words)
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_CAL     = CSR_ADDR_WIDTH'(13);  // R/W: live PHY read-eye cal
                                                                              //      (see bit map above)
    // issue #13 instrumented-build registers (words 14..20). Word 15 is EMAP status; the {addr,got,exp}
    // payload is random-access (IDX/ADDR/DATA at words 18/19/20) — a REGISTERED RAM read (A5), NOT
    // pop-on-read, so the 64x64 emap RAM never fans an async mux into the CSR read path.
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_DBG       = CSR_ADDR_WIDTH'(14);  // R/W: dbg_* bundle image
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_EMAP_STAT = CSR_ADDR_WIDTH'(15);  // R  : {ov,valid,count}
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_PAT       = CSR_ADDR_WIDTH'(16);  // R/W: pattern select
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_WRAP      = CSR_ADDR_WIDTH'(17);  // R/W: wrapped-write probe arm
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_EMAP_IDX  = CSR_ADDR_WIDTH'(18);  // R/W: emap read index
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_EMAP_ADDR = CSR_ADDR_WIDTH'(19);  // R  : emap[IDX] word address
    localparam logic [CSR_ADDR_WIDTH-1:0] REG_EMAP_DATA = CSR_ADDR_WIDTH'(20);  // R  : emap[IDX] {got,exp}

    localparam int unsigned EMAP_DEPTH = 64;   // wound-map FIFO depth (frozen §0)

    // ------------------------------------------------------------------------
    // Deterministic, address-seeded per-word data pattern (xorshift32 folded to
    // DATA_WIDTH). Pure function of the WORD address, so write and read phases
    // agree without any stored expectation memory.
    // ------------------------------------------------------------------------
    function automatic logic [DATA_WIDTH-1:0] gen_pattern(input logic [ADDR_WIDTH-1:0] a);
        logic [31:0] x;
        x = 32'(a);
        x = x ^ (x << 7);
        x = x ^ (x >> 9);
        x = x ^ (x << 8);
        return x[DATA_WIDTH-1:0];
    endfunction

    // ------------------------------------------------------------------------
    // Architectural state (CSR-visible)
    // ------------------------------------------------------------------------
    logic                  st_busy, st_done, st_error;
    logic [31:0]           r_len;         // words to test
    logic [ADDR_WIDTH-1:0] r_base;        // starting word address
    logic [31:0]           r_wr_cycles;   // WRITE-phase cycle count
    logic [31:0]           r_rd_cycles;   // READ-phase  cycle count
    logic [31:0]           r_err_count;   // read mismatches
    // First-mismatch capture (diagnostics): latched once per run on the first bad read word.
    logic [ADDR_WIDTH-1:0] r_err_addr;    // word address of the first mismatch
    logic [DATA_WIDTH-1:0] r_err_got;     // value returned at the first mismatch
    logic [DATA_WIDTH-1:0] r_err_exp;     // expected value at the first mismatch
    logic                  r_err_latched; // first mismatch has been captured this run

    // Runtime PHY read-eye calibration image (REG_CAL). Plain R/W — unlike REG_BURSTW there is NO
    // "0 => default" carve-out, since 0 is a legitimate cal value. Drives the cal_* outputs live.
    logic [31:0]           r_cal;
    assign cal_capture_phase = r_cal[0];
    assign cal_preamble_skip = r_cal[3:1];
    assign cal_rx_tap        = r_cal[8:4];
    assign cal_pair_skew     = r_cal[9];

    // ------------------------------------------------------------------------
    // issue #13 instrumented-build state (REG_DBG / REG_PAT / REG_WRAP / REG_EMAP_*)
    // ------------------------------------------------------------------------
    // REG_DBG store. Bit [8] (cr0_reprog) is a WRITE-1 strobe that is NEVER stored — it fires a
    // single-cycle dbg_cr0_reprog pulse (below) and reads back 0. All other bits latch and drive the
    // dbg_* bundle live (same clk domain as ctrl; no synchronizer). Resets to DBG_RESET => legacy.
    logic [31:0]           r_dbg;
    assign dbg_wr_lat_trim   = r_dbg[3:0];
    assign dbg_lat_clocks    = r_dbg[7:4];
    assign dbg_prewin_drive  = r_dbg[9];
    assign dbg_prewin_n      = r_dbg[12:10];
    assign dbg_prewin_marker = r_dbg[13];
    assign dbg_postwin_hold  = r_dbg[14];
    assign dbg_ck_stretch_off = r_dbg[15];
    assign dbg_prewin_contig = r_dbg[16];   // round 2 (A)
    assign dbg_end_cwrite     = r_dbg[17];   // round 3 (B): end-of-row commit-WRITE (was end-READ)
    // dbg_cr0_reprog is a generated 1-cycle pulse (in the sequential block), NOT a decode of r_dbg[8].

    // REG_PAT store (pattern select) and REG_WRAP arm (target word address + go-strobe).
    logic [1:0]            r_pat;
    logic [ADDR_WIDTH-1:0] r_wrap_addr;   // REG_WRAP: target word address B for the wrapped-write probe
    logic                  wrap_go;       // 1 = a wrapped-write probe is armed (consumed in S_IDLE)

    // Wound-map FIFO: 64 x {addr[31:0], got[15:0], exp[15:0]}. Pushed on EVERY read mismatch (in
    // parallel with the legacy single-shot first-error latch) so a multi-wound run decodes in ONE
    // pass. Readout is a REGISTERED RAM read (A5, MLAB/M20K-friendly): host writes REG_EMAP_IDX, then
    // reads REG_EMAP_ADDR/DATA on later JTAG transactions (many clk later) — emap_rd_q has settled.
    logic [63:0]           emap_mem [EMAP_DEPTH];
    logic [63:0]           emap_rd_q;     // registered read of emap_mem[r_emap_idx]
    logic [6:0]            emap_count;    // number of wounds captured this run (saturates at 64)
    logic                  emap_ov;       // sticky-per-run: a wound arrived after the FIFO filled
    logic [5:0]            r_emap_idx;    // host-set read index (0..63)

    // ------------------------------------------------------------------------
    // Pattern mux (REG_PAT). Applied to BOTH the write launch (pat_q) and the read expectation
    // (exp_q) so write and read always agree. Constant/echo backgrounds make wound-content
    // attribution unambiguous — gen(0)=0 means PAT=0 cannot distinguish a zeroed wound from real
    // data at low addresses, so any pass assertion must use PAT!=0 (or BASE!=0).
    // ------------------------------------------------------------------------
    function automatic logic [DATA_WIDTH-1:0] pat_of(input logic [ADDR_WIDTH-1:0] a);
        unique case (r_pat)
            2'd0:    pat_of = gen_pattern(a);      // xorshift (today's default)
            2'd1:    pat_of = {DATA_WIDTH{1'b1}};  // constant 0xFFFF
            2'd2:    pat_of = {DATA_WIDTH{1'b0}};  // constant 0x0000
            default: pat_of = a[DATA_WIDTH-1:0];   // addr-echo (low 16 bits of the WORD address)
        endcase
    endfunction

    // Wrapped-write per-beat address arithmetic (A4). The device wraps inside the CR0 wrap group, so
    // the REG_WRAP probe must write pat_of() of the address the device ACTUALLY stores to, letting the
    // follow-up RO probe assert repair: waddr(beat) = (B & ~(W-1)) | ((B + beat) & (W-1)), W=group.
    // (Before the wrap point this equals B+beat; after it, it wraps back to the group base.)
    function automatic logic [ADDR_WIDTH-1:0] wrap_addr(input logic [ADDR_WIDTH-1:0] b,
                                                        input logic [LEN_WIDTH-1:0]  bt);
        logic [ADDR_WIDTH-1:0] wmask;
        wmask     = ADDR_WIDTH'(WRAP_PROBE_WORDS) - ADDR_WIDTH'(1);   // low-bit group mask (W-1)
        wrap_addr = (b & ~wmask) | ((b + ADDR_WIDTH'(bt)) & wmask);
    endfunction

    // ------------------------------------------------------------------------
    // Working state (the traffic sequencer)
    // ------------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE,     // wait for CTRL.start
        S_WSTART,   // decide next write burst / end of write phase
        S_WBEAT,    // stream a write burst (command + data) to the slave
        S_RSTART,   // decide next read burst / end of read phase (=> done)
        S_RCMD,     // assert a read command, wait for it to be accepted
        S_RDATA,    // capture + check returning read words
        S_WRAP      // issue #13: single wrapped-write probe burst (REG_WRAP; wrap_en=1)
    } state_e;
    state_e state;

    logic [ADDR_WIDTH-1:0] cur_addr;    // base word address of the current burst
    // Pipelined data pattern (timing fix, 200 MHz fabric): gen_pattern() is a multi-level xorshift;
    // computed combinationally from cur_addr/beat it was the design's WORST fabric path (bench
    // address register -> xorshift -> Avalon -> ctrl -> PHY DQ I/O cell, Fmax 184 MHz). pat_q holds
    // the CURRENT write beat's pattern, exp_q the CURRENT read beat's expected value; both are
    // computed one cycle ahead at every (cur_addr, beat) change, so the xorshift terminates in a
    // local register instead of the I/O launch path.
    logic [DATA_WIDTH-1:0] pat_q;       // gen_pattern(cur_addr + beat) for the write phase
    logic [DATA_WIDTH-1:0] exp_q;       // gen_pattern(cur_addr + beat) for the read phase
    logic [31:0]           words_left;  // words remaining in the current phase
    logic [LEN_WIDTH-1:0]  beat;        // words done within the current burst
    logic                  wr_started;  // WR_CYCLES gate (first cmd asserted .. last beat)
    logic                  rd_started;  // RD_CYCLES gate (first cmd asserted .. last word)

    // Words in the current burst = min(active_burstw, words_left). The WRITE phase uses r_burstw and
    // the READ phase uses r_rburstw — both RUNTIME-programmable (REG_BURSTW / REG_RBURSTW, default
    // BURST_WORDS) so the host can sweep write and read burst lengths INDEPENDENTLY without a
    // recompile: e.g. write a single (correct) burst while splitting the read into many, isolating the
    // multi-burst READ path from the split-WRITE commit quirk (issue #2).
    logic [LEN_WIDTH-1:0]  r_burstw;
    logic [LEN_WIDTH-1:0]  r_rburstw;
    wire                   in_read_phase = (state == S_RSTART) || (state == S_RCMD) ||
                                           (state == S_RDATA);
    wire [LEN_WIDTH-1:0]   active_burstw = in_read_phase ? r_rburstw : r_burstw;
    logic [LEN_WIDTH-1:0]  this_burst;
    always_comb begin
        if (words_left >= 32'(active_burstw)) this_burst = active_burstw;
        else                                  this_burst = words_left[LEN_WIDTH-1:0];
    end

    // CSR start strobe: a write of CTRL bit0 while the CSR bus is ready.
    logic start_stroke;
    always_comb begin
        start_stroke = csr_write && !csr_waitrequest &&
                       (csr_address == REG_CTRL) && csr_writedata[0];
    end

    // ------------------------------------------------------------------------
    // Avalon-MM master combinational outputs
    //   * write: address/burstcount/writedata held for the whole burst; m_write
    //     high in S_WBEAT. A word is accepted whenever !m_waitrequest.
    //   * read : m_read high in S_RCMD until the command is accepted; data then
    //     streams back on m_readdatavalid (the master never back-pressures reads).
    // ------------------------------------------------------------------------
    // S_WRAP re-uses this datapath for the wrapped-write probe: it presents a single Avalon write
    // burst at base=cur_addr(=r_wrap_addr) with burstcount=WRAP_PROBE_WORDS; wrap_en=1 tells the
    // front-end to raise cmd_wrap so the device wraps inside its CR0 group. Normal runs keep
    // wrap_en=0 (=> FE cmd_wrap tied 0 = today's linear behavior).
    always_comb begin
        m_address    = cur_addr;
        m_burstcount = (state == S_WRAP) ? LEN_WIDTH'(WRAP_PROBE_WORDS) : this_burst;
        m_read       = (state == S_RCMD);
        m_write      = (state == S_WBEAT) || (state == S_WRAP);
        m_writedata  = pat_q;   // pipelined gen_pattern/pat_of (see pat_q declaration)
    end
    assign wrap_en = (state == S_WRAP);

    // ------------------------------------------------------------------------
    // CSR read (combinational, 0 wait states)
    // ------------------------------------------------------------------------
    always_comb begin
        csr_waitrequest = 1'b0;
        unique case (csr_address)
            REG_CTRL:   csr_readdata = {29'b0, st_error, st_done, st_busy};  // STATUS
            REG_LEN:    csr_readdata = r_len;
            REG_BASE:   csr_readdata = 32'(r_base);
            REG_WRCYC:  csr_readdata = r_wr_cycles;
            REG_RDCYC:  csr_readdata = r_rd_cycles;
            REG_ERRCNT: csr_readdata = r_err_count;
            REG_BYTES:  csr_readdata = 32'(BYTES_PER_WORD);
            REG_MAGIC:  csr_readdata = VERSION_MAGIC;
            REG_ERRADDR: csr_readdata = 32'(r_err_addr);
            REG_ERRGOT:  csr_readdata = 32'(r_err_got);
            REG_ERREXP:  csr_readdata = 32'(r_err_exp);
            REG_BURSTW:  csr_readdata = 32'(r_burstw);
            REG_RBURSTW: csr_readdata = 32'(r_rburstw);
            REG_CAL:     csr_readdata = r_cal;
            // issue #13 instrumented-build readback
            REG_DBG:       csr_readdata = {r_dbg[31:9], 1'b0, r_dbg[7:0]};          // bit8 (strobe) reads 0
            REG_PAT:       csr_readdata = {30'b0, r_pat};
            REG_WRAP:      csr_readdata = 32'(r_wrap_addr);
            REG_EMAP_STAT: csr_readdata = {23'b0, emap_ov, (emap_count != 7'd0), emap_count};
            REG_EMAP_IDX:  csr_readdata = {26'b0, r_emap_idx};
            REG_EMAP_ADDR: csr_readdata = emap_rd_q[63:32];                          // A5 registered read
            REG_EMAP_DATA: csr_readdata = emap_rd_q[31:0];                           // {got[31:16], exp[15:0]}
            default:    csr_readdata = 32'h0;
        endcase
    end

    // ------------------------------------------------------------------------
    // Sequential: CSR config writes + the traffic/scoreboard FSM
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            st_busy     <= 1'b0;
            st_done     <= 1'b0;
            st_error    <= 1'b0;
            r_len       <= 32'd0;
            r_base      <= '0;
            r_wr_cycles <= 32'd0;
            r_rd_cycles <= 32'd0;
            r_err_count <= 32'd0;
            r_err_addr    <= '0;
            r_err_got     <= '0;
            r_err_exp     <= '0;
            r_err_latched <= 1'b0;
            r_burstw    <= LEN_WIDTH'(BURST_WORDS);   // default; host may reprogram via REG_BURSTW
            r_rburstw   <= LEN_WIDTH'(BURST_WORDS);   // default; host may reprogram via REG_RBURSTW
            r_cal       <= CAL_RESET;                 // POR cal image; host may reprogram via REG_CAL
            cur_addr    <= '0;
            words_left  <= 32'd0;
            beat        <= '0;
            wr_started  <= 1'b0;
            rd_started  <= 1'b0;
            // issue #13 instrumented-build seeds: r_dbg=DBG_RESET => dbg_wr_lat_trim/dbg_lat_clocks
            // bit-identical to the ctrl WR_LAT_TRIM/LATENCY_CLOCKS params, all other dbg bits 0 => legacy.
            r_dbg          <= DBG_RESET;
            r_pat          <= 2'd0;                   // PAT=0 => pat_of == gen_pattern (today)
            r_wrap_addr    <= '0;
            wrap_go        <= 1'b0;
            r_emap_idx     <= 6'd0;
            emap_count     <= 7'd0;
            emap_ov        <= 1'b0;
            emap_rd_q      <= 64'd0;
            dbg_cr0_reprog <= 1'b0;
        end else begin
            // Registered EMAP readout (A5): emap_rd_q trails r_emap_idx by 1..2 clk; host reads
            // ADDR/DATA on a later JTAG transaction so it has always settled. MLAB/M20K-friendly.
            emap_rd_q      <= emap_mem[r_emap_idx];
            // dbg_cr0_reprog is a single-cycle pulse: default 0, set for exactly one clock on a REG_DBG
            // write with bit[8]=1 (below). The ctrl edge-consumes it only when idle (host contract).
            dbg_cr0_reprog <= 1'b0;
            // ---- CSR configuration writes (LEN / BASE) -----------------------
            // Accepted any time; they only take effect at the next start. CTRL is
            // decoded as the start strobe in S_IDLE below.
            if (csr_write && !csr_waitrequest) begin
                unique case (csr_address)
                    REG_LEN:    r_len    <= csr_writedata;
                    REG_BASE:   r_base   <= csr_writedata[ADDR_WIDTH-1:0];
                    REG_BURSTW: r_burstw  <= (csr_writedata[LEN_WIDTH-1:0] == '0)
                                             ? LEN_WIDTH'(BURST_WORDS)        // 0 => reset to default
                                             : csr_writedata[LEN_WIDTH-1:0];
                    REG_RBURSTW: r_rburstw <= (csr_writedata[LEN_WIDTH-1:0] == '0)
                                             ? LEN_WIDTH'(BURST_WORDS)        // 0 => reset to default
                                             : csr_writedata[LEN_WIDTH-1:0];
                    REG_CAL:    r_cal      <= csr_writedata;                  // plain R/W (no 0-carve-out)
                    // issue #13 instrumented-build config writes.
                    REG_DBG: begin
                        // Store all bits EXCEPT [8] (cr0_reprog strobe, forced 0 so it reads 0); a
                        // write with bit[8]=1 fires the one-cycle dbg_cr0_reprog pulse (§2.2 host
                        // contract: poke only while STATUS.busy=0, else the ctrl drops the pulse).
                        r_dbg <= {csr_writedata[31:9], 1'b0, csr_writedata[7:0]};
                        if (csr_writedata[8]) dbg_cr0_reprog <= 1'b1;
                    end
                    REG_PAT:  r_pat <= csr_writedata[1:0];
                    // REG_WRAP arms ONE wrapped-write probe: nonzero V = target word address B. Only
                    // accepted at S_IDLE (mutually exclusive with a normal run; a write while busy
                    // arms nothing). Consumed one clk later in the S_IDLE FSM branch below.
                    REG_WRAP: if (state == S_IDLE) begin
                        r_wrap_addr <= csr_writedata[ADDR_WIDTH-1:0];
                        wrap_go     <= (csr_writedata != 32'd0);
                    end
                    REG_EMAP_IDX: r_emap_idx <= csr_writedata[5:0];
                    default:  /* CTRL + read-only regs: no stored effect */ ;
                endcase
            end

            // ---- traffic sequencer ------------------------------------------
            unique case (state)
                // --------------------------------------------------------------
                S_IDLE: begin
                    if (start_stroke && !st_busy) begin
                        st_busy     <= 1'b1;
                        st_done     <= 1'b0;
                        st_error    <= 1'b0;
                        r_wr_cycles <= 32'd0;
                        r_rd_cycles <= 32'd0;
                        r_err_count <= 32'd0;
                        r_err_latched <= 1'b0;
                        emap_count  <= 7'd0;         // issue #13: clear wound-map at every run start
                        emap_ov     <= 1'b0;         //   (both normal AND read-only runs)
                        cur_addr    <= r_base;
                        words_left  <= r_len;
                        beat        <= '0;
                        wr_started  <= 1'b0;
                        rd_started  <= 1'b0;
                        // CTRL bit1 = READ-ONLY run: skip the write phase and score the readback
                        // against the pattern of a PREVIOUS normal run over the same LEN/BASE.
                        // The device-pending-buffer probe: read a region back later, WITHOUT the
                        // usual rewrite, to see what actually reached the memory array.
                        state       <= csr_writedata[1] ? S_RSTART : S_WSTART;
                    end else if (wrap_go && !st_busy) begin
                        // issue #13 REG_WRAP probe: launch ONE wrapped-write burst. No read/scoreboard
                        // phase — the host follows with an RO run to probe the array. STATUS mirrors a
                        // normal run (busy while streaming, done at completion).
                        st_busy       <= 1'b1;
                        st_done       <= 1'b0;
                        st_error      <= 1'b0;
                        r_err_count   <= 32'd0;
                        r_err_latched <= 1'b0;
                        emap_count    <= 7'd0;
                        emap_ov       <= 1'b0;
                        wrap_go       <= 1'b0;
                        cur_addr      <= r_wrap_addr;
                        words_left    <= 32'(WRAP_PROBE_WORDS);
                        beat          <= '0;
                        pat_q         <= pat_of(r_wrap_addr);   // beat 0: wrap_addr(B,0) == B
                        state         <= S_WRAP;
                    end
                end

                // ---- WRITE phase --------------------------------------------
                S_WSTART: begin
                    if (wr_started) r_wr_cycles <= r_wr_cycles + 32'd1;  // inter-burst bubble
                    if (words_left == 32'd0) begin
                        // Write phase complete -> set up the READ phase.
                        cur_addr   <= r_base;
                        words_left <= r_len;
                        beat       <= '0;
                        state      <= S_RSTART;
                    end else begin
                        beat       <= '0;
                        pat_q      <= pat_of(cur_addr);        // beat 0 pattern, ready on S_WBEAT entry
                        wr_started <= 1'b1;
                        state      <= S_WBEAT;
                    end
                end

                S_WBEAT: begin
                    r_wr_cycles <= r_wr_cycles + 32'd1;
                    // A word is accepted on any cycle the slave is ready.
                    if (!m_waitrequest) begin
                        if (beat + LEN_WIDTH'(1) == this_burst) begin
                            // Final beat of this burst accepted this cycle.
                            cur_addr   <= cur_addr + ADDR_WIDTH'(this_burst);
                            words_left <= words_left - 32'(this_burst);
                            if (words_left == 32'(this_burst))
                                wr_started <= 1'b0;         // last burst -> stop counting after this cycle
                            state      <= S_WSTART;
                        end else begin
                            beat  <= beat + LEN_WIDTH'(1);
                            pat_q <= pat_of(cur_addr + ADDR_WIDTH'(beat) + ADDR_WIDTH'(1));
                        end
                    end
                end

                // ---- READ phase ---------------------------------------------
                S_RSTART: begin
                    if (rd_started) r_rd_cycles <= r_rd_cycles + 32'd1;  // inter-burst bubble
                    if (words_left == 32'd0) begin
                        // Read phase complete -> whole test done.
                        st_busy <= 1'b0;
                        st_done <= 1'b1;
                        state   <= S_IDLE;
                    end else begin
                        beat       <= '0;
                        exp_q      <= pat_of(cur_addr);        // beat 0 expectation, ready before data
                        rd_started <= 1'b1;
                        state      <= S_RCMD;
                    end
                end

                S_RCMD: begin
                    r_rd_cycles <= r_rd_cycles + 32'd1;
                    // Command accepted when the slave drops waitrequest.
                    if (!m_waitrequest) state <= S_RDATA;
                end

                S_RDATA: begin
                    r_rd_cycles <= r_rd_cycles + 32'd1;
                    if (m_readdatavalid) begin
                        if (m_readdata != exp_q) begin
                            r_err_count <= r_err_count + 32'd1;
                            st_error    <= 1'b1;
                            if (!r_err_latched) begin       // capture the FIRST mismatch only
                                r_err_latched <= 1'b1;
                                r_err_addr    <= cur_addr + ADDR_WIDTH'(beat);
                                r_err_got     <= m_readdata;
                                r_err_exp     <= exp_q;
                            end
                            // issue #13 wound-map: push {addr,got,exp} on EVERY mismatch, in PARALLEL
                            // with the single-shot latch above, so a multi-wound run decodes in one
                            // pass. Master never back-pressures reads => <=1 push/clk (no rate hazard).
                            // Keep the FIRST 64 (the wound zones nearest BASE); flag overflow after.
                            if (emap_count < 7'd64) begin
                                emap_mem[emap_count[5:0]] <= {cur_addr + ADDR_WIDTH'(beat),
                                                              m_readdata, exp_q};
                                emap_count <= emap_count + 7'd1;
                            end else begin
                                emap_ov <= 1'b1;
                            end
                        end
                        if (beat + LEN_WIDTH'(1) == this_burst) begin
                            cur_addr   <= cur_addr + ADDR_WIDTH'(this_burst);
                            words_left <= words_left - 32'(this_burst);
                            if (words_left == 32'(this_burst))
                                rd_started <= 1'b0;         // last burst -> stop counting after this cycle
                            state      <= S_RSTART;
                        end else begin
                            beat  <= beat + LEN_WIDTH'(1);
                            exp_q <= pat_of(cur_addr + ADDR_WIDTH'(beat) + ADDR_WIDTH'(1));
                        end
                    end
                end

                // ---- issue #13 wrapped-write probe (REG_WRAP) ---------------
                // ONE Avalon write burst at base=cur_addr(=B), burstcount=WRAP_PROBE_WORDS, wrap_en=1
                // (=> front-end cmd_wrap=1 so the device wraps inside its CR0 group). Per-beat data =
                // pat_of(wrap_addr(B,beat)) so the follow-up RO probe can assert repair against what
                // the device actually stored (A4). No read/scoreboard phase here.
                S_WRAP: begin
                    if (!m_waitrequest) begin
                        if (beat + LEN_WIDTH'(1) == LEN_WIDTH'(WRAP_PROBE_WORDS)) begin
                            st_busy <= 1'b0;
                            st_done <= 1'b1;
                            state   <= S_IDLE;
                        end else begin
                            beat  <= beat + LEN_WIDTH'(1);
                            pat_q <= pat_of(wrap_addr(cur_addr, beat + LEN_WIDTH'(1)));
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
