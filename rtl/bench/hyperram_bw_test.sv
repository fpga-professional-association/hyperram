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
//            |      | STATUS(on read)      |  R     | bit0 = busy, bit1 = done, bit2 = error
//    0x04    |  1   | LEN                  |  R/W   | number of words to test (per phase)
//    0x08    |  2   | BASE_ADDR            |  R/W   | starting WORD address (keep MSB=0 => memory space)
//    0x0C    |  3   | WR_CYCLES            |  R     | clk cycles of the WRITE phase (see above)
//    0x10    |  4   | RD_CYCLES            |  R     | clk cycles of the READ phase  (see above)
//    0x14    |  5   | ERR_COUNT            |  R     | number of read words that mismatched
//    0x18    |  6   | DATA_BYTES_PER_WORD  |  R     | constant = 2 (bytes per HyperBus word)
//    0x1C    |  7   | VERSION / MAGIC      |  R     | constant identifier (default 0x48425754 = "HBWT")
//    0x20    |  8   | ERR_ADDR             |  R     | WORD address of the first read mismatch
//    0x24    |  9   | ERR_GOT              |  R     | read data returned at the first mismatch
//    0x28    | 10   | ERR_EXP              |  R     | expected pattern at the first mismatch
//    0x2C    | 11   | BURSTW               |  R/W   | WRITE-phase HyperBus burst length (words); 0 => default
//    0x30    | 12   | RBURSTW              |  R/W   | READ-phase  HyperBus burst length (words); 0 => default
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
    parameter int unsigned CSR_ADDR_WIDTH = 4,                      // 16 word-registers (0x00..0x3C)
    parameter logic [31:0] VERSION_MAGIC  = 32'h4842_5754           // "HBWT"
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
    input  logic                        m_waitrequest
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

    // ------------------------------------------------------------------------
    // Working state (the traffic sequencer)
    // ------------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE,     // wait for CTRL.start
        S_WSTART,   // decide next write burst / end of write phase
        S_WBEAT,    // stream a write burst (command + data) to the slave
        S_RSTART,   // decide next read burst / end of read phase (=> done)
        S_RCMD,     // assert a read command, wait for it to be accepted
        S_RDATA     // capture + check returning read words
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
    always_comb begin
        m_address    = cur_addr;
        m_burstcount = this_burst;
        m_read       = (state == S_RCMD);
        m_write      = (state == S_WBEAT);
        m_writedata  = pat_q;   // pipelined gen_pattern (see pat_q declaration)
    end

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
            cur_addr    <= '0;
            words_left  <= 32'd0;
            beat        <= '0;
            wr_started  <= 1'b0;
            rd_started  <= 1'b0;
        end else begin
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
                        cur_addr    <= r_base;
                        words_left  <= r_len;
                        beat        <= '0;
                        wr_started  <= 1'b0;
                        rd_started  <= 1'b0;
                        state       <= S_WSTART;
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
                        pat_q      <= gen_pattern(cur_addr);   // beat 0 pattern, ready on S_WBEAT entry
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
                            pat_q <= gen_pattern(cur_addr + ADDR_WIDTH'(beat) + ADDR_WIDTH'(1));
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
                        exp_q      <= gen_pattern(cur_addr);   // beat 0 expectation, ready before data
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
                        end
                        if (beat + LEN_WIDTH'(1) == this_burst) begin
                            cur_addr   <= cur_addr + ADDR_WIDTH'(this_burst);
                            words_left <= words_left - 32'(this_burst);
                            if (words_left == 32'(this_burst))
                                rd_started <= 1'b0;         // last burst -> stop counting after this cycle
                            state      <= S_RSTART;
                        end else begin
                            beat  <= beat + LEN_WIDTH'(1);
                            exp_q <= gen_pattern(cur_addr + ADDR_WIDTH'(beat) + ADDR_WIDTH'(1));
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
