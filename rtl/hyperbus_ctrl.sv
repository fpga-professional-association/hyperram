// hyperbus_ctrl — HyperBus master controller (protocol layer, PHY-agnostic).
//
// Normative spec: Infineon/Cypress HyperBus 001-99253 Rev *H, distilled in docs/SPEC_DIGEST.md.
// Architecture / clocking: docs/DESIGN.md. Frozen ports: docs/INTERFACES.md. Shared defs:
// rtl/hyperbus_pkg.sv. Clean-room implementation; no vendor primitives; fully Verilator-simulable.
//
// Responsibilities (DESIGN.md §4/§5):
//   * Accept one native command per transaction and build the 48-bit Command-Address (hb_pack_ca).
//   * Drive CS#, run CK (phy_ck_en), and serialize CA/write data as DDR word pairs to the PHY.
//   * Handle FIXED and VARIABLE initial latency. In variable mode the slave-driven RWDS level during
//     the CA phase (phy_rwds_i High => 2x latency) doubles the configured latency count.
//   * DDR reads: arm the PHY receiver (phy_rd_arm) and count RWDS-gated recovered words
//     (phy_dq_i_valid). Row/page latency gaps are absorbed because completion is word-counted, not
//     clock-counted. RWDS stalled Low >= 32 clocks => err_timeout + abort (SPEC_DIGEST §4/§7).
//   * DDR writes (with latency): RWDS is master-driven byte-mask (High = masked; = ~wr_strb).
//   * Register-space / zero-latency writes: data follows CA immediately, master does NOT drive RWDS,
//     full-word only (SPEC_DIGEST §5/§6).
//   * Linear + wrapped bursts (CA[45] = ~wrap). Linear bursts are chopped to MAX_BURST_WORDS so CS#
//     never exceeds tCSM, transparently to the caller (DESIGN.md §5.6).
//   * POR init: pulse device reset, wait POR_DELAY_CYCLES, optionally program CR0 (PROGRAM_CR), then
//     raise init_done. User commands are gated until init_done.
//
// Controller <-> PHY timing contract (a companion GENERIC PHY / model is written to match):
//   CS# is asserted one setup cycle (CK idle) before the CA phase. The 3 CA words are then clocked
//   over 3 consecutive cycles (byte A in [PHYW-1:DQ_WIDTH], byte B in [DQ_WIDTH-1:0]). Non-zero
//   latency inserts LATENCY_CLOCKS (x2 if variable+doubled) CK cycles before the data phase.
//
// Sync reset (active-high) for all architectural state; no async reset, no datapath clock gating.
`ifndef HYPERBUS_CTRL_SV
`define HYPERBUS_CTRL_SV
module hyperbus_ctrl
  import hyperbus_pkg::*;
#(
    parameter int unsigned DQ_WIDTH         = HB_DQ_WIDTH_DEFAULT,
    parameter int unsigned DATA_WIDTH       = HB_DATA_WIDTH_DEFAULT,
    parameter int unsigned ADDR_WIDTH       = HB_ADDR_WIDTH,
    parameter int unsigned LEN_WIDTH        = HB_LEN_WIDTH_DEFAULT,
    parameter int unsigned LATENCY_CLOCKS   = HB_LATENCY_CLOCKS_DEFAULT,
    parameter bit          FIXED_LATENCY    = HB_FIXED_LATENCY_DEFAULT,
    parameter int unsigned MAX_BURST_WORDS  = 0,                 // 0 = no chopping; else tCSM/tCK
    parameter bit          PROGRAM_CR       = 1'b1,              // program CR0 at init
    parameter int unsigned POR_DELAY_CYCLES = 0,                 // ~150us in cycles (0 in sim)
    parameter logic [3:0]  INIT_LATENCY_CODE= hb_clocks_to_latency_code(LATENCY_CLOCKS),
    // CR0 image programmed at init: [15]=1 normal, [14:12]=000 drive, [11:8]=1111 reserved-as-1,
    // [7:4]=latency code, [3]=fixed-latency, [2]=1 legacy wrap, [1:0]=11 (32B). (SPEC_DIGEST §8.1)
    parameter logic [15:0] INIT_CR0         = {1'b1, 3'b000, 4'b1111, INIT_LATENCY_CODE,
                                               FIXED_LATENCY, 3'b111},
    // -- Device-specific multi-burst work-arounds (Winbond W957D8NB, AXC3000). Both DEFAULT OFF so
    //    existing instantiations are bit-identical; the board top / bench enable them explicitly. --
    // A linear segment is chopped so it NEVER crosses a BURST_BOUNDARY_WORDS-aligned WORD boundary
    // (0 = disabled). The W957D8NB releases the bus ~1.5 CK into a burst that crosses a 0x2000-word
    // boundary, so everything past it reads as floating junk; chopping at the boundary avoids it
    // (DESIGN.md §5.6 / issue: 0x2000-word boundary chop).
    parameter int unsigned BURST_BOUNDARY_WORDS = 0,
    // After every SPLIT memory-write segment, self-issue an internal COMMIT-READ that spans the last
    // written word (the device only commits a write burst's final word when the next command is a
    // read that covers it; a following write drops it). 1 = interpose the commit-read (issue #1).
    parameter bit          WR_COMMIT_READ       = 1'b0
) (
    input  logic                    clk,
    input  logic                    rst,            // synchronous, active high

    // -- native command channel (slave) --
    input  logic                    cmd_valid,
    output logic                    cmd_ready,
    input  logic                    cmd_read,       // 1 = read  (CA[47])
    input  logic                    cmd_reg,        // 1 = register space (CA[46])
    input  logic                    cmd_wrap,       // 1 = wrapped burst (CA[45] = ~wrap)
    input  logic [ADDR_WIDTH-1:0]   cmd_addr,       // WORD address
    input  logic [LEN_WIDTH-1:0]    cmd_len,        // burst length in words, >= 1

    // -- native write-data channel (slave) --
    input  logic                    wr_valid,
    output logic                    wr_ready,
    input  logic [DATA_WIDTH-1:0]   wr_data,        // byte A = [DATA_WIDTH-1:DQ_WIDTH]
    input  logic [DATA_WIDTH/8-1:0] wr_strb,        // per-byte write-enable (1 = write)
    input  logic                    wr_last,        // final word of burst (informational)

    // -- native read-data channel (master) --
    output logic                    rd_valid,
    input  logic                    rd_ready,
    output logic [DATA_WIDTH-1:0]   rd_data,
    output logic                    rd_last,

    // -- status --
    output logic                    busy,
    output logic                    init_done,
    output logic                    err_underrun,   // pulse: write data not delivered in time
    output logic                    err_timeout,    // pulse: read RWDS stalled >= 32 clocks

    // -- PHY interface (master) : TX --
    output logic                    phy_cs_n,
    output logic                    phy_rst_n,
    output logic                    phy_ck_en,
    output logic [2*DQ_WIDTH-1:0]   phy_dq_o,       // [PHYW-1:DQ_WIDTH]=byte A, [DQ_WIDTH-1:0]=byte B
    output logic                    phy_dq_oe,
    output logic [1:0]              phy_rwds_o,      // [1]=1st phase, [0]=2nd phase (write mask)
    output logic                    phy_rwds_oe,
    output logic                    phy_rd_arm,

    // -- PHY interface (master) : RX --
    input  logic [2*DQ_WIDTH-1:0]   phy_dq_i,       // recovered read word (byte A in high half)
    input  logic                    phy_dq_i_valid,
    input  logic                    phy_rwds_i,      // synchronized RWDS level

    // -- DEBUG taps (bring-up only; leave unconnected in normal instantiations) --
    output logic [3:0]              dbg_state,      // FSM state (state_e ordinal)
    output logic [5:0]              dbg_rd_wptr,    // (repurposed) rem_left[5:0]  — words remaining in burst
    output logic [5:0]              dbg_rd_rptr     // (repurposed) seg_left[5:0]  — words remaining in segment
);

  // ------------------------------------------------------------------------
  // Derived widths / internal constants
  // ------------------------------------------------------------------------
  localparam int unsigned STRB_WIDTH       = DATA_WIDTH / 8;
  localparam int unsigned RESET_CYCLES     = 8;                    // RST# low pulse (>= tRP, SPEC §9)
  localparam int unsigned RECOVERY_CYCLES  = 4;                    // tCSHI + tRWR gap (SPEC §6)
  localparam int unsigned TAIL_CYCLES      = 2;                    // extra CS# Low after last word (tCSH note)
  localparam int unsigned READ_STALL_LIMIT = 32;                   // RWDS Low >= 32 clk => timeout (SPEC §4/§7)
  // Read over-stream drain/settle (AXC3000 multi-burst fix). After a read burst the real HyperRAM
  // keeps driving a few EXTRA source-synchronous words: its read output pipeline drains past the
  // master's CK-stop (which has flight/pipeline latency), so stray words keep arriving AFTER the master
  // has counted its burst. If the NEXT read arms its receiver while those stragglers are still arriving,
  // they are captured as phantom leading words and corrupt / hang the next burst (see tb_multiburst).
  // ST_RD_DRAIN therefore follows every read burst: CK is stopped (no new device words) but the PHY
  // receiver is kept ARMED so the over-stream words are drained out (phy_dq_i_valid) and DISCARDED,
  // until the PHY has been quiet for DRAIN_QUIET_CYCLES consecutive cycles (over-stream ended). Only
  // then does the transaction recover and re-arm/accept the next read, so no straggler leaks across the
  // burst boundary. Word-rate (phy_dq_i_valid, one pulse per straggler — no RWDS-level aliasing);
  // adaptive (robust to a VARIABLE extra-word count, not a hardcoded 7); bounded by DRAIN_MAX_CYCLES so
  // a stuck receiver can never hang the drain.
  localparam int unsigned DRAIN_QUIET_CYCLES = 4;                  // consecutive quiet cycles => drained
  localparam int unsigned DRAIN_MAX_CYCLES   = 128;               // safety bound on the drain wait (cycles)
  localparam int unsigned RD_FIFO_DEPTH    = 32;                  // read holding buffer (rd_ready slack).
                                                                    // Must exceed the largest single read
                                                                    // segment (Avalon burstcount) + the SDR
                                                                    // PHY read-pipeline depth so the final,
                                                                    // rd_last-tagged word is never dropped on
                                                                    // rd_fifo_full: on the AXC3000 SDR path a
                                                                    // depth of 8 capped completing reads at ~5
                                                                    // words (bursts >=6 hung). 32 covers the
                                                                    // board's 16-word bursts with slack.
  localparam int unsigned RD_AW            = $clog2(RD_FIFO_DEPTH);
  // Internal commit-read length (WR_COMMIT_READ). Must be >= 2 words (a 1-word dummy read does NOT
  // trigger the device write-commit; issue #1 approach #3) and small enough to never itself cross a
  // BURST_BOUNDARY_WORDS boundary (it reads backwards from the just-written last word, which lies in
  // an un-crossed segment). 4 matches the on-silicon "working read phase" trigger.
  localparam int unsigned COMMIT_READ_WORDS = 4;

  typedef enum logic [3:0] {
    ST_RESET,   // device reset asserted
    ST_POR,     // reset released, POR delay
    ST_INIT,    // launch optional CR0 write
    ST_IDLE,    // ready for a user command
    ST_CS,      // CS# setup (CK idle), first CA word presented
    ST_CA,      // 3 CA words clocked out
    ST_LAT,     // initial latency
    ST_READ,    // read-data phase (RWDS-gated word count)
    ST_RD_ABORT,// read aborted (RWDS timeout): drain remaining beats as flagged filler
    ST_RD_DRAIN,// post-read: CK stopped, receiver kept armed to absorb+discard over-stream stragglers
    ST_WRITE,   // write-data phase
    ST_TAIL,    // CS# held Low after last word (tCSH note)
    ST_RECOVER  // CS# High, tCSHI/tRWR recovery
  } state_e;

  state_e                  state;
  logic [31:0]             cnt;          // general dwell counter (reset/POR/latency/tail/recovery)
  logic [5:0]              stall_cnt;    // read RWDS-Low stall counter

  // Read over-stream drain/settle (see DRAIN_* localparams above).
  logic [7:0]              drain_cnt;    // consecutive cycles with no straggler word (quiet detector)
  logic [7:0]              drain_dwell;  // total cycles spent in ST_RD_DRAIN (safety bound)

  // transaction context
  logic                    cur_read;
  logic                    cur_reg;
  logic                    cur_wrap;     // 1 = wrapped
  logic [ADDR_WIDTH-1:0]   cur_addr;     // current segment start (WORD address)
  logic [LEN_WIDTH-1:0]    rem_left;     // words remaining in whole burst (incl. current)
  logic [LEN_WIDTH-1:0]    seg_left;     // words remaining in current CS# segment (incl. current)
  logic [LEN_WIDTH-1:0]    seg_count;    // words in current segment (for linear address advance)
  hb_ca_t                  ca_reg;       // packed 48-bit CA for the current segment
  logic [1:0]              ca_idx;       // CA word index 0..2
  logic                    rwds_hi;      // RWDS sampled High during CA (latency doubling indicator)
  logic                    lat_extra_done; // the additional (2x) latency count has been inserted
  logic                    doing_init;   // current transaction is the internal CR0 write

  // -- Write-commit interpose (WR_COMMIT_READ). A memory-write burst's final word is only committed
  //    by the device when a subsequent read covers it; between two writes the master interposes an
  //    internal COMMIT-READ that spans that word. `doing_commit` mirrors `doing_init` for this
  //    internal read (its data is discarded); `wr_pending_commit` marks that a write segment closed
  //    and still needs committing; `commit_resume` distinguishes an intra-command split (restore the
  //    shadow write context afterwards) from a deferred write->write interpose (return to idle). --
  logic                    doing_commit;
  logic                    wr_pending_commit;
  logic                    commit_resume;
  logic [ADDR_WIDTH-1:0]   last_wr_addr; // last WORD address written in the most recent write segment
  logic [ADDR_WIDTH-1:0]   sv_addr;      // shadow: write-segment start to resume after a chop commit-read
  logic [LEN_WIDTH-1:0]    sv_rem;       // shadow: words remaining to resume after a chop commit-read

  // read holding FIFO ({last, data})
  logic [DATA_WIDTH:0]     rd_fifo [RD_FIFO_DEPTH];
  logic [RD_AW:0]          rd_wptr, rd_rptr;

  // ------------------------------------------------------------------------
  // Combinational helpers
  // ------------------------------------------------------------------------
  wire                     zlw = ~cur_read & cur_reg;             // zero-latency (register) write

  // Deferred write->write commit interpose (WR_COMMIT_READ): when a NEW memory-write command is
  // presented while a previous write still needs committing, DON'T accept it — gate cmd_ready and
  // self-issue the internal commit-read first (launched in ST_IDLE below). A read/register command
  // is accepted normally (a covering read commits the pending write on silicon).
  wire                     commit_gate = WR_COMMIT_READ & wr_pending_commit &
                                         cmd_valid & ~cmd_read & ~cmd_reg;

  // write-data source: internal CR0 image during init, else the native write channel
  wire [DATA_WIDTH-1:0]    wsrc_data  = doing_init ? INIT_CR0 : wr_data;
  wire [STRB_WIDTH-1:0]    wsrc_strb  = doing_init ? {STRB_WIDTH{1'b1}} : wr_strb;
  wire                     wsrc_valid = doing_init ? 1'b1 : wr_valid;

  // Initial-latency doubling select (SPEC_DIGEST §3; spec 663-664/779-780; §5.2.4). The master ALWAYS
  // decodes the slave-driven RWDS level presented during CA to choose 1x vs 2x latency counts — this
  // is UNCONDITIONAL and must NOT branch on FIXED_LATENCY. A fixed-latency device simply drives RWDS to
  // a constant level; HyperRAM commonly fixes it HIGH to always require two counts (§5.2.4). One
  // latency count = LATENCY_CLOCKS cycles; when doubled a SECOND count of LATENCY_CLOCKS is inserted
  // back-to-back in ST_LAT. `rwds_hi` latches the indicator across CA *and* the latency window so the
  // decision is robust to the PHY's RWDS synchroniser pipeline delay (cf. OpenHBMC hbmc_ctrl.v:557,
  // which likewise samples RWDS-during-CA regardless of the fixed/variable parameter). FIXED_LATENCY
  // now only seeds the CR0[3] image at init (INIT_CR0 default).
  wire                     lat_double = rwds_hi | phy_rwds_i;

  // Read over-stream considered drained once the PHY has stopped delivering straggler words for
  // DRAIN_QUIET_CYCLES consecutive cycles (the over-stream has ended), or the safety bound is hit.
  wire                     drain_done = (drain_cnt   >= 8'(DRAIN_QUIET_CYCLES)) |
                                        (drain_dwell >= 8'(DRAIN_MAX_CYCLES));

  wire                     rd_fifo_empty = (rd_wptr == rd_rptr);
  wire                     rd_fifo_full  = (rd_wptr[RD_AW-1:0] == rd_rptr[RD_AW-1:0]) &
                                           (rd_wptr[RD_AW]     != rd_rptr[RD_AW]);
  wire [RD_AW-1:0]         rd_wa = rd_wptr[RD_AW-1:0];
  wire [RD_AW-1:0]         rd_ra = rd_rptr[RD_AW-1:0];

  // 16-bit CA word selected by index (MSB word first, SPEC_DIGEST §2)
  function automatic logic [DATA_WIDTH-1:0] ca_slice(input logic [1:0] idx);
    unique case (idx)
      2'd0:    return ca_reg[47:32];
      2'd1:    return ca_reg[31:16];
      default: return ca_reg[15:0];
    endcase
  endfunction

  // segment size for a (remaining) burst starting at `addr`. Linear bursts are chopped so a single
  // CS# segment never (a) exceeds MAX_BURST_WORDS (tCSM), nor (b) crosses a BURST_BOUNDARY_WORDS-word
  // aligned boundary (the W957D8NB bus-release quirk). Wrapped bursts stay in their group -> never
  // chopped. `addr` is the WORD start of the segment (only used for the boundary limit).
  function automatic logic [LEN_WIDTH-1:0] seg_size(input logic [LEN_WIDTH-1:0]  total,
                                                    input logic                  wrapped,
                                                    input logic [ADDR_WIDTH-1:0] addr);
    logic [LEN_WIDTH-1:0] lim;
    logic [LEN_WIDTH-1:0] to_bound;
    lim = total;
    if ((MAX_BURST_WORDS != 0) && !wrapped && (lim > LEN_WIDTH'(MAX_BURST_WORDS)))
      lim = LEN_WIDTH'(MAX_BURST_WORDS);
    if ((BURST_BOUNDARY_WORDS != 0) && !wrapped) begin
      // words from `addr` to the next boundary = BOUND - (addr mod BOUND); chop `lim` down to it.
      to_bound = LEN_WIDTH'(BURST_BOUNDARY_WORDS - (32'(addr) % BURST_BOUNDARY_WORDS));
      if (lim > to_bound) lim = to_bound;
    end
    return lim;
  endfunction

  // Base WORD address for the internal commit-read so that a COMMIT_READ_WORDS-word linear read
  // [base .. base+COMMIT_READ_WORDS-1] SPANS (ends on) the just-written word `la` (issue #1: the read
  // must cover the pending address). Clamped at 0 for very low addresses.
  function automatic logic [ADDR_WIDTH-1:0] commit_base(input logic [ADDR_WIDTH-1:0] la);
    if (la >= ADDR_WIDTH'(COMMIT_READ_WORDS - 1))
      return la - ADDR_WIDTH'(COMMIT_READ_WORDS - 1);
    else
      return '0;
  endfunction

  // ------------------------------------------------------------------------
  // Read data-out (FIFO front)
  // ------------------------------------------------------------------------
  assign rd_valid = ~rd_fifo_empty;
  assign rd_data  = rd_fifo[rd_ra][DATA_WIDTH-1:0];
  assign rd_last  = rd_fifo[rd_ra][DATA_WIDTH];

  // ------------------------------------------------------------------------
  // PHY drive + native handshakes (combinational, Moore on `state`)
  // ------------------------------------------------------------------------
  always_comb begin
    cmd_ready   = 1'b0;
    wr_ready    = 1'b0;
    phy_cs_n    = 1'b1;
    phy_rst_n   = 1'b1;
    phy_ck_en   = 1'b0;
    phy_dq_o    = '0;
    phy_dq_oe   = 1'b0;
    phy_rwds_o  = 2'b00;
    phy_rwds_oe = 1'b0;
    phy_rd_arm  = 1'b0;
    busy        = 1'b0;

    unique case (state)
      ST_RESET: phy_rst_n = 1'b0;

      ST_IDLE:  cmd_ready = init_done & rd_fifo_empty & ~commit_gate;

      ST_CS: begin
        phy_cs_n  = 1'b0;
        phy_dq_oe = 1'b1;
        phy_dq_o  = ca_slice(2'd0);
        busy      = ~doing_init;
      end

      ST_CA: begin
        phy_cs_n  = 1'b0;
        phy_ck_en = 1'b1;
        phy_dq_oe = 1'b1;
        phy_dq_o  = ca_slice(ca_idx);
        busy      = ~doing_init;
      end

      ST_LAT: begin
        phy_cs_n  = 1'b0;
        phy_ck_en = 1'b1;
        if (cur_read) begin
          phy_rd_arm = 1'b1;              // bus turn-around; slave drives DQ/RWDS
        end else begin
          phy_rwds_oe = 1'b1;             // latency write: master drives RWDS Low preamble (mask)
          phy_rwds_o  = 2'b00;
        end
        busy = ~doing_init;
      end

      ST_READ: begin
        phy_cs_n   = 1'b0;
        phy_ck_en  = 1'b1;
        phy_rd_arm = 1'b1;
        busy       = ~doing_init;
      end

      ST_WRITE: begin
        phy_cs_n  = 1'b0;
        phy_ck_en = 1'b1;
        phy_dq_oe = 1'b1;
        phy_dq_o  = wsrc_data;
        if (!zlw) begin
          phy_rwds_oe = 1'b1;            // RWDS = byte mask (High = masked); underrun => mask both
          phy_rwds_o  = wsrc_valid ? {~wsrc_strb[1], ~wsrc_strb[0]} : 2'b11;
        end
        if (wsrc_valid & ~doing_init) wr_ready = 1'b1;
        busy = ~doing_init;
      end

      ST_TAIL:     busy = ~doing_init;
      ST_RECOVER:  busy = ~doing_init;
      ST_RD_ABORT: busy = ~doing_init;   // CS# High (idle bus); draining flagged filler to the FIFO

      ST_RD_DRAIN: begin
        // Post-read over-stream drain: CS# held Low (still selected, tail window), CK STOPPED so the
        // device produces no new words, but the PHY receiver stays ARMED so any straggler/over-stream
        // words the device already launched are recovered (phy_dq_i_valid) and DISCARDED here — not
        // written to rd_fifo, not counted. Held until the PHY goes quiet, so nothing leaks to the next
        // read burst.
        phy_cs_n   = 1'b0;
        phy_rd_arm = 1'b1;
        busy       = ~doing_init;
      end

      default: ; // ST_POR, ST_INIT: idle bus
    endcase

    if (state == ST_TAIL) phy_cs_n = 1'b0;   // hold CS# Low through the tail
  end

  // Launch an internal COMMIT-READ (WR_COMMIT_READ): a linear memory read of COMMIT_READ_WORDS words
  // based at `base`, marked doing_commit so its recovered data is discarded (never enters rd_fifo).
  // Drives the transaction context exactly like a user read accepted in ST_IDLE, then enters ST_CS.
  task automatic launch_commit_read(input logic [ADDR_WIDTH-1:0] base);
    doing_commit <= 1'b1;
    cur_read     <= 1'b1;
    cur_reg      <= 1'b0;
    cur_wrap     <= 1'b0;
    cur_addr     <= base;
    rem_left     <= LEN_WIDTH'(COMMIT_READ_WORDS);
    seg_count    <= LEN_WIDTH'(COMMIT_READ_WORDS);
    seg_left     <= LEN_WIDTH'(COMMIT_READ_WORDS);
    ca_reg       <= hb_pack_ca(1'b1, 1'b0, 1'b1, base);   // read, memory, linear
    rwds_hi      <= 1'b0;
    ca_idx       <= 2'd0;
    state        <= ST_CS;
  endtask

  // ------------------------------------------------------------------------
  // Sequential FSM + datapath state
  // ------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    // registered error pulses default Low (one-cycle strobes)
    err_underrun <= 1'b0;
    err_timeout  <= 1'b0;

    if (rst) begin
      state        <= ST_RESET;
      cnt          <= 32'(RESET_CYCLES - 1);
      stall_cnt    <= '0;
      drain_cnt    <= '0;
      drain_dwell  <= '0;
      init_done    <= 1'b0;
      doing_init   <= 1'b0;
      doing_commit <= 1'b0;
      wr_pending_commit <= 1'b0;
      commit_resume     <= 1'b0;
      last_wr_addr <= '0;
      sv_addr      <= '0;
      sv_rem       <= '0;
      cur_read     <= 1'b0;
      cur_reg      <= 1'b0;
      cur_wrap     <= 1'b0;
      cur_addr     <= '0;
      rem_left     <= '0;
      seg_left     <= '0;
      seg_count    <= '0;
      ca_reg       <= '0;
      ca_idx       <= 2'd0;
      rwds_hi      <= 1'b0;
      lat_extra_done <= 1'b0;
      rd_wptr      <= '0;
      rd_rptr      <= '0;
    end else begin
      // read FIFO drain (independent of the FSM)
      if (~rd_fifo_empty & rd_ready) rd_rptr <= rd_rptr + 1'b1;

      unique case (state)
        // ---------------- POR / init ----------------
        ST_RESET: begin
          if (cnt == 32'd0) begin
            state <= ST_POR;
            cnt   <= 32'(POR_DELAY_CYCLES);
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        ST_POR: begin
          if (cnt == 32'd0) state <= ST_INIT;
          else              cnt   <= cnt - 1'b1;
        end

        ST_INIT: begin
          if (PROGRAM_CR) begin
            // internal register-space (zero-latency) write of CR0
            doing_init <= 1'b1;
            cur_read   <= 1'b0;
            cur_reg    <= 1'b1;
            cur_wrap   <= 1'b0;
            cur_addr   <= HB_REG_CR0[ADDR_WIDTH-1:0];
            rem_left   <= LEN_WIDTH'(1);
            seg_left   <= LEN_WIDTH'(1);
            seg_count  <= LEN_WIDTH'(1);
            ca_reg     <= hb_pack_ca(1'b0, 1'b1, 1'b1, HB_REG_CR0[ADDR_WIDTH-1:0]);
            rwds_hi    <= 1'b0;
            ca_idx     <= 2'd0;
            state      <= ST_CS;
          end else begin
            init_done <= 1'b1;
            state     <= ST_IDLE;
          end
        end

        // ---------------- idle / accept ----------------
        ST_IDLE: begin
          // Flush the read holding FIFO between transactions. A real HyperRAM keeps streaming read
          // data for a few extra words after the master has counted its burst (the master's CK stop
          // has pipeline latency), so a completed read can leave stray words buffered. Left in place
          // they keep rd_fifo non-empty, block the next command (cmd_ready gates on rd_fifo_empty),
          // and hang multi-burst reads on hardware. In ST_IDLE the previous transaction is fully
          // drained (its rd_last was delivered), so clearing the pointers only discards those extras.
          rd_wptr <= '0;
          rd_rptr <= '0;
          if (init_done & rd_fifo_empty) begin
            if (commit_gate) begin
              // DEFERRED write->write interpose: a new memory write is pending but the previous
              // write still needs committing. Self-issue the commit-read (spanning last_wr_addr);
              // the pending write command is NOT accepted (cmd_ready gated) and is taken after the
              // commit-read completes and returns to ST_IDLE.
              launch_commit_read(commit_base(last_wr_addr));
              commit_resume <= 1'b0;               // deferred: return to ST_IDLE afterwards
            end else if (cmd_valid) begin
              cur_read  <= cmd_read;
              cur_reg   <= cmd_reg;
              cur_wrap  <= cmd_wrap;
              cur_addr  <= cmd_addr;
              rem_left  <= cmd_len;
              seg_count <= seg_size(cmd_len, cmd_wrap, cmd_addr);
              seg_left  <= seg_size(cmd_len, cmd_wrap, cmd_addr);
              ca_reg    <= hb_pack_ca(cmd_read, cmd_reg, ~cmd_wrap, cmd_addr);
              rwds_hi   <= 1'b0;
              ca_idx    <= 2'd0;
              wr_pending_commit <= 1'b0;           // a normally-accepted command clears the pending
                                                   //   flag (a covering read commits the prior write)
              state     <= ST_CS;
            end
          end
        end

        // ---------------- CA phase ----------------
        ST_CS: begin
          state  <= ST_CA;
          ca_idx <= 2'd0;
        end

        ST_CA: begin
          if (phy_rwds_i) rwds_hi <= 1'b1;         // sample slave latency indicator
          if (ca_idx == 2'd2) begin
            if (zlw) begin
              state <= ST_WRITE;                   // zero-latency write: data follows CA immediately
            end else begin
              cnt            <= 32'(LATENCY_CLOCKS - 1);  // one latency count; doubled in ST_LAT if RWDS-high
              lat_extra_done <= 1'b0;
              state          <= ST_LAT;
            end
          end else begin
            ca_idx <= ca_idx + 1'b1;
          end
        end

        // ---------------- initial latency ----------------
        // Keep decoding the slave RWDS level throughout the latency window (not just CA): with the PHY
        // RWDS synchroniser the CA-High indicator may only become visible a cycle or two into ST_LAT.
        // When doubling is required, a SECOND latency count of LATENCY_CLOCKS is inserted before data
        // (SPEC_DIGEST §3: "an additional latency count is inserted"), so read/write data alignment
        // tracks the device's 2x initial latency exactly.
        ST_LAT: begin
          if (phy_rwds_i) rwds_hi <= 1'b1;
          if (cnt == 32'd0) begin
            if (lat_double & ~lat_extra_done) begin
              lat_extra_done <= 1'b1;
              cnt            <= 32'(LATENCY_CLOCKS - 1);   // insert the additional (2x) latency count
            end else if (cur_read) begin
              stall_cnt <= '0;
              state     <= ST_READ;
            end else begin
              state <= ST_WRITE;
            end
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        // ---------------- read data ----------------
        ST_READ: begin
          if (phy_dq_i_valid) begin
            // Commit-read data is DISCARDED (doing_commit): count the word for burst completion but
            // never push it to rd_fifo, so it is invisible to the front-end.
            if (!rd_fifo_full & ~doing_commit) begin
              rd_fifo[rd_wa] <= {(rem_left == LEN_WIDTH'(1)), phy_dq_i[DATA_WIDTH-1:0]};
              rd_wptr        <= rd_wptr + 1'b1;
            end
            stall_cnt <= '0;
            seg_left  <= seg_left - 1'b1;
            rem_left  <= rem_left - 1'b1;
            if (seg_left == LEN_WIDTH'(1)) begin
              if (rem_left != LEN_WIDTH'(1))       // chopped: advance to next linear segment
                cur_addr <= cur_addr + ADDR_WIDTH'(seg_count);
              // Enter the over-stream drain (CK stops, receiver stays armed) before the CS# tail, so
              // the device's post-CK-stop stragglers are absorbed and cannot leak into the next read.
              drain_cnt   <= '0;
              drain_dwell <= '0;
              state       <= ST_RD_DRAIN;
            end
          end else if (!phy_rwds_i) begin
            if (stall_cnt == 6'(READ_STALL_LIMIT - 1)) begin
              // RWDS stalled >= 32 clk (SPEC_DIGEST §4/§7): abort the read. Raise CS# (ST_RD_ABORT is
              // a bus-idle state) and, so the AXI/Avalon front-end's R channel still terminates with
              // exactly cmd_len beats + a final rd_last (AXI A3.4.1 / Avalon burst completion), drain
              // the remaining rem_left words as flagged filler rather than silently dropping rd_last.
              // A commit-read is internal (data discarded); a stall there must NOT surface as a
              // user-visible error, but it still drains through ST_RD_ABORT to recover cleanly.
              err_timeout <= ~doing_commit;
              state       <= ST_RD_ABORT;
            end else begin
              stall_cnt <= stall_cnt + 1'b1;
            end
          end
        end

        // ---------------- read over-stream drain (post burst) ----------------
        // CK is stopped (see combinational drive) so the device sources no new words, but the receiver
        // is held armed: any straggler/over-stream words the device already launched are recovered as
        // phy_dq_i_valid and DISCARDED here (never written to rd_fifo, never counted). Stay until the
        // PHY has been quiet for DRAIN_QUIET_CYCLES cycles (over-stream ended), bounded by
        // DRAIN_MAX_CYCLES. This absorbs a VARIABLE number of extra words so none leak into the next
        // read burst (the AXC3000 multi-burst hang). Then take the normal CS# tail + recovery.
        ST_RD_DRAIN: begin
          drain_dwell <= drain_dwell + 8'd1;
          if (phy_dq_i_valid) drain_cnt <= '0;             // a straggler arrived: not quiet yet
          else                drain_cnt <= drain_cnt + 8'd1;
          if (drain_done) begin
            cnt   <= 32'(TAIL_CYCLES - 1);
            state <= ST_TAIL;
          end
        end

        // ---------------- read abort drain (post-timeout) ----------------
        ST_RD_ABORT: begin
          if (rem_left == LEN_WIDTH'(0)) begin
            cnt   <= 32'(RECOVERY_CYCLES - 1);
            state <= ST_RECOVER;
          end else if (doing_commit | !rd_fifo_full) begin
            // Commit-read abort: discard (no fifo write, no back-pressure); else drain flagged filler.
            if (~doing_commit) begin
              rd_fifo[rd_wa] <= {(rem_left == LEN_WIDTH'(1)), {DATA_WIDTH{1'b0}}};
              rd_wptr        <= rd_wptr + 1'b1;
            end
            rem_left       <= rem_left - 1'b1;
            seg_left       <= (seg_left == LEN_WIDTH'(0)) ? seg_left : seg_left - 1'b1;
            if (rem_left == LEN_WIDTH'(1)) begin
              cnt   <= 32'(RECOVERY_CYCLES - 1);
              state <= ST_RECOVER;
            end
          end
        end

        // ---------------- write data ----------------
        ST_WRITE: begin
          if (!wsrc_valid) err_underrun <= 1'b1;   // host underrun: word gets masked, burst continues
          seg_left <= seg_left - 1'b1;
          rem_left <= rem_left - 1'b1;
          if (seg_left == LEN_WIDTH'(1)) begin
            // Last word of this write segment: record its (pre-advance) WORD address so a subsequent
            // commit-read (WR_COMMIT_READ) can span it.
            last_wr_addr <= cur_addr + ADDR_WIDTH'(seg_count) - ADDR_WIDTH'(1);
            if (rem_left != LEN_WIDTH'(1))          // chopped: advance to next linear segment
              cur_addr <= cur_addr + ADDR_WIDTH'(seg_count);
            cnt   <= 32'(TAIL_CYCLES - 1);
            state <= ST_TAIL;
          end
        end

        // ---------------- tail / recovery ----------------
        ST_TAIL: begin
          if (cnt == 32'd0) begin
            cnt   <= 32'(RECOVERY_CYCLES - 1);
            state <= ST_RECOVER;
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        ST_RECOVER: begin
          if (cnt == 32'd0) begin
            if (rem_left != LEN_WIDTH'(0)) begin
              // more linear segments remain in THIS transaction (chopped by tCSM / boundary)
              if (WR_COMMIT_READ & ~cur_read & ~cur_reg & ~doing_commit) begin
                // tCSM / boundary CHOP interpose: commit the just-closed write segment's last word
                // with an internal commit-read, remembering where to resume the write afterwards.
                sv_addr           <= cur_addr;        // next write-segment start (advanced in ST_WRITE)
                sv_rem            <= rem_left;
                commit_resume     <= 1'b1;
                wr_pending_commit <= 1'b1;            // segment closed -> needs committing
                launch_commit_read(commit_base(last_wr_addr));
              end else begin
                // normal reopen (read / reg / commit-read, or WR_COMMIT_READ disabled)
                seg_count <= seg_size(rem_left, cur_wrap, cur_addr);
                seg_left  <= seg_size(rem_left, cur_wrap, cur_addr);
                ca_reg    <= hb_pack_ca(cur_read, cur_reg, ~cur_wrap, cur_addr);
                rwds_hi   <= 1'b0;
                ca_idx    <= 2'd0;
                state     <= ST_CS;
              end
            end else if (doing_commit) begin
              // Internal commit-read finished (its recovered data was discarded).
              doing_commit      <= 1'b0;
              wr_pending_commit <= 1'b0;
              if (commit_resume) begin
                // Intra-command split write: restore the shadow context, reopen the write segment.
                commit_resume <= 1'b0;
                cur_read  <= 1'b0;
                cur_reg   <= 1'b0;
                cur_wrap  <= 1'b0;
                cur_addr  <= sv_addr;
                rem_left  <= sv_rem;
                seg_count <= seg_size(sv_rem, 1'b0, sv_addr);
                seg_left  <= seg_size(sv_rem, 1'b0, sv_addr);
                ca_reg    <= hb_pack_ca(1'b0, 1'b0, 1'b1, sv_addr);   // write, memory, linear
                rwds_hi   <= 1'b0;
                ca_idx    <= 2'd0;
                state     <= ST_CS;
              end else begin
                state <= ST_IDLE;                     // deferred interpose done: take the pending cmd
              end
            end else begin
              // Normal user / init transaction complete.
              if (~cur_read & ~cur_reg & ~doing_init)
                wr_pending_commit <= 1'b1;            // a memory write closed -> last word pending
              if (doing_init) begin
                init_done  <= 1'b1;
                doing_init <= 1'b0;
              end
              state <= ST_IDLE;
            end
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        default: state <= ST_IDLE;
      endcase
    end
  end

  // wr_last is informational only (burst length is authoritative); waive unused-signal lint.
  /* verilator lint_off UNUSEDSIGNAL */
  logic _unused_wr_last;
  /* verilator lint_on UNUSEDSIGNAL */
  always_comb _unused_wr_last = wr_last;

  // DEBUG taps.
  assign dbg_state   = 4'(state);
  assign dbg_rd_wptr = 6'(rem_left);   // repurposed: rem_left (words remaining in whole burst)
  assign dbg_rd_rptr = 6'(seg_left);   // repurposed: seg_left (words remaining in current segment)

endmodule
`endif
