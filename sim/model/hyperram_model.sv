// hyperram_model — behavioral, spec-accurate HyperRAM DEVICE (slave) model.
//
// GOLDEN reference model for the HyperBus master IP testbenches. Clean-room from
// docs/SPEC_DIGEST.md (Infineon/Cypress HyperBus 001-99253 Rev *H); no reference code copied.
//
// Scope (SPEC_DIGEST §2–§8):
//   * Decodes the 48-bit Command-Address (CA), MSB byte first, over the first 6 CK edges.
//   * Honors fixed / variable initial latency (SPEC_DIGEST §3): drives RWDS during CA as the
//     latency indicator (High = 2x, Low = 1x); variable-latency doubling is driven by a
//     deterministic, parameterized internal refresh-collision model (there is no external
//     collision port in the frozen interface — see REFRESH_EVERY).
//   * Drives RWDS as the source-synchronous read strobe, edge-aligned to read data.
//   * Accepts the master-driven RWDS write byte-mask during latency-bearing memory writes
//     (RWDS High => byte masked / array unchanged).
//   * Register-space writes are zero-latency, full-word, unmasked (SPEC_DIGEST §5/§6).
//   * Linear, legacy-wrapped and hybrid-wrapped bursts (SPEC_DIGEST §7).
//   * CR0/CR1 (writable) and ID0/ID1 (read-only) register space with reset values from the
//     package, always big-endian (byte A = word[15:8]).
//   * A backing memory array; optional mid-burst row-crossing latency gap (RWDS held Low).
//
// This is a PROTOCOL-accurate model, not an AC-timing model. hb_ck is a real DDR clock: one DQ
// byte is transferred on every CK edge (two bytes / cycle). Byte A (word[15:8]) is on the CK-High
// (rising) phase, byte B (word[7:0]) on the CK-Low (falling) phase (SPEC_DIGEST §4).
//
// Alignment contract for read data (race-free under verilator --binary):
//   During the read-data phase the model drives RWDS = CK and presents the current word
//   combinationally (byte A while CK High, byte B while CK Low). Data and RWDS transition together
//   at CK edges and each byte is valid for a full half-period, so a PHY that samples the eye centre
//   (e.g. off a 90-degree-shifted clock) captures cleanly. During a row-crossing gap RWDS is held
//   Low and no strobe is produced (SPEC_DIGEST §4/§7).
//
// Split-driver pins (Verilator-safe; the TB resolves the shared bus — no inout in the model). The
// model consumes what the master drives (hb_dq_i / hb_rwds_i, with the master's *_ie enables) and
// exposes what it drives (hb_dq_o / hb_rwds_o with its own *_oe). See docs/INTERFACES.md.
`ifndef HYPERRAM_MODEL_SV
`define HYPERRAM_MODEL_SV
`timescale 1ns/1ps
module hyperram_model
  import hyperbus_pkg::*;
#(
    parameter int unsigned DQ_WIDTH       = HB_DQ_WIDTH_DEFAULT,      // 8 (HyperBus is 8-bit DQ)
    parameter int unsigned MEM_WORDS      = 1 << 16,                  // backing array depth (words)
    parameter int unsigned LATENCY_CLOCKS = HB_LATENCY_CLOCKS_DEFAULT,// initial latency (CK cycles)
    parameter bit          FIXED_LATENCY  = HB_FIXED_LATENCY_DEFAULT, // 1 = fixed, 0 = variable
    parameter int unsigned ROW_WORDS      = 64,                       // row size (words); 0 disables gaps
    parameter int unsigned ROW_PENALTY    = 4,                        // extra latency (CK cycles) per row crossing
    parameter int unsigned REFRESH_EVERY  = 0,                        // variable-latency collision cadence:
                                                                      //   0 = never; N>0 = every N-th transaction
                                                                      //   inserts the additional latency count.
    parameter bit          FIXED_2X       = 1'b0,                     // SPEC_DIGEST §5.2.4 fixed-latency
                                                                      //   variant: when in FIXED mode, drive
                                                                      //   RWDS HIGH during CA and always use
                                                                      //   two latency counts (2x). This is the
                                                                      //   common Cypress/Winbond behaviour.
    parameter int unsigned STALL_AT       = 0,                        // read-stall injection: word index into a
    parameter int unsigned STALL_CLOCKS   = 0,                        //   read burst at which to hold RWDS Low
                                                                      //   for STALL_CLOCKS CK cycles (0=never).
                                                                      //   Models the >=32-clk error stall.
    parameter int unsigned RD_PREAMBLE_CLOCKS = 0,                    // read-strobe PREAMBLE (CK cycles): before
                                                                      //   the first real read byte the device
                                                                      //   toggles RWDS (=CK) with DQ Hi-Z (reads
                                                                      //   as 0x00). Reproduces the real Winbond
                                                                      //   W957D8NB read turnaround captured on the
                                                                      //   AXC3000 board (cap idx85-88). 0 = none
                                                                      //   (spec-ideal; keeps existing TBs aligned).
    parameter int unsigned RD_OVERSTREAM_WORDS = 0,                   // READ over-stream (CK-stop pipeline latency):
                                                                      //   after the master STOPS CK ending a read burst
                                                                      //   the real W957D8NB keeps pushing this many EXTRA
                                                                      //   source-synchronous words (RWDS + DQ) before its
                                                                      //   read output pipeline drains — the master's CK-stop
                                                                      //   has flight/pipeline latency, so the device over-runs
                                                                      //   the master's word count (AXC3000: ~23 words seen for
                                                                      //   a 16-word request). The ideal edge-driven data phase
                                                                      //   below stops the instant CK stops and never exercised
                                                                      //   this; the SELF-TIMED tail below reproduces it.
                                                                      //   0 = ideal device (keeps all existing TBs aligned).
    parameter bit          WR_COMMIT_QUIRK = 1'b0,                     // W957D8NB SPLIT-WRITE commit quirk (issue #1):
                                                                      //   when 1, the FINAL word of every memory WRITE
                                                                      //   burst is held pending (NOT committed to mem[])
                                                                      //   and is committed only when a subsequent READ
                                                                      //   transaction covers that address and delivers
                                                                      //   >=2 words; a subsequent WRITE drops it. 0 =
                                                                      //   ideal device (immediate commit; keeps TBs aligned).
    parameter int unsigned BURST_BOUNDARY_WORDS = 0,                  // W957D8NB 0x2000-WORD boundary quirk: when a
                                                                      //   single burst crosses this WORD-aligned boundary
                                                                      //   the device RELEASES the bus (stops driving on
                                                                      //   reads / stops capturing on writes) for the rest
                                                                      //   of that CS#; reads past it return floating junk.
                                                                      //   0 = disabled (keeps existing TBs aligned).
    parameter logic [15:0] ID0_RESET      = HB_ID0_RESET,             // read-only device ID (mfr nibble)
    parameter logic [15:0] ID1_RESET      = HB_ID1_RESET,             // read-only device ID (type nibble)
    parameter logic [15:0] CR0_RESET      = HB_CR0_RESET,             // config register 0 reset image
    parameter logic [15:0] CR1_RESET      = HB_CR1_RESET              // config register 1 reset image
) (
    input  logic                 hb_ck,      // clock from master
    input  logic                 hb_ck_n,    // complementary clock (tied if single-ended)
    input  logic                 hb_cs_n,    // chip select, active low
    input  logic                 hb_rst_n,   // device reset, active low
    input  logic [DQ_WIDTH-1:0]  hb_dq_i,    // DQ driven BY MASTER (CA / write data)
    input  logic                 hb_dq_ie,   // master DQ output-enable (turnaround awareness)
    output logic [DQ_WIDTH-1:0]  hb_dq_o,    // DQ driven BY MODEL (read data)
    output logic                 hb_dq_oe,   // model DQ output-enable
    input  logic                 hb_rwds_i,  // RWDS driven BY MASTER (write byte-mask)
    input  logic                 hb_rwds_ie, // master RWDS output-enable
    output logic                 hb_rwds_o,  // RWDS driven BY MODEL (CA latency indicator + read strobe)
    output logic                 hb_rwds_oe  // model RWDS output-enable
);

  // ------------------------------------------------------------------------
  // Local geometry. HyperBus words are 16-bit; DQ is DQ_WIDTH (8) so one word = 2 bytes.
  // ------------------------------------------------------------------------
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16-bit HyperBus word
  localparam int unsigned AW         = $clog2(MEM_WORDS);     // word-address width into the array
  localparam int unsigned CA_SR_BITS = HB_CA_BITS - DQ_WIDTH; // CA bytes 0..4 retained; byte 5 is live

  // Backing array (word-addressed). Not cleared by hardware reset (spec: reset restores config
  // registers, not the array).
  logic [DATA_WIDTH-1:0] mem [MEM_WORDS];

  // Config / ID registers (16-bit, always big-endian on the wire).
  logic [DATA_WIDTH-1:0] id0, id1, cr0, cr1;
  logic                  cr0_written; // once CR0 is programmed, latency follows the register

  // ------------------------------------------------------------------------
  // Transaction state (all updated in the single CK-edge process below).
  // ------------------------------------------------------------------------
  logic                    ck_q;            // previous CK level, for edge detection
  logic                    cs_q;            // previous CS# level, for start-of-transaction detection
  logic [15:0]             beat;            // number of CK edges processed since CS# fell
  logic [CA_SR_BITS-1:0]   ca_sr;           // CA shift register (MSB byte first; holds bytes 0..4)
  logic                    cur_rw;          // CA[47] : 1 = read
  logic                    cur_as;          // CA[46] : 1 = register space
  logic                    cur_linear;      // CA[45] : 1 = linear burst
  logic [HB_ADDR_WIDTH-1:0]cur_reg_addr;    // decoded word address (register-space compare)
  logic [15:0]             first_data_beat; // beat index at which the first data byte (A) appears
  logic                    collide;         // this transaction inserts the additional latency count
  logic [31:0]             txn_cnt;         // transactions since reset (drives the refresh model)

  logic [AW-1:0]           addr;            // running memory word address (data phase)
  logic [31:0]             wcnt;            // words delivered so far (wrap / hybrid bookkeeping)
  logic [AW-1:0]           group_base;      // wrap-group base address
  logic [AW-1:0]           group_top;       // wrap-group top address
  logic [AW-1:0]           wrap_words_r;    // wrap-group size in words (latched at CA decode)
  logic [15:0]             pen;             // row-crossing gap countdown (CK edges), reads only
  logic [15:0]             pen_pending;     // gap armed at byte B, activated at the next word slot
  logic [DATA_WIDTH-1:0]   out_word;        // read word currently being driven onto DQ
  logic [DQ_WIDTH-1:0]     wr_hi;           // captured byte A during a register write
  logic                    stall_done;      // read-stall (STALL_CLOCKS) already injected this txn

  // ---- split-write commit quirk (WR_COMMIT_QUIRK) ----
  // `hold` = the most recent memory-write word within the CURRENT burst (delayed one word so the
  // burst's LAST word is never committed here); at CS# deassert it moves to `pend`. `pend` = the last
  // burst's held word, awaiting a covering read (commit) or the next write (drop).
  logic                    hold_valid;
  logic [AW-1:0]           hold_addr;
  logic [DATA_WIDTH-1:0]   hold_word;
  logic [1:0]              hold_we;         // per-byte write-enable {A,B} (unmasked bytes)
  logic                    whi_we;          // byte-A write-enable, captured on the rising edge
  logic                    pend_valid;
  logic [AW-1:0]           pend_addr;
  logic [DATA_WIDTH-1:0]   pend_word;
  logic [1:0]              pend_we;

  // ---- 0x2000-word boundary release quirk (BURST_BOUNDARY_WORDS) ----
  logic                    bnd_rel;         // this transaction crossed a boundary -> bus released

  // ---- read over-stream tail (self-timed; models the CK-stop pipeline latency) ----
  realtime                 os_half;         // measured read-data CK half-period (tail cadence)
  realtime                 os_last_edge;    // $realtime of the most recent read-data CK edge (0 = none)
  logic                    os_run;          // 1 while the device drives the over-stream tail
  logic                    os_rwds_o;       // tail RWDS drive
  logic [DQ_WIDTH-1:0]     os_dq_o;         // tail DQ byte drive

  // ------------------------------------------------------------------------
  // Combinational helpers.
  // ------------------------------------------------------------------------
  // Selected + not in reset.
  wire busy = hb_rst_n & ~hb_cs_n;

  // The 48-bit CA becomes complete on the 6th CA byte (this cycle's hb_dq_i is the last byte).
  wire [HB_CA_BITS-1:0]     full_ca   = {ca_sr, hb_dq_i};
  /* verilator lint_off UNUSEDSIGNAL */                            // upper address bits > array depth
  wire [HB_ADDR_WIDTH-1:0]  full_addr = hb_ca_addr(full_ca);       // decoded WORD address
  /* verilator lint_on UNUSEDSIGNAL */
  wire [AW-1:0]             start_addr= full_addr[AW-1:0];         // into the backing array
  wire [AW-1:0]             wrap_w    = AW'(hb_wrap_words(cr0[1:0]));

  // Effective fixed/variable and latency count: parameter defaults until CR0 is programmed, then
  // the CR0 register governs (SPEC_DIGEST §3, CR0[3] / CR0[7:4]).
  wire        fixed_active = cr0_written ? cr0[3] : (FIXED_LATENCY != 1'b0);
  wire [15:0] lat_active   = cr0_written ? 16'(hb_latency_code_to_clocks(cr0[7:4]))
                                         : 16'(LATENCY_CLOCKS);

  // Register-space readback value (aliased decode on the low word address).
  wire [DATA_WIDTH-1:0] reg_val =
      (cur_reg_addr == HB_REG_ID0) ? id0 :
      (cur_reg_addr == HB_REG_ID1) ? id1 :
      (cur_reg_addr == HB_REG_CR0) ? cr0 :
      (cur_reg_addr == HB_REG_CR1) ? cr1 : '0;

  // Next-address generator: linear increment, legacy wrap-in-group, or hybrid (wrap once then
  // linear from the next group boundary). SPEC_DIGEST §7 / Table 5.4.
  function automatic logic [AW-1:0] next_addr(input logic [AW-1:0] a, input logic [31:0] cnt);
    logic [AW-1:0] n;
    if (cur_linear) begin
      n = a + 1'b1;                                   // linear
    end else if (cr0[2]) begin
      n = (a == group_top) ? group_base : a + 1'b1;   // legacy wrap: stay in group forever
    end else begin                                    // hybrid wrap: one traversal, then linear
      if (cnt == {{(32-AW){1'b0}}, wrap_words_r} - 1)
        n = group_base + wrap_words_r;                // leave the group at the next boundary
      else if (cnt >= {{(32-AW){1'b0}}, wrap_words_r})
        n = a + 1'b1;                                 // linear tail
      else
        n = (a == group_top) ? group_base : a + 1'b1; // still wrapping
    end
    return n;
  endfunction

  // ------------------------------------------------------------------------
  // Backing-store initialization. Deterministic byte pattern so read-before-write is predictable
  // and matches the byte-addressed convention (byte at byte-address i = i*13+7): word a holds
  // {byte(2a), byte(2a+1)} = {A=high, B=low}.
  // ------------------------------------------------------------------------
  initial begin
    for (int unsigned a = 0; a < MEM_WORDS; a++)
      mem[a] = { DQ_WIDTH'((2*a  )*13 + 7),
                 DQ_WIDTH'((2*a+1)*13 + 7) };
  end

  // ------------------------------------------------------------------------
  // Single edge-detecting process. hb_ck is not free-running (idle while CS# High), so CS#/RST#
  // are in the sensitivity list; a stored ck_q recovers both CK edges (DDR). Verified to elaborate
  // and simulate clean under verilator --binary 5.020.
  // ------------------------------------------------------------------------
  always @(hb_ck or hb_cs_n or hb_rst_n) begin
    if (!hb_rst_n) begin
      // Hardware reset: restore config/ID registers, drop to idle (SPEC_DIGEST §9).
      beat        <= '0;
      ca_sr       <= '0;
      pen         <= '0;
      pen_pending <= '0;
      wcnt        <= '0;
      stall_done  <= 1'b0;
      hold_valid  <= 1'b0;
      pend_valid  <= 1'b0;
      bnd_rel     <= 1'b0;
      cs_q        <= 1'b1;
      ck_q        <= hb_ck;
      txn_cnt     <= '0;
      cr0_written <= 1'b0;
      id0         <= DATA_WIDTH'(ID0_RESET);
      id1         <= DATA_WIDTH'(ID1_RESET);
      cr0         <= DATA_WIDTH'(CR0_RESET);
      cr1         <= DATA_WIDTH'(CR1_RESET);
    end else if (hb_cs_n) begin
      // Idle between transactions: clear per-transaction state, keep registers.
      beat  <= '0;
      ca_sr <= '0;
      pen   <= '0;
      pen_pending <= '0;
      wcnt  <= '0;
      cs_q  <= 1'b1;
      ck_q  <= hb_ck;
      bnd_rel <= 1'b0;                         // boundary-release latch is per-transaction
      // Split-write quirk: the word still held at CS# deassert is this burst's UNCOMMITTED last word.
      // Move it to `pend` to await a covering read (commit) or the next write (drop). Idempotent.
      if (WR_COMMIT_QUIRK && hold_valid) begin
        pend_valid <= 1'b1;
        pend_addr  <= hold_addr;
        pend_word  <= hold_word;
        pend_we    <= hold_we;
        hold_valid <= 1'b0;
      end
    end else if (cs_q) begin
      // CS# just fell: start of transaction. Decide the refresh-collision for this transaction
      // now, so the RWDS latency indicator is stable throughout the whole CA period.
      cs_q    <= 1'b0;
      ck_q    <= hb_ck;
      beat    <= '0;
      ca_sr   <= '0;
      pen     <= '0;
      pen_pending <= '0;
      wcnt    <= '0;
      stall_done <= 1'b0;
      collide <= (!fixed_active) && (REFRESH_EVERY != 0) &&
                 (((txn_cnt + 1) % REFRESH_EVERY) == 0);
      txn_cnt <= txn_cnt + 1;
    end else if (hb_ck !== ck_q) begin
      // A CK edge while selected. hb_ck High => rising edge (byte A); Low => falling (byte B).
      logic [15:0] bnew;
      ck_q <= hb_ck;
      bnew = beat + 16'd1;   // 1-based index of the edge being processed

      if (bnew <= 16'd6) begin
        // -------- Command-Address phase (6 bytes, MSB first) --------
        ca_sr <= {ca_sr[CA_SR_BITS-DQ_WIDTH-1:0], hb_dq_i};
        if (bnew == 16'd6) begin
          // Full CA captured this edge: decode and set up the data phase.
          logic [15:0] eff;
          cur_rw       <= hb_ca_read(full_ca);
          cur_as       <= hb_ca_reg(full_ca);
          cur_linear   <= hb_ca_linear(full_ca);
          cur_reg_addr <= full_addr;

          // Effective initial latency (CK cycles). Register-space writes are zero-latency
          // (SPEC_DIGEST §5/§6); variable mode doubles the count on a refresh collision.
          if (hb_ca_reg(full_ca) && !hb_ca_read(full_ca))
            eff = 16'd0;
          else if (fixed_active)
            eff = FIXED_2X ? (lat_active << 1) : lat_active;   // §5.2.4 fixed-2x variant
          else
            eff = collide ? (lat_active << 1) : lat_active;
          // First data byte (A) lands on the rising edge after CA (6 edges) + eff latency cycles:
          //   edge index = 6 + 2*eff + 1 = 7 + 2*eff.
          first_data_beat <= 16'd7 + (eff << 1);

          // Split-write quirk: a NEW memory WRITE drops any pending (uncommitted) word from the
          // previous write burst (models "a following WRITE leaves the pending word uncommitted").
          // A read (incl. the master's commit-read) does NOT drop it — it commits it below.
          if (WR_COMMIT_QUIRK && !hb_ca_read(full_ca) && !hb_ca_reg(full_ca))
            pend_valid <= 1'b0;

          // Data-phase address / burst setup.
          addr         <= start_addr;
          wcnt         <= '0;
          wrap_words_r <= wrap_w;
          group_base   <= start_addr & ~(wrap_w - 1'b1);
          group_top    <= (start_addr & ~(wrap_w - 1'b1)) | (wrap_w - 1'b1);
        end
      end else if (bnew >= first_data_beat) begin
        // -------- Data phase --------
        if (cur_rw) begin
          // READ: the model sources data. Advance/latch on the rising edge (byte A) that starts a
          // word; deliver byte B on the falling edge; a row-crossing gap is armed at byte B and
          // activated at the *next* word slot so it never shadows the completing word's byte B.
          if (pen != 0) begin
            pen <= pen - 16'd1;                    // inside a row-crossing gap: no transfer
          end else if (pen_pending != 0) begin     // gap begins now (a would-be byte-A rising edge)
            pen         <= pen_pending - 16'd1;     // hold RWDS Low for the row-crossing penalty
            pen_pending <= 16'd0;
          end else if (hb_ck && (STALL_CLOCKS != 0) && !stall_done &&
                       (wcnt == 32'(STALL_AT))) begin
            // Inject a long RWDS-Low stall at word STALL_AT (models the >=32-clk error stall,
            // SPEC_DIGEST §4/§7). Hold RWDS Low for STALL_CLOCKS CK cycles (2 edges each) without
            // delivering a word; the master must abort. Only injected once per transaction.
            pen        <= 16'(2 * STALL_CLOCKS);
            stall_done <= 1'b1;
          end else if (hb_ck) begin                // rising edge: start of a word
            // Boundary-release quirk: stepping ONTO a BURST_BOUNDARY_WORDS-aligned address past the
            // first word crosses the boundary -> the device releases the bus for the rest of this
            // CS# (see the drive block). Sticky until CS# deassert.
            if ((BURST_BOUNDARY_WORDS != 0) && (wcnt != 32'd0) &&
                ((32'(addr) % BURST_BOUNDARY_WORDS) == 32'd0))
              bnd_rel <= 1'b1;
            // Split-write quirk: reading the pending address returns the held (uncommitted) value;
            // if the read delivers >=2 words (this is at least the 2nd, wcnt>=1) it COMMITS it.
            if (WR_COMMIT_QUIRK && pend_valid && !cur_as && (addr == pend_addr)) begin
              out_word <= pend_word;
              if (wcnt >= 32'd1) begin
                if (pend_we[1]) mem[pend_addr][DATA_WIDTH-1:DQ_WIDTH] <= pend_word[DATA_WIDTH-1:DQ_WIDTH];
                if (pend_we[0]) mem[pend_addr][DQ_WIDTH-1:0]          <= pend_word[DQ_WIDTH-1:0];
                pend_valid <= 1'b0;
              end
            end else begin
              out_word <= cur_as ? reg_val : mem[addr];
            end
            addr     <= next_addr(addr, wcnt);
            wcnt     <= wcnt + 32'd1;
          end else begin                           // falling edge: byte B just delivered
            if ((ROW_WORDS != 0) && !cur_as && ((addr % AW'(ROW_WORDS)) == 0))
              pen_pending <= 16'(2 * ROW_PENALTY);  // upcoming word starts a new row -> arm a gap
          end
        end else begin
          // WRITE: the master sources data; the model captures on both edges. Memory writes honor
          // the RWDS byte-mask (High = masked); register writes are unmasked full words.
          if (hb_ck) begin                         // rising edge: byte A (word[15:8])
            if (cur_as)
              wr_hi <= hb_dq_i;
            else if (WR_COMMIT_QUIRK) begin
              wr_hi  <= hb_dq_i;                    // delayed-commit: buffer byte A + its mask
              whi_we <= ~hb_rwds_i;
            end else if (!hb_rwds_i && !bnd_rel)
              mem[addr][DATA_WIDTH-1:DQ_WIDTH] <= hb_dq_i;
          end else begin                           // falling edge: byte B (word[7:0])
            // Boundary-release quirk (writes): once the burst steps onto a boundary the device stops
            // capturing for the rest of this CS# -> post-boundary words are never stored.
            if ((BURST_BOUNDARY_WORDS != 0) && (wcnt != 32'd0) &&
                ((32'(addr) % BURST_BOUNDARY_WORDS) == 32'd0))
              bnd_rel <= 1'b1;
            if (cur_as) begin
              if (cur_reg_addr == HB_REG_CR0) begin
                cr0         <= {wr_hi, hb_dq_i};    // big-endian: A=high, B=low
                cr0_written <= 1'b1;
              end else if (cur_reg_addr == HB_REG_CR1) begin
                cr1 <= {wr_hi, hb_dq_i};
              end
              // ID0/ID1 are read-only: writes ignored.
            end else if (WR_COMMIT_QUIRK) begin
              // Commit the PREVIOUS held word (never the current), so this burst's LAST word stays in
              // `hold` at CS# deassert (-> pend). Honor per-byte masks; skip stores once bus-released.
              if (hold_valid && !bnd_rel) begin
                if (hold_we[1]) mem[hold_addr][DATA_WIDTH-1:DQ_WIDTH] <= hold_word[DATA_WIDTH-1:DQ_WIDTH];
                if (hold_we[0]) mem[hold_addr][DQ_WIDTH-1:0]          <= hold_word[DQ_WIDTH-1:0];
              end
              hold_addr  <= addr;
              hold_word  <= {wr_hi, hb_dq_i};
              hold_we    <= {whi_we, ~hb_rwds_i};
              hold_valid <= 1'b1;
            end else if (!hb_rwds_i && !bnd_rel) begin
              mem[addr][DQ_WIDTH-1:0] <= hb_dq_i;
            end
            addr <= next_addr(addr, wcnt);
            wcnt <= wcnt + 32'd1;
          end
        end
      end
      // else: initial-latency edges (bnew in 7..first_data_beat-1) — nothing captured/driven here.

      beat <= bnew;
    end
  end

  // ------------------------------------------------------------------------
  // Read over-stream tail (self-timed) — models the on-silicon CK-stop pipeline latency.
  //
  // Physical effect (AXC3000 W957D8NB): after the master stops CK to end a read burst the device
  // keeps driving a few more source-synchronous words before its read output pipeline empties, so
  // the master's CK-stop over-runs its word count. The edge-driven data phase above stops the instant
  // hb_ck stops (ideal device) and never reproduced this. Here a self-timed generator, armed once the
  // master stops clocking mid-read, drives RD_OVERSTREAM_WORDS extra {RWDS-rise, RWDS-fall} word
  // strobes at the measured CK cadence — exactly the stray words the master's read FIFOs must discard.
  //
  // os_last_edge tracks the most recent read-data CK edge (cleared to 0 when CS# is High / between
  // read bursts and after a tail is emitted, so the tail arms exactly once per read burst).
  // ------------------------------------------------------------------------
  always @(hb_ck or hb_cs_n) begin
    if (hb_cs_n) begin
      os_last_edge = 0.0;                                  // idle: disarm the tail watchdog
    end else if (cur_rw && (beat >= first_data_beat)) begin
      if (os_last_edge != 0.0) os_half = $realtime - os_last_edge;  // half-period between CK edges
      os_last_edge = $realtime;
    end
  end

  initial begin
    os_run = 1'b0; os_rwds_o = 1'b0; os_dq_o = '0;
    os_half = 10ns; os_last_edge = 0.0;
  end

  // Watchdog: when the gap since the last read-data CK edge exceeds ~1.8 half-periods the master has
  // stopped clocking mid-read; drive the over-stream tail, then release and re-arm on the next burst.
  always begin
    #1;
    if ((RD_OVERSTREAM_WORDS != 0) && !os_run && hb_rst_n && !hb_cs_n &&
        cur_rw && (beat >= first_data_beat) && (os_last_edge != 0.0) &&
        (($realtime - os_last_edge) > (os_half * 1.8))) begin
      logic [AW-1:0]         os_addr;
      logic [DATA_WIDTH-1:0] os_word;
      os_run  = 1'b1;
      os_addr = addr;                                       // continue from where the data phase left off
      for (int k = 0; (k < int'(RD_OVERSTREAM_WORDS)) && hb_rst_n; k++) begin
        os_word = mem[os_addr];
        os_addr = os_addr + 1'b1;
        os_rwds_o = 1'b1; os_dq_o = os_word[DATA_WIDTH-1:DQ_WIDTH]; #(os_half);  // byte A (RWDS rise)
        os_rwds_o = 1'b0; os_dq_o = os_word[DQ_WIDTH-1:0];          #(os_half);  // byte B (RWDS fall)
      end
      os_rwds_o    = 1'b0;
      os_last_edge = 0.0;                                   // consumed: re-arm on the next read burst
      os_run       = 1'b0;
    end
  end

  // ------------------------------------------------------------------------
  // Combinational bus drive. Ownership per interface state, SPEC_DIGEST §4 Table 7.1:
  //   CA period          : model drives RWDS = latency indicator (High => additional count).
  //   Read init latency  : model drives RWDS Low; DQ High-Z (bus turn-around).
  //   Read data          : model drives DQ + RWDS (RWDS = CK strobe, edge-aligned; Low in gaps).
  //   Write latency/data : model releases DQ and RWDS (master owns both).
  // ------------------------------------------------------------------------
  always_comb begin
    hb_dq_o   = '0;
    hb_dq_oe  = 1'b0;
    hb_rwds_o = 1'b0;
    hb_rwds_oe= 1'b0;
    if (busy) begin
      if (beat < 16'd6) begin
        // Command-Address: slave drives the RWDS latency indicator (High = 2x latency count).
        // Variable mode: High only on a refresh collision. Fixed-2x variant (§5.2.4): always High.
        hb_rwds_oe = 1'b1;
        hb_rwds_o  = collide | (fixed_active & FIXED_2X);
      end else if (cur_rw) begin
        if (beat < first_data_beat) begin
          // Read initial access latency: DQ Hi-Z (turn-around), RWDS driven by the slave.
          hb_rwds_oe = 1'b1;
          // Read PREAMBLE (SPEC-real device): for the final RD_PREAMBLE_CLOCKS CK cycles of the
          // turn-around window the device already toggles RWDS (= CK) while DQ is still Hi-Z (reads
          // 0x00). This is the AXC3000-captured W957D8NB behaviour (cap idx85-88): RWDS edges appear
          // BEFORE the first real data byte. DQ stays Hi-Z (hb_dq_oe=0 => resolves to 0 on the bus).
          if ((RD_PREAMBLE_CLOCKS != 0) &&
              (beat >= (first_data_beat - 16'(2*RD_PREAMBLE_CLOCKS))))
            hb_rwds_o = hb_ck;      // preamble strobe: RWDS toggles like CK, DQ = 0
          else
            hb_rwds_o = 1'b0;       // plain turn-around: RWDS held Low
        end else if (bnd_rel) begin
          // Boundary-release quirk: the device has let go of the bus (DQ + RWDS Hi-Z). The master
          // sees no strobe -> it reads floating junk and eventually stalls/aborts (SPEC_DIGEST §4/§7).
          hb_dq_oe   = 1'b0;
          hb_rwds_oe = 1'b0;
        end else begin
          // Read data: source-synchronous strobe + data, edge-aligned.
          hb_dq_oe   = 1'b1;
          hb_rwds_oe = 1'b1;
          if (pen != 0) begin
            // Row-crossing gap: hold RWDS Low (no strobe); data is don't-care to the master.
            hb_rwds_o = 1'b0;
            hb_dq_o   = out_word[DATA_WIDTH-1:DQ_WIDTH];
          end else begin
            hb_rwds_o = hb_ck;   // strobe transitions with CK (edge-aligned to data)
            hb_dq_o   = hb_ck ? out_word[DATA_WIDTH-1:DQ_WIDTH]   // byte A on CK High
                              : out_word[DQ_WIDTH-1:0];           // byte B on CK Low
          end
        end
      end
      // WRITE (cur_rw==0), beat>=6: model releases DQ and RWDS — the master drives them.
    end
    // Over-stream tail overrides the bus: the device keeps driving RWDS+DQ after CK stops (the
    // read output pipeline drains past the master's CK-stop). Wins over the CK-gated read drive
    // above (hb_ck is idle here) and over the idle release when CS# has already gone High.
    if (os_run) begin
      hb_dq_o    = os_dq_o;
      hb_dq_oe   = 1'b1;
      hb_rwds_o  = os_rwds_o;
      hb_rwds_oe = 1'b1;
    end
  end

  // ------------------------------------------------------------------------
  // Intentionally-unused inputs (differential clock leg + master output-enable hints). The model
  // derives all phase information from hb_ck and the transaction FSM.
  // ------------------------------------------------------------------------
  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, hb_ck_n, hb_dq_ie, hb_rwds_ie};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
`endif
