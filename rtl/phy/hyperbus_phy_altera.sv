// hyperbus_phy_altera — Intel/Altera **Agilex 3** DDR-IO HyperBus PHY (REAL synthesis target).
//
// Drop-in variant of the frozen `hyperbus_phy` contract (docs/INTERFACES.md): identical port list
// and identical controller-facing timing contract as `hyperbus_phy_generic`, but the DDR serialise /
// deserialise and the forwarded HyperBus clock are built from Agilex-3 (tennm_ph2 / "PH2") hard I/O
// primitives instead of the generic behavioural muxes. This is the file the board build compiles.
//
//   *** NOT Verilator-simulable — that is expected. ***
//   It instantiates device primitives (tennm_ph2_ddio_out / _ddio_oe / _ddio_in) that exist only in
//   the Quartus/Agilex simulation & synthesis libraries. For RTL/algorithm simulation use
//   PHY_VARIANT="GENERIC" (hyperbus_phy_generic), which this module is a bit-for-bit behavioural
//   match of at the controller boundary. Correctness of THIS file is proven by the Quartus fitter /
//   timing reports and by on-hardware bring-up, not by Verilator.
//
//   *** READ-EYE CALIBRATION NOTE ***
//   The read datapath is source-synchronous: read DQ is captured with a DDIO clocked by the returning
//   RWDS strobe, which must be phase-shifted ~90° (quarter bit) so its edges land in the CENTRE of the
//   DQ data eye. The nominal shift here is a compile-time default (RX_STROBE_DLY_TAPS, centre of the
//   tap range) plus a byte-pairing/framing select (RX_PAIR_SKEW). On real silicon the correct tap and
//   pairing are PVT/board dependent and MUST be found by a hardware read-eye sweep (write a known
//   pattern, sweep the tap/skew, pick the widest passing window). Treat the defaults as a starting
//   point only — see the "RX read-capture delay" block below and docs handoff.
//
// ---------------------------------------------------------------------------------------------------
// Primitive map (all from the Agilex-3 "tennm_ph2" I/O library; usage cross-checked against the
// device-generated altera_gpio.sv reference wrapper):
//
//   tennm_ph2_ddio_out  — output DDR register: launches datainhi on the CK-high sub-phase and
//                         datainlo on the CK-low sub-phase of each `clk` cycle. Used for:
//                           * each hb_dq_o[i]   : {byte A, byte B}   (byte A = 1st edge = high)
//                           * hb_rwds_o         : {rwds phase1, phase0} write byte-mask
//                           * hb_ck / hb_ck_n   : forwarded HyperBus clock, clocked by clk90
//   tennm_ph2_ddio_oe   — output-enable DDR register: registers the tri-state enable in the I/O cell
//                         so DQ/RWDS turnaround is deterministic. Used for hb_dq_oe / hb_rwds_oe.
//   tennm_ph2_ddio_in   — input DDR register: captures datain on both edges of its clock and presents
//                         regouthi (rising sample) / regoutlo (falling sample) retimed to the rising
//                         edge. Clocked by the phase-shifted RWDS strobe → source-synchronous read
//                         capture. Used for each hb_dq_i[i].
//
// The bidirectional pad buffers (tennm_ph2_io_obuf / _io_ibuf) are NOT in this module: per the frozen
// contract the PHY exposes SPLIT hb_dq_o / hb_dq_oe / hb_dq_i (no `inout`) so it stays Verilator-shaped
// and reusable, and the true tri-state pad lives in the board wrapper. See
// fpga/axc3000/hyperbus_pads_altera.sv, which instantiates io_obuf/io_ibuf around these split ports.
//
// ---------------------------------------------------------------------------------------------------
// Clocking contract (supplied by the board IOPLL; see fpga/axc3000/):
//   clk     — HyperBus word clock. One 16-bit word per cycle; DQ moves 2 bytes/cycle (DDR). All TX
//             DDIO_OUT/OE and the whole controller run on this clock.
//   clk90   — SAME frequency as clk, +90° phase (lags clk by a quarter period). Clocks ONLY the CK-
//             forwarding DDIO so hb_ck's edges sit in the centre of the DQ eye that clk launches.
//   clk_ref — optional delay/calibration reference; unused for functional correctness here (tie-off).
//   rst     — synchronous, active-high (architectural state only; DDR datapath regs are reset-less).
// clk and clk90 MUST come from the same PLL/VCO (fixed, calibrated phase) — a jittery/independent
// clk90 corrupts write centring.
// ---------------------------------------------------------------------------------------------------

`timescale 1ns/1ps

/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off DECLFILENAME */
module hyperbus_phy_altera
  import hyperbus_pkg::*;
#(
    // ---- frozen contract parameters (passed by hyperbus_phy wrapper) ----
    parameter int unsigned DQ_WIDTH    = HB_DQ_WIDTH_DEFAULT,
    parameter int unsigned DATA_WIDTH  = 2 * DQ_WIDTH,
    parameter int unsigned ADDR_WIDTH  = HB_ADDR_WIDTH,
    parameter int unsigned LEN_WIDTH   = HB_LEN_WIDTH_DEFAULT,
    parameter              PHY_VARIANT = "INTEL",
    parameter bit          DIFF_CK     = 1'b1,

    // ---- non-frozen Agilex-only knobs (defaulted; the wrapper never overrides these) ----
    // Read-strobe eye-centring delay, expressed as a tap index into a keep-buffer delay line. The
    // line length is RX_STROBE_MAX_TAPS+1; the default selects the middle tap (~centre of the eye at
    // nominal PVT). REPLACE/RECALIBRATE on hardware — see the read-eye note in the header.
    parameter int unsigned RX_STROBE_MAX_TAPS = 16,
    parameter int unsigned RX_STROBE_DLY_TAPS = RX_STROBE_MAX_TAPS/2,
    // Read byte-pairing / half-word framing select. The DDIO_IN presents {rising sample, falling
    // sample} where the falling sample belongs to the PRECEDING rising edge; RX_PAIR_SKEW=1 re-pairs
    // byteA(n) with the falling byteB that FOLLOWS it (matches the generic PHY's {A, next-B} order).
    // If a hardware read-eye sweep shows the halves swapped, build with RX_PAIR_SKEW=0.
    parameter bit          RX_PAIR_SKEW       = 1'b1
) (
    input  logic                clk,
    input  logic                clk90,
    input  logic                clk_ref,    // calibration/delay reference (unused for correctness)
    input  logic                rst,

    // ---- ctrl-facing (slave; mirror of hyperbus_ctrl TX/RX) ----
    input  logic                phy_cs_n,
    input  logic                phy_rst_n,
    input  logic                phy_ck_en,
    input  logic [2*DQ_WIDTH-1:0] phy_dq_o,   // [hi]=byte A (1st edge), [lo]=byte B (2nd edge)
    input  logic                phy_dq_oe,
    input  logic [1:0]          phy_rwds_o,   // [1]=1st phase mask, [0]=2nd phase mask
    input  logic                phy_rwds_oe,
    input  logic                phy_rd_arm,
    output logic [2*DQ_WIDTH-1:0] phy_dq_i,   // recovered read word (byte A in high half)
    output logic                phy_dq_i_valid,
    output logic                phy_rwds_i,

    // ---- device pins (split; board wrapper adds tri-state via io_obuf/io_ibuf) ----
    output logic                hb_ck,
    output logic                hb_ck_n,
    output logic                hb_cs_n,
    output logic                hb_rst_n,
    output logic [DQ_WIDTH-1:0] hb_dq_o,
    output logic                hb_dq_oe,
    input  logic [DQ_WIDTH-1:0] hb_dq_i,
    output logic                hb_rwds_o,
    output logic                hb_rwds_oe,
    input  logic                hb_rwds_i
);

  localparam int unsigned PHYW = 2 * DQ_WIDTH;  // one HyperBus word

  // ==================================================================================================
  //  TX : chip-select / device-reset control pipeline  (architectural state → synchronous reset)
  //  One uniform clk of latency, matching the DDIO_OUT/OE launch latency below, keeps cs#, CK-enable,
  //  the DQ/RWDS bytes and the output enables mutually aligned on the pins.
  // ==================================================================================================
  logic ck_en_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      hb_cs_n  <= 1'b1;    // idle: chip deselected
      hb_rst_n <= 1'b0;    // hold device in reset while core is reset
      ck_en_q  <= 1'b0;
    end else begin
      hb_cs_n  <= phy_cs_n;
      hb_rst_n <= phy_rst_n;
      ck_en_q  <= phy_ck_en;
    end
  end

  // ==================================================================================================
  //  TX : DQ data — one tennm_ph2_ddio_out per DQ bit
  //  datainhi = byte A (phy_dq_o high half, 1st/CK-rising edge), datainlo = byte B (low half, 2nd/CK-
  //  falling edge). The DDIO registers both halves on the clk rising edge and serialises them onto the
  //  pin across the following cycle → exactly the generic PHY's 1-cycle launch latency & A-then-B order.
  // ==================================================================================================
  genvar gi;
  generate
    for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_dq_out
      tennm_ph2_ddio_out #(
        .mode      ("MODE_DDR"),
        .asclr_ena ("ASCLR_ENA_NONE"),
        .sclr_ena  ("SCLR_ENA_NONE")
      ) u_dq_ddio_out (
        .ena      (1'b1),
        .areset   (1'b0),
        .sreset   (1'b0),
        .datainhi (phy_dq_o[DQ_WIDTH + gi]),  // byte A bit i (1st edge, CK high)
        .datainlo (phy_dq_o[gi]),             // byte B bit i (2nd edge, CK low)
        .dataout  (hb_dq_o[gi]),
        .clk      (clk)
      );
    end
  endgenerate

  // ==================================================================================================
  //  TX : RWDS write byte-mask — one tennm_ph2_ddio_out, same A/B phase mapping as DQ.
  // ==================================================================================================
  tennm_ph2_ddio_out #(
    .mode      ("MODE_DDR"),
    .asclr_ena ("ASCLR_ENA_NONE"),
    .sclr_ena  ("SCLR_ENA_NONE")
  ) u_rwds_ddio_out (
    .ena      (1'b1),
    .areset   (1'b0),
    .sreset   (1'b0),
    .datainhi (phy_rwds_o[1]),   // 1st-phase write mask
    .datainlo (phy_rwds_o[0]),   // 2nd-phase write mask
    .dataout  (hb_rwds_o),
    .clk      (clk)
  );

  // ==================================================================================================
  //  TX : tri-state enables — plain (reset-less) pipeline registers, NOT tennm_ph2_ddio_oe.
  //  The enable is a whole-cycle value (both DDR sub-phases carry the same level), so a single-rate
  //  register is functionally identical to a MODE_DDR DDIO_OE here. It is deliberately a *soft*
  //  register rather than a hard I/O-cell DDIO_OE because hb_dq_oe is ONE bit shared across the whole
  //  x8 DQ bus: a hard DDIO_OE I/O-cell register can drive only ONE pin's output buffer, so sharing it
  //  across 8 DQ pads makes the Agilex fitter fail with "cannot place 1 DDIO_OE / no routing
  //  connectivity" (error 175001). A soft register is instead REPLICATED by the fitter into each DQ
  //  I/O cell's fast output-enable register, preserving the same single-clk launch latency as the DQ
  //  DDIO_OUT so drive/turnaround stays byte-aligned. RWDS (a single pad) is treated identically for
  //  symmetry. Reset-less per the Hyperflex datapath discipline (PLAN §3 LV1); the controller holds
  //  phy_*_oe Low out of reset, so the pins are released until the first real transaction.
  //  (These enables are exposed on the split hb_*_oe ports and drive the inferred tri-state pads.)
  // ==================================================================================================
  always_ff @(posedge clk) begin
    hb_dq_oe   <= phy_dq_oe;
    hb_rwds_oe <= phy_rwds_oe;
  end

  // ==================================================================================================
  //  TX : HyperBus clock generation — forwarded, centre-aligned, glitch-free
  //  A DDIO_OUT clocked by clk90 with datainhi=ck_en_q, datainlo=0 re-emits clk90 while enabled and
  //  holds Low while idle: output = clk90 during its high sub-phase, 0 during its low sub-phase, so
  //  hb_ck's RISING edge coincides with clk90's rising edge = the CENTRE of the byte-A eye that the DQ
  //  DDIOs (clocked by clk) launch. Gating is glitch-free because ck_en_q only changes on a clk rising
  //  edge — a moment when clk90 is Low — so a pulse is never chopped and CK idles Low (SPEC_DIGEST §1).
  //  This mirrors the generic PHY's `hb_ck = ck_en_q ? clk90 : 0`, realised in the I/O cell.
  // ==================================================================================================
  tennm_ph2_ddio_out #(
    .mode      ("MODE_DDR"),
    .asclr_ena ("ASCLR_ENA_NONE"),
    .sclr_ena  ("SCLR_ENA_NONE")
  ) u_ck_ddio_out (
    .ena      (1'b1),
    .areset   (1'b0),
    .sreset   (1'b0),
    .datainhi (ck_en_q),   // high sub-phase follows clk90 when enabled
    .datainlo (1'b0),      // low  sub-phase forced Low → clean single pulse, idles Low
    .dataout  (hb_ck),
    .clk      (clk90)
  );

  generate
    if (DIFF_CK) begin : g_ckn
      // Complementary clock: inverted DDR phases (datainhi=0, datainlo=ck_en_q) → 180° of hb_ck.
      // The AXC3000 HyperRAM is single-ended (no hb_ck_n board pin); keep this for pseudo-diff boards
      // and drive it from its own DDIO so both legs share the clk90 launch path.
      tennm_ph2_ddio_out #(
        .mode      ("MODE_DDR"),
        .asclr_ena ("ASCLR_ENA_NONE"),
        .sclr_ena  ("SCLR_ENA_NONE")
      ) u_ckn_ddio_out (
        .ena      (1'b1),
        .areset   (1'b0),
        .sreset   (1'b0),
        .datainhi (1'b0),
        .datainlo (ck_en_q),
        .dataout  (hb_ck_n),
        .clk      (clk90)
      );
    end else begin : g_ckn_tie
      assign hb_ck_n = 1'b1;   // single-ended: idle CK Low ⇒ idle CK# High
    end
  endgenerate

  // ==================================================================================================
  //  RX : read-capture strobe delay (eye-centring, calibratable)
  //  Read DQ and RWDS leave the device edge-aligned and return with the same round-trip flight delay,
  //  so the only delay-tolerant sampling clock is RWDS itself, phase-shifted ~90° so its edges land in
  //  the centre of the DQ eye. A fixed local-clock phase only works at zero flight delay and mis-pairs
  //  bytes on real hardware. Here the shift is a KEEP'd combinational buffer line with a tap mux; the
  //  selected tap is the calibration knob (default = middle tap). On silicon the per-tap picoseconds
  //  are PVT/board dependent — sweep on hardware (header note). A production build may instead route
  //  this through the I/O input-delay chain / a dedicated RWDS PLL phase; the tap select stays the
  //  calibration handle either way.
  // ==================================================================================================
  localparam int unsigned RX_TAP =
      (RX_STROBE_DLY_TAPS > RX_STROBE_MAX_TAPS) ? RX_STROBE_MAX_TAPS : RX_STROBE_DLY_TAPS;

  (* keep = 1 *) logic [RX_STROBE_MAX_TAPS:0] rwds_dly_line;
  assign rwds_dly_line[0] = hb_rwds_i;
  generate
    for (gi = 0; gi < RX_STROBE_MAX_TAPS; gi = gi + 1) begin : g_rwds_dly
      // Buffer stage; `keep` stops synthesis from collapsing the chain so each stage adds real delay.
      (* keep = 1 *) logic dly_stage;
      assign dly_stage             = rwds_dly_line[gi];
      assign rwds_dly_line[gi + 1] = dly_stage;
    end
  endgenerate

  wire rx_strobe = rwds_dly_line[RX_TAP];   // phase-shifted RWDS = read-capture clock

  // ==================================================================================================
  //  RX : DDR read capture — one tennm_ph2_ddio_in per DQ bit, clocked by the shifted RWDS strobe.
  //  regouthi = sample at the strobe RISING edge (byte A), regoutlo = sample at the FALLING edge.
  //  Both are retimed to the rising edge, so a full {A,B} pair is available once per strobe period in
  //  the RWDS-strobe clock domain.
  // ==================================================================================================
  logic [DQ_WIDTH-1:0] rx_hi;   // rising-edge captured byte (byte A of this period)
  logic [DQ_WIDTH-1:0] rx_lo;   // falling-edge captured byte
  generate
    for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_dq_in
      tennm_ph2_ddio_in #(
        .mode      ("MODE_DDR"),
        .asclr_ena ("ASCLR_ENA_NONE"),
        .sclr_ena  ("SCLR_ENA_NONE")
      ) u_dq_ddio_in (
        .ena      (1'b1),
        .areset   (1'b0),
        .sreset   (1'b0),
        .datain   (hb_dq_i[gi]),
        .clk      (rx_strobe),
        .regouthi (rx_hi[gi]),
        .regoutlo (rx_lo[gi])
      );
    end
  endgenerate

  // ==================================================================================================
  //  RX : byte pairing + RWDS→clk elastic FIFO  (the one true CDC of the system, DESIGN §2)
  //  Re-assemble the 16-bit word in the RWDS-strobe domain and push it into a small gray-pointer FIFO
  //  drained by clk — identical structure & controller contract to hyperbus_phy_generic (one
  //  clk-synchronous word + valid per FIFO entry). Capture is gated by phy_rd_arm (High only across the
  //  read latency + data window, when RWDS is a real strobe). The write pointer takes the single
  //  designated async clear on rst (the only async reset in the IP).
  //
  //  Pairing (RX_PAIR_SKEW): the DDIO's regoutlo at rising edge n is the falling sample that PRECEDED
  //  edge n, i.e. byte B of word n-1. To reproduce the generic PHY's {byteA(n), the byteB that follows
  //  it} order, hold byte A one strobe and pair it with the next regoutlo (RX_PAIR_SKEW=1). If a
  //  hardware sweep shows the halves swapped, RX_PAIR_SKEW=0 pairs {rx_hi, rx_lo} of the same edge.
  // ==================================================================================================
  localparam int unsigned RXF_DEPTH = 8;
  localparam int unsigned RXF_AW    = $clog2(RXF_DEPTH);

  logic [PHYW-1:0]     rxf_mem [RXF_DEPTH];
  logic [DQ_WIDTH-1:0] rx_hi_hold;                // byte A held one strobe (for RX_PAIR_SKEW)
  logic                rx_prime;                  // 1 after the first strobe (skew pipeline primed)
  logic [RXF_AW:0]     wptr_bin;                  // strobe-domain binary write pointer (+1 wrap MSB)
  logic [RXF_AW:0]     rptr_bin;                  // clk-domain binary read pointer

  wire  [PHYW-1:0] rx_word_pair = RX_PAIR_SKEW ? {rx_hi_hold, rx_lo}   // {byteA(n-1), byteB(n-1)}
                                               : {rx_hi,      rx_lo};  // {byteA(n),   byteB(n)}
  wire             rx_push_ok   = RX_PAIR_SKEW ? rx_prime : 1'b1;

  /* verilator lint_off SYNCASYNCNET */
  always_ff @(posedge rx_strobe or posedge rst) begin
    if (rst) begin
      wptr_bin   <= '0;
      rx_hi_hold <= '0;
      rx_prime   <= 1'b0;
    end else if (phy_rd_arm) begin
      rx_hi_hold <= rx_hi;
      rx_prime   <= 1'b1;
      if (rx_push_ok) begin
        rxf_mem[wptr_bin[RXF_AW-1:0]] <= rx_word_pair;
        wptr_bin                      <= wptr_bin + 1'b1;
      end
    end else begin
      rx_prime <= 1'b0;   // re-prime the skew pipeline for the next armed read burst
    end
  end
  /* verilator lint_on SYNCASYNCNET */

  // Gray-code the write pointer and 2-flop synchronise it into clk (standard async-FIFO CDC).
  wire  [RXF_AW:0] wptr_gray = wptr_bin ^ (wptr_bin >> 1);
  logic [RXF_AW:0] wgray_s1, wgray_s2;
  always_ff @(posedge clk) begin
    if (rst) begin
      wgray_s1 <= '0;
      wgray_s2 <= '0;
    end else begin
      wgray_s1 <= wptr_gray;
      wgray_s2 <= wgray_s1;
    end
  end

  function automatic logic [RXF_AW:0] gray2bin(input logic [RXF_AW:0] g);
    logic [RXF_AW:0] b;
    for (int i = RXF_AW; i >= 0; i--)
      b[i] = (i == RXF_AW) ? g[RXF_AW] : (b[i+1] ^ g[i]);
    return b;
  endfunction
  wire [RXF_AW:0] wptr_bin_s = gray2bin(wgray_s2);
  wire            rxf_empty  = (rptr_bin == wptr_bin_s);

  // Read side (clk domain): one recovered word + valid pulse per FIFO entry.
  always_ff @(posedge clk) begin
    if (rst) begin
      rptr_bin       <= '0;
      phy_dq_i       <= '0;
      phy_dq_i_valid <= 1'b0;
    end else begin
      phy_dq_i_valid <= 1'b0;
      if (!rxf_empty) begin
        phy_dq_i       <= rxf_mem[rptr_bin[RXF_AW-1:0]];
        phy_dq_i_valid <= 1'b1;
        rptr_bin       <= rptr_bin + 1'b1;
      end
    end
  end

  // ==================================================================================================
  //  RX : RWDS level synchroniser (clk domain) — variant-independent 2-flop sync of the raw RWDS pin
  //  for the controller's CA latency-select (High during CA ⇒ 2× latency) and read-stall detection.
  // ==================================================================================================
  logic rwds_s1;
  always_ff @(posedge clk) begin
    if (rst) begin
      rwds_s1    <= 1'b0;
      phy_rwds_i <= 1'b0;
    end else begin
      rwds_s1    <= hb_rwds_i;
      phy_rwds_i <= rwds_s1;
    end
  end

  // Contract-only / calibration-reference tie-offs (kept so all PHY variants share one port+param list).
  logic _unused_ok;
  assign _unused_ok = &{1'b0, clk_ref, ADDR_WIDTH[0], LEN_WIDTH[0], DATA_WIDTH[0],
                        PHY_VARIANT == "INTEL"};

endmodule
/* verilator lint_on DECLFILENAME */
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
