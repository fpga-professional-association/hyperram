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
    parameter bit          RX_PAIR_SKEW       = 1'b1,
    // Read-strobe PREAMBLE skip (non-frozen, defaulted 0) — same semantics as hyperbus_phy_sdr.sv:
    // a real HyperRAM (Winbond W957D8NB) toggles RWDS for RD_PREAMBLE_SKIP CK cycles with DQ Hi-Z
    // (=0x00) BEFORE the first read-data byte. Each preamble cycle is one rx_strobe rising edge here;
    // without the skip those edges push phantom {0x00,0x00} words and shift the whole burst
    // (on-silicon fingerprint: ERR_COUNT = LEN-1). Ignore the first RD_PREAMBLE_SKIP rising edges
    // after each arm. 0 = spec-ideal device (no preamble). GitHub issue #7.
    parameter int unsigned RD_PREAMBLE_SKIP   = 0,
    // HyperBus CK forwarding scheme (GitHub issue #8, parent #3 direction 1):
    //   "CLK90"   — (legacy default) CK DDIOs clocked by clk90 (+90 deg IOPLL phase). Centres CK in
    //               the DQ eye by construction but needs TWO clock phases in the I/O periphery —
    //               the Agilex-3 Bank-3A Fitter blocker (err 24403/24404).
    //   "CLK_DLY" — CK DDIOs clocked by the SAME clk as the DQ DDIOs (ONE periphery clock, no
    //               24403/24404). CK leaves the pin edge-aligned with the DQ launch; the required
    //               quarter-period eye-centring shift is applied OFF-FABRIC by a hard per-pin
    //               output-delay-chain assignment on the hb_ck pin (board .qsf; ~tCK/4, e.g.
    //               1.25 ns @ 200 MHz). clk90 is unused in this scheme (tie to clk at the board top).
    parameter              CK_SCHEME          = "CLK90",
    // Read DQ capture scheme:
    //   "LOCAL2X" (default) — the SDR PHY's RX front-end verbatim: DQ/RWDS registered into the
    //       free-running 2x core clock (clk90 in FABRIC2X guise), RWDS edges DETECTED (not used as
    //       a clock), byte pairing + preamble skip + synchronous disarm flush — the structure
    //       proven on this board's silicon to a 350 MHz capture clock with zero tuning.
    //   "FABRIC" — strobe-clocked fabric regs. Worked at 50 MHz; at 175 MHz the undelayed strobe
    //       races the byte transitions (one-byte slip, INPUT_DELAY_CHAIN taps too small to fix).
    //   "IOREG" — tennm_ph2_ddio_in in the I/O cell — read back constant zeros on the AXC3000.
    parameter              RX_CAP_SCHEME      = "LOCAL2X"
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
    // ---- runtime read-eye calibration (mandatory, no defaults; shared port contract, docs/INTERFACES.md
    //      v9). Elaboration-only tie-offs here until #3 unblocks the Agilex fit: the RX tap / byte-pairing
    //      stay the compile-time RX_STROBE_DLY_TAPS / RX_PAIR_SKEW parameters (no RX/TX/CK changes). ----
    input  logic                              cal_capture_phase,
    input  logic [HB_CAL_PREAMBLE_SKIP_WIDTH-1:0] cal_preamble_skip,
    input  logic [HB_CAL_RX_TAP_WIDTH-1:0]        cal_rx_tap,
    input  logic                              cal_pair_skew,
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
  // One-clk delay of the byte-B (LO) TX streams — pairs each word's byte B with the FABRIC2X CK
  // falling edge that arrives one half-slot into the following cycle (see the CK generate below).
  logic [DQ_WIDTH-1:0] dq_b_dly;
  logic                rwds_b_dly;
  always_ff @(posedge clk) begin
    dq_b_dly   <= phy_dq_o[DQ_WIDTH-1:0];
    rwds_b_dly <= phy_rwds_o[0];
  end

  genvar gi;
  generate
    for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_dq_out
      tennm_ph2_ddio_out #(
        .mode      ("MODE_DDR"),
        .asclr_ena ("ASCLR_ENA_NONE"),
        .sclr_ena  ("SCLR_ENA_NONE")
      ) u_dq_ddio_out (
        .ena      (1'b1),
        .areset   (1'b1),   // ACTIVE-LOW reset_n on ph2 atoms (vendor altera_gpio ties ~aclr = 1'b1;
                        // tying 0 holds the atom in async clear — DDIO_IN then reads constant 0)
        .sreset   (1'b0),
        // ON-SILICON ALIGNMENT (AXC3000, 2026-07-09): with the FABRIC2X CK generator, the CK
        // falling edge lands in the NEXT cycle's LO half-slot, so byte B rides a one-clk-delayed
        // LO stream (dq_b_dly) to meet it; byte A stays on datainhi where the rising edge samples
        // it. Verified against wire capture: without the delay the device stores {A(k), B(k+1)}.
        .datainhi (phy_dq_o[DQ_WIDTH + gi]),  // byte A bit i
        .datainlo (dq_b_dly[gi]),             // byte B bit i, delayed one clk
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
    .areset   (1'b1),   // ACTIVE-LOW reset_n on ph2 atoms (vendor altera_gpio ties ~aclr = 1'b1;
                        // tying 0 holds the atom in async clear — DDIO_IN then reads constant 0)
    .sreset   (1'b0),
    // Same one-clk LO-stream delay as the DQ DDIOs (see dq_b_dly note).
    .datainhi (phy_rwds_o[1]),   // 1st-phase write mask
    .datainlo (rwds_b_dly),      // 2nd-phase write mask, delayed one clk
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
  // CK generation, selected by CK_SCHEME (issue #8):
  //   "FABRIC2X" — (recommended on Agilex-3) port of the SILICON-PROVEN SDR-PHY CK generator
  //       (hyperbus_phy_sdr.sv, clean to a 350 MHz fabric clock on this board): the clk90 port
  //       carries a 2x-CK, 0-deg, CORE-ONLY fabric clock; a clk-domain toggle is synchronised into
  //       it, and hb_ck is emitted from an ordinary fabric register on the clk90 NEGEDGE — rising
  //       edge at T/4, falling at 3T/4 of the word period, i.e. centred in both byte eyes BY
  //       CONSTRUCTION. No I/O-cell CK register, no pin delay chain, and the I/O periphery still
  //       sees exactly ONE clock (clk, on the DQ/RWDS DDIOs) — hb_ck reaches its pad as data.
  //   "CLK_DLY"  — CK DDIO_OUT clocked by the same clk as the DQ DDIOs + hard output-delay-chain
  //       centring on the pin (bw.qsf). On the AXC3000 this produced NO observable CK activity on
  //       the wire (device never clocked a CA in; RWDS never strobed) — kept for future dissection.
  //   "CLK90"    — legacy: CK DDIO_OUT on a +90 deg IOPLL phase; needs two periphery clock phases
  //       (Fitter 24403/24404 on Bank 3A) — unbuildable on this board.
  generate
    if (CK_SCHEME == "FABRIC2X") begin : g_ck_fab2x
      // 2-flop reset into the clk90 (2x) domain (structure ported from hyperbus_phy_sdr.sv).
      logic rst2x_meta, rst2x;
      always_ff @(posedge clk90) begin
        rst2x_meta <= rst;
        rst2x      <= rst2x_meta;
      end
      // clk-domain word-phase toggle → synchronise + edge-detect in clk90 → beat_a marks the
      // byte-A (first) half of each clk word period.
      logic tgl;
      always_ff @(posedge clk) tgl <= rst ? 1'b0 : ~tgl;
      logic tgl_s1, tgl_s2, tgl_s3;
      always_ff @(posedge clk90) begin
        if (rst2x) begin tgl_s1 <= 1'b0; tgl_s2 <= 1'b0; tgl_s3 <= 1'b0; end
        else       begin tgl_s1 <= tgl;  tgl_s2 <= tgl_s1; tgl_s3 <= tgl_s2; end
      end
      wire beat_a = tgl_s2 ^ tgl_s3;
      logic beat_a_d1;
      always_ff @(posedge clk90) beat_a_d1 <= rst2x ? 1'b0 : beat_a;
      // Latch the CK enable per word at beat_a so a pulse is never chopped (glitch-free gating).
      logic cken_w;
      always_ff @(posedge clk90) begin
        if (rst2x)       cken_w <= 1'b0;
        else if (beat_a) cken_w <= ck_en_q;
      end
      // Emit CK on the clk90 negedge (beat_a_d1 view — the EXACT generator that runs the bus on
      // silicon). Its edges land one half-slot later than the DQ DDIO's word boundary: the rising
      // edge samples the wire's HI half-slot (byte A — correct) and the falling edge samples the
      // NEXT cycle's LO half-slot. The TX datapath below compensates by delaying the byte-B (LO)
      // stream one clk, so that late falling edge lands on the CURRENT word's byte B. Empirical:
      // moving the CK edge earlier instead (beat_a direct, with or without an early cken preload)
      // kills the CA entirely on silicon — do not "simplify" this back.
      logic ck_r;
      always_ff @(negedge clk90) ck_r <= rst2x ? 1'b0 : (cken_w & beat_a_d1);
      assign hb_ck   = ck_r;
      assign hb_ck_n = DIFF_CK ? ~ck_r : 1'b1;
    end else begin : g_ck_ddio
      wire ck_launch_clk;
      if (CK_SCHEME == "CLK_DLY") begin : g_ck_clkdly
        assign ck_launch_clk = clk;     // one periphery clock; centring via pin output-delay (~tCK/4)
      end else begin : g_ck_clk90
        assign ck_launch_clk = clk90;   // legacy: +90 deg phase centres CK in the DQ eye directly
      end

      tennm_ph2_ddio_out #(
        .mode      ("MODE_DDR"),
        .asclr_ena ("ASCLR_ENA_NONE"),
        .sclr_ena  ("SCLR_ENA_NONE")
      ) u_ck_ddio_out (
        .ena      (1'b1),
        .areset   (1'b1),   // ACTIVE-LOW reset_n on ph2 atoms (vendor altera_gpio ties ~aclr = 1'b1)
        .sreset   (1'b0),
        .datainhi (ck_en_q),   // high sub-phase follows the launch clock when enabled
        .datainlo (1'b0),      // low  sub-phase forced Low → clean single pulse, idles Low
        .dataout  (hb_ck),
        .clk      (ck_launch_clk)
      );

      if (DIFF_CK) begin : g_ckn
        // Complementary clock: inverted DDR phases (datainhi=0, datainlo=ck_en_q) → 180° of hb_ck.
        tennm_ph2_ddio_out #(
          .mode      ("MODE_DDR"),
          .asclr_ena ("ASCLR_ENA_NONE"),
          .sclr_ena  ("SCLR_ENA_NONE")
        ) u_ckn_ddio_out (
          .ena      (1'b1),
          .areset   (1'b1),   // ACTIVE-LOW reset_n on ph2 atoms (vendor altera_gpio ties ~aclr = 1'b1)
          .sreset   (1'b0),
          .datainhi (1'b0),
          .datainlo (ck_en_q),
          .dataout  (hb_ck_n),
          .clk      (ck_launch_clk)
        );
      end else begin : g_ckn_tie
        assign hb_ck_n = 1'b1;   // single-ended: idle CK Low ⇒ idle CK# High
      end
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
  //  RX : DDR read capture, selected by RX_CAP_SCHEME:
  //    "FABRIC" (default) — plain fabric registers on both edges of the shifted strobe, sampling the
  //        DQ ibuf outputs directly. Same structure class the SDR PHY proved on silicon to a 350 MHz
  //        capture clock with zero tuning; needs no I/O-cell input register, so the DQ pad's raw
  //        input can route to core with no P2X-term conflict. Pairing: at rising edge n+1 the push
  //        process's pre-edge view is {rx_hi = byte A(n), rx_lo = byte B(n)} — a matched same-word
  //        pair, so the effective pair skew is forced to 0 in this scheme (EFF_PAIR_SKEW below).
  //    "IOREG" — one tennm_ph2_ddio_in per DQ bit in the I/O cell (the original scheme). On this
  //        board it read back constant zeros through five build variants (areset polarity, strobe
  //        delay, preamble skip all swept) — kept for reference/other devices, not trusted on
  //        Agilex-3 until the atom's fabric-clock + regout semantics are vendor-clarified.
  // ==================================================================================================
  logic [DQ_WIDTH-1:0] rx_hi;   // rising-edge captured byte (byte A)
  logic [DQ_WIDTH-1:0] rx_lo;   // falling-edge captured byte
  generate
    if (RX_CAP_SCHEME == "IOREG") begin : g_rx_ioreg
      for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_dq_in
        tennm_ph2_ddio_in #(
          .mode      ("MODE_DDR"),
          .asclr_ena ("ASCLR_ENA_NONE"),
          .sclr_ena  ("SCLR_ENA_NONE")
        ) u_dq_ddio_in (
          .ena      (1'b1),
          .areset   (1'b1),   // ACTIVE-LOW reset_n on ph2 atoms (vendor altera_gpio ties ~aclr = 1'b1)
          .sreset   (1'b0),
          .datain   (hb_dq_i[gi]),
          .clk      (rx_strobe),
          .regouthi (rx_hi[gi]),
          .regoutlo (rx_lo[gi])
        );
      end
    end else if (RX_CAP_SCHEME == "LOCAL2X") begin : g_rx_local2x_tie
      // LOCAL2X does its capture inside the write-side generate below (clk90 domain);
      // the strobe-clocked rx_hi/rx_lo rails are unused.
      assign rx_hi = '0;
      assign rx_lo = '0;
    end else begin : g_rx_fabric
      // Reset-less datapath capture registers (Hyperflex discipline): byte A on the strobe rising
      // edge, byte B on the falling edge. Downstream pairing reads them one edge later (pre-edge
      // view), which yields matched {A(n), B(n)} words with EFF_PAIR_SKEW = 0.
      logic [DQ_WIDTH-1:0] rx_hi_r, rx_lo_r;
      always_ff @(posedge rx_strobe) rx_hi_r <= hb_dq_i;
      /* verilator lint_off SYNCASYNCNET */
      always_ff @(negedge rx_strobe) rx_lo_r <= hb_dq_i;
      /* verilator lint_on SYNCASYNCNET */
      assign rx_hi = rx_hi_r;
      assign rx_lo = rx_lo_r;
    end
  endgenerate

  // Effective byte pairing for the scheme in use (see header note above the generate).
  localparam bit EFF_PAIR_SKEW = (RX_CAP_SCHEME == "IOREG") ? RX_PAIR_SKEW : 1'b0;

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
  localparam int unsigned RXF_DEPTH = 32;  // elastic read FIFO. Deepened from 8 (matching the SDR
                                           // PHY's silicon fix, issue #7) so a full board read burst
                                           // plus device over-stream plus the strobe->clk gray-pointer
                                           // hand-off latency never laps the pointer.
  localparam int unsigned RXF_AW    = $clog2(RXF_DEPTH);

  logic [PHYW-1:0]     rxf_mem [RXF_DEPTH];
  logic [DQ_WIDTH-1:0] rx_hi_hold;                // byte A held one strobe (for RX_PAIR_SKEW)
  logic                rx_prime;                  // 1 after the first strobe (skew pipeline primed)
  logic [RXF_AW:0]     wptr_bin;                  // strobe-domain binary write pointer (+1 wrap MSB)
  logic [RXF_AW:0]     rptr_bin;                  // clk-domain binary read pointer

  // Leading rx_strobe rising edges still to discard as read-strobe preamble (issue #7). Reloaded by
  // rx_flush_q between bursts, so every armed read re-skips its own device preamble.
  localparam int unsigned SKIPW = (RD_PREAMBLE_SKIP == 0) ? 1 : $clog2(RD_PREAMBLE_SKIP + 1);
  logic [SKIPW-1:0]    pre_skip = SKIPW'(RD_PREAMBLE_SKIP);

  generate
    if (RX_CAP_SCHEME == "LOCAL2X") begin : g_rxw_local2x
      // =============================================================================================
      // SDR-PHY RX front-end, ported verbatim (hyperbus_phy_sdr.sv — silicon-proven at 350 MHz on
      // this board): DQ/RWDS registered into the free-running 2x clock (clk90), RWDS edges DETECTED
      // in that domain, rising edge captures byte A, falling edge completes {A,B} and pushes. The
      // disarm flush and preamble reload are SYNCHRONOUS (free-running clock — no async apparatus).
      // =============================================================================================
      logic rxrst_meta, rxrst;
      always_ff @(posedge clk90) begin
        rxrst_meta <= rst;
        rxrst      <= rxrst_meta;
      end

      logic [DQ_WIDTH-1:0] dq_cap;
      logic                rwds_cap, rwds_cap_q;
      logic [DQ_WIDTH-1:0] rx_byte_a;
      logic                have_a;
      logic                rdarm_s1, rdarm_s2;

      always_ff @(posedge clk90) begin
        dq_cap   <= hb_dq_i;      // registered input sampling on the local 2x clock
        rwds_cap <= hb_rwds_i;
        if (rxrst) begin rdarm_s1 <= 1'b0; rdarm_s2 <= 1'b0; end
        else       begin rdarm_s1 <= phy_rd_arm; rdarm_s2 <= rdarm_s1; end
      end

      wire rwds_rise = rwds_cap & ~rwds_cap_q;   // start of byte A
      wire rwds_fall = ~rwds_cap & rwds_cap_q;   // start of byte B (word completes)

      always_ff @(posedge clk90) begin
        if (rxrst) begin
          rx_byte_a  <= '0;
          have_a     <= 1'b0;
          rwds_cap_q <= 1'b0;
          wptr_bin   <= '0;
          pre_skip   <= SKIPW'(RD_PREAMBLE_SKIP);
        end else begin
          rwds_cap_q <= rwds_cap;
          if (!rdarm_s2) begin
            // Between bursts: flush the write side, re-arm the preamble skip (SDR PHY semantics).
            have_a   <= 1'b0;
            pre_skip <= SKIPW'(RD_PREAMBLE_SKIP);
            wptr_bin <= '0;
          end else begin
            if (rwds_rise) begin
              if (pre_skip != '0) begin
                pre_skip <= pre_skip - 1'b1;   // discard preamble rise (DQ Hi-Z here)
                have_a   <= 1'b0;
              end else begin
                rx_byte_a <= dq_cap;
                have_a    <= 1'b1;
              end
            end else if (rwds_fall && have_a) begin
              rxf_mem[wptr_bin[RXF_AW-1:0]] <= {rx_byte_a, dq_cap};   // {byte A, byte B}
              wptr_bin                      <= wptr_bin + 1'b1;
              have_a                        <= 1'b0;
            end
          end
        end
      end
    end else begin : g_rxw_strobe
      // Strobe-clocked write side (FABRIC / IOREG capture schemes). rx_strobe only toggles inside a
      // burst's data window, so disarm-time reload is done by an ASYNC hold (rx_flush_q, a
      // clk-registered glitch-free level; single async control per flop). Release timing is safe by
      // protocol construction: first strobe edge >= CA + initial latency after phy_rd_arm rises.
      logic rx_flush_q;
      always_ff @(posedge clk) begin
        if (rst) rx_flush_q <= 1'b1;
        else     rx_flush_q <= ~phy_rd_arm;
      end

      wire  [PHYW-1:0] rx_word_pair = EFF_PAIR_SKEW ? {rx_hi_hold, rx_lo}   // {byteA(n-1), byteB(n-1)}
                                                    : {rx_hi,      rx_lo};  // {byteA(n),   byteB(n)}
      wire             rx_push_ok   = (RX_CAP_SCHEME == "IOREG") ? (EFF_PAIR_SKEW ? rx_prime : 1'b1)
                                                                 : rx_prime;

      /* verilator lint_off SYNCASYNCNET */
      always_ff @(posedge rx_strobe or posedge rx_flush_q) begin
        if (rx_flush_q) begin
          wptr_bin   <= '0;
          rx_hi_hold <= '0;
          rx_prime   <= 1'b0;
          pre_skip   <= SKIPW'(RD_PREAMBLE_SKIP);
        end else if (pre_skip != '0) begin
          pre_skip   <= pre_skip - 1'b1;
          rx_hi_hold <= rx_hi;
          rx_prime   <= 1'b0;
        end else begin
          rx_hi_hold <= rx_hi;
          rx_prime   <= 1'b1;
          if (rx_push_ok) begin
            rxf_mem[wptr_bin[RXF_AW-1:0]] <= rx_word_pair;
            wptr_bin                      <= wptr_bin + 1'b1;
          end
        end
      end
      /* verilator lint_on SYNCASYNCNET */
    end
  endgenerate

  // Gray-code the write pointer and 2-flop synchronise it into clk (standard async-FIFO CDC).
  // While the receiver is disarmed the write side is flushed to 0 by rx_flush_q — a MULTI-BIT gray
  // transition a plain 2-flop sync can mis-sample mid-flight (the SDR PHY's on-silicon multi-burst
  // leak, issue #7). Force the synchronised copy to 0 while disarmed (the source is 0 anyway) so
  // rxf_empty is deterministic; normal +1 gray sync resumes from 0 when the next read arms.
  wire  [RXF_AW:0] wptr_gray = wptr_bin ^ (wptr_bin >> 1);
  logic [RXF_AW:0] wgray_s1, wgray_s2;
  always_ff @(posedge clk) begin
    if (rst || !phy_rd_arm) begin
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

  // Read side (clk domain): one recovered word + valid pulse per FIFO entry. While the receiver is
  // disarmed hold the read pointer at 0 — paired with the rx_flush_q write-side flush above, both
  // sides start every read burst at 0, discarding trailing over-streamed words (issue #7).
  always_ff @(posedge clk) begin
    if (rst) begin
      rptr_bin       <= '0;
      phy_dq_i       <= '0;
      phy_dq_i_valid <= 1'b0;
    end else begin
      phy_dq_i_valid <= 1'b0;
      if (!phy_rd_arm) begin
        rptr_bin <= '0;
      end else if (!rxf_empty) begin
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
  // The four runtime cal_* knobs are accumulated here too: they are elaboration-only for this variant
  // until #3 lands (see the port comment); the tap / pairing stay the compile-time params above.
  logic _unused_ok;
  assign _unused_ok = &{1'b0, clk_ref, ADDR_WIDTH[0], LEN_WIDTH[0], DATA_WIDTH[0],
                        PHY_VARIANT == "INTEL",
                        cal_capture_phase, cal_preamble_skip, cal_rx_tap, cal_pair_skew};

endmodule
/* verilator lint_on DECLFILENAME */
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
