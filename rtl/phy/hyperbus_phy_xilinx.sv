// hyperbus_phy_xilinx — AMD/Xilinx **7-series** DDR-IO HyperBus PHY (real synthesis target).
//
// Drop-in variant of the frozen `hyperbus_phy` contract (docs/INTERFACES.md): identical port list and
// identical controller-facing timing contract as `hyperbus_phy_generic`, but TX serialise / CK forward
// and RX deserialise are built from 7-series hard-IP primitives (ODDR / IDDR / IDELAYE2 / IDELAYCTRL /
// BUFIO / BUFR / OBUF / OBUFDS) instead of the generic behavioural muxes. This is the file a Xilinx
// board build compiles.
//
//   *** SIMULATES via a Verilator-only primitive shim (sim/model/xilinx_prims_sim.sv). ***
//   The genuine primitives live only in the Vivado `unisim` library, but the shim provides functional
//   stand-ins so `bash sim/run.sh` exercises THIS datapath end-to-end (tb_xilinx), against both the
//   ideal and the non-ideal (read-preamble + over-stream) device model. Still NOT hardware-proven and
//   NOT timing-closed: the read-eye taps, byte-pairing polarity and preamble skip are bring-up knobs
//   that must be swept on real silicon (see the notes below). The shim is best-effort — run xvlog/xelab
//   against the real unisim to catch any misremembered primitive port before synthesis.
//
// ---------------------------------------------------------------------------------------------------
// CLOCKING (from the board MMCM/PLL; the one-periphery-clock rule of the Agilex-3 SDR/DDIO variants
// does NOT apply here — clk90 keeps its documented 90°-CK-centring meaning):
//   clk     — HyperBus word clock. One 16-bit word / cycle; DQ moves 2 bytes/cycle (DDR). Clocks the
//             controller, the TX ODDRs (DQ/RWDS/OE) and the RX FIFO read side.
//   clk90   — SAME frequency as clk, +90° phase. Clocks ONLY the CK-forwarding ODDR so hb_ck's edges
//             sit in the CENTRE of the DQ eye that the clk-launched DQ ODDRs drive.
//   clk_ref — ~200 MHz IDELAYCTRL reference (calibrates the IDELAYE2 tap). REAL clock required here
//             (unlike GENERIC, where clk_ref is a tie-off).
//   rst     — synchronous, active-high (architectural TX state + the clk-domain RX FIFO/CDC). The one
//             async element is the RWDS-strobe→clk read-capture boundary (docs/PHY_PORTING §2, DESIGN
//             §2) — that is where the CDC belongs.
//
// BOARD REQUIREMENT (RX): RWDS must land on a clock-capable MRCC/SRCC pin in the SAME I/O bank as
// DQ[7:0], so the delayed strobe can drive a BUFIO (the DQ IDDRs) + a BUFR (the pairing FSM). Add the
// input-delay / clock exceptions from docs/INTEGRATION.md; sweep RX_STROBE_DLY_TAPS on hardware.
//
// ---------------------------------------------------------------------------------------------------
// UltraScale / UltraScale+ port (DOCUMENT ONLY — this file is 7-series):
//   ODDR   → ODDRE1        IDDR   → IDDRE1        IDELAYE2 → IDELAYE3 (+ ODELAYE3 / native mode)
//   BUFIO  → (BITSLICE / native RX clocking)      BUFR   → BUFGCE_DIV      IDELAYCTRL → per-bank RIU
// The controller-facing behaviour and the byte-pairing/FIFO logic below are device-independent; only
// the primitive instantiations swap.
// ---------------------------------------------------------------------------------------------------

`timescale 1ns/1ps

/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off DECLFILENAME */
module hyperbus_phy_xilinx
  import hyperbus_pkg::*;
#(
    // ---- frozen contract parameters (passed by the hyperbus_phy wrapper) ----
    parameter int unsigned DQ_WIDTH    = HB_DQ_WIDTH_DEFAULT,
    parameter int unsigned DATA_WIDTH  = 2 * DQ_WIDTH,
    parameter int unsigned ADDR_WIDTH  = HB_ADDR_WIDTH,
    parameter int unsigned LEN_WIDTH   = HB_LEN_WIDTH_DEFAULT,
    parameter              PHY_VARIANT = "XILINX",
    parameter bit          DIFF_CK     = 1'b1,

    // ---- non-frozen 7-series knobs (defaulted; the wrapper never overrides the first two) ----
    // IDELAYE2 FIXED read-strobe delay, in taps. Centres the returning RWDS strobe (~90° / quarter
    // bit) in the DQ eye. Default centres the eye at the tb CK rate under the behavioural shim; on
    // silicon a tap is ~78 ps @200 MHz REFCLK and the correct value is a hardware read-eye sweep.
    parameter int unsigned RX_STROBE_DLY_TAPS = 16,
    // Read byte-pairing escape hatch (mirrors hyperbus_phy_altera.sv:83). 0 = pair this word's rising
    // IDDR sample (byte A) with its own centred falling data (byte B) — correct under the shim and the
    // default here. 1 = hold byte A one strobe (pair the PREVIOUS rising sample), for a board whose
    // capture pipeline lands byte A of word N with byte B of word N+1. Signature of a wrong setting:
    // byte A of word N mixed with byte B of the neighbouring word — flip this and re-sweep on hardware.
    parameter bit          RX_PAIR_SKEW       = 1'b0,
    // Read-strobe PREAMBLE skip (mirrors the SDR variant). A real HyperRAM toggles RWDS for
    // RD_PREAMBLE_SKIP CK cycles with DQ Hi-Z before the first read byte; ignore that many leading
    // rwds rising edges so pairing starts on the real data window. 0 = spec-ideal device.
    parameter int unsigned RD_PREAMBLE_SKIP   = 0
) (
    input  logic                clk,
    input  logic                clk90,
    input  logic                clk_ref,    // ~200 MHz IDELAYCTRL reference
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

    // ---- device pins (split; board wrapper adds tri-state via IOBUF) ----
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

  genvar gi;

  // ==================================================================================================
  //  TX : chip-select / device-reset / CK-enable control pipeline (architectural state → sync reset).
  //  One uniform clk of latency, matching the ODDR launch latency below, keeps cs#, CK, the DQ/RWDS
  //  bytes and the output enables mutually aligned on the pins (mirror hyperbus_phy_altera.sv:123).
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
  //  TX : tri-state enables — PLAIN single-rate registers, deliberately NOT ODDR-based.
  //  The enable is a whole-cycle level (both DDR sub-phases carry the same value), so a single-rate
  //  register is functionally identical to a MODE_DDR OE here. On Xilinx the tri-state control is the
  //  per-pin `IOBUF.T` in the board wrapper (one T per DQ/RWDS pad) — there is no shared hard OE macro
  //  and therefore NONE of the Agilex "1 DDIO_OE across 8 DQ pins" Fitter hazard (altera err 175001,
  //  hyperbus_phy_altera.sv:179): nothing to "fix" into an ODDR. Reset-less datapath (Hyperflex); the
  //  controller holds phy_*_oe Low out of reset so the pads stay released until the first transaction.
  // ==================================================================================================
  always_ff @(posedge clk) begin
    hb_dq_oe   <= phy_dq_oe;
    hb_rwds_oe <= phy_rwds_oe;
  end

  // ==================================================================================================
  //  TX : DQ data — one ODDR per bit. D1 = byte A (phy_dq_o high half, 1st/CK-rising edge),
  //  D2 = byte B (low half, 2nd/CK-falling edge), clocked by clk. SAME_EDGE registers both halves off
  //  one clk rising edge and serialises A-then-B across the following cycle — exactly the generic PHY's
  //  1-cycle launch latency and A-then-B order. The tri-state IOBUF (driven by hb_dq_oe) is external.
  // ==================================================================================================
  generate
    for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_dq_out
      ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_dq_oddr (
        .Q  (hb_dq_o[gi]),
        .C  (clk),
        .CE (1'b1),
        .D1 (phy_dq_o[DQ_WIDTH + gi]),   // byte A bit i (1st edge, CK High)
        .D2 (phy_dq_o[gi]),              // byte B bit i (2nd edge, CK Low)
        .R  (1'b0),
        .S  (1'b0)
      );
    end
  endgenerate

  // ==================================================================================================
  //  TX : RWDS write byte-mask — one ODDR, same A/B phase mapping as DQ.
  // ==================================================================================================
  ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_rwds_oddr (
    .Q  (hb_rwds_o),
    .C  (clk),
    .CE (1'b1),
    .D1 (phy_rwds_o[1]),   // 1st-phase write mask
    .D2 (phy_rwds_o[0]),   // 2nd-phase write mask
    .R  (1'b0),
    .S  (1'b0)
  );

  // ==================================================================================================
  //  TX : HyperBus clock — ONE ODDR clocked by clk90 fed {ck_en_q, 1'b0}, feeding ONE OBUFDS (not two
  //  independently-placed single-ended legs — the OBUFDS pair has better-guaranteed P/N skew,
  //  docs/PHY_PORTING §3). Output = clk90 while enabled, 0 while idle: hb_ck's RISING edge coincides
  //  with clk90's rising edge = the CENTRE of the byte-A eye the DQ ODDRs (clocked by clk) launch.
  //  Gating is glitch-free because ck_en_q only changes on a clk rising edge — a moment clk90 is Low —
  //  so a pulse is never chopped and CK idles Low (SPEC_DIGEST §1). This is also the fix for the old
  //  skeleton's `hb_ck_n = DIFF_CK ? 1'b1 : 1'b1` bug: hb_ck_n now truly tracks ~hb_ck when DIFF_CK.
  // ==================================================================================================
  logic ck_ddr;
  ODDR #(.DDR_CLK_EDGE("SAME_EDGE"), .INIT(1'b0), .SRTYPE("SYNC")) u_ck_oddr (
    .Q  (ck_ddr),
    .C  (clk90),
    .CE (1'b1),
    .D1 (ck_en_q),   // high sub-phase follows clk90 when enabled
    .D2 (1'b0),      // low  sub-phase forced Low → single centred pulse, idles Low
    .R  (1'b0),
    .S  (1'b0)
  );
  generate
    if (DIFF_CK) begin : g_ckbuf_diff
      OBUFDS u_ckbuf (.I(ck_ddr), .O(hb_ck), .OB(hb_ck_n));
    end else begin : g_ckbuf_se
      OBUF u_ckbuf (.I(ck_ddr), .O(hb_ck));
      assign hb_ck_n = 1'b1;   // single-ended: idle CK Low ⇒ idle CK# High
    end
  endgenerate

  // ==================================================================================================
  //  RX : read-strobe eye-centring delay + regional clock buffers.
  //  IDELAYE2 (FIXED tap — no runtime CSR path through the frozen ports for VAR_LOAD) delays the
  //  returning RWDS ~90° so its edges land in the centre of the DQ eye; IDELAYCTRL calibrates the tap
  //  reference off clk_ref (~200 MHz). The delayed strobe fans out through a BUFIO (I/O-column clock →
  //  the DQ IDDRs) and a parallel BUFR (regional clock → the byte-pairing FSM / FIFO write side) — a
  //  BUFG would not reach the I/O capture cells with the needed low skew.
  // ==================================================================================================
  localparam int unsigned RX_TAP = (RX_STROBE_DLY_TAPS > 31) ? 31 : RX_STROBE_DLY_TAPS;

  wire rwds_dly;      // eye-centred RWDS
  wire idelay_rdy;    // IDELAYCTRL ready (functionally inert for FIXED taps)
  wire rx_ck_io;      // BUFIO copy → IDDRs
  wire rx_ck_r;       // BUFR  copy → pairing FSM / FIFO write side

  IDELAYCTRL u_idelayctrl (.RDY(idelay_rdy), .REFCLK(clk_ref), .RST(rst));

  IDELAYE2 #(
    .IDELAY_TYPE      ("FIXED"),
    .IDELAY_VALUE     (RX_TAP),
    .REFCLK_FREQUENCY (200.0),
    .DELAY_SRC        ("IDATAIN"),
    .SIGNAL_PATTERN   ("DATA")
  ) u_rwds_idelay (
    .DATAOUT     (rwds_dly),
    .CNTVALUEOUT (),
    .C           (clk_ref),
    .CE          (1'b0),
    .INC         (1'b0),
    .LD          (1'b0),
    .LDPIPEEN    (1'b0),
    .CINVCTRL    (1'b0),
    .REGRST      (1'b0),
    .IDATAIN     (hb_rwds_i),
    .DATAIN      (1'b0),
    .CNTVALUEIN  (5'd0)
  );

  BUFIO u_rwds_bufio (.O(rx_ck_io), .I(rwds_dly));
  BUFR #(.BUFR_DIVIDE("BYPASS")) u_rwds_bufr (.O(rx_ck_r), .I(rwds_dly), .CE(1'b1), .CLR(1'b0));

  // ==================================================================================================
  //  RX : DDR read capture — one IDDR per DQ bit clocked by the BUFIO strobe copy. Q1 = the sample at
  //  the strobe RISING edge (byte A, mid byte-A eye); Q2 = the sample at the FALLING edge (byte B).
  // ==================================================================================================
  logic [DQ_WIDTH-1:0] rx_hi;   // IDDR rising sample  (byte A)
  logic [DQ_WIDTH-1:0] rx_lo;   // IDDR falling sample (byte B; see pairing note)
  generate
    for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_dq_in
      IDDR #(.DDR_CLK_EDGE("SAME_EDGE_PIPELINED"), .INIT_Q1(1'b0), .INIT_Q2(1'b0), .SRTYPE("SYNC"))
      u_dq_iddr (
        .Q1 (rx_hi[gi]),
        .Q2 (rx_lo[gi]),
        .C  (rx_ck_io),
        .CE (1'b1),
        .D  (hb_dq_i[gi]),
        .R  (1'b0),
        .S  (1'b0)
      );
    end
  endgenerate

  // ==================================================================================================
  //  RX : arm/disarm flush generation (clk domain). phy_rd_arm is a controller Moore level, High only
  //  across the read latency + data + drain window and Low between bursts AND on the read-abort/timeout
  //  path (hyperbus_ctrl ST_RD_ABORT never arms). 2-flop it for a clean edge, then pulse `rx_flush` on
  //  EITHER edge:
  //    * the ARM (rising) edge RELOADS pre_skip (=RD_PREAMBLE_SKIP) and clears have_a/wptr BEFORE this
  //      burst's preamble strobe edges arrive — so EVERY read, including the very first after reset,
  //      starts with its preamble skip armed. (The strobe clock is gated silent between bursts, so the
  //      strobe-domain FSM cannot reload pre_skip on its own like the free-running-clock SDR variant
  //      does at hyperbus_phy_sdr.sv:316; the arm-edge flush is that reload.)
  //    * the DISARM (falling) edge flushes any device over-stream the drain window pushed, and covers
  //      the read-abort/timeout path.
  //  This clk-domain pulse ASYNC-clears the strobe-domain pairing FSM — NOT a level sync — because the
  //  gated strobe could otherwise strand a stale "armed" value with no trailing edge to resolve it.
  //  `rst` is a separate DIRECT async-clear term (kept out of this comb pulse so it stays a clean async
  //  reset in one place, matching hyperbus_phy_generic.sv:180's `posedge rst` idiom).
  // ==================================================================================================
  logic rdarm_s1, rdarm_s2;
  always_ff @(posedge clk) begin
    if (rst) begin
      rdarm_s1 <= 1'b0;
      rdarm_s2 <= 1'b0;
    end else begin
      rdarm_s1 <= phy_rd_arm;
      rdarm_s2 <= rdarm_s1;
    end
  end
  wire rx_flush = rdarm_s1 ^ rdarm_s2;   // pulse on ANY phy_rd_arm edge (arm reload + disarm flush)

  // ==================================================================================================
  //  RX : byte pairing + preamble skip (RWDS-strobe / BUFR domain), with the flush as ASYNC CLEAR.
  //
  //  Byte A is the IDDR rising sample (rx_hi), captured at the strobe rising edge and STABLE through
  //  the following falling edge. The word is pushed on the FALLING edge — reading byte A off the
  //  OPPOSITE (rising-captured) register is race-free, and the falling edge of the SAME word always
  //  occurs (even for the last word of a burst), so no trailing strobe edge is needed to flush it.
  //  Byte B is taken from the eye-centred bus at the falling edge (RX_PAIR_SKEW=0). A registered IDDR
  //  falling sample (rx_lo) would arrive one strobe late and, under a gated strobe, would strand the
  //  last word — hence byte B is the centred read here; rx_lo/RX_PAIR_SKEW=1 are the hardware escape
  //  hatch for a board whose pipeline pairs differently (see the RX_PAIR_SKEW parameter note).
  //
  //  Preamble: each leading rwds rising edge is a strobe rising edge; while pre_skip != 0 we discard it
  //  (have_a stays Low) so the following falling edge cannot complete a phantom {0x00,0x00} word.
  //  wptr_bin is the strobe-domain FIFO write pointer; the flush resets it (and re-arms pre_skip) so a
  //  fresh burst starts clean and any device over-stream past the master's word count is dropped.
  // ==================================================================================================
  localparam int unsigned RXF_DEPTH = 32;   // matches hyperbus_phy_sdr: a full 16-word burst + the
  localparam int unsigned RXF_AW    = $clog2(RXF_DEPTH);  // clk-domain gray-pointer hand-off never laps.

  localparam int unsigned SKIPW = (RD_PREAMBLE_SKIP == 0) ? 1 : $clog2(RD_PREAMBLE_SKIP + 1);

  logic [PHYW-1:0]     rxf_mem [RXF_DEPTH];
  logic [DQ_WIDTH-1:0] rx_hi_hold;   // byte A held one strobe (RX_PAIR_SKEW escape hatch)
  logic                have_a;       // past the preamble: a real byte A is in flight this word
  logic [SKIPW-1:0]    pre_skip;     // leading rwds rising edges still to discard this burst
  logic [RXF_AW:0]     wptr_bin;     // strobe-domain binary write pointer (extra wrap MSB)
  logic [RXF_AW:0]     rptr_bin;     // clk-domain binary read pointer

  wire [DQ_WIDTH-1:0] byte_a = RX_PAIR_SKEW ? rx_hi_hold : rx_hi;   // pairing escape hatch
  wire [PHYW-1:0]     rx_word = {byte_a, hb_dq_i};                  // {byte A, byte B (centred)}

  // Strobe RISING edge: preamble skip + arm byte A. Async-cleared by rst and the arm/disarm flush pulse.
  /* verilator lint_off SYNCASYNCNET */
  always_ff @(posedge rx_ck_r or posedge rst or posedge rx_flush) begin
    if (rst || rx_flush) begin
      have_a     <= 1'b0;
      pre_skip   <= SKIPW'(RD_PREAMBLE_SKIP);
      rx_hi_hold <= '0;
    end else begin
      rx_hi_hold <= rx_hi;              // hold this word's byte A for the skew escape hatch
      if (pre_skip != '0) begin
        pre_skip <= pre_skip - 1'b1;    // discard a preamble rising edge (DQ Hi-Z here)
        have_a   <= 1'b0;
      end else begin
        have_a   <= 1'b1;               // real read-data window: byte A captured by the IDDR
      end
    end
  end
  /* verilator lint_on SYNCASYNCNET */

  // Strobe FALLING edge: assemble {byte A, byte B} and push. Async-cleared by rst and the flush pulse.
  /* verilator lint_off SYNCASYNCNET */
  always_ff @(negedge rx_ck_r or posedge rst or posedge rx_flush) begin
    if (rst || rx_flush) begin
      wptr_bin <= '0;
    end else if (have_a) begin
      rxf_mem[wptr_bin[RXF_AW-1:0]] <= rx_word;   // {byte A (hi), byte B (lo)}
      wptr_bin                       <= wptr_bin + 1'b1;
    end
  end
  /* verilator lint_on SYNCASYNCNET */

  // ==================================================================================================
  //  RX : gray-pointer CDC (strobe → clk). Copied from hyperbus_phy_sdr.sv:344-362 verbatim: while the
  //  receiver is disarmed the write side resets wptr_bin to 0 (a multi-bit gray transition a plain
  //  2-flop sync can mis-sample mid-flight), so force the synchronised copy directly to 0 while
  //  disarmed — the flush is then deterministic (rxf_empty cleanly asserted) with no reliance on the
  //  gray-pointer reset surviving the CDC. Normal +1 gray sync resumes from 0 when the next read arms.
  // ==================================================================================================
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

  // ==================================================================================================
  //  RX : FIFO read side (clk word domain). Copied from hyperbus_phy_sdr.sv:378-393 verbatim: one
  //  recovered word + valid per FIFO entry; while disarmed hold the read pointer at 0 so the elastic
  //  FIFO is flushed to empty — write side (wptr_bin) is likewise flushed by rx_flush — so every read
  //  burst starts at 0 and any trailing over-streamed words from the previous burst are discarded.
  // ==================================================================================================
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
  //  (Kept from the previous skeleton unchanged; SPEC_DIGEST §3/§4.)
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

  // Contract-only / calibration tie-offs (kept so all PHY variants share one port+param list).
  logic _unused_ok;
  assign _unused_ok = &{1'b0, idelay_rdy, rx_lo, ADDR_WIDTH[0], LEN_WIDTH[0], DATA_WIDTH[0],
                        PHY_VARIANT == "XILINX"};

endmodule
/* verilator lint_on DECLFILENAME */
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
