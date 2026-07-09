// hyperbus_gpio_io — AXC3000 board I/O layer for the 200 MHz DDIO build, built from VENDOR-WRAPPED
// altera_gpio cells instead of raw tennm_ph2 atoms (qsys/hbgpio.qsys: hbgpio_dq_cell, hbgpio_ck_cell).
//
// Why this exists (2026-07-09 bring-up findings, see the ddio-200 branch history):
//   * Raw tennm_ph2_ddio_out for CK produced NO wire activity on this board in any configuration; the
//     FABRIC2X workaround (SDR-PHY CK generator) works but needs a 2x core clock whose minimum pulse
//     width caps CK at ~176 MHz. The vendor ck_cell (altera_gpio, DDR out + CKE) runs the I/O at 1x —
//     no 2x clock anywhere — which is the only honest route to the device's 200 MHz rating.
//   * dq_cell is a bidir DDR GPIO with DDIO_WITH_DELAY (MODE_DDR_W_DLY): the cell delays the DATA
//     input against its input-register clock (ck_in = raw RWDS), giving the hold margin that the
//     undelayed strobe-clocked fabric capture lacked at 175 MHz.
//   * RWDS itself stays on the bring-up-proven path: tennm TX atom (write mask) + inferred tristate
//     pad + raw input (it must supply the raw strobe for dq_cell.ck_in and the controller level sync).
//
// This module presents the hyperbus_ctrl-facing phy_* interface (same shape as the frozen PHY
// contract's ctrl side) and OWNS the hb_dq / hb_rwds / hb_ck pins (pad ring inside the GPIO cells /
// inferred tristate). hb_cs_n / hb_rst_n are plain registered outputs. Board-only file — not part of
// the portable IP; compiled by Quartus only.
//
// Alignment knobs (the wire tells the truth — sweep with the capture module + first-error CSRs):
//   TX_B_DLY   : 1 = delay the byte-B (lo) TX stream one clk (the alignment the FABRIC2X CK needed;
//                the vendor CK cell's phase may differ — flip if the {A(k),B(k+1)} signature appears).
//   CK_DIN_HI  : 1 = ck_cell.din = {1,0} (high sub-phase first); 0 = {0,1} (half-cycle CK shift).
//   RD_PREAMBLE_SKIP : device read-strobe preamble CK cycles to discard (W957D8NB: 1).

`timescale 1ns/1ps

/* verilator lint_off UNDRIVEN */
/* verilator lint_off UNUSEDSIGNAL */
module hyperbus_gpio_io #(
    parameter int unsigned DQ_WIDTH         = 8,
    parameter int unsigned RD_PREAMBLE_SKIP = 1,
    parameter bit          TX_B_DLY         = 1'b1,
    parameter bit          CK_DIN_HI        = 1'b1
) (
    input  logic                  clk,          // CK-rate word clock (controller + all launches)
    input  logic                  rst,          // synchronous, active-high

    // ---- ctrl-facing (mirror of hyperbus_ctrl's PHY master interface) ----
    input  logic                  phy_cs_n,
    input  logic                  phy_rst_n,
    input  logic                  phy_ck_en,
    input  logic [2*DQ_WIDTH-1:0] phy_dq_o,     // [hi]=byte A (1st edge), [lo]=byte B (2nd edge)
    input  logic                  phy_dq_oe,
    input  logic [1:0]            phy_rwds_o,   // [1]=1st-phase mask, [0]=2nd-phase mask
    input  logic                  phy_rwds_oe,
    input  logic                  phy_rd_arm,
    output logic [2*DQ_WIDTH-1:0] phy_dq_i,     // recovered read word (byte A high half)
    output logic                  phy_dq_i_valid,
    output logic                  phy_rwds_i,

    // ---- board pins (this module owns the pad ring for dq/rwds/ck) ----
    output logic                  hb_ck,
    output logic                  hb_cs_n,
    output logic                  hb_rst_n,
    inout  wire  [DQ_WIDTH-1:0]   hb_dq,
    inout  wire                   hb_rwds
);

  localparam int unsigned PHYW = 2 * DQ_WIDTH;

  // ================================================================================================
  //  Control pipeline — one uniform clk of latency so cs#/rst#/enables track the GPIO-cell DDR
  //  launch latency (the cells register din on ck_out).
  // ================================================================================================
  logic csn_q, rstn_q, dqoe_q, rwoe_q, cken_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      csn_q  <= 1'b1;
      rstn_q <= 1'b0;
      dqoe_q <= 1'b0;
      rwoe_q <= 1'b0;
      cken_q <= 1'b0;
    end else begin
      csn_q  <= phy_cs_n;
      rstn_q <= phy_rst_n;
      dqoe_q <= phy_dq_oe;
      rwoe_q <= phy_rwds_oe;
      cken_q <= phy_ck_en;
    end
  end
  assign hb_cs_n  = csn_q;
  assign hb_rst_n = rstn_q;

  // ================================================================================================
  //  TX byte-B one-clk delay (silicon alignment, see header). Reset-less datapath regs.
  // ================================================================================================
  logic [DQ_WIDTH-1:0] dq_b_dly;
  logic                rwds_b_dly;
  always_ff @(posedge clk) begin
    dq_b_dly   <= phy_dq_o[DQ_WIDTH-1:0];
    rwds_b_dly <= phy_rwds_o[0];
  end
  wire [DQ_WIDTH-1:0] tx_b = TX_B_DLY ? dq_b_dly : phy_dq_o[DQ_WIDTH-1:0];
  wire                tx_rwds_b = TX_B_DLY ? rwds_b_dly : phy_rwds_o[0];

  // ================================================================================================
  //  DQ — raw tennm_ph2_ddio_out TX atoms (proven at 175 in the atom PHY) + inferred tristate pads.
  //  The vendor dq_cell was retired: its strobe-clocked input DDIO registers read constant zeros
  //  (hard input DDR regs cannot be clocked from a fabric-routed data pin on this silicon — confirmed
  //  for raw atoms AND vendor cells), and leaving them configured-but-unused recreates the pad's
  //  P2X input-term conflict. RX is LOCAL1X fabric sampling below; the raw ibuf nets feed it.
  // ================================================================================================
  logic [DQ_WIDTH-1:0] dq_ddr_out;
  genvar gi;
  generate
    for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_dq_tx
      tennm_ph2_ddio_out #(
        .mode      ("MODE_DDR"),
        .asclr_ena ("ASCLR_ENA_NONE"),
        .sclr_ena  ("SCLR_ENA_NONE")
      ) u_dq_tx (
        .ena      (1'b1),
        .areset   (1'b1),   // ACTIVE-LOW reset_n on ph2 atoms
        .sreset   (1'b0),
        .datainhi (phy_dq_o[DQ_WIDTH + gi]),  // byte A bit i
        .datainlo (tx_b[gi]),                 // byte B bit i (TX_B_DLY-aligned)
        .dataout  (dq_ddr_out[gi]),
        .clk      (clk)
      );
      assign hb_dq[gi] = dqoe_q ? dq_ddr_out[gi] : 1'bz;
    end
  endgenerate

  wire [DQ_WIDTH-1:0] dq_raw   = hb_dq;     // raw input-buffer view (fabric)
  wire                rwds_raw = hb_rwds;   // raw RWDS view

  // ================================================================================================
  //  CK — vendor DDR-out GPIO cell with clock-enable: emits clk on the pin while cke is high.
  //  CK_DIN_HI selects which sub-phase carries the high level (a half-cycle phase knob).
  // ================================================================================================
  hbgpio_ck_cell u_ck_cell (
    .ck      (clk),
    .din     (CK_DIN_HI ? 2'b10 : 2'b01),
    .cke     (cken_q),
    .pad_out (hb_ck)
  );

  // ================================================================================================
  //  RWDS — bring-up-proven path: tennm TX atom for the DDR write mask + inferred tristate pad;
  //  the raw input feeds dq_cell.ck_in (above), the strobe-domain RX below, and the level sync.
  // ================================================================================================
  logic rwds_ddr_out;
  tennm_ph2_ddio_out #(
    .mode      ("MODE_DDR"),
    .asclr_ena ("ASCLR_ENA_NONE"),
    .sclr_ena  ("SCLR_ENA_NONE")
  ) u_rwds_tx (
    .ena      (1'b1),
    .areset   (1'b1),   // ACTIVE-LOW reset_n on ph2 atoms
    .sreset   (1'b0),
    .datainhi (phy_rwds_o[1]),
    .datainlo (tx_rwds_b),
    .dataout  (rwds_ddr_out),
    .clk      (clk)
  );
  assign hb_rwds = rwoe_q ? rwds_ddr_out : 1'bz;

  // ================================================================================================
  //  RX : LOCAL1X — both-edge fabric sampling at the 1x clock + the silicon-proven SDR edge-detect
  //  pairing. posedge and negedge registers each sample DQ/RWDS once per clk, yielding the same
  //  2-samples-per-CK stream the SDR PHY's algorithm consumed from its 2x clock — with NO 2x clock.
  //  The pairing FSM is fully synchronous in the clk domain and processes both samples per cycle in
  //  arrival order (pos_q from the previous posedge, then neg_q from the mid-cycle negedge). RWDS
  //  RISING edges across consecutive samples tag byte A, falling edges complete {A,B} words —
  //  self-aligning against the device's tCKDS flight-delay phase, exactly like the SDR PHY.
  // ================================================================================================
  localparam int unsigned RXF_DEPTH = 32;
  localparam int unsigned RXF_AW    = $clog2(RXF_DEPTH);
  localparam int unsigned SKIPW     = (RD_PREAMBLE_SKIP == 0) ? 1 : $clog2(RD_PREAMBLE_SKIP + 1);

  // Both-edge input sampling (reset-less datapath). neg-edge regs are retimed into the posedge
  // domain half a cycle later (a constrained 2.5 ns path at 200 MHz).
  logic [DQ_WIDTH-1:0] dq_pos_q, dq_neg_q, dq_neg_ret;
  logic                rwds_pos_q, rwds_neg_q, rwds_neg_ret;
  always_ff @(posedge clk) begin
    dq_pos_q   <= dq_raw;
    rwds_pos_q <= rwds_raw;
  end
  /* verilator lint_off SYNCASYNCNET */
  always_ff @(negedge clk) begin
    dq_neg_q   <= dq_raw;
    rwds_neg_q <= rwds_raw;
  end
  /* verilator lint_on SYNCASYNCNET */
  always_ff @(posedge clk) begin
    dq_neg_ret   <= dq_neg_q;
    rwds_neg_ret <= rwds_neg_q;
  end

  logic [PHYW-1:0]  rxf_mem [RXF_DEPTH];
  logic [RXF_AW:0]  wptr_bin, rptr_bin;
  logic [SKIPW-1:0] pre_skip;
  logic [DQ_WIDTH-1:0] rx_byte_a;
  logic have_a, rwds_prev;

  // Two-sample-per-cycle SDR pairing, unrolled. Sample order each posedge k:
  //   s0 = {rwds_neg_ret, dq_neg_ret}  (captured at negedge k-1.5 -> retimed; OLDER)
  //   s1 = {rwds_pos_q,   dq_pos_q}    (captured at posedge k-1;   NEWER)
  wire s0_rise =  rwds_neg_ret & ~rwds_prev;
  wire s0_fall = ~rwds_neg_ret &  rwds_prev;
  wire s1_rise =  rwds_pos_q   & ~rwds_neg_ret;
  wire s1_fall = ~rwds_pos_q   &  rwds_neg_ret;

  // At a CK-rate strobe, the two samples of one clk cycle can carry at most ONE completed word
  // (a fall needs a preceding rise; two falls per cycle would need a 2x-rate strobe). Single-push
  // logic, cases enumerated:
  //   (s0_fall)            -> completes the word begun in an earlier cycle
  //   (s0_rise, s1_fall)   -> rise and completion within this cycle ({dq_neg_ret, dq_pos_q})
  //   (s1_fall)            -> completes the word begun earlier
  //   rises without falls  -> just load/skip byte A
  wire s0_ok = (pre_skip == '0);
  logic            push;
  logic [PHYW-1:0] pushval;
  always_comb begin
    push    = 1'b0;
    pushval = '0;
    if (s0_fall && have_a) begin
      push    = 1'b1;
      pushval = {rx_byte_a, dq_neg_ret};
    end else if (s1_fall && (s0_rise ? s0_ok : have_a)) begin
      push    = 1'b1;
      pushval = {s0_rise ? dq_neg_ret : rx_byte_a, dq_pos_q};
    end
  end

  always_ff @(posedge clk) begin
    if (rst || !phy_rd_arm) begin
      wptr_bin  <= '0;
      have_a    <= 1'b0;
      rwds_prev <= 1'b0;
      pre_skip  <= SKIPW'(RD_PREAMBLE_SKIP);
      rx_byte_a <= '0;
    end else begin
      rwds_prev <= rwds_pos_q;
      // preamble skip consumes rising edges (at most one rise per cycle at a CK-rate strobe)
      if ((s0_rise || s1_rise) && pre_skip != '0) begin
        pre_skip <= pre_skip - 1'b1;
        have_a   <= 1'b0;
      end else begin
        // byte-A load: the newest unconsumed rise wins; a push consumes the pending A
        if (s1_rise)                 begin rx_byte_a <= dq_pos_q;  have_a <= 1'b1; end
        else if (s0_rise && !s1_fall) begin rx_byte_a <= dq_neg_ret; have_a <= 1'b1; end
        else if (push)               have_a <= 1'b0;
      end
      if (push) begin
        rxf_mem[wptr_bin[RXF_AW-1:0]] <= pushval;
        wptr_bin                      <= wptr_bin + 1'b1;
      end
    end
  end

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

  // RWDS level sync for the controller's latency select / stall watch.
  logic rwds_s1;
  always_ff @(posedge clk) begin
    if (rst) begin
      rwds_s1    <= 1'b0;
      phy_rwds_i <= 1'b0;
    end else begin
      rwds_s1    <= rwds_raw;
      phy_rwds_i <= rwds_s1;
    end
  end

endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNDRIVEN */
