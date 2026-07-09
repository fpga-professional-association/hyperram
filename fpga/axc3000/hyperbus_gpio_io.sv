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
  //  DQ — vendor bidir DDR GPIO cell (hbgpio_dq_cell): din/dout are per-pin interleaved pairs
  //  {bit 2i+1 = first/hi sub-phase, bit 2i = second/lo sub-phase}; oe is per pin. ck_out = clk
  //  (launch), ck_in = raw RWDS (source-synchronous capture; the cell's DDIO_WITH_DELAY gives the
  //  data-vs-strobe margin). pad_io is the real pin.
  // ================================================================================================
  logic [2*DQ_WIDTH-1:0] dq_din;
  logic [2*DQ_WIDTH-1:0] dq_dout;
  genvar gi;
  generate
    for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_dqmap
      assign dq_din[2*gi+1] = phy_dq_o[DQ_WIDTH + gi];  // byte A bit i — first sub-phase
      assign dq_din[2*gi]   = tx_b[gi];                 // byte B bit i — second sub-phase (delayed)
    end
  endgenerate

  wire rwds_raw = hb_rwds;   // raw input buffer view of the (bidir) RWDS pin

  hbgpio_dq_cell u_dq_cell (
    .ck_in  (rwds_raw),
    .ck_out (clk),
    .dout   (dq_dout),
    .din    (dq_din),
    .oe     ({DQ_WIDTH{dqoe_q}}),
    .pad_io (hb_dq)
  );

  // RX pair rails from the cell's input DDIO registers (rwds domain).
  logic [DQ_WIDTH-1:0] rx_hi, rx_lo;
  generate
    for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_rxmap
      assign rx_hi[gi] = dq_dout[2*gi+1];   // sampled at strobe rise (byte A)
      assign rx_lo[gi] = dq_dout[2*gi];     // sampled at strobe fall (byte B)
    end
  endgenerate

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
  //  RX : strobe-domain byte pairing + preamble skip + elastic FIFO + clk-side drain.
  //  Same machinery proven in the DDIO bring-up (hyperbus_phy_altera FABRIC scheme): the pairing
  //  process clocks on the raw strobe and reads the cell's dout rails one edge late (pre-edge
  //  views => matched {A(n),B(n)} pairs, prime-gated first push); the whole write side is held in
  //  async reset by rx_flush_q (clk-registered, glitch-free) while the receiver is disarmed.
  // ================================================================================================
  localparam int unsigned RXF_DEPTH = 32;
  localparam int unsigned RXF_AW    = $clog2(RXF_DEPTH);
  localparam int unsigned SKIPW     = (RD_PREAMBLE_SKIP == 0) ? 1 : $clog2(RD_PREAMBLE_SKIP + 1);

  logic [PHYW-1:0]  rxf_mem [RXF_DEPTH];
  logic [RXF_AW:0]  wptr_bin, rptr_bin;
  logic [SKIPW-1:0] pre_skip = SKIPW'(RD_PREAMBLE_SKIP);
  logic             rx_prime;

  logic rx_flush_q;
  always_ff @(posedge clk) begin
    if (rst) rx_flush_q <= 1'b1;
    else     rx_flush_q <= ~phy_rd_arm;
  end

  /* verilator lint_off SYNCASYNCNET */
  always_ff @(posedge rwds_raw or posedge rx_flush_q) begin
    if (rx_flush_q) begin
      wptr_bin <= '0;
      rx_prime <= 1'b0;
      pre_skip <= SKIPW'(RD_PREAMBLE_SKIP);
    end else if (pre_skip != '0) begin
      pre_skip <= pre_skip - 1'b1;
      rx_prime <= 1'b0;
    end else begin
      rx_prime <= 1'b1;
      if (rx_prime) begin
        rxf_mem[wptr_bin[RXF_AW-1:0]] <= {rx_hi, rx_lo};
        wptr_bin                      <= wptr_bin + 1'b1;
      end
    end
  end
  /* verilator lint_on SYNCASYNCNET */

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
