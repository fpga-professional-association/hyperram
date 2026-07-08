// hyperbus_phy_xilinx — AMD/Xilinx (7-series/UltraScale) DDR-IO PHY variant. SYNTH-ONLY SKELETON.
//
// Same frozen port list as hyperbus_phy_generic / the `hyperbus_phy` contract (docs/INTERFACES.md).
// This variant is a PLACEHOLDER: it marks where the vendor primitives go for a real board build. It
// is NOT expected to simulate under Verilator — use PHY_VARIANT="GENERIC" for simulation. The behavi-
// oural body below merely keeps the module elaboratable and its outputs defined; every spot that must
// become a hard-IP primitive in silicon is flagged `TODO(xilinx)`.
//
// Semantics to preserve when filling this in (see hyperbus_phy_generic for the reference behaviour):
//   * DQ / RWDS transmit  : ODDR (DDR_CLK_EDGE="SAME_EDGE"), byte A on the 1st sub-phase (phy_dq_o
//                           high half), byte B on the 2nd; OBUFT tri-state from the *_oe registers.
//   * CK generation       : ODDR clocked by clk90 fed {phy_ck_en,1'b0} so CK is centre-aligned to the
//                           DQ eye and idles Low; OBUFDS when DIFF_CK, else OBUF (hb_ck_n unused).
//   * DQ / RWDS receive   : per-bit IDELAYE2 on RWDS (~90 deg, needs IDELAYCTRL on clk_ref=200 MHz) to
//                           move the strobe into the data eye, IDDR/ISERDESE2 clocked by the delayed
//                           RWDS to recover byte A/B, then an async FIFO (RWDS->clk) for the elastic
//                           hand-off. Emit one clk-domain word per recovered pair on phy_dq_i/_valid.
//   * RWDS level          : 2-flop synchroniser of the raw RWDS pin into clk for phy_rwds_i.

`timescale 1ns/1ps

/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNUSEDSIGNAL */
module hyperbus_phy_xilinx
  import hyperbus_pkg::*;
#(
    parameter int unsigned DQ_WIDTH    = HB_DQ_WIDTH_DEFAULT,
    parameter int unsigned DATA_WIDTH  = 2 * DQ_WIDTH,
    parameter int unsigned ADDR_WIDTH  = HB_ADDR_WIDTH,
    parameter int unsigned LEN_WIDTH   = HB_LEN_WIDTH_DEFAULT,
    parameter              PHY_VARIANT = "XILINX",
    parameter bit          DIFF_CK     = 1'b1
) (
    input  logic                clk,
    input  logic                clk90,
    input  logic                clk_ref,    // 200 MHz IDELAYCTRL reference
    input  logic                rst,

    input  logic                phy_cs_n,
    input  logic                phy_rst_n,
    input  logic                phy_ck_en,
    input  logic [2*DQ_WIDTH-1:0] phy_dq_o,
    input  logic                phy_dq_oe,
    input  logic [1:0]          phy_rwds_o,
    input  logic                phy_rwds_oe,
    input  logic                phy_rd_arm,
    output logic [2*DQ_WIDTH-1:0] phy_dq_i,
    output logic                phy_dq_i_valid,
    output logic                phy_rwds_i,

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
  localparam int unsigned PHYW = 2 * DQ_WIDTH;

  // ==================================================================
  //  TODO(xilinx): replace this placeholder body with vendor primitives.
  //  ODDR / IDDR / ISERDESE2 / IDELAYE2 (+ IDELAYCTRL on clk_ref) / OBUFT / OBUFDS.
  //  The assignments below are only tie-offs so the module elaborates; they are
  //  NOT DDR-correct and this variant is not intended to simulate.
  // ==================================================================

  // --- control pipeline (same intent as the generic variant) ---
  always_ff @(posedge clk) begin
    if (rst) begin
      hb_cs_n <= 1'b1; hb_rst_n <= 1'b0; hb_dq_oe <= 1'b0; hb_rwds_oe <= 1'b0;
    end else begin
      hb_cs_n <= phy_cs_n; hb_rst_n <= phy_rst_n; hb_dq_oe <= phy_dq_oe; hb_rwds_oe <= phy_rwds_oe;
    end
  end

  // --- TX DQ / RWDS: TODO instantiate ODDR per bit (byte A = high half, byte B = low half) ---
  assign hb_dq_o   = phy_dq_o[PHYW-1:DQ_WIDTH];  // TODO(xilinx): ODDR {phy_dq_o[hi], phy_dq_o[lo]}
  assign hb_rwds_o = phy_rwds_o[1];              // TODO(xilinx): ODDR {phy_rwds_o[1], phy_rwds_o[0]}

  // --- CK: TODO ODDR on clk90 fed {phy_ck_en,1'b0}; OBUFDS if DIFF_CK ---
  assign hb_ck   = 1'b0;                          // TODO(xilinx)
  assign hb_ck_n = DIFF_CK ? 1'b1 : 1'b1;         // TODO(xilinx): OBUFDS complementary

  // --- RX: TODO IDELAYE2 + IDDR/ISERDESE2 on delayed RWDS, then RWDS->clk async FIFO ---
  assign phy_dq_i       = '0;                     // TODO(xilinx): recovered read word
  assign phy_dq_i_valid = 1'b0;                   // TODO(xilinx)

  // --- RWDS level 2-flop synchroniser (this part is variant-independent) ---
  logic rwds_s1;
  always_ff @(posedge clk) begin
    if (rst) begin rwds_s1 <= 1'b0; phy_rwds_i <= 1'b0; end
    else     begin rwds_s1 <= hb_rwds_i; phy_rwds_i <= rwds_s1; end
  end

  // keep contract-only inputs from tripping unused-net checks in this skeleton
  logic _unused_ok;
  assign _unused_ok = &{1'b0, clk90, clk_ref, phy_dq_o, phy_rwds_o, phy_rd_arm, hb_dq_i,
                        ADDR_WIDTH[0], LEN_WIDTH[0], DATA_WIDTH[0]};

endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
