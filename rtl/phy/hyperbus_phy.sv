// hyperbus_phy — PHY wrapper: selects the DDR-IO variant by string parameter PHY_VARIANT.
//
// Frozen port list (docs/INTERFACES.md §hyperbus_phy). This is a pure structural selector: it
// instantiates exactly one of the variant implementations (generic | sdr | intel/altera | xilinx)
// inside a generate-if so that only the selected variant is elaborated. For sim, pick
// PHY_VARIANT="GENERIC" (behavioural DDR) or PHY_VARIANT="SDR" (portable single-clock-phase SDR,
// hyperbus_phy_sdr — see that file); the vendor variants are synth-only skeletons.
//
// All variants share this exact port list, so the wrapper just forwards every signal 1:1.
// NOTE for "SDR": that variant REPURPOSES the `clk90` port as a 2x byte-serialisation clock (single
// PLL, 0deg — NOT a 90deg phase); see hyperbus_phy_sdr.sv. Port names/widths are unchanged.
`ifndef HYPERBUS_PHY_SV
`define HYPERBUS_PHY_SV
`timescale 1ns/1ps
module hyperbus_phy
  import hyperbus_pkg::*;
#(
    parameter int unsigned DQ_WIDTH    = HB_DQ_WIDTH_DEFAULT,
    parameter int unsigned DATA_WIDTH  = 2 * DQ_WIDTH,
    parameter int unsigned ADDR_WIDTH  = HB_ADDR_WIDTH,
    parameter int unsigned LEN_WIDTH   = HB_LEN_WIDTH_DEFAULT,
    parameter              PHY_VARIANT = "GENERIC",   // GENERIC | INTEL | XILINX
    parameter bit          DIFF_CK     = 1'b1
) (
    input  logic                clk,
    input  logic                clk90,
    input  logic                clk_ref,
    input  logic                rst,

    // ---- ctrl-facing (slave; mirror of hyperbus_ctrl TX/RX) ----
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

    // ---- device pins (split; board wrapper adds tristate) ----
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

  generate
    if (PHY_VARIANT == "INTEL" || PHY_VARIANT == "ALTERA") begin : g_altera
      hyperbus_phy_altera #(
        .DQ_WIDTH(DQ_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH),
        .LEN_WIDTH(LEN_WIDTH), .PHY_VARIANT(PHY_VARIANT), .DIFF_CK(DIFF_CK)
      ) u_var (
        .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
        .phy_cs_n(phy_cs_n), .phy_rst_n(phy_rst_n), .phy_ck_en(phy_ck_en),
        .phy_dq_o(phy_dq_o), .phy_dq_oe(phy_dq_oe), .phy_rwds_o(phy_rwds_o),
        .phy_rwds_oe(phy_rwds_oe), .phy_rd_arm(phy_rd_arm),
        .phy_dq_i(phy_dq_i), .phy_dq_i_valid(phy_dq_i_valid), .phy_rwds_i(phy_rwds_i),
        .hb_ck(hb_ck), .hb_ck_n(hb_ck_n), .hb_cs_n(hb_cs_n), .hb_rst_n(hb_rst_n),
        .hb_dq_o(hb_dq_o), .hb_dq_oe(hb_dq_oe), .hb_dq_i(hb_dq_i),
        .hb_rwds_o(hb_rwds_o), .hb_rwds_oe(hb_rwds_oe), .hb_rwds_i(hb_rwds_i)
      );
    end else if (PHY_VARIANT == "SDR") begin : g_sdr
      hyperbus_phy_sdr #(
        .DQ_WIDTH(DQ_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH),
        .LEN_WIDTH(LEN_WIDTH), .PHY_VARIANT(PHY_VARIANT), .DIFF_CK(DIFF_CK)
      ) u_var (
        .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
        .phy_cs_n(phy_cs_n), .phy_rst_n(phy_rst_n), .phy_ck_en(phy_ck_en),
        .phy_dq_o(phy_dq_o), .phy_dq_oe(phy_dq_oe), .phy_rwds_o(phy_rwds_o),
        .phy_rwds_oe(phy_rwds_oe), .phy_rd_arm(phy_rd_arm),
        .phy_dq_i(phy_dq_i), .phy_dq_i_valid(phy_dq_i_valid), .phy_rwds_i(phy_rwds_i),
        .hb_ck(hb_ck), .hb_ck_n(hb_ck_n), .hb_cs_n(hb_cs_n), .hb_rst_n(hb_rst_n),
        .hb_dq_o(hb_dq_o), .hb_dq_oe(hb_dq_oe), .hb_dq_i(hb_dq_i),
        .hb_rwds_o(hb_rwds_o), .hb_rwds_oe(hb_rwds_oe), .hb_rwds_i(hb_rwds_i)
      );
    end else if (PHY_VARIANT == "XILINX") begin : g_xilinx
      hyperbus_phy_xilinx #(
        .DQ_WIDTH(DQ_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH),
        .LEN_WIDTH(LEN_WIDTH), .PHY_VARIANT(PHY_VARIANT), .DIFF_CK(DIFF_CK)
      ) u_var (
        .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
        .phy_cs_n(phy_cs_n), .phy_rst_n(phy_rst_n), .phy_ck_en(phy_ck_en),
        .phy_dq_o(phy_dq_o), .phy_dq_oe(phy_dq_oe), .phy_rwds_o(phy_rwds_o),
        .phy_rwds_oe(phy_rwds_oe), .phy_rd_arm(phy_rd_arm),
        .phy_dq_i(phy_dq_i), .phy_dq_i_valid(phy_dq_i_valid), .phy_rwds_i(phy_rwds_i),
        .hb_ck(hb_ck), .hb_ck_n(hb_ck_n), .hb_cs_n(hb_cs_n), .hb_rst_n(hb_rst_n),
        .hb_dq_o(hb_dq_o), .hb_dq_oe(hb_dq_oe), .hb_dq_i(hb_dq_i),
        .hb_rwds_o(hb_rwds_o), .hb_rwds_oe(hb_rwds_oe), .hb_rwds_i(hb_rwds_i)
      );
    end else begin : g_generic
      hyperbus_phy_generic #(
        .DQ_WIDTH(DQ_WIDTH), .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH),
        .LEN_WIDTH(LEN_WIDTH), .PHY_VARIANT(PHY_VARIANT), .DIFF_CK(DIFF_CK)
      ) u_var (
        .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
        .phy_cs_n(phy_cs_n), .phy_rst_n(phy_rst_n), .phy_ck_en(phy_ck_en),
        .phy_dq_o(phy_dq_o), .phy_dq_oe(phy_dq_oe), .phy_rwds_o(phy_rwds_o),
        .phy_rwds_oe(phy_rwds_oe), .phy_rd_arm(phy_rd_arm),
        .phy_dq_i(phy_dq_i), .phy_dq_i_valid(phy_dq_i_valid), .phy_rwds_i(phy_rwds_i),
        .hb_ck(hb_ck), .hb_ck_n(hb_ck_n), .hb_cs_n(hb_cs_n), .hb_rst_n(hb_rst_n),
        .hb_dq_o(hb_dq_o), .hb_dq_oe(hb_dq_oe), .hb_dq_i(hb_dq_i),
        .hb_rwds_o(hb_rwds_o), .hb_rwds_oe(hb_rwds_oe), .hb_rwds_i(hb_rwds_i)
      );
    end
  endgenerate

endmodule
`endif
