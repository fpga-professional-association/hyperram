// hyperbus_pads_altera — Agilex-3 bidirectional HyperBus pad ring for the AXC3000 board build.
//
// The frozen `hyperbus_phy` contract deliberately exposes SPLIT pins (hb_dq_o / hb_dq_oe / hb_dq_i,
// and likewise RWDS) with no `inout`, so the PHY stays reusable/Verilator-shaped and the true
// tri-state pad lives here, at the board boundary (docs/INTERFACES.md, "Board integration").
//
//   *** synthesis-only board file (targets the AXC3000 / Agilex-3 fitter) ***
//
// I/O style — INFERRED tri-state (docs/INTERFACES.md board note: `assign hb_dq = hb_dq_oe ? hb_dq_o
// : 'z`). We deliberately do NOT wrap the pins in explicit tennm_ph2_io_obuf/io_ibuf primitives:
// the PHY drives a SINGLE registered DQ output-enable (hb_dq_oe, one bit for the whole x8 bus, from
// one tennm_ph2_ddio_oe). A single hard DDIO_OE I/O-cell register can only serve ONE pin's output
// buffer, so feeding it to 8 explicit io_obuf.oe inputs makes the Fitter fail with
// "cannot place 1 DDIO_OE / no routing connectivity" (u_dq_ddio_oe). With inferred tri-states the
// Fitter is free to REPLICATE the single-bit enable's register into each DQ I/O cell (ordinary
// register replication, which a hard primitive forbids), preserving the 1-clk OE launch latency
// that matches the DQ data DDIO — so drive/turnaround stays aligned. The per-bit DQ data still comes
// from the PHY's per-pin tennm_ph2_ddio_out and packs into the inferred output buffers.
//
// I/O standard / slew / termination come from the .qsf pin assignments (pins.tcl, 1.2 V).

`timescale 1ns/1ps

module hyperbus_pads_altera #(
    parameter int unsigned DQ_WIDTH = 8
) (
    // ---- split PHY side (to hyperbus_phy .hb_* ports) ----
    input  logic                phy_hb_ck,
    input  logic                phy_hb_ck_n,
    input  logic                phy_hb_cs_n,
    input  logic                phy_hb_rst_n,
    input  logic [DQ_WIDTH-1:0] phy_hb_dq_o,
    input  logic                phy_hb_dq_oe,
    output logic [DQ_WIDTH-1:0] phy_hb_dq_i,
    input  logic                phy_hb_rwds_o,
    input  logic                phy_hb_rwds_oe,
    output logic                phy_hb_rwds_i,

    // ---- device pads (to .qsf / pins.tcl) ----
    output logic                hb_ck,
    output logic                hb_ck_n,     // single-ended AXC3000: tied by the PHY, no board pin
    output logic                hb_cs_n,
    output logic                hb_rst_n,
    inout  wire  [DQ_WIDTH-1:0] hb_dq,
    inout  wire                 hb_rwds
);

  // ---- output-only clock/control pads (inferred output buffers) ----
  assign hb_ck    = phy_hb_ck;
  assign hb_ck_n  = phy_hb_ck_n;
  assign hb_cs_n  = phy_hb_cs_n;
  assign hb_rst_n = phy_hb_rst_n;

  // ---- bidirectional DQ[7:0]: inferred tri-state, shared registered OE replicated per pin ----
  genvar gi;
  generate
    for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_dq_pad
      assign hb_dq[gi]     = phy_hb_dq_oe ? phy_hb_dq_o[gi] : 1'bz;
      assign phy_hb_dq_i[gi] = hb_dq[gi];
    end
  endgenerate

  // ---- bidirectional RWDS: write-mask out (OE from PHY) / read strobe + latency indicator in ----
  assign hb_rwds       = phy_hb_rwds_oe ? phy_hb_rwds_o : 1'bz;
  assign phy_hb_rwds_i = hb_rwds;

endmodule
