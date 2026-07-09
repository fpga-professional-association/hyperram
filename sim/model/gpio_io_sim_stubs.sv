// gpio_io_sim_stubs — Verilator-only behavioral stand-ins for the vendor atoms used by
// fpga/axc3000/hyperbus_gpio_io.sv, so the LOCAL1X RX pairing FSM can be simulated and swept
// across flight-delay phases (sim/tb_local1x.sv) instead of debugged one Quartus compile at a time.
//
// Semantics modeled from the on-silicon findings on the ddio-200 branch:
//   * tennm_ph2_ddio_out: registers {datainhi, datainlo} on posedge clk; emits datainLO during the
//     FIRST half of the following cycle and datainHI during the SECOND half (lo-first, as observed
//     on the wire), with a small launch delay.
//   * hbgpio_ck_cell: DDR-out GPIO with clock-enable — emits din[1] during the first half-cycle and
//     din[0] during the second while cke (registered per cycle) is high; idles low.
// Compiled ONLY by the Verilator testbench (never listed in bw.qsf).

`timescale 1ns/1ps

module tennm_ph2_ddio_out #(
    parameter mode      = "MODE_DDR",
    parameter asclr_ena = "ASCLR_ENA_NONE",
    parameter sclr_ena  = "SCLR_ENA_NONE"
) (
    input  logic ena,
    input  logic areset,   // active-low reset_n (vendor ties 1'b1 when unused)
    input  logic sreset,
    input  logic datainhi,
    input  logic datainlo,
    output logic dataout,
    input  logic clk
);
  logic hi_q, lo_q;
  always_ff @(posedge clk) begin
    if (ena) begin
      hi_q <= datainhi;
      lo_q <= datainlo;
    end
  end
  // lo-first emission across the following cycle
  assign dataout = (areset === 1'b0) ? 1'b0 : (clk ? lo_q : hi_q);
endmodule

module hbgpio_ck_cell (
    input  logic       ck,
    input  logic [1:0] din,
    input  logic       cke,
    output logic       pad_out
);
  logic cke_q;
  logic [1:0] din_q;
  always_ff @(posedge ck) begin
    cke_q <= cke;
    din_q <= din;
  end
  assign pad_out = cke_q ? (ck ? din_q[1] : din_q[0]) : 1'b0;
endmodule
