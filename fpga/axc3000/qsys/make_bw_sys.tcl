# make_bw_sys.tcl — qsys-script that constructs + saves fpga/axc3000/qsys/bw_sys.qsys
#
# The on-chip control/clock backbone for the AXC3000 HyperBus bandwidth test:
#   * 25 MHz board clock in  (CLK_25M_C)
#   * Agilex-3 IOPLL: outclk0 = clk (CK_MHZ, 0 deg = HyperBus CK word clock; the ONLY clock in the
#                     I/O periphery — clocks the controller, fabric, and ALL of the DDIO PHY's I/O
#                     cells under CK_SCHEME="CLK_DLY", issue #8),
#                     outclk1 = (2x CK_MHZ, 0 deg) CORE-ONLY fabric clock: the DDIO PHY's FABRIC2X
#                     CK generator + the hyperbus_capture debug sampler. It clocks NO I/O cell. (The export interface is still named "clk90" for the frozen port
#                     list; it is NOT a 90 deg phase — that second phase is exactly what the DDIO
#                     PHY could not route into Bank 3A, Fitter err 24403/24404.)
#   * reset bridge + reset controller (synchronous, active-high fabric reset, 50 MHz domain)
#   * Altera JTAG-to-Avalon-MM master bridge — its Avalon-MM master is EXPORTED to top.sv,
#     where it drives the hyperram_bw_test CSR slave (LEN/BASE/CTRL/STATUS/…).
#
# Run (headless, in the Quartus-Pro docker; see fpga/axc3000/README.md):
#   qsys-script --script=qsys/make_bw_sys.tcl
#   qsys-generate qsys/bw_sys.qsys --synthesis=VERILOG --family="Agilex 3" \
#       --part=A3CY100BM16AE7S --output-directory=qsys/bw_sys
#
# CLOCK PLAN (DDIO, issue #8): the frozen controller is WORD-per-clk and the DDIO PHY moves 2 bytes
# per clk on both CK edges, so EVERYTHING — controller, fabric, Qsys backbone, and every I/O-cell
# DDIO register — runs on outclk0 = CK_MHZ. hb_ck = CK_MHZ (DDIO-forwarded, delay-centred at the
# pin). outclk1 (same freq, 0 deg) only clocks the fabric debug capture.

package require -exact qsys 26.1

# ---------------------------------------------------------------------------
create_system bw_sys
set_project_property DEVICE_FAMILY {Agilex 3}
set_project_property DEVICE       {A3CY100BM16AE7S}

# ===========================================================================
# 25 MHz board clock input
# ===========================================================================
add_instance clk_in altera_clock_bridge
set_instance_parameter_value clk_in EXPLICIT_CLOCK_RATE {25000000.0}
set_instance_parameter_value clk_in NUM_CLOCK_OUTPUTS   {1}

# ===========================================================================
# IOPLL: 25 MHz ref -> outclk0 = CK_MHZ @0deg (clk: CK word clock + ALL I/O periphery),
#                      outclk1 = CK_MHZ @0deg (fabric-only debug-capture clock — exported as "clk90")
# Both outputs are 0 deg: there is NO second PLL phase into the I/O periphery (CK eye-centring is a
# hard output-delay on the hb_ck pin, bw.qsf). This is the 24403/24404 fix (issue #8).
# ===========================================================================
# SPEED-RAMP knob: CK_MHZ = hb_ck target (= clk word clock = outclk0).
# DDIO PHY clock plan (issue #8, CK_SCHEME="CLK_DLY"): the I/O runs at 1x CK — the DQ *and* CK DDIOs
# are all clocked by outclk0 (ONE periphery clock, no 24403/24404; CK eye-centring comes from the
# hard output-delay assignment on the hb_ck pin in bw.qsf). outclk1 is no longer a 2x byte clock:
# it only feeds the hyperbus_capture debug sampler in the fabric, so it runs at 1x CK too (a 2x
# clock would be 400 MHz at the 200 MHz device ceiling — unclosable and unneeded).
# Peak throughput ~= 2 bytes * CK_MHZ MB/s per direction (DDR x8).
#   50 -> ~100 MB/s | 100 -> ~200 | 175 -> ~350 (SDR ceiling was here) | 200 -> ~400 (device max).
set CK_MHZ   175.0
set BYTE_MHZ [expr {2.0 * $CK_MHZ}]

add_instance iopll altera_iopll
set_instance_parameter_value iopll gui_reference_clock_frequency {25.0}
set_instance_parameter_value iopll gui_operation_mode            {direct}
set_instance_parameter_value iopll gui_use_locked                {1}
set_instance_parameter_value iopll gui_number_of_clocks          {2}
set_instance_parameter_value iopll gui_output_clock_frequency0   $CK_MHZ
set_instance_parameter_value iopll gui_phase_shift_deg0          {0.0}
set_instance_parameter_value iopll gui_output_clock_frequency1   $BYTE_MHZ
set_instance_parameter_value iopll gui_phase_shift_deg1          {0.0}

# Clock bridges: an IOPLL output clock that is BOTH exported AND fanned to internal sinks loses its
# internal connections on save (the export wins). So the internal fabric (rst_ctrl, jtag_master)
# taps iopll.outclkN directly, while a dedicated clock bridge per output carries the SAME clock to
# the top-level export. Mirrors the reference clock_system.qsys idiom (and the rst_out bridge below).
add_instance clkbr0 altera_clock_bridge
set_instance_parameter_value clkbr0 EXPLICIT_CLOCK_RATE [expr {$CK_MHZ   * 1.0e6}]
set_instance_parameter_value clkbr0 NUM_CLOCK_OUTPUTS   {1}
add_instance clkbr1 altera_clock_bridge
set_instance_parameter_value clkbr1 EXPLICIT_CLOCK_RATE [expr {$BYTE_MHZ * 1.0e6}]
set_instance_parameter_value clkbr1 NUM_CLOCK_OUTPUTS   {1}

# ===========================================================================
# Reset in (async board button) -> synchronised, active-high
# ===========================================================================
add_instance reset_in altera_reset_bridge
set_instance_parameter_value reset_in ACTIVE_LOW_RESET  {0}
set_instance_parameter_value reset_in SYNCHRONOUS_EDGES {deassert}
set_instance_parameter_value reset_in NUM_RESET_OUTPUTS {1}
set_instance_parameter_value reset_in USE_RESET_REQUEST {0}

# Reset controller: re-synchronise the reset into the 50 MHz (outclk0) domain
add_instance rst_ctrl altera_reset_controller
set_instance_parameter_value rst_ctrl NUM_RESET_INPUTS       {1}
set_instance_parameter_value rst_ctrl SYNC_DEPTH             {3}
set_instance_parameter_value rst_ctrl OUTPUT_RESET_SYNC_EDGES {deassert}
set_instance_parameter_value rst_ctrl RESET_REQUEST_PRESENT   {0}

# Fan the synchronised fabric reset back out to a bridge so it can be exported to top.sv
add_instance rst_out altera_reset_bridge
set_instance_parameter_value rst_out ACTIVE_LOW_RESET  {0}
set_instance_parameter_value rst_out SYNCHRONOUS_EDGES {none}
set_instance_parameter_value rst_out NUM_RESET_OUTPUTS {1}
set_instance_parameter_value rst_out USE_RESET_REQUEST {0}

# ===========================================================================
# JTAG-to-Avalon-MM master bridge — control plane. Its Avalon-MM master is exported.
# ===========================================================================
add_instance jtag_master altera_jtag_avalon_master
set_instance_parameter_value jtag_master USE_PLI {0}

# ===========================================================================
# Connections
# ===========================================================================
# clocks
add_connection clk_in.out_clk iopll.refclk

add_connection iopll.outclk0 rst_ctrl.clk
add_connection iopll.outclk0 jtag_master.clk
add_connection iopll.outclk0 clkbr0.in_clk
add_connection iopll.outclk1 clkbr1.in_clk

# resets
add_connection reset_in.out_reset iopll.reset
add_connection reset_in.out_reset rst_ctrl.reset_in0
add_connection rst_ctrl.reset_out rst_out.in_reset
add_connection rst_ctrl.reset_out jtag_master.clk_reset

# The reset_in bridge (SYNCHRONOUS_EDGES=deassert) needs a clock domain (raw board clock).
# rst_out (SYNCHRONOUS_EDGES=none) is a pure pass-through and has no clock interface.
add_connection clk_in.out_clk reset_in.clk

# ===========================================================================
# Exports (top-level interfaces)
# ===========================================================================
set_interface_property clk_25   EXPORT_OF clk_in.in_clk
set_interface_property reset    EXPORT_OF reset_in.in_reset
set_interface_property clk      EXPORT_OF clkbr0.out_clk
set_interface_property clk90    EXPORT_OF clkbr1.out_clk
set_interface_property locked   EXPORT_OF iopll.locked
set_interface_property sys_reset EXPORT_OF rst_out.out_reset
set_interface_property master   EXPORT_OF jtag_master.master

# ---------------------------------------------------------------------------
save_system qsys/bw_sys.qsys
