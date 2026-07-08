# make_bw_sys.tcl — qsys-script that constructs + saves fpga/axc3000/qsys/bw_sys.qsys
#
# The on-chip control/clock backbone for the AXC3000 HyperBus bandwidth test:
#   * 25 MHz board clock in  (CLK_25M_C)
#   * Agilex-3 IOPLL: outclk0 = clk (~50 MHz, 0 deg = HyperBus CK word clock),
#                     outclk1 = clk2x (~100 MHz, 0 deg = SDR byte clock).
#     (The export interface is still named "clk90"; for the SDR PHY it carries the 100 MHz 2x byte
#      clock, NOT a 90 deg phase — that phase is what the OLD DDIO PHY could not route into Bank 3A,
#      Fitter err 24403/24404. The SDR PHY needs only ONE clock in the I/O periphery.)
#   * reset bridge + reset controller (synchronous, active-high fabric reset, 50 MHz domain)
#   * Altera JTAG-to-Avalon-MM master bridge — its Avalon-MM master is EXPORTED to top.sv,
#     where it drives the hyperram_bw_test CSR slave (LEN/BASE/CTRL/STATUS/…).
#
# Run (headless, in the Quartus-Pro docker; see fpga/axc3000/README.md):
#   qsys-script --script=qsys/make_bw_sys.tcl
#   qsys-generate qsys/bw_sys.qsys --synthesis=VERILOG --family="Agilex 3" \
#       --part=A3CY100BM16AE7S --output-directory=qsys/bw_sys
#
# CLOCK PLAN (SDR): the frozen controller is WORD-per-clk, so the fabric byte engine runs at 2x CK.
# clk = 50 MHz drives the controller/fabric/Qsys backbone; clk2x = 100 MHz (0 deg) is the ONLY clock
# at the Bank-3A SDR I/O registers + hb_ck generator. hb_ck = clk2x/2 = 50 MHz => ~100 MB/s per
# direction on the x8 bus. Chosen low on purpose so the un-calibrated read eye is wide.

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
# IOPLL: 25 MHz ref -> outclk0 = 50 MHz @0deg (clk, CK word clock),
#                      outclk1 = 100 MHz @0deg (clk2x, SDR byte clock — exported as "clk90")
# Both outputs are 0 deg: there is NO second PLL phase into the I/O periphery (the SDR PHY derives
# the CK-centring quarter-period shift from clk2x's own negedge). This is the 24403/24404 fix.
# ===========================================================================
add_instance iopll altera_iopll
set_instance_parameter_value iopll gui_reference_clock_frequency {25.0}
set_instance_parameter_value iopll gui_operation_mode            {direct}
set_instance_parameter_value iopll gui_use_locked                {1}
set_instance_parameter_value iopll gui_number_of_clocks          {2}
set_instance_parameter_value iopll gui_output_clock_frequency0   {50.0}
set_instance_parameter_value iopll gui_phase_shift_deg0          {0.0}
set_instance_parameter_value iopll gui_output_clock_frequency1   {100.0}
set_instance_parameter_value iopll gui_phase_shift_deg1          {0.0}

# Clock bridges: an IOPLL output clock that is BOTH exported AND fanned to internal sinks loses its
# internal connections on save (the export wins). So the internal fabric (rst_ctrl, jtag_master)
# taps iopll.outclkN directly, while a dedicated clock bridge per output carries the SAME clock to
# the top-level export. Mirrors the reference clock_system.qsys idiom (and the rst_out bridge below).
add_instance clkbr0 altera_clock_bridge
set_instance_parameter_value clkbr0 EXPLICIT_CLOCK_RATE {50000000.0}
set_instance_parameter_value clkbr0 NUM_CLOCK_OUTPUTS   {1}
add_instance clkbr1 altera_clock_bridge
set_instance_parameter_value clkbr1 EXPLICIT_CLOCK_RATE {100000000.0}
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
