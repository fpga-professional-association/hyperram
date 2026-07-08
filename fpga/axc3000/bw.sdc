# bw.sdc — timing constraints for the AXC3000 HyperBus bandwidth-test build.
#
# Clock architecture: one 25 MHz board XO -> IOPLL -> clk (50 MHz, 0deg) + clk90 (50 MHz, +90deg).
# The IOPLL-generated clocks and the JTAG-to-Avalon bridge's TCK are constrained by the Qsys IP's own
# generated .sdc (pulled in through bw_sys.qip); here we only anchor the board clock, let Quartus
# derive the PLL outputs from it, add uncertainty, and cut the asynchronous push-button reset.

# ---- board reference clock (25 MHz on CLK_25M_C) ----
create_clock -name CLK_25M_C -period 40.000 [get_ports CLK_25M_C]

# ---- derive the IOPLL output clocks (clk / clk90) from the reference ----
derive_pll_clocks
derive_clock_uncertainty

# ---- asynchronous, debounced-in-firmware push button: do not time it ----
set_false_path -from [get_ports USER_BTN] -to [all_registers]

# The user LEDs are slow status indicators; cut them from timing.
set_false_path -to [get_ports {LED1 RLED GLED}]
