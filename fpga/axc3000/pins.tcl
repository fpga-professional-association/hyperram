# pins.tcl — AXC3000 pin + I/O-standard assignments for top.sv.
#
# Values copied verbatim from
#   agilex_3_ai_benchmarks/quartus/constraints/axc3000_board.tcl
# (Arrow "AXC3000 Evaluation Board: User Guide" v1.2.1, cross-checked vs refdes-agilex3), for ONLY
# the ports top.sv actually uses. Sourced from bw.qsf after FAMILY/DEVICE are set.
#
# RESOLVED 2026-07-08 on real silicon: hb_cs_n / hb_ck use the Arrow refdes values D8/D7 (NOT the
# User Guide's C7/B5). With C7/B5 the HyperRAM never received a clock/select and no transaction
# completed on the board (writes drove into the void, reads hung: STATUS busy+error, never done).
# Source of truth: ArrowElectronics/refdes-agilex3 axc3000_pin_assignment.tcl (HR_CSn=D8, HR_CLK=D7);
# both are legal Bank-3A DQ12-group balls per SCH-TEI0131-01-P001.PDF page 7.
#
# AXC3000 HyperRAM is SINGLE-ENDED: there is no hb_ck_n board pin, so top.sv does not expose one.

########################################################################
# 25 MHz board clock (fixed XO, single-ended)
set_location_assignment PIN_A7  -to CLK_25M_C
set_instance_assignment -name IO_STANDARD "1.2 V" -to CLK_25M_C
# PIN_A7 is a general (non-dedicated-PLL-refclk) I/O on this device, so the 25 MHz reference cannot
# reach the IOPLL on a dedicated route (Fitter error 23527). Promote it to the global clock network,
# which the Fitter accepts as an IOPLL refclk source.
set_instance_assignment -name GLOBAL_SIGNAL "GLOBAL CLOCK" -to CLK_25M_C

########################################################################
# Reset — active-low USER button (S2), needs internal weak pull-up
set_location_assignment PIN_A12 -to USER_BTN
set_instance_assignment -name IO_STANDARD "1.2 V" -to USER_BTN
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to USER_BTN

########################################################################
# User LEDs (active-low, 3.3-V LVCMOS)
set_location_assignment PIN_AG21 -to LED1
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to LED1
set_location_assignment PIN_AH22 -to RLED
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to RLED
set_location_assignment PIN_AK21 -to GLED
set_instance_assignment -name IO_STANDARD "3.3-V LVCMOS" -to GLED

########################################################################
# HyperRAM (Winbond W957D8NB, 1.2 V, x8 HyperBus DDR)
set_location_assignment PIN_C3 -to hb_dq[0]
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_dq[0]
set_location_assignment PIN_C2 -to hb_dq[1]
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_dq[1]
set_location_assignment PIN_B4 -to hb_dq[2]
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_dq[2]
set_location_assignment PIN_B6 -to hb_dq[3]
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_dq[3]
set_location_assignment PIN_D3 -to hb_dq[4]
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_dq[4]
set_location_assignment PIN_A4 -to hb_dq[5]
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_dq[5]
set_location_assignment PIN_B3 -to hb_dq[6]
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_dq[6]
set_location_assignment PIN_C6 -to hb_dq[7]
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_dq[7]

set_location_assignment PIN_A6 -to hb_rwds
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_rwds

set_location_assignment PIN_F7 -to hb_rst_n
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_rst_n

# CSn / CLK — User Guide values (see discrepancy note above)
set_location_assignment PIN_D8 -to hb_cs_n
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_cs_n
set_location_assignment PIN_D7 -to hb_ck
set_instance_assignment -name IO_STANDARD "1.2 V" -to hb_ck
