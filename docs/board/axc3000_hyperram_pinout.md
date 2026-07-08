# AXC3000 HyperRAM pinout — from the board schematic (authoritative)

Source: Arrow/Trenz **AXC3000 schematic** `SCH-TEI0131-01-P001.PDF` (Rev A, 2025-05-28),
page 7 (FPGA `A3CY100BM16AE7S`, sheet "FPGA2") + page 13 (HyperRAM `W957D8NBRA4I`, U5).
Download: <https://github.com/ArrowElectronics/Agilex-3/blob/main/images/AXC3000/SCH-TEI0131-01-P001.PDF>

**All HyperRAM I/O is in FPGA `IO Bank 3A`, 1.2 V.** Pin → FPGA-ball → bank pin-function
(from the schematic's Bank 3A pin list):

| Signal | FPGA ball | Bank-3A function | Notes |
|---|---|---|---|
| `hb_dq[0]` | C3 | IO26, DIFF_IO_3A_B11P, DQ13 | |
| `hb_dq[1]` | C2 | IO24, DIFF_IO_3A_B12P, DQ13 | |
| `hb_dq[2]` | B4 | IO31, **PLL_3A_B_CLKOUT0N**, DQ13 | on a PLL clock-output pin |
| `hb_dq[3]` | B6 | IO35, **CLK_B_3A_0N**, DQ13 | on a clock-capable pin |
| `hb_dq[4]` | D3 | IO25, DIFF_IO_3A_B12N, DQ13 | |
| `hb_dq[5]` | A4 | IO30, **PLL_3A_B_CLKOUT0P / FB0**, DQ13 | on a PLL clock-output pin |
| `hb_dq[6]` | B3 | IO27, DIFF_IO_3A_B11N, DQ13 | |
| `hb_dq[7]` | C6 | IO33, DIFF_IO_3A_B8N, DQ13 | |
| `hb_rwds` | A6 | IO28, DIFF_IO_3A_B10P, **DQS13/CQ13** | DQS **positive** of the pair |
| `hb_ck`  | B5 | IO29, DIFF_IO_3A_B10N, **DQSn13/CQn13** | DQS **negative** of the pair |
| `hb_cs_n`| C7 | IO32, DIFF_IO_3A_B8P, DQ13 | (see CS/CK note) |
| `hb_rst_n`| F7 | IO45, DIFF_IO_3A_B2N, DQ12 | |
| `CLK_25M_C` (ref clk) | A7 | IO34, **CLK_B_3A_0P**, DQ13 | clock-capable input (pair of B6) |

## The finding that matters for the fit blocker

`hb_ck` (B5) and `hb_rwds` (A6) are the **DQS13 differential pair** (`DQSn13`/`DQS13`), and
the DQ bits are `DQ13`-group pins in the **same Bank 3A**. In the Agilex I/O architecture these
DQS pins carry the **dedicated DDR strobe/clock resources** for that DQ group.

This explains — and reframes — the PNR failure (`fpga/axc3000`, Quartus Err 24403/24404: the
IOPLL could not route both `clk` (0°) and `clk90` (90°) into the Bank-3A DDIO region). The
hand-rolled PHY routed a **second free IOPLL phase to the I/O** to forward `hb_ck`; the device
expects the HyperBus clock to be driven through the **DQS / hardened DDR-I/O clock resource** on
its DQS pin, not via a general fabric-PLL phase into the I/O periphery. **The pins are correct and
legal (the fit accepted them) — the fix is a PHY *clocking-architecture* change, not a pin move:**
drive `hb_ck` from the DQS/DDR-I/O clock tree (or the hardened EMIF-style DDR I/O), so only one
PLL phase reaches the periphery. This is the documented "hardened DDR-I/O path" remaining-work
item (see `metrics/.../defects` constraint-timing).

## Open item carried over: CS vs CK pin

Two Arrow sources disagreed on 2 of 12 pins (User Guide: `CS#=C7, CK=B5`; refdes
`axc3000_pin_assignment.tcl`: `CS#=D8, CK=D7`). This repo/`pins.tcl` uses the User-Guide values
(C7/B5), which are legal Bank-3A pins and fit-accepted. The schematic page-7 net-to-ball wiring
would settle it definitively; the extracted text confirms C7, B5, D7, D8 are all valid Bank-3A
balls but does not cleanly pair the CS/CK *nets* to balls. Cross-check against the vendor
reference `axc3000_pin_assignment.tcl` before trusting on silicon.
