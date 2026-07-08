# AXC3000 HyperBus bandwidth test — board build

On-hardware bring-up of the HyperBus master IP on the **Arrow AXC3000** (Agilex 3
`A3CY100BM16AE7S`, Winbond **W957D8NB** HyperRAM). The synthesizable bandwidth engine
(`rtl/bench/hyperram_bw_test`) streams a WRITE then a READ phase over the real HyperRAM and counts
the on-chip `clk` cycles of each phase; a host reads the counters back over JTAG and computes MB/s.

**JTAG is control plane only** — the measured `WR_CYCLES`/`RD_CYCLES` cover the on-chip Avalon
datapath, never the JTAG access time, so the reported MB/s is the true HyperBus throughput.

## Clock plan (conservative, by design)

| Clock   | Source            | Freq / phase | Drives |
|---------|-------------------|--------------|--------|
| board XO | `CLK_25M_C` (A7) | 25 MHz       | IOPLL refclk |
| `clk`   | IOPLL `outclk0`   | **50 MHz, 0°**  | whole fabric + PHY TX (word clock) |
| `clk90` | IOPLL `outclk1`   | **50 MHz, +90°** | PHY CK forwarding (write-data centring) |

50 MHz word clock ⇒ DDR `hb_ck` ≈ 50 MHz ⇒ ~100 MB/s/direction theoretical on the x8 bus. Chosen
deliberately **low** so the un-calibrated Agilex read eye is wide and first bring-up does not need a
read-strobe tap sweep. Push higher later (and calibrate `RX_STROBE_DLY_TAPS`/`RX_PAIR_SKEW` in
`rtl/phy/hyperbus_phy_altera.sv`) once basic reads pass.

## What's here

| File | Role |
|------|------|
| `qsys/make_bw_sys.tcl` | qsys-script that builds `qsys/bw_sys.qsys` (IOPLL + reset + JTAG-to-Avalon master) |
| `qsys/bw_sys.qsys` | generated Platform Designer system (checked in; regenerate with the flow below) |
| `top.sv` | board top: `bw_sys` (clocks/reset/JTAG master) → `hyperram_bw_top` (bw engine + Agilex PHY) → `hyperbus_pads_altera` (I/O pads) |
| `hyperbus_pads_altera.sv` | `tennm_ph2` I/O buffer ring turning the split PHY pins into real bidir pads |
| `pins.tcl` | pin + I/O-standard assignments (copied from the board constraints file) |
| `bw.qsf` / `bw.qpf` | Quartus project (`FAMILY "Agilex 3"`, `DEVICE A3CY100BM16AE7S`, `TOP top`) |
| `bw.sdc` | timing constraints (25 MHz create_clock, derive_pll_clocks, false_path on the button/LEDs) |
| `sysconsole/bw_read.tcl` | System Console script: program the run, poll done, print WRITE/READ MB/s |

Top-level ports (board signal names, matching `pins.tcl`): `CLK_25M_C`, `USER_BTN` (active-low
reset), `hb_dq[7:0]`, `hb_rwds`, `hb_cs_n`, `hb_ck`, `hb_rst_n`, and LEDs `LED1` (STATUS.done),
`RLED` (STATUS.error), `GLED` (PLL locked) — all active-low. The AXC3000 HyperRAM is single-ended,
so there is no `hb_ck_n` board pin (`DIFF_CK=0`).

## Build

Everything runs headless in the Quartus-Pro 26.1 Docker image. Define a helper once:

```bash
QPRO() { docker run --rm -i --user $(id -u):$(id -g) -e HOME=/tmp \
  -v /home/tcovert/projects/hyperram:/workspace \
  -v /dev/null:/usr/lib/x86_64-linux-gnu/libudev.so.1 \
  -w /workspace/fpga/axc3000 alterafpga/quartus-pro:26.1-agilex3 "$@"; }
```

1. **Build + generate the Qsys system** (produces `qsys/bw_sys/` HDL and the per-instance IP under
   `qsys/ip/bw_sys/`). The `--quartus-project` flag is REQUIRED — without it the sub-IP (IOPLL,
   JTAG master, reset controller) are emitted as empty black boxes and synthesis fails:

   ```bash
   QPRO qsys-script --script=qsys/make_bw_sys.tcl
   QPRO qsys-generate qsys/bw_sys.qsys --synthesis=VERILOG --quartus-project=bw --rev=bw
   ```

2. **Compile** (Analysis & Synthesis → Fit → Assembler → Timing):

   ```bash
   QPRO quartus_sh --flow compile bw -c bw
   ```

   Bitstream lands in `output_files/bw.sof`. Run the tensor-mode / timing audit as usual.

## Program the board

Program over the on-board USB-Blaster III (see the project memory note on the AXC3000 JTAG path —
program via a root + `--privileged` + `/dev/bus/usb` container, NOT the compile container):

```bash
QPRO quartus_pgm -c 1 -m jtag -o "p;output_files/bw.sof"
```

`GLED` lights when the IOPLL locks. Press `USER_BTN` (S2) to reset the fabric.

## Run the bandwidth test

```bash
QPRO system-console --script=sysconsole/bw_read.tcl 4096 0x0
#   args: <LEN_words> <BASE_word_addr_hex>   (defaults: 4096 words, base 0x0)
```

It opens the JTAG-Avalon master, checks the `"HBWT"` magic, programs `LEN`/`BASE_ADDR`, pulses
`CTRL.start`, polls `STATUS.done`, then prints WRITE/READ MB/s and a PASS/FAIL integrity result.
`LED1` (done) and `RLED` (error) mirror `STATUS` each time the host polls it.

## Build status

- **Qsys generation**: clean (IOPLL implements 25→50/50@90° with user settings; JTAG master, reset
  controller, clock bridges all real IP).
- **Analysis & Synthesis**: **clean, 0 errors** — the whole hierarchy elaborates: `bw_sys`, the
  bandwidth engine, `hyperram_avalon` + `hyperbus_ctrl` + the Agilex `hyperbus_phy_altera`
  (`tennm_ph2` DDIO primitives), and the inferred I/O pad ring. 5 in / 7 out / 9 bidir pins.
- **Simulation** (`sim/run.sh`, generic PHY): `tb_bw` **PASS**, 0 data errors — unaffected by the
  board work.
- **Fitter**: reaches final periphery placement, then **blocks at one clock-routing constraint**
  (Fitter error 24403/24404): the IOPLL cannot route BOTH `clk` (0°, to the DQ/RWDS output DDIOs)
  AND `clk90` (90°, to the CK-forwarding DDIO) into the single HyperRAM I/O sub-bank. Two fitter
  blockers found on the way were fixed and are checked in:
    1. **DDIO_OE placement** (error 175001): the PHY drove one shared `tennm_ph2_ddio_oe` (a hard,
       1-pin I/O-cell register) onto all 8 DQ pads. Changed `hyperbus_phy_altera.sv` to register the
       OE in ordinary (replicable) flops so the fitter copies the enable into each DQ I/O cell —
       same 1-clk latency, and the pad ring is inferred tri-state (`hyperbus_pads_altera.sv`).
    2. **IOPLL refclk routing** (error 23527): `CLK_25M_C` (PIN_A7) is not a dedicated PLL refclk
       pin, so the reference could not reach the IOPLL. Promoted it to a global clock in `pins.tcl`.

  The remaining 24403 is architectural: forwarding a 90°-shifted CK from a **fabric-shared** IOPLL
  output into the same I/O sub-bank as the DQ pins is not routable here (both `clk` and `clk90` must
  reach DDIOs in one sub-bank; the shared clock spine can carry only one). The AXC3000 reference
  designs avoid this by clocking high-speed I/O from **hardened** controllers (the MIPI XSPI MC),
  not a hand-rolled two-phase DDIO PHY — corroborating the limitation. See Hardware handoff below.

## Hardware handoff / notes

- **Fitter 24403 (two-phase I/O clock) — resolution options**, in order of preference:
  1. **Move CK to its own clock region** by resolving the **disputed `hb_ck` pin** (User Guide B5
     vs refdes D7 — see `pins.tcl`) against the schematic and, if needed, floorplanning the CK pad
     into a different I/O sub-bank than the DQ pads, so `clk` (→DQ) and `clk90` (→CK) no longer share
     a sub-bank clock spine. Needs the schematic + package I/O-bank map.
  2. **Use the hardened EMIF / DDR I/O path** (as the vendor MIPI reference does) instead of the
     hand-rolled two-phase DDIO PHY — the supported way to clock high-speed I/O on this device.
  Both need the physical board and/or the schematic and are beyond this RTL-integration task.
  (Note: centring CK in the DQ eye fundamentally requires two clock phases at the I/O, so the only
  way to keep it in one sub-bank is to physically separate the CK and DQ pads — option 1.)
- **HyperRAM `hb_cs_n`/`hb_ck` pins are unverified** — the board User Guide and the refdes repo
  disagree (UG: C7/B5; refdes: D8/D7). `pins.tcl` uses the User Guide values (see the note in that
  file and `docs/board_bringup.md`). Confirm against the schematic before trusting bring-up.

- **HyperRAM `hb_cs_n`/`hb_ck` pins are unverified** — the board User Guide and the refdes repo
  disagree (UG: C7/B5; refdes: D8/D7). `pins.tcl` uses the User Guide values (see the note in that
  file and `docs/board_bringup.md`). Confirm against the schematic before trusting bring-up.
- **Read-eye calibration**: if reads mismatch (`ERR_COUNT>0`) but writes complete, sweep
  `RX_STROBE_DLY_TAPS` / `RX_PAIR_SKEW` in `hyperbus_phy_altera.sv` (source-synchronous read
  capture — see that file's header). The conservative 50 MHz clock is meant to make the first pass
  work without this.
- Everything above the physical program/run steps is reproducible from a clean checkout by the two
  build commands; the program + `system-console` steps require the physical board + USB-Blaster III.
