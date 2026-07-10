# AXC3000 HyperBus bandwidth test — board build

On-hardware bring-up of the HyperBus master IP on the **Arrow AXC3000** (Agilex 3
`A3CY100BM16AE7S`, Winbond **W957D8NBRA4I** HyperRAM 2.1 — the 4 ns/250 MHz speed grade, wired
single-ended-CK on this board). The synthesizable bandwidth engine (`rtl/bench/hyperram_bw_test`)
streams a WRITE then a READ phase over the real HyperRAM and counts the on-chip `clk` cycles of
each phase; a host reads the counters back over JTAG and computes MB/s.

**JTAG is control plane only** — the measured `WR_CYCLES`/`RD_CYCLES` cover the on-chip Avalon
datapath, never the JTAG access time, so the reported MB/s is the true HyperBus throughput.

## Architecture (DDIO/GPIO-cell build, `ddio-200` branch)

The board build inlines the IP's front-end and controller and pairs them with a **board-local I/O
layer** instead of a portable PHY:

```
bw_sys (IOPLL + JTAG-Avalon master)          fpga/axc3000/qsys/make_bw_sys.tcl
  └─ u_bw   hyperram_bw_test                 traffic gen + scoreboard + CSRs (JTAG-addressable)
      └─ u_fe   hyperbus_avalon              Avalon-MM front-end   (rtl/if/)
          └─ u_ctrl  hyperbus_ctrl           frozen controller     (rtl/)
              └─ u_io   hyperbus_gpio_io     BOARD-ONLY I/O layer  (fpga/axc3000/)
```

`hyperbus_gpio_io.sv` owns the pad ring and the silicon-proven bring-up findings:

- **TX**: raw `tennm_ph2_ddio_out` DDR atoms for DQ/RWDS (+ inferred tristate pads), byte-B
  one-`clk` delay (`TX_B_DLY=1` — the device stores `{A(k),B(k+1)}` without it).
- **CK**: `CK_GEN="FABRIC2X"` — an SDR-style fabric generator on the 2× core clock. It is the
  ONLY electrically clean CK source found on this board; every 1× I/O-cell generator (vendor
  `altera_gpio` cell, CKE- or DIN-gated, and the raw atom) produces page-crossing bit errors.
  Its cost: the 2× clock's ~353 MHz minimum-pulse limit caps CK at **~176 MHz**.
- **RX**: QUAD1X — four samples per CK (0/90/180/270°, from `clk` pos/neg + the 2× clock) with
  edge-detect pairing; `RD_PREAMBLE_SKIP=1` discards the device's read-strobe preamble;
  `ARM_DELAY_CYCLES=16` blinds the receiver through the RWDS float window after the latency
  indicator (200 MHz hazard).

## Clock plan

| Clock    | Source          | Freq / phase        | Drives |
|----------|-----------------|---------------------|--------|
| board XO | `CLK_25M_C` (A7)| 25 MHz              | IOPLL refclk |
| `clk`    | IOPLL `outclk0` | **CK_MHZ (175), 0°**| controller, Qsys backbone, ALL I/O launches (word clock = HyperBus CK rate) |
| `clk2x`  | IOPLL `outclk1` | **2×CK (350), 0°**  | CORE-ONLY: FABRIC2X CK generator, QUAD1X sampling grid, debug capture |

`hb_ck` itself is generated in fabric (FABRIC2X) and leaves through an ordinary output pin with a
hard `OUTPUT_DELAY_CHAIN 63` assignment in `bw.qsf` for eye centring. The speed knob is `CK_MHZ`
in `qsys/make_bw_sys.tcl` (currently **175.0**).

## Controller configuration (`top.sv`, silicon-calibrated)

| Parameter | Value | Why |
|-----------|-------|-----|
| `LATENCY_CLOCKS` | 6 | CR0-programmed initial latency |
| `MAX_BURST_WORDS` | 512 | tCSM chop budget (~2.9 µs @175 vs ~15 µs limit) |
| `BURST_BOUNDARY_WORDS` | 0x2000 | device releases the bus when a burst crosses 16 KB |
| `WR_COALESCE` / `WR_COALESCE_WAIT` | 1 / 8 | splice contiguous write commands into one CS# burst (issue #1) |
| `WR_CHOP_REPLAY` / `WR_REPLAY_WORDS` | 1 / 4 | re-send the 4-word tail the device drops at every forced chop (issue #1) |
| `WR_COMMIT_READ` | 0 | silicon-proven ineffective for write→write; replay supersedes |
| `WR_LAT_TRIM` | 3 | measured: device write window opens 3 CK after the spec-anchored point |
| `u_bw CAL_RESET` | 0x2 | REG_CAL POR seed (preamble_skip=1); readback-only on this build — the DDIO runtime cal paths are issue-#3-gated |

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

   **Changing `CK_MHZ` (or any qsys knob) needs a CLEAN regen** — stale caches silently keep the
   old clocks. Remove `qsys/bw_sys`, `qsys/ip/bw_sys`, `qsys/bw_sys.qsys`, `output_files`, `qdb`,
   `dni`, and any `qsys/make_bw_sys.qpf/.qsf` the script emitted, then rerun both commands.

2. **Compile** (Synthesis → Fit → Assembler → Timing; ~15–25 min):

   ```bash
   QPRO quartus_sh --flow compile bw -c bw
   ```

   Bitstream lands in `output_files/bw.sof`. A known-good 175 MHz bitstream is checked in at
   `bitstreams/ddio_gpio_175_pass.sof`.

## Devkit sharing — REQUIRED lock protocol

Multiple agents/sessions may use this board. **Every** board access (programming AND
`system-console`) must hold the shared lock — `flock` auto-releases on process exit, so a crashed
holder never wedges the board:

```bash
flock -w 600 /tmp/axc3000-devkit.lock -c '<your docker quartus_pgm / system-console command>'
```

Compiles and STA need no lock (they don't touch the board). If `flock` times out after 600 s,
another agent is mid-session — retry, don't steal.

## Program the board + run the benchmark

Programming and System Console need a **root `--privileged` container with `/dev/bus/usb`
mounted** (USB Blaster III over usbipd/WSL2) — NOT the user-mode compile container above:

```bash
PGM() { docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb \
  -v /home/tcovert/projects/hyperram:/workspace \
  -w /workspace/fpga/axc3000 alterafpga/quartus-pro:26.1-agilex3 "$@"; }

flock -w 600 /tmp/axc3000-devkit.lock -c 'PGM quartus_pgm -c 1 -m jtag -o "p;output_files/bw.sof"'
flock -w 600 /tmp/axc3000-devkit.lock -c \
  'PGM bash -c "jtagconfig >/dev/null 2>&1; sleep 1; system-console --script=sysconsole/bw_read.tcl 768 0x0 768 175"'
```

**Gotcha:** `system-console` alone finds NO devices in a fresh container; running `jtagconfig`
first in the SAME container primes jtagd. `quartus_pgm` has no such problem.

`bw_read.tcl` args: `<LEN_words> <BASE_hex> <BURST_words> <CK_MHz> [REG_CAL_image]`
(`BURST_WORDS=0` keeps the bitstream default; arg 5 pokes the live read-eye cal CSR, `-1`/absent
keeps the POR seed). It programs the run, polls done, prints WRITE/READ MB/s, `ERR_COUNT`, and
the first-error CSRs (`0x20` addr / `0x24` got / `0x28` expected — the offset decode used for
`WR_LAT_TRIM` calibration). `LED1`=done, `RLED`=error, `GLED`=PLL lock.

## Measured on hardware (W957D8NBRA4I)

**Clean, integrity-verified ceiling — 175 MHz CK, DDR x8** (LEN=768, single coalesced burst,
`ERR_COUNT=0`):

| hb_ck | WRITE MB/s | READ MB/s | notes |
|-------|-----------:|----------:|-------|
| **175 MHz** | **344.7** | **337.3** | FABRIC2X CK; ceiling set by the 2× clock's ~353 MHz min-pulse limit (~176 MHz CK max) |

- **200 MHz**: everything the FPGA controls is proven — fit + static timing (Fmax 210 MHz), CA
  decode, reads pace 383.5 MB/s / writes 389.9 MB/s — but data integrity degrades with a
  **page-crossing artifact** (one bit0 flip per 32-byte device page, ~70–95 errs / 768 words):
  every 1× CK generator shows it, FABRIC2X doesn't, and the board wires CK **single-ended** on a
  1.2 V-class device (Infineon guidance specifies differential CK/CK# for this class). Formally
  roadmapped in issue #12 (250 MHz needs a dedicated differential PLL clock-output pair).
- **Multi-burst writes**: the W957D8NB discards the last 4 words of a write burst when the next
  command is also a write (issue #1). Fixed in the controller: `WR_COALESCE` splices contiguous
  write commands into one CS# burst, and `WR_CHOP_REPLAY` re-sends the 4-word tail at every
  forced chop (tCSM / 16 KB boundary). Residual, inherent to the device: a stream that ends
  write-then-idle-forever leaves its final 4 words uncommitted unless any covering read follows.
- Diagnostic CSRs: burst size (`0x2C` write / `0x30` read), first-error triplet
  (`0x20/0x24/0x28`), live read-eye cal (`0x34`, issue #10).

## Bring-up history (hard-won, condensed)

- **Fitter 24403/24404** ("can't route two IOPLL phases into Bank 3A I/O") — resolved: only ONE
  periphery clock (`clk`); CK eye-centring moved to a hard output delay chain on the pin. Also
  fixed on the way: DDIO_OE placement (err 175001), IOPLL refclk promotion (err 23527).
- **`hb_cs_n=D8`, `hb_ck=D7`** (Arrow `axc3000_pin_assignment.tcl`, schematic-confirmed). The
  User Guide's C7/B5 are WRONG. See `docs/board/axc3000_hyperram_pinout.md`.
- **ph2 atom `areset` is ACTIVE-LOW** (tie `1'b1`); Quartus Pro deletes `(* keep *)` fabric delay
  chains; a pad's raw-input fanout conflicts with in-cell input DDR registers (single P2X term);
  `OUTPUT_DELAY_CHAIN` max 63 on output pins / 15 on bidir; qsys `altera_gpio` needs the
  `gui_io_reg_mode {DDIO}`-style GUI parameter names (`REGISTER_MODE`/`ddr` silently revert).
- **CK-source matrix**: vendor 1× CK cell (CKE- and DIN-gated) and the raw 1× atom all show the
  page-crossing artifact (the raw atom worst, ~490 errs at 175); the fabric FABRIC2X generator is
  clean — the artifact is generator quality + single-ended CK, not frequency.
- The SDR-PHY era (50→175 MHz single-clock bring-up, `hyperbus_phy_sdr`) proved the board, the
  pinout, tCSM chopping, and the write-latency calibration method before the DDIO work; its
  ceiling matched FABRIC2X (~176 MHz) for the same min-pulse reason.
