# PHY Porting Guide

The controller (`hyperbus_ctrl`) is PHY-agnostic: it speaks a small **DDR-parallel,
single-clock** interface and never instantiates an I/O cell. Everything
technology-specific — DDR serialization, CK generation, input capture, and the one
RWDS→`clk` clock-domain crossing — lives behind the frozen `hyperbus_phy` port list.
This guide explains how to swap the **generic** (simulation) PHY for a
vendor-primitive variant for a real board.

- Frozen PHY port list: [`INTERFACES.md §hyperbus_phy`](INTERFACES.md).
- Reference behavior to preserve: `rtl/phy/hyperbus_phy_generic.sv`.
- Clocking/CDC rationale: [`DESIGN.md §2 / §4`](DESIGN.md).

> **Status.** `hyperbus_phy_generic` is complete and simulation-validated.
> `hyperbus_phy_xilinx` is a **real 7-series datapath** (`ODDR`/`IDDR`/`IDELAYE2`/
> `IDELAYCTRL`/`BUFIO`/`BUFR`/`OBUF`/`OBUFDS`) that **simulates via a Verilator-only
> primitive shim** (`sim/model/xilinx_prims_sim.sv`, `tb_xilinx`) — but is **still
> not hardware-proven or timing-closed**: the read-eye taps (`RX_STROBE_DLY_TAPS`),
> byte-pairing polarity (`RX_PAIR_SKEW`) and preamble skip (`RD_PREAMBLE_SKIP`) are
> bring-up knobs to sweep on real silicon. `hyperbus_phy_altera` wraps Quartus/Agilex
> hard-IP primitives and is **not** Verilator-simulable (validated by the fitter +
> hardware). For either, calibrating the RWDS strobe and closing static timing
> against a device datasheet is per-target hardware work.

---

## 1. How variant selection works

`hyperbus_phy` is a pure structural selector. It instantiates exactly one variant
inside a `generate` guarded by the string parameter `PHY_VARIANT`:

| `PHY_VARIANT` | Variant instantiated | Simulates | Purpose |
|---|---|---|---|
| `"GENERIC"` (default) | `hyperbus_phy_generic` | yes | simulation + inference on any FPGA |
| `"INTEL"` / `"ALTERA"` | `hyperbus_phy_altera` | no (Quartus-only prims) | Intel/Altera board build |
| `"XILINX"` | `hyperbus_phy_xilinx` | yes (via primitive shim) | AMD/Xilinx 7-series board build |

All three share one port list, so switching is a single parameter change on the top
(`hyperram_axi` / `hyperram_avalon`). You do **not** touch the controller or
front-ends.

To add a new vendor (e.g. Lattice): copy a skeleton to
`rtl/phy/hyperbus_phy_lattice.sv`, add an `else if (PHY_VARIANT == "LATTICE")` branch
to `hyperbus_phy.sv` mirroring an existing branch verbatim (same `.port(port)`
wiring), and fill in the primitives below.

---

## 2. The contract every variant must honor

Ports (see `INTERFACES.md` for widths; `PHYW = 2*DQ_WIDTH`):

**Transmit (ctrl → PHY), all in the `clk` domain:**

| Signal | Meaning the PHY must realize |
|---|---|
| `phy_cs_n`, `phy_rst_n` | register into the pin domain; keep aligned with DQ/CK |
| `phy_ck_en` | run `hb_ck` this cycle; CK idles **Low** when Low |
| `phy_dq_o[PHYW-1:0]` | DDR out word: **`[PHYW-1:DQ_WIDTH]` = byte A (1st / CK-rising edge)**, `[DQ_WIDTH-1:0]` = byte B (2nd / CK-falling edge) |
| `phy_dq_oe` | DQ output enable (tri-state control) |
| `phy_rwds_o[1:0]` | DDR RWDS out (write byte-mask): `[1]` = 1st phase, `[0]` = 2nd |
| `phy_rwds_oe` | RWDS output enable |
| `phy_rd_arm` | pulses when a read-data phase begins: enable RWDS-clocked capture and reset the elastic buffer |

**Receive (PHY → ctrl), delivered into the `clk` domain:**

| Signal | Meaning the PHY must produce |
|---|---|
| `phy_dq_i[PHYW-1:0]` | one recovered read **word** (byte A in the high half), already deskewed |
| `phy_dq_i_valid` | one pulse per recovered word |
| `phy_rwds_i` | the RWDS **level**, 2-flop synchronized into `clk` (CA latency-select + stall watch) |

**Invariants (do not break these):**

1. **Center-aligned CK.** `hb_ck` edges must land in the center of each DQ byte eye
   for writes/CA (that is why `clk90` exists). CK must idle **Low** between pulses,
   gated by `phy_ck_en`.
2. **Byte A = high half.** `phy_dq_o[PHYW-1:DQ_WIDTH]` is the first edge, byte B the
   second. Registers are big-endian (SPEC_DIGEST §4). Keep this on both TX and RX.
3. **RWDS-sourced read capture.** Read DQ is source-synchronous to the *device's*
   RWDS strobe, not to `hb_ck`. Capture DQ on RWDS edges, then cross into `clk`. A
   fixed local-clock phase will not track the DQ/RWDS round-trip flight delay.
4. **CDC is the PHY's job.** The RWDS→`clk` crossing is the single true CDC in the
   system and must stay inside the PHY (async FIFO / vendor input FIFO). No other
   module hand-rolls a synchronizer.
5. **Split pins, no `inout`.** Drive `hb_dq_o`/`hb_dq_oe`/`hb_dq_i` (and RWDS)
   separately; the board wrapper owns the tristate (see `INTEGRATION.md §4`), unless
   the vendor DDR-IO cell integrates it.

Verify a filled-in variant by re-pointing the existing testbenches at it (build with
`PHY_VARIANT` set) in a vendor simulator with the primitive libraries; the generic
PHY remains the golden reference for what "correct" looks like functionally.

---

## 3. Mapping the generic behavior to primitives

The generic PHY does five things. Here is the primitive that replaces each, per
vendor. (Sites are marked `TODO(altera)` / `TODO(xilinx)` in the skeletons.)

| Generic behavior | AMD/Xilinx | Intel/Altera | Lattice |
|---|---|---|---|
| DDR **DQ/RWDS out** (byte A→1st, byte B→2nd sub-phase), tri-stated by `_oe` | `ODDR` (`DDR_CLK_EDGE="SAME_EDGE"`) + `OBUFT`/`IOBUF` | `altera_gpio` / DDIO_OUT (or `ALTDDIO_OUT`) with tri-state | `ODDRX1F` + `BB` |
| **CK generation** on `clk90` fed `{phy_ck_en, 1'b0}`, idle Low; diff pair if `DIFF_CK` | `ODDR` on `clk90` + `OBUFDS` | DDIO_OUT on `clk90` + pseudo-diff pair | `ODDRX1F` on `clk90` |
| DDR **DQ in** capture, RWDS-clocked | `IDDR` (`DDR_CLK_EDGE="SAME_EDGE_PIPELINED"`) clocked by delayed RWDS | DDIO_IN clocked by delayed RWDS | `IDDRX1F` |
| **RWDS strobe eye-centering** (~90° / quarter-bit shift; `RX_STROBE_DELAY` in sim) | `IDELAYE2/E3` (calibrated) or MMCM phase | DPA / PLL phase or `altera_gpio` delay chain; `clk_ref` drives calibration | `DELAYG` / PLL phase |
| **RWDS→clk elastic** hand-off (gray FIFO in generic) | `xpm_fifo_async` or LUTRAM async FIFO | DCFIFO (async) | `pmi_fifo_dc` |
| **RWDS level** 2-flop synchronizer (variant-independent) | 2 FF | 2 FF | 2 FF |

`clk_ref` is unused by the generic PHY (tie-off). In the vendor variants it is the
delay/SERDES **calibration reference** (e.g. the 200 MHz `IDELAYCTRL` ref clock on
Xilinx); route a real clock to it there.

---

## 4. Step-by-step for a vendor variant

1. **Start from the skeleton** (`hyperbus_phy_altera.sv` or `_xilinx.sv`). It already
   registers `hb_cs_n`/`hb_rst_n`/`hb_dq_oe`/`hb_rwds_oe` on `clk` and synchronizes
   the RWDS level — keep those.
2. **TX DQ/RWDS.** Replace the placeholder `assign hb_dq_o = phy_dq_o[hi]` with a
   per-bit DDR-out primitive fed `{phy_dq_o[PHYW-1:DQ_WIDTH][i], phy_dq_o[DQ_WIDTH-1:0][i]}`
   (byte A first). Same for RWDS with `{phy_rwds_o[1], phy_rwds_o[0]}`. Drive the
   tri-state from `phy_dq_oe` / `phy_rwds_oe`.
3. **CK.** Instantiate a DDR-out on `clk90` fed `{phy_ck_en, 1'b0}` so CK toggles
   only when armed and idles Low; add the complementary output when `DIFF_CK`.
4. **RX capture.** Delay RWDS by ~90° (calibrated primitive), use it as the capture
   clock for a DDR-in on DQ, and pack the pair into a `PHYW` word (byte A high half).
   Gate/reset capture with `phy_rd_arm`.
5. **CDC.** Push each recovered word through an async FIFO from the RWDS domain to
   `clk`; emit `phy_dq_i` + a one-cycle `phy_dq_i_valid` per word on the `clk` side.
6. **Constrain.** Add the input-delay/DQS-group and CDC exceptions from
   [`INTEGRATION.md §5`](INTEGRATION.md). Sweep the RWDS delay tap to center the read
   eye on real silicon.
7. **Validate.** Re-run the `tb_*` suite against the DUT built with your
   `PHY_VARIANT` in a simulator that has the vendor primitive libraries, comparing to
   the generic PHY's known-good behavior. Then bring up on hardware at a low CK rate,
   reading ID0/ID1 first.

Keep the port list byte-for-byte identical to the contract — the whole point of this
structure is that the controller, front-ends, and testbenches never change when the
PHY does.
