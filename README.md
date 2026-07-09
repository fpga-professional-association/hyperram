# HyperBus / HyperRAM Controller IP

A clean-room, technology-agnostic **HyperBus master controller** for HyperRAM /
HyperFlash-class devices, with both **AXI4** and **Avalon-MM** front-ends and a
swappable DDR PHY. The protocol engine contains **no vendor primitives** and
simulates end-to-end under `verilator --binary` (tested with Verilator 5.020).

> **Normative source:** Infineon / Cypress **HyperBus Specification, doc
> 001-99253 Rev. \*H** (Feb 6 2019). An implementation-oriented digest lives in
> [`docs/SPEC_DIGEST.md`](docs/SPEC_DIGEST.md); every controller decision cites a
> section there.

---

## ⚡ Performance & test status (at a glance)

**Tested on real silicon:** Arrow **AXC3000** dev board — Intel **Agilex 3 `A3CY100BM16AE7S`** FPGA
+ Winbond **W957D8NB** HyperRAM (128 Mb, ×8, 1.2 V), Quartus Prime Pro 26.1, **SDR PHY**.

**Measured HyperRAM bandwidth** (single burst, data-integrity-verified on every row, `ERR_COUNT=0`):

| HyperBus CK | Byte clock | Write | Read |
|------------:|-----------:|------:|-----:|
| 50 MHz  | 100 MHz | 96.8  | 94.8  MB/s |
| 100 MHz | 200 MHz | 193.6 | 189.3 |
| 150 MHz | 300 MHz | 290.4 | 283.9 |
| **175 MHz** | **350 MHz** | **342.4** | **337.3 MB/s** |

- **Peak ~342 / ~337 MB/s at 175 MHz CK — 3.5× the bring-up baseline.** Change the clock with the
  single `CK_MHZ` knob in [`fpga/axc3000/qsys/make_bw_sys.tcl`](fpga/axc3000/qsys/make_bw_sys.tcl).
- **175 MHz is the SDR-PHY ceiling** (its 2×-byte clock hits a min-pulse-width limit). The device's
  **200 MHz / 400 MB/s** maximum needs the DDIO PHY — tracked in
  [issue #3](https://github.com/fpga-professional-association/hyperram/issues/3).
- **Single bursts commit clean up to ~768 words** (≥1024 hits the device refresh window, tCSM ≈15 µs).
  *Split* multi-burst **writes** (LEN > burst size) drop the last word of each non-final burst — a
  W957D8NB write-commit quirk ([issue #1](https://github.com/fpga-professional-association/hyperram/issues/1));
  workaround: drive writes as single bursts ≤512 words.
- **Vendor-neutral core:** the controller + AXI4/Avalon front-ends contain no vendor primitives and
  simulate end-to-end under Verilator; only the PHY is device-specific. See
  **[Porting to your device](#porting-to-your-device)**.

---

## Burst-length limits & the write-commit workaround

HyperBus transfers a **linear burst** under one CS# assertion, auto-incrementing the address. Two limits
shape how you should drive writes on this device:

- **tCSM — CS# maximum-Low time (spec §6 / §9).** A self-refreshing HyperRAM can only refresh while CS# is
  High, so the spec **requires** that a single CS# assertion not exceed **tCSM**; longer transfers must be
  split into multiple bursts. This IP does that automatically via `MAX_BURST_WORDS` (= tCSM / tCK) so the
  chop is transparent to the caller. Empirically the W957D8NB holds a clean single burst up to ~768 words
  (≈15 µs) at these clocks; a ≥1024-word single burst overruns tCSM and corrupts.

- **Split-write commit quirk (device — [issue #1](https://github.com/fpga-professional-association/hyperram/issues/1)).**
  When a *write* transfer is split across CS# boundaries — whether the caller issues several bursts or the
  tCSM chopper does — the W957D8NB fails to commit the **last word of every non-final write burst** (it
  reads back `0x0000`). It is deterministic, is *not* reproduced in an ideal-clock model, and the device
  commits that word only if the **next command is a read**. Multi-burst *reads* are unaffected.

**Is the single-burst workaround spec-compliant? Yes.** Issuing a write as a **single linear burst within
one CS# assertion (≤ tCSM, ≤512 words with margin)** is not a workaround *around* the spec — it is exactly
a normal HyperBus write transaction as the spec defines it. The CA, initial latency, DDR data phase, and
RWDS byte-masking are all unchanged; you simply keep the transfer inside one CS# assertion, which the spec
already permits up to tCSM. What you cannot yet *rely* on is back-to-back write bursts to a contiguous
region committing every boundary word — and that is a **device** behavior to design around, not a spec
feature this IP omits. Transfers that must exceed tCSM (and therefore *must* cross a CS# boundary, per
spec) are where the quirk bites; [issue #1](https://github.com/fpga-professional-association/hyperram/issues/1)
tracks the real fix (the device commits a pending write on a subsequent read).

---

## Features

- **Two host interfaces, one engine.** `hyperram_axi` (AXI4 slave) and
  `hyperram_avalon` (Avalon-MM slave) are *thin* front-ends over a single native
  command/write/read valid-ready interface (`hyperbus_ctrl`). No protocol logic
  is duplicated between them.
- **Full 48-bit Command-Address** encode/decode (R/W#, address-space,
  burst-type, word address) per spec §3, built in `hyperbus_pkg::hb_pack_ca()`.
- **Fixed and variable initial latency.** The controller always decodes the
  slave-driven RWDS level during CA and doubles the latency count when the device
  requests 2× — even in fixed-latency mode (spec §3.2 / §5.2.4). Verified by
  `tb_fixed2x`.
- **RWDS-gated read completion.** Read words are counted on the source-synchronous
  RWDS strobe, not a free-running clock, so mid-burst row/page latency gaps are
  absorbed transparently (spec §3.2 / §7).
- **Read-stall timeout + abort.** RWDS held Low ≥ 32 clocks raises `err_timeout`,
  terminates the native read cleanly (`rd_last`), and maps to AXI **SLVERR** — no
  deadlock. Verified by `tb_timeout`.
- **Byte-masked writes** (RWDS = inverted per-byte strobe) and **zero-latency
  register/config writes** (no RWDS mask, full-word only) per spec §5 / §6.
- **Linear and wrapped bursts.** `CA[45] = ~wrap`; linear bursts are chopped to
  `MAX_BURST_WORDS` (= tCSM / tCK) so CS# never exceeds the device's maximum
  Low-time, transparently to the caller (spec §6 / §7).
- **POR init + CR programming.** Reset pulse, configurable power-up delay, then an
  optional CR0 write built from parameters; `init_done` gates user traffic.
- **Register / ID access is first-class**, routed through the same native
  interface (CR0/CR1/ID0/ID1), not a side channel.
- **Swappable PHY** behind one frozen port list: a **generic inferrable-DDR**
  variant (simulation + any FPGA), an **AMD/Xilinx** 7-series DDR variant that
  simulates via a Verilator-only primitive shim (still not hardware-proven), and an
  **Intel/Altera** DDIO variant for board bring-up (see *Status & scope*).
- **Hyperflex-friendly RTL:** single clock domain in the controller/front-ends,
  synchronous active-high reset for architectural state, no async reset and no
  datapath clock gating. The one true CDC (RWDS → `clk`) is isolated inside the PHY.

---

## Block diagram

```
                 ┌───────────────────────────── hyperram_axi / hyperram_avalon (TOP) ─────────────────────────────┐
                 │                                                                                                 │
  AXI4  ────────►│  ┌───────────────┐   native cmd / wr / rd   ┌──────────────┐   DDR-parallel PHY IF  ┌────────┐  │
   or            │  │ hyperbus_axi  │ ───────────────────────► │              │ ─────────────────────► │        │  │   hb_ck / hb_ck_n
  Avalon-MM ────►│  │      or       │                          │ hyperbus_ctrl│                        │hyperbus│  │──► hb_cs_n / hb_rst_n
  slave          │  │hyperbus_avalon│ ◄─────────────────────── │  (protocol   │ ◄───────────────────── │ _phy   │  │◄─► hb_dq[7:0]
                 │  └───────────────┘       read data          │   engine)    │   recovered read data  │ (SERDES│  │◄─► hb_rwds
                 │   front-end / bus adapter                   └──────────────┘   + RWDS→clk CDC        │  + IO) │  │
                 │       (thin, no protocol)                     no vendor prims  ──────────────────►   └────────┘  │
                 │                                                                    init_done                     │  GENERIC | INTEL | XILINX
                 └─────────────────────────────────────────────────────────────────────────────────────────────────┘
                       clk / clk90 / clk_ref / rst  (one PLL, phase-related; ctrl uses only clk)

  Simulation only:  the same hb_* device pins connect to sim/model/hyperram_model.sv (golden HyperRAM model),
                    with DQ/RWDS bus resolution done in the testbench (split-driver, Verilator-safe, no inout).
```

Module roles (frozen boundaries in [`docs/INTERFACES.md`](docs/INTERFACES.md);
architecture in [`docs/DESIGN.md`](docs/DESIGN.md)):

| Module | Role | Vendor prims | Verilator |
|---|---|---|---|
| `hyperbus_pkg` | params, typedefs, CA pack/unpack, latency & wrap tables | no | yes |
| `hyperbus_ctrl` | protocol engine (CA, latency, RWDS-gated read, write mask, burst chop, POR init) | **no** | yes |
| `hyperbus_phy` | PHY wrapper; selects one variant by `PHY_VARIANT` | — | — |
| `hyperbus_phy_generic` | inferrable DDR I/O + RWDS→clk CDC | **no** | **yes** |
| `hyperbus_phy_altera` | Intel/Altera DDR-IO variant | yes | skeleton |
| `hyperbus_phy_xilinx` | AMD/Xilinx 7-series ODDR/IDDR variant | yes | yes (via primitive shim) |
| `hyperbus_avalon` | Avalon-MM slave → native (thin) | no | yes |
| `hyperbus_axi` | AXI4 slave → native (thin) | no | yes |
| `hyperram_axi` / `hyperram_avalon` | tops = front-end + ctrl + phy | per PHY variant | yes (generic) |
| `hyperram_model` | behavioral device model (sim only) | no | yes |

---

## Host interfaces

Full signal tables are in [`docs/INTERFACES.md`](docs/INTERFACES.md); a summary follows.

### AXI4 slave (`hyperram_axi`)

Standard AXI4 (AW / W / B / AR / R). Data beats map 1:1 to 16-bit HyperBus words
when `AXI_DATA_WIDTH == 16`.

| Channel | Ports | Notes |
|---|---|---|
| Address write | `awid, awaddr, awlen, awsize, awburst, awvalid, awready` | `awaddr` is a byte address; MSB selects register space |
| Write data | `wdata, wstrb, wlast, wvalid, wready` | `wstrb` → HyperBus byte mask |
| Write resp | `bid, bresp, bvalid, bready` | `bresp = SLVERR` on controller error |
| Address read | `arid, araddr, arlen, arsize, arburst, arvalid, arready` | INCR / WRAP / FIXED decomposed into linear native segments |
| Read data | `rid, rdata, rresp, rlast, rvalid, rready` | `rresp = SLVERR` on timeout |

Burst handling: **INCR** → one segment; **WRAP** → two segments reproducing AXI
order for any boundary (WRAP2/4…); **FIXED** → N single-word segments. A narrow
`AxSIZE` is accepted but flagged SLVERR. Ready/valid outputs are held Low in reset.

### Avalon-MM slave (`hyperram_avalon`)

| Port | Dir | Notes |
|---|---|---|
| `avs_address` | I | word address; MSB selects register space |
| `avs_read` / `avs_write` | I | request |
| `avs_writedata` / `avs_byteenable` | I | write word + byte enables |
| `avs_burstcount` | I | words in burst (linear; `cmd_wrap` tied 0) |
| `avs_readdata` / `avs_readdatavalid` | O | read return |
| `avs_waitrequest` | O | back-pressure |

### HyperBus device pins (both tops, from `hyperbus_phy`)

`hb_ck, hb_ck_n, hb_cs_n, hb_rst_n, hb_dq_o[DQ_WIDTH-1:0], hb_dq_oe,
hb_dq_i[DQ_WIDTH-1:0], hb_rwds_o, hb_rwds_oe, hb_rwds_i`, plus status `init_done`.
Pins are **split** (separate `_o`/`_oe`/`_i`); the board wrapper adds the tristate
buffers (`IOBUF`) — see [`docs/INTEGRATION.md`](docs/INTEGRATION.md). Clocking:
`clk` (bus word rate), `clk90` (CK centering), `clk_ref` (vendor PHY delay ref;
tie for generic), `rst` (synchronous active-high).

---

## Parameters

Common to every module (defaults from `hyperbus_pkg`):

| Parameter | Default | Meaning |
|---|---|---|
| `DQ_WIDTH` | 8 | HyperBus DQ pins |
| `DATA_WIDTH` | 16 | native word = `2*DQ_WIDTH` (one HyperBus word) |
| `ADDR_WIDTH` | 32 | word-address width |
| `LEN_WIDTH` | 16 | burst-length counter width (words) |

Controller (`hyperbus_ctrl`, and forwarded through the tops):

| Parameter | Default | Meaning |
|---|---|---|
| `LATENCY_CLOCKS` | 6 | initial latency, CA1→data, in clocks (spec Table 5.3) |
| `FIXED_LATENCY` | 1 | 1 = fixed initial latency (POR default) |
| `MAX_BURST_WORDS` | 0 | 0 = no chop; else tCSM/tCK — linear bursts chopped to this |
| `PROGRAM_CR` | 1 | write CR0 during POR init |
| `POR_DELAY_CYCLES` | 0 | power-up delay in clocks (set to ~150 µs worth on hardware) |
| `INIT_LATENCY_CODE` | derived | CR0[7:4] code written at init |
| `INIT_CR0` | `0x0008` | CR0 image written at init |

AXI front-end adds `ID_WIDTH` (4), `AXI_DATA_WIDTH` (16), `AXI_ADDR_WIDTH`
(`ADDR_WIDTH+1`). PHY adds `PHY_VARIANT` (`"GENERIC"` | `"INTEL"` | `"XILINX"`)
and `DIFF_CK` (1 = drive `hb_ck_n`).

---

## Quick start

### Prerequisites

- **Verilator ≥ 5.020** (`--binary` + `--timing`), a C++17 toolchain, and `bash`.
  On Debian/Ubuntu: `sudo apt-get install -y verilator build-essential`.

### Run the simulation

```bash
git clone <this-repo> hyperram
cd hyperram
bash sim/run.sh
```

`sim/run.sh` builds and runs each self-checking testbench with
`verilator --binary --timing -Wall` and exits non-zero on any build,
elaboration, or simulation failure. Expected tail:

```
== Running tb_avalon    TB_RESULT: PASS
== Running tb_axi       TB_RESULT: PASS
== Running tb_fixed2x   TB_RESULT: PASS
== Running tb_timeout   TB_RESULT: PASS
ALL TESTBENCHES PASSED
```

| Testbench | What it exercises |
|---|---|
| `tb_avalon` | Avalon-MM POR init, single + burst write/read-back, CR/ID access |
| `tb_axi` | AXI4 INCR single + burst, WRAP burst read, CR0 write/read-back, ID0 read, B/R = OKAY |
| `tb_fixed2x` | fixed-latency device that drives RWDS High during CA → controller must use 2× latency for reads *and* writes |
| `tb_timeout` | mid-burst RWDS stall (40 clocks) → `err_timeout`, clean `rd_last`, SLVERR, no deadlock |

Each testbench wires the DUT top to the golden `hyperram_model` and checks
byte-exact read-back; any mismatch raises `$fatal` (non-zero exit).

### Instantiate

See [`docs/INTEGRATION.md`](docs/INTEGRATION.md) for a complete board wrapper
(tristate IOBUFs, PLL clock plan, `.sdc` notes) and
[`docs/PHY_PORTING.md`](docs/PHY_PORTING.md) for filling in the vendor PHY
primitives.

---

## Porting to your device

The controller (`hyperbus_ctrl`), the AXI4/Avalon front-ends, and the generic PHY are
**device-independent** and simulate on any toolchain. Bringing this IP up on a new FPGA + HyperRAM is
almost entirely a **PHY + board** job:

1. **Pick a PHY** (`hyperbus_phy` selects by `PHY_VARIANT`):
   - `"GENERIC"` — inferrable DDR; simulation + any FPGA (not I/O-timing-tuned).
   - `"SDR"` (`hyperbus_phy_sdr`) — **portable, ONE clock in the I/O periphery**: runs the byte engine at
     2×CK and derives CK-centring from that clock's negedge. This is the variant proven to **~342 MB/s**
     on the AXC3000. Best first target on any device — no vendor DDR primitives, no per-bit calibration.
   - `"XILINX"` (`hyperbus_phy_xilinx`) — real 7-series ODDR/IDDR/IDELAYE2 DDR-I/O (I/O at 1×CK);
     **simulates via a Verilator-only primitive shim, still not hardware-proven or timing-closed**.
   - `"INTEL"` (`hyperbus_phy_altera`) — hard DDR-I/O for the highest speed (I/O at 1×CK); vendor
     DDIO/IDELAY primitives (see `docs/PHY_PORTING.md`).
2. **Clock plan** (one PLL, phase-related):
   - *SDR PHY* — `clk` = HyperBus CK word rate; `clk90` is **repurposed** as the 2×CK byte clock (0°). Only
     one clock reaches the I/O. Scale both together (the `CK_MHZ` knob) to trade clock for bandwidth.
   - *DDIO PHY* — `clk` (0°) + `clk90` (90°, CK-centring); **both** phases enter the I/O, so the device/bank
     must route two periphery phases (the AXC3000 could not → the SDR PHY is the workaround).
3. **Board wrapper.** The PHY exposes **split** `hb_*_o` / `_oe` / `_i` (no `inout`, stays Verilator-shaped);
   add the tri-state pad ring in a board wrapper (`fpga/axc3000/hyperbus_pads_altera.sv`,
   [`docs/INTEGRATION.md`](docs/INTEGRATION.md)).
4. **Pins + I/O standard** for your HyperRAM (`hb_dq[7:0]`, `hb_rwds`, `hb_cs_n`, `hb_ck`, `hb_rst_n`;
   `hb_ck_n` only if `DIFF_CK=1`). Get `hb_ck` / `hb_cs_n` right — wrong pins = the device never responds.
5. **Timing (`.sdc`).** For bring-up, `false_path` the off-chip HyperBus pins and close on the internal
   fabric; production high-speed needs real source-synchronous input/output-delay closure.
6. **Device specifics from your datasheet:** `LATENCY_CLOCKS`, `FIXED_LATENCY`, `INIT_CR0` (CR0
   latency/drive/burst fields), and `MAX_BURST_WORDS` (= tCSM / tCK, so CS# never exceeds the device's max
   Low time). Register/ID addresses default to W957D8NB-family values.
7. **Read-eye calibration on hardware.** SDR PHY: sweep `CAPTURE_PHASE` (and `RD_PREAMBLE_SKIP` if your
   device drives a read preamble). DDIO PHY: sweep the input delay taps / DPA. Write a known pattern, sweep,
   pick the widest passing window.

The AXC3000 build in [`fpga/axc3000/`](fpga/axc3000/) is a complete worked example of all of the above
(SDR PHY, IOPLL clock plan, pins, `.sdc`, and the JTAG bandwidth-test harness).

---

## Spec-compliance summary

Against **Infineon/Cypress HyperBus Specification 001-99253 Rev \*H**. "Simulated"
means verified against the golden `hyperram_model` under Verilator; it is **not**
a claim of silicon timing closure (see *Status & scope*).

| Spec area | §     | Status | Where / evidence |
|---|---|---|---|
| 48-bit CA encoding (R/W#, AS, burst type, word address) | §2/§3 | Implemented, simulated | `hyperbus_pkg::hb_pack_ca`; `tb_axi`/`tb_avalon` |
| DDR data phase, byte A / byte B ordering, big-endian registers | §4 | Implemented, simulated | `hyperbus_phy_generic`; all TBs |
| Initial latency code table (3–16 clks, codes 1110/1111 = 3/4) | §3 | Implemented | `hb_latency_code_to_clocks` / `_to_latency_code` |
| Fixed latency + RWDS-during-CA 2× select | §3.2/§5.2.4 | Implemented; only the *constant* fixed-2× case simulated | `hyperbus_ctrl`; `tb_fixed2x`. True **variable** latency (alternating 1×/2×) not simulated — [#4](https://github.com/fpga-professional-association/hyperram/issues/4) |
| RWDS-gated read completion; row/page latency gaps absorbed | §3.2/§7 | Implemented, simulated | `hyperbus_ctrl`; `hyperram_model` row penalty |
| Read RWDS-stall ≥ 32 clks → abort + error | §3.2/§4 | Implemented, simulated | `err_timeout`; `tb_timeout` |
| Byte-masked writes (RWDS = ~strobe, High = mask) | §4 | Implemented; **not** simulated (all TBs write full-word) | `hyperbus_ctrl:295` — [#4](https://github.com/fpga-professional-association/hyperram/issues/4) |
| Zero-latency register/config writes (no mask, full word) | §5/§6 | Implemented, simulated | `hyperbus_ctrl`; `tb_axi` CR0 write |
| Linear bursts (CA[45]=1) | §7 | Implemented, simulated | all TBs |
| Wrapped/hybrid bursts (CA[45]=0) | §7 | Implemented; **not** simulated — front-ends tie `cmd_wrap=0`; `tb_axi` "WRAP" is AXI-wrap decomposed to *linear* native segments | ctrl wrap path; [#4](https://github.com/fpga-professional-association/hyperram/issues/4) |
| Wrap boundary from CR0[1:0] (128/64/16/32 B) | §7 | Table implemented; **not** exercised (no wrapped CA; only 32 B configured) | `hb_wrap_words` — [#4](https://github.com/fpga-professional-association/hyperram/issues/4) |
| Burst chopping to tCSM (MAX_BURST_WORDS) | §6 | Implemented; **not** simulated (all TBs set `MAX_BURST_WORDS=0` → chop/re-open FSM never runs) | `hyperbus_ctrl` — [#4](https://github.com/fpga-professional-association/hyperram/issues/4) |
| CR0 / ID0 register access | §5/§8 | Implemented, simulated | `tb_axi` CR0 rw + ID0 rd; `tb_avalon` |
| CR1 / ID1 register access | §8.2/§8.3 | Decoded in model; **not** simulated (no CR1/ID1 access in any TB). CR1 also **not** programmed at init | [#4](https://github.com/fpga-professional-association/hyperram/issues/4), [#5](https://github.com/fpga-professional-association/hyperram/issues/5) |
| POR init + CR0 programming, `init_done` gating | §8/§9 | Implemented, simulated (CR0 only) | `hyperbus_ctrl` init; all TBs wait `init_done`. CR1 init + tRP/tRPH/tRH/tVCS timing not done — [#5](https://github.com/fpga-professional-association/hyperram/issues/5) |
| Deep Power-Down, active clock-stop | §5.2.1/§8.7, §1 | **Not** implemented | [#5](https://github.com/fpga-professional-association/hyperram/issues/5) |
| Differential vs single-ended CK (`DIFF_CK`) | §1 | Parameterized | `hb_ck_n` driven when `DIFF_CK` |
| AC timing (tRWR, tCSHI, tCSS, tACC…) closure | §9 | **Not** provided in RTL | device/board `.sdc` — hardware work, see below |
| CR1 bit layout, latency↔frequency map | §8.2 | Device-specific, not hard-coded | pull from W957D8NB datasheet |

Device register addresses / reset values used in the package and model are
**W957D8NB / HyperRAM-family** values (flagged `[device, not generic spec]` in the
digest), cross-checked against the project BFM — the generic spec defers these to
each datasheet.

---

## Performance (measured on AXC3000 silicon)

Real, integrity-verified HyperRAM bandwidth on the Arrow **AXC3000** (Agilex 3
`A3CY100BM16AE7S` + Winbond **W957D8NB** HyperRAM), via the **SDR PHY**, read back over
JTAG-Avalon with on-chip cycle counters (`fpga/axc3000/`). The SDR PHY runs a `CK_MHZ` word
clock plus a 2×CK byte clock from one IOPLL, so bandwidth scales directly with the clock:

| HyperBus CK | Byte clk | Write (LEN=512) | Read (LEN=512) | Notes |
|------------:|---------:|----------------:|---------------:|-------|
| 50 MHz  | 100 MHz | 96.8  | 94.8  MB/s | original bring-up |
| 100 MHz | 200 MHz | 193.6 | 189.3 | |
| 133 MHz | 266 MHz | 258.1 | 252.4 | |
| 150 MHz | 300 MHz | 290.4 | 283.9 | |
| 160 MHz | 320 MHz | 309.7 | 302.9 | |
| **175 MHz** | **350 MHz** | **342.4** | **337.3 MB/s** | **SDR ceiling** (LEN=768) |

Every row is `STATUS.done`, `ERR_COUNT=0`, integrity PASS, re-confirmed on repeats. Bandwidth also
grows with **burst length** as the fixed per-transaction overhead (6-beat CA + initial latency)
amortizes. The read eye holds all the way to the 350 MHz byte clock with `CAPTURE_PHASE=0` (no tuning).

- **Fit** (A3CY100, Quartus Pro 26.1, timing closed at 175 MHz CK): outclk0 (fabric) Fmax 189 MHz,
  outclk1 (byte) restricted Fmax **352.98 MHz** — the min-pulse-width limit that caps this 2×-byte SDR
  architecture at **~176 MHz CK**. ~1 k ALM / ~1.4 k reg / few M20K / 0 DSP / 1 PLL.
- **Reaching the 200 MHz / 400 MB/s device max** requires the DDIO PHY (I/O at 1×CK), still blocked on
  the Fitter's 24403/24404 two-clock-phase routing —
  [issue #3](https://github.com/fpga-professional-association/hyperram/issues/3).

**Scope / open items.** Single bursts commit clean up to ~768 words (≥1024 hits tCSM refresh, ≈15 µs).
Multi-burst *reads* complete correctly (over-stream drain). Multi-burst (split) *writes* drop the last
word of each non-final burst — a W957D8NB write-commit quirk
([issue #1](https://github.com/fpga-professional-association/hyperram/issues/1); workaround: single
bursts ≤512 words). The board build adds runtime burst-size + first-error diagnostic CSRs and an
on-chip logic analyzer (`fpga/axc3000/hyperbus_capture.sv`); details in `fpga/axc3000/README.md`.

## Status & scope

- **Simulation-validated.** The controller, both front-ends, and the **generic** and
  **SDR** PHYs are verified against a behavioral HyperRAM model (incl. the real read
  preamble) under Verilator 5.020 — protocol correctness, not electrical timing.
- **Hardware-validated (AXC3000).** The SDR-PHY build is timing-closed, programmed,
  and **measured on real silicon** (see Performance above): single-burst HyperRAM
  read/write with verified integrity. This is the first on-silicon bandwidth for
  this controller.
- **The generic PHY is simulation-oriented.** It uses plain clocked registers and
  behavioral DDR muxes (and a modeled RWDS strobe delay). It will *infer* on any
  FPGA but is not tuned for a specific device's I/O timing.
- **The AMD/Xilinx PHY simulates via a primitive shim; the Intel/Altera PHY does not.**
  `hyperbus_phy_xilinx` is a real 7-series datapath (`ODDR`/`IDDR`/`IDELAYE2`/`IDELAYCTRL`/
  `BUFIO`/`BUFR`/`OBUF`/`OBUFDS`) that **simulates end-to-end under `verilator --binary --timing`**
  through a Verilator-only shim (`sim/model/xilinx_prims_sim.sv`, `tb_xilinx`) — but is **still not
  hardware-proven or timing-closed**: the read-eye taps (`RX_STROBE_DLY_TAPS`), byte-pairing polarity
  (`RX_PAIR_SKEW`) and preamble skip (`RD_PREAMBLE_SKIP`) are bring-up knobs to sweep on real silicon.
  `hyperbus_phy_altera` uses Quartus/Agilex hard-IP primitives and is therefore **not** expected to
  simulate under Verilator (validated by the fitter + on-hardware bring-up). Neither is a drop-in for a
  new board without pin/XDC constraints and static timing closure. See
  [`docs/PHY_PORTING.md`](docs/PHY_PORTING.md).
- **Hardware measurement** is the AXC3000 SDR-PHY result in **Performance** above: single-burst
  read/write clean and integrity-verified from 50 up to **175 MHz CK (~342/337 MB/s)**, and multi-burst
  *reads* validated. Open on hardware: split multi-burst *writes* (issue #1) and the 200 MHz DDIO PHY
  (issue #3). The generic/vendor DDIO PHY variants are not yet hardware-validated.

---

## Repository layout

```
rtl/
  hyperbus_pkg.sv            params, typedefs, CA/latency/wrap functions
  hyperbus_ctrl.sv          protocol engine (native slave ⇄ PHY master)
  hyperram_axi.sv           top: AXI4  + ctrl + phy
  hyperram_avalon.sv        top: Avalon-MM + ctrl + phy
  if/hyperbus_axi.sv        AXI4 slave front-end
  if/hyperbus_avalon.sv     Avalon-MM slave front-end
  phy/hyperbus_phy.sv       PHY wrapper (selects variant)
  phy/hyperbus_phy_generic.sv   inferrable DDR + RWDS→clk CDC (sim + any FPGA)
  phy/hyperbus_phy_altera.sv    Intel/Altera DDR-IO skeleton
  phy/hyperbus_phy_xilinx.sv    AMD/Xilinx 7-series ODDR/IDDR DDR-I/O (simulates via primitive shim)
sim/
  model/hyperram_model.sv       behavioral golden device
  model/xilinx_prims_sim.sv     Verilator-only 7-series primitive shim (ODDR/IDDR/IDELAYE2/… ; tb_xilinx)
  tb_avalon / tb_axi / tb_fixed2x / tb_timeout / tb_xilinx …   self-checking testbenches
  run.sh                        build + run all TBs (Verilator)
docs/
  SPEC_DIGEST.md  DESIGN.md  INTERFACES.md  INTEGRATION.md  PHY_PORTING.md
```

---

## License

Apache-2.0. Copyright 2026 FPGA Professional Association. See [`LICENSE`](LICENSE)
and [`NOTICE`](NOTICE). This is an **original clean-room** implementation of the
public HyperBus specification; MJoergen/HyperRAM (MIT) and OpenHBMC (Apache-2.0)
were consulted as design references only — no code was copied.
