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
  variant (simulation + any FPGA) plus **Intel/Altera** and **AMD/Xilinx**
  vendor-primitive skeletons for board bring-up (see *Status & scope*).
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
| `hyperbus_phy_xilinx` | AMD/Xilinx ODDR/IDDR variant | yes | skeleton |
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

## Spec-compliance summary

Against **Infineon/Cypress HyperBus Specification 001-99253 Rev \*H**. "Simulated"
means verified against the golden `hyperram_model` under Verilator; it is **not**
a claim of silicon timing closure (see *Status & scope*).

| Spec area | §     | Status | Where / evidence |
|---|---|---|---|
| 48-bit CA encoding (R/W#, AS, burst type, word address) | §2/§3 | Implemented, simulated | `hyperbus_pkg::hb_pack_ca`; `tb_axi`/`tb_avalon` |
| DDR data phase, byte A / byte B ordering, big-endian registers | §4 | Implemented, simulated | `hyperbus_phy_generic`; all TBs |
| Initial latency code table (3–16 clks, codes 1110/1111 = 3/4) | §3 | Implemented | `hb_latency_code_to_clocks` / `_to_latency_code` |
| Fixed vs variable latency; RWDS-during-CA 1×/2× select | §3.2/§5.2.4 | Implemented, simulated | `hyperbus_ctrl`; `tb_fixed2x` |
| RWDS-gated read completion; row/page latency gaps absorbed | §3.2/§7 | Implemented, simulated | `hyperbus_ctrl`; `hyperram_model` row penalty |
| Read RWDS-stall ≥ 32 clks → abort + error | §3.2/§4 | Implemented, simulated | `err_timeout`; `tb_timeout` |
| Byte-masked writes (RWDS = ~strobe, High = mask) | §4 | Implemented, simulated | `hyperbus_ctrl`; `tb_axi` write path |
| Zero-latency register/config writes (no mask, full word) | §5/§6 | Implemented, simulated | `hyperbus_ctrl`; `tb_axi` CR0 write |
| Linear + wrapped bursts (CA[45]) | §7 | Implemented, simulated | `tb_axi` WRAP read; wrap tables in `hyperbus_pkg` |
| Wrap boundary from CR0[1:0] (128/64/16/32 B) | §7 | Table implemented | `hb_wrap_words` |
| Burst chopping to tCSM (MAX_BURST_WORDS) | §6 | Implemented | `hyperbus_ctrl` (`MAX_BURST_WORDS`) |
| CR0/CR1/ID0/ID1 register access | §5/§8 | Implemented, simulated | `tb_axi` CR0 + ID0; `tb_avalon` |
| POR init + CR programming, `init_done` gating | §8/§9 | Implemented, simulated | `hyperbus_ctrl` init; all TBs wait `init_done` |
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
`A3CY100BM16AE7S` + Winbond **W957D8NB** HyperRAM), via the **SDR PHY** at a
conservative **50 MHz** HyperBus clock (100 MHz fabric byte clock), read back over
JTAG-Avalon with on-chip cycle counters (`fpga/axc3000/`). Independently re-confirmed.

| Metric | Value | Conditions |
|---|---|---|
| **Write bandwidth** | **48.48 MB/s** | 16-word burst · `STATUS.done`, `ERR_COUNT=0`, integrity PASS |
| **Read bandwidth**  | **36.36 MB/s** | 16-word burst · `STATUS.done`, `ERR_COUNT=0`, integrity PASS |
| HyperBus CK | **50 MHz** (×8 SDR, 1 byte / 100 MHz fabric cycle; ~100 MB/s/dir theoretical peak) | overhead amortizes with burst length |
| SDR-PHY system fit (A3CY100) | fmax 169.6 MHz `clk` / 373 MHz byte clock · **1,006 ALM / 1,393 reg / 2 M20K / 0 DSP** | Quartus Pro 26.1, timing met |

Bandwidth scales with burst length as the fixed per-transaction overhead (6-beat CA
+ initial latency) amortizes: LEN=4 → 12.5 MB/s read, LEN=5 → 15.2, LEN=16 → 36.36.

**Scope of the measurement:** single bursts up to 16 words complete with verified
data integrity. Multi-burst reads (LEN > 16) currently hang on silicon — a read-path
timing effect *not* reproduced in the ideal-clock sim; captured + documented in
`fpga/axc3000/README.md` (on-chip logic analyzer `hyperbus_capture.sv`). The
DDIO/high-speed PHY (toward the device's 250 MHz DDR spec) is future work.

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
- **Vendor PHY variants are skeletons.** `hyperbus_phy_altera` and
  `hyperbus_phy_xilinx` present the frozen port list and mark every spot that must
  become a hard-IO primitive (`ODDR`/`IDDR`, `ALTDDIO`/DDIO, IDELAY/DPA, input
  FIFO) with `TODO(vendor)`. They are **not** expected to simulate and are **not**
  timing-closed. Real board bring-up — primitive instantiation, RWDS strobe
  delay/DPA calibration, and static timing closure against a device datasheet — is
  per-target hardware work. See [`docs/PHY_PORTING.md`](docs/PHY_PORTING.md).
- **Hardware measurement** is limited to the AXC3000 SDR-PHY single-burst result in
  **Performance** above; the generic/vendor PHY variants and multi-burst reads are
  not yet hardware-validated.

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
  phy/hyperbus_phy_xilinx.sv    AMD/Xilinx ODDR/IDDR skeleton
sim/
  model/hyperram_model.sv   behavioral golden device
  tb_avalon / tb_axi / tb_fixed2x / tb_timeout   self-checking testbenches
  run.sh                    build + run all TBs (Verilator)
docs/
  SPEC_DIGEST.md  DESIGN.md  INTERFACES.md  INTEGRATION.md  PHY_PORTING.md
```

---

## License

Apache-2.0. Copyright 2026 FPGA Professional Association. See [`LICENSE`](LICENSE)
and [`NOTICE`](NOTICE). This is an **original clean-room** implementation of the
public HyperBus specification; MJoergen/HyperRAM (MIT) and OpenHBMC (Apache-2.0)
were consulted as design references only — no code was copied.
