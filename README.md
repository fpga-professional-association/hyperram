# HyperRAM / HyperBus Controller IP

A clean-room, vendor-neutral **HyperBus master** for HyperRAM devices: one protocol engine
(`hyperbus_ctrl`) behind your choice of **AXI4** or **Avalon-MM** slave front-end and a swappable
PHY (generic-inferrable, portable-SDR, AMD/Xilinx, Intel/Altera). Verified end-to-end against a
behavioral device model under Verilator (23 self-checking testbenches) and on real silicon.

**Measured on hardware** (Arrow AXC3000: Agilex 3 `A3CY100BM16AE7S` + Winbond **W957D8NBRA4I**,
128 Mb ×8, 1.2 V, HyperRAM 2.1):

| Clock | Interface | Write | Read | Integrity |
|------:|-----------|------:|-----:|-----------|
| **175 MHz CK** | DDR x8 | **341.1 MB/s** | **332.3 MB/s** | `ERR_COUNT=0`, 25-run soak clean |

Full-row 1024-word bursts pace 343.3 / 337.8 MB/s. The board's clean ceiling is ~176 MHz (FPGA
clock-generation limit); the path to the device's rated 250 MHz / 500 MB/s is
[issue #12](https://github.com/fpga-professional-association/hyperram/issues/12).

---

## Features

**Protocol (HyperBus spec 001-99253 Rev \*H):**
- Full 48-bit Command-Address encode (R/W#, address space, burst type, word address).
- Fixed **and** variable initial latency: the controller decodes the device's RWDS level during CA
  and inserts the 2× latency count whenever the device requests it, for reads and writes.
- RWDS-gated read completion — mid-burst row/page latency gaps are absorbed transparently.
- Byte-masked writes (RWDS = inverted strobe), zero-latency register/config writes.
- Linear and wrapped/hybrid bursts (all four CR0 wrap sizes).
- POR/reset sequencing with ns-derived AC timing (`CLK_FREQ_MHZ` + `T_RP_NS/T_RPH_NS/T_RH_NS/
  T_VCS_US`), CR0 **and optional CR1** programming at init, `init_done` gating.
- **Deep Power-Down**: entry via a host CR0 write is detected, and the next command transparently
  performs the guarded wake (CS# pulse + `tDPDOUT`).
- **Active clock-stop** during read back-pressure (`ACTIVE_CLK_STOP`).
- Read-stall watchdog: RWDS silent ≥ 32 clocks → clean abort, `rd_last` preserved, AXI `SLVERR` —
  no deadlock.

**Throughput & device management:**
- **Write coalescing** (`WR_COALESCE`): contiguous write commands arriving back-to-back are
  spliced into one CS# burst — a stream of small writes runs at full bus rate with no
  per-command CA/latency cost and no CS# boundaries inside a row.
- **Row-aligned segmenting** (`MAX_BURST_WORDS`, `BURST_BOUNDARY_WORDS`): long transfers are
  chopped so no burst ever crosses the device row — the invariant the W957D8NB requires (see
  *Device limitations* below). Chopping is transparent to the caller.
- Deterministic, spec-legal CS#-low time per segment (tCSM compliance by construction).

**Runtime calibration & diagnostics:**
- **Live read-eye calibration ports** (`cal_capture_phase`, `cal_preamble_skip`, `cal_rx_tap`,
  `cal_pair_skew`): retune the read capture at runtime with a CSR write — no recompile. The
  bench exposes them at `REG_CAL` (word 13).
- The synthesizable bandwidth/integrity engine (`rtl/bench/hyperram_bw_test.sv`): cycle-exact
  write/read MB/s counters, per-word integrity check, first-error address/got/expected CSRs,
  runtime burst-size registers, and a **read-only run mode** (score a region without rewriting
  it — the instrument that characterized the device below).

**Portability & verification:**
- Controller + front-ends contain **no vendor primitives**; single clock domain, synchronous
  reset, Hyperflex-friendly. The one true CDC (RWDS → `clk`) is isolated inside the PHY.
- Four PHY variants behind one frozen port list (see *Porting*).
- 23 Verilator testbenches against a golden device model that reproduces the real silicon's
  behaviors (read preamble, variable latency, row quirks — see below); `bash sim/run.sh` runs
  everything and fails non-zero on any mismatch.

---

## Device limitations and how they are handled

Silicon characterization of the W957D8NBRA4I (read-only-probe + wound-map verified, 2026-07;
full trail: [`fpga/axc3000/README.md`](fpga/axc3000/README.md), `docs/INTERFACES.md`,
issues [#1](https://github.com/fpga-professional-association/hyperram/issues/1) and
[#13](https://github.com/fpga-professional-association/hyperram/issues/13)) reduced the
2026-07-09 "three laws" to **one root mechanism** — a 4-word write-commit pipeline in the
device — and issue #13 built a runtime-provable **fix set that drives every measured write
shape to zero loss**.

**The mechanism** (marker-attribution proven — the wound content is literally whatever is
driven on DQ, `0xA500..03` landed on demand):

| # | Device behavior (all RO/EMAP-verified) | What the core does about it |
|---|---|---|
| 1 | **Open-sampling commit**: at every memory-write CS# open at word B, the device shifts DQ bus state through its 4-word pipe across the write-latency window and commits the last 4 slots to `[B-4, B)`. Idle bus ⇒ the classic "wound" (zeros); the bus is never idle if the core drives it. | **Pre-window heal** (`REG_DBG[9]`, width `[12:10]`, sweep-proven n=4): re-drives the true `[B-4..B-1]` from the retained write shadow during the last 4 latency clocks — the commit *is* the heal. Works at internal row chops out of the box; `REG_DBG[16]` (contiguity qualifier) extends it across contiguous command-edge reopens. |
| 2 | **Row wrap**: a linear burst must never cross the **1024-word (2 KB) row** — writes wrap onto the row start, reads release the bus. | Row-bounded, row-aligned segments (`MAX_BURST_WORDS=1024` = `BURST_BOUNDARY_WORDS='h400`): no burst ever crosses a row. Transparent. |
| 3 | **Row-end orphan**: a write burst *closing* exactly on a row multiple never commits its last 4 words — they park in a device register (a **discard**, not a garble: old content stays; the historic "0x5050" was stale data). Orphans coexist, survive all writes (masked included). | **End-commit-write** (`REG_DBG[17]`): after a row-aligned final close the core opens one masked 4-beat write at `B=end` — its open-sampling commit (fact 1) heals the tail home; the mask keeps `[end, end+4)` untouched. |
| 4 | **Orphan spray**: at the first READ after parking, each orphan fires once — its 4 words land at `[home-1028, home-1024)`, exactly the *previous* boundary's home (below-zero targets are dropped). Reads never place it home (lengths 1..1024 swept). This is why the historic `WR_COMMIT_READ` and every read-shaped repair failed. | **Spray defuse** (`REG_DBG[18]`): at each interior row boundary the core fires the spray deterministically with an internal read, then re-heals the one known casualty from a retained boundary-tail history. First boundary needs nothing (its spray lands in the already-dead below-base zone or is dropped). |

**Measured result (issue #13 acceptance, 175 MHz, `PAT=addr-echo`, RO-ground-truth confirmed,
25-run soak clean):** with the fix set enabled (`prewin=1 pn=4 contig=1 endcw=1 defuse=1`,
i.e. `REG_DBG=0x0007_1263`), **768 / 1024 / 1536 / 2048 / 4096-by-256 / 16 KB-crosser / 8192
all read back `ERR_COUNT=0`**. In-row throughput is unchanged (341.1 W / 332.3 R MB/s); an
8-row stream pays ~7 % write throughput for the per-boundary defuse (315.9 W / 329.5 R MB/s).

**Defaults and enabling:** the fix set is **default-off** (POR `REG_DBG` seeds the legacy
trim/latency only) — the legacy contract below still describes default behavior. Enable at
runtime (`sysconsole/dbg_poke.tcl prewin 1 / pn 4 / contig 1 / endcw 1 / defuse 1`) or bake it
in with the bench parameter `DBG_RESET=32'h0007_1263`.

**Legacy (fix set off) cost model — measured, exactly predictive:** in-row transfers are
loss-free; a write stream spanning rows loses exactly 4 words per row transition at
`[N·1024−4, N·1024)`. Reads are safe *unless a row-end orphan is pending* (fact 4: the first
read then sprays 4 stale words one row below the orphan's home — this is the corrected form of
the old "reads never wound" claim).

**What an application must still mind:**

1. **Non-contiguous fresh writes** wound `[base-4, base)` (fact 1 with an idle bus and no valid
   shadow). Keep an 8-byte guard gap below independently-written bases, write abutting regions
   descending, or align bases to rows. (With the fix set on, *contiguous* appends heal
   automatically — the old "no write-only repair" rule is superseded: a contiguous re-write
   with the heal active repairs in place, E5-proven.)
2. **Read throughput:** reads never coalesce (each read pays CA + latency); prefer large
   burstcounts. 
3. **Rebuild discipline:** the DQ/CK pad launch is calibration-based (`WR_LAT_TRIM`), not
   SDC-constrained — a refit can be silicon-marginal even with STA met (seen once: 2 words per
   row-end close uncommitted). Re-run the bw_read shape suite after **every** recompile; the
   known-good bitstream is banked at
   `fpga/axc3000/bitstreams/ddio_row_175_issue13_fixset_seed4_20260711.sof`.
4. **Different silicon:** these are single-sample W957D8NB findings (temperature/voltage/second
   -unit margining still open — issue #13 L-G). The knobs are parameters/CSRs
   (`MAX_BURST_WORDS`, `BURST_BOUNDARY_WORDS`, `WR_COALESCE`, `DBG_RESET`), and the repo ships
   the probe method: read-only runs (`bw_read.tcl` arg 6), the 64-deep wound map
   (`emap_dump.tcl`), pattern scrubbing (`pat_set.tcl`), and marker attribution
   (`dbg_poke.tcl marker 1`) to verify *your* part's behavior directly.

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
                 │       (thin, no protocol)                     no vendor prims                        └────────┘  │
                 └─────────────────────────────────────────────────────────────────────────────────────────────────┘
                       clk / clk90 / clk_ref / rst  (one PLL, phase-related; ctrl uses only clk)

  Simulation:  the same hb_* device pins connect to sim/model/hyperram_model.sv (golden HyperRAM model,
               including the row/wound behaviors above as opt-in knobs), bus resolution in the testbench.
```

| Module | Role | Vendor prims | Verilator |
|---|---|---|---|
| `hyperbus_pkg` | params, typedefs, CA pack/unpack, latency & wrap tables | no | yes |
| `hyperbus_ctrl` | protocol engine (CA, latency, RWDS-gated read, write mask, coalescing, row segmenting, POR/CR init, DPD, clock-stop) | **no** | yes |
| `hyperbus_phy` | PHY wrapper; selects one variant by `PHY_VARIANT` | — | — |
| `hyperbus_phy_generic` | inferrable DDR I/O + RWDS→clk CDC | **no** | **yes** |
| `hyperbus_phy_sdr` | portable single-periphery-clock variant (byte engine at 2×CK) | **no** | **yes** |
| `hyperbus_phy_xilinx` | AMD/Xilinx 7-series ODDR/IDDR/IDELAYE2 datapath | yes | yes (via primitive shim) |
| `hyperbus_phy_altera` | Intel/Altera DDIO variant | yes | no (fitter/hardware-validated) |
| `hyperbus_avalon` / `hyperbus_axi` | bus slave → native (thin) | no | yes |
| `hyperram_avalon` / `hyperram_axi` | tops = front-end + ctrl + phy | per PHY | yes |
| `hyperram_bw_test` (`rtl/bench/`) | synthesizable bandwidth/integrity engine + CSRs | no | yes |
| `hyperram_model` (`sim/`) | behavioral golden device (sim only) | no | yes |

Frozen module boundaries: [`docs/INTERFACES.md`](docs/INTERFACES.md) (currently v10).
Architecture rationale: [`docs/DESIGN.md`](docs/DESIGN.md).

---

## Host interfaces

Full signal tables in [`docs/INTERFACES.md`](docs/INTERFACES.md); summary:

### AXI4 slave (`hyperram_axi`)

Standard AXI4 (AW/W/B/AR/R); data beats map 1:1 to 16-bit HyperBus words at
`AXI_DATA_WIDTH=16`. `awaddr`/`araddr` MSB selects register space (CR0/CR1/ID0/ID1). INCR bursts
→ one native segment; WRAP → two segments in AXI order; FIXED → per-beat segments. Controller
errors surface as `SLVERR` on B/R.

### Avalon-MM slave (`hyperram_avalon`)

Word-addressed pipelined slave with `burstcount`, `byteenable`, `readdatavalid`, `waitrequest`;
address MSB selects register space. An `err_underrun` strobe reports write-data underruns.

### HyperBus device pins (from `hyperbus_phy`)

`hb_ck`, `hb_ck_n` (if `DIFF_CK=1`), `hb_cs_n`, `hb_rst_n`, and **split** data pins
(`hb_dq_o/_oe/_i`, `hb_rwds_o/_oe/_i`) — the board wrapper adds the tristate pads
([`docs/INTEGRATION.md`](docs/INTEGRATION.md)). Status: `init_done`. Runtime calibration inputs:
`cal_capture_phase`, `cal_preamble_skip[2:0]`, `cal_rx_tap[4:0]`, `cal_pair_skew`.

---

## Key parameters

Common: `DQ_WIDTH` (8), `DATA_WIDTH` (16), `ADDR_WIDTH` (32), `LEN_WIDTH` (16).

Controller (`hyperbus_ctrl`, forwarded through both tops):

| Parameter | Default | Meaning |
|---|---|---|
| `LATENCY_CLOCKS` / `FIXED_LATENCY` | 6 / 1 | initial latency (spec Table 5.3) and fixed/variable select |
| `MAX_BURST_WORDS` | 0 (off) | segment cap — set to the device **row size** (W957D8NB: 1024) |
| `BURST_BOUNDARY_WORDS` | 0 (off) | never let a burst cross this boundary — set to the **row** (`'h400`) |
| `WR_COALESCE` / `WR_COALESCE_WAIT` | 0 / 8 | splice contiguous write commands into one CS# burst |
| `WR_LAT_TRIM` | 0 | write-window calibration offset (CK); board-measured (AXC3000: 3) |
| `PROGRAM_CR` / `INIT_CR0` / `INIT_LATENCY_CODE` | 1 / device | CR0 programming at init |
| `PROGRAM_CR1` / `INIT_CR1` | 0 / device | optional CR1 programming at init |
| `CLK_FREQ_MHZ` + `T_RP_NS/T_RPH_NS/T_RH_NS/T_VCS_US` | 0 (legacy) | ns-derived POR/reset AC timing |
| `SUPPORT_DPD` / `TDPDOUT_CYCLES` | 0 | Deep-Power-Down detect + guarded wake |
| `ACTIVE_CLK_STOP` | 0 | stop CK during read back-pressure |
| `WR_COMMIT_READ*`, `WR_CHOP_REPLAY*`, `WR_CHOP_PAUSE*` | off | parked experiment family (see limitations §5) |

PHY adds `PHY_VARIANT` (`"GENERIC"`\|`"SDR"`\|`"XILINX"`\|`"INTEL"`), `DIFF_CK`,
`RD_PREAMBLE_SKIP`, and per-variant capture knobs (all runtime-tunable via the `cal_*` ports on
the SDR variant). AXI adds `ID_WIDTH`, `AXI_DATA_WIDTH`, `AXI_ADDR_WIDTH`.

---

## Quick start

Prerequisites: **Verilator ≥ 5.020**, C++17 toolchain, `bash`.

```bash
git clone <this-repo> hyperram
cd hyperram
bash sim/run.sh        # builds + runs all 23 self-checking TBs; exits non-zero on any failure
```

Coverage by area:

| Area | Testbenches |
|---|---|
| Bus front-ends | `tb_avalon`, `tb_axi`, `tb_axi_wrap` (WRAP-write decomposition, AR/AW arbiter) |
| Latency & timing | `tb_fixed2x`, `tb_varlat` (per-transaction 1×/2×), `tb_timeout`, `tb_por_timing` |
| Bursts & data | `tb_chop`, `tb_wrap` (all four CR0 wrap sizes), `tb_masked`, `tb_multiburst`, `tb_multiburst_generic` |
| Registers & init | `tb_reg`, `tb_cr1init` |
| Power management | `tb_dpd`, `tb_clkstop` |
| Device quirks | `tb_commit` (wound/row model: coalescing, row segmenting, the parked replay family — 11 stacks); `tb_dbg` (issue #13: REG_DBG knobs, heal/marker, EMAP wound map, REG_PAT, wrapped-write probe, orphan/spray model + the defuse — 12 checks) |
| PHY variants | `tb_sdr`, `tb_preamble`, `tb_preamble_generic`, `tb_xilinx` (7-series datapath via shim), `tb_cal` (live REG_CAL retune, no recompile) |
| Bench engine | `tb_bw` (bandwidth engine + CSRs) |

(A 24th testbench, `tb_local1x`, phase-sweeps the AXC3000 board I/O layer and lives outside
`run.sh` because it references board files.)

Every testbench checks byte-exact read-back against the golden model and `$fatal`s on mismatch.

---

## Porting to your device

The controller, front-ends, and generic/SDR PHYs are device-independent. Bring-up on a new
FPGA + HyperRAM is a PHY + board job:

1. **Pick a PHY** (`PHY_VARIANT`):
   - `"GENERIC"` — inferrable DDR; simulation and any-FPGA starting point.
   - `"SDR"` — **recommended first hardware target**: one clock in the I/O periphery (byte engine
     at 2×CK), no vendor primitives, no per-bit calibration; silicon-proven on the AXC3000.
   - `"XILINX"` — real 7-series ODDR/IDDR/IDELAYE2 datapath; simulates under Verilator via the
     included primitive shim; not yet hardware-proven — sweep its `RX_STROBE_DLY_TAPS` /
     `RX_PAIR_SKEW` / `RD_PREAMBLE_SKIP` on silicon.
   - `"INTEL"` — Agilex DDIO variant. The complete, silicon-proven Agilex 3 board build (I/O
     layer, clocking, constraints, benchmark harness) is in [`fpga/axc3000/`](fpga/axc3000/) —
     use it as the worked example; its README documents every board-level pitfall.
2. **Clock plan:** one PLL; `clk` = CK word rate; the SDR variant repurposes `clk90` as the 2×CK
   byte clock (only one clock enters the I/O periphery).
3. **Board wrapper:** the PHY exposes split `_o/_oe/_i` pins (Verilator-shaped, no `inout`); add
   the tristate pad ring at the top ([`docs/INTEGRATION.md`](docs/INTEGRATION.md)).
4. **Pins + I/O standard** from your board; get `hb_cs_n`/`hb_ck` right or the device never
   responds.
5. **Timing:** false-path the HyperBus pins for bring-up; real source-synchronous closure for
   production speeds.
6. **Device parameters from your datasheet:** `LATENCY_CLOCKS`, `INIT_CR0`, and the row/segment
   knobs (`MAX_BURST_WORDS` = row words, `BURST_BOUNDARY_WORDS` = row words, `WR_COALESCE=1`).
   Then verify the device-limitation model on *your* part with the read-only-probe method
   (limitations §5).
7. **Read-eye calibration:** write a pattern, sweep the capture knobs (`CAPTURE_PHASE` /
   `cal_*` CSR on SDR; delay taps on the DDR variants), pick the widest passing window.
   `WR_LAT_TRIM` calibrates the write window (the first-error CSR offset tracks it 1:1).

Porting details: [`docs/PHY_PORTING.md`](docs/PHY_PORTING.md).

---

## Spec compliance

Against Infineon/Cypress **HyperBus Specification 001-99253 Rev \*H**. "Simulated" = verified
against the golden model under Verilator; electrical/AC closure is board work.

| Spec area | § | Status |
|---|---|---|
| 48-bit CA encode (R/W#, AS, burst type, address) | §2/§3 | Implemented, simulated |
| DDR data phase, byte ordering, big-endian registers | §4 | Implemented, simulated |
| Initial latency codes (3–16 clocks) | §3 | Implemented |
| Fixed latency + RWDS-during-CA 2× select | §3.2/§5.2.4 | Implemented, simulated (constant + per-transaction variable) |
| RWDS-gated read completion, latency-gap absorption | §3.2/§7 | Implemented, simulated |
| Read RWDS-stall ≥ 32 clk → abort + error | §3.2/§4 | Implemented, simulated |
| Byte-masked writes | §4 | Implemented, simulated (all strobe combos) |
| Zero-latency register writes | §5/§6 | Implemented, simulated |
| Linear bursts + tCSM segmenting | §6/§7 | Implemented, simulated + silicon-verified |
| Wrapped/hybrid bursts, all CR0[1:0] sizes | §7 | Implemented, simulated |
| CR0/CR1/ID0/ID1 access | §5/§8 | Implemented, simulated |
| POR init, CR0 + optional CR1 programming, ns-derived tRP/tRPH/tRH/tVCS | §8/§9 | Implemented, simulated |
| Deep Power-Down (entry detect + guarded wake) | §5.2.1/§8.7 | Implemented, simulated |
| Active clock-stop | §1 | Implemented, simulated |
| Differential / single-ended CK (`DIFF_CK`) | §1 | Implemented, simulated |
| AC timing closure (tRWR, tCSS, tACC…) | §9 | Board `.sdc` work — see `fpga/axc3000/` for the closed example |

Register addresses/reset values default to W957D8NB-family; override from your datasheet.

---

## Performance

Measured on the AXC3000 (Agilex 3 + W957D8NBRA4I, DDR x8 board build in
[`fpga/axc3000/`](fpga/axc3000/), JTAG-read cycle counters, every word integrity-checked):

| Shape | Write | Read | Errors |
|---|---:|---:|---|
| 768-word bursts (in-row) | 341.1 MB/s | 332.3 MB/s | 0 (25-run soak) |
| 1024-word full-row bursts | 343.3 MB/s | 337.8 MB/s | the known 4-word row-transition cost |
| Coalesced 64-word write commands | 331.9 MB/s | — | 0 |

Bandwidth scales with CK (`CK_MHZ` knob in `qsys/make_bw_sys.tcl`) and with burst length as the
CA + latency overhead amortizes.

### Reaching the device's rated 250 MHz / 500 MB/s

The W957D8NBRA4I is the 4 ns (250 MHz) speed grade; this IP's engine is not the bottleneck —
the HyperBus **clock path** is:

- **CK must come from a dedicated PLL clock-output pin pair** (true clock-network drive), not
  from I/O-cell data registers: on the AXC3000, every I/O-cell CK source shows a page-boundary
  bit-margin artifact above ~176 MHz, while the fabric generator is clean but min-pulse-capped
  at ~176 MHz.
- **CK/CK# should be differential** on this 1.2 V-class device (Infineon guidance); set
  `DIFF_CK=1` (the IP already drives both legs) and route a matched pair. The AXC3000 wires CK
  single-ended, which is why its ceiling stands at ~176 MHz.
- Then: latency code per the 250 MHz AC table, `WR_LAT_TRIM` recalibration, fabric timing at
  250 MHz (Fmax today: 210). The complete plan with acceptance criteria is
  [issue #12](https://github.com/fpga-professional-association/hyperram/issues/12).

---

## Repository layout

```
rtl/
  hyperbus_pkg.sv               params, typedefs, CA/latency/wrap functions
  hyperbus_ctrl.sv              protocol engine (native slave ⇄ PHY master)
  hyperram_axi.sv               top: AXI4      + ctrl + phy
  hyperram_avalon.sv            top: Avalon-MM + ctrl + phy
  if/hyperbus_axi.sv            AXI4 slave front-end
  if/hyperbus_avalon.sv         Avalon-MM slave front-end
  phy/hyperbus_phy.sv           PHY wrapper (selects variant)
  phy/hyperbus_phy_generic.sv   inferrable DDR + RWDS→clk CDC
  phy/hyperbus_phy_sdr.sv       portable single-periphery-clock variant
  phy/hyperbus_phy_xilinx.sv    AMD/Xilinx 7-series datapath (simulates via shim)
  phy/hyperbus_phy_altera.sv    Intel/Altera DDIO variant
  bench/hyperram_bw_test.sv     synthesizable bandwidth/integrity engine + CSRs
  bench/hyperram_bw_top.sv      bench top (engine + IP top)
sim/
  model/hyperram_model.sv       behavioral golden device (incl. wound/row knobs)
  model/xilinx_prims_sim.sv     Verilator-only 7-series primitive shim
  tb_*.sv                       23 self-checking testbenches
  run.sh                        build + run everything (Verilator)
fpga/axc3000/                   complete Agilex 3 board build: I/O layer, qsys clocking,
                                constraints, bitstreams, JTAG benchmark + probe scripts
docs/
  SPEC_DIGEST.md  DESIGN.md  INTERFACES.md  INTEGRATION.md  PHY_PORTING.md
```

---

## License

Apache-2.0. Copyright 2026 FPGA Professional Association. See [`LICENSE`](LICENSE) and
[`NOTICE`](NOTICE). This is an **original clean-room** implementation of the public HyperBus
specification; MJoergen/HyperRAM (MIT) and OpenHBMC (Apache-2.0) were consulted as design
references only — no code was copied.
