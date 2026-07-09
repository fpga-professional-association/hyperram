# HyperBus Master IP — Design & Architecture

**Status:** interface-frozen (see `INTERFACES.md`). **Normative spec digest:** `SPEC_DIGEST.md`
(Infineon/Cypress HyperBus 001-99253 Rev *H). **Shared definitions:** `rtl/hyperbus_pkg.sv`.

This IP is a clean-room, technology-agnostic HyperBus master (HyperRAM / HyperFlash class devices).
Design goals, in priority order:

1. **PHY-agnostic controller.** The protocol engine contains **no vendor primitives** and no I/O
   cells. It talks to the outside world only through a small DDR-parallel *PHY interface* and is
   therefore fully simulable under `verilator --binary` (5.020).
2. **One native interface.** There is exactly **one** native controller command/data interface
   (`hyperbus_ctrl`). AXI4 and Avalon-MM are **thin** front-ends that translate their bus protocol to
   this native interface and add nothing else.
3. **Swappable PHY.** The PHY has a **generic inferrable-DDR** variant (behaves correctly in
   simulation and infers on any FPGA), an **AMD/Xilinx** 7-series `ODDR`/`IDDR`/`IDELAYE2` variant that
   **simulates via a Verilator-only primitive shim** (still not hardware-proven or timing-closed), and
   an **Intel/Altera** DDIO variant that wraps Quartus/Agilex hard-IP primitives (not Verilator-
   simulable; validated by the fitter + hardware). All variants present the identical port list.
4. **Register-space (CR/ID) access is first-class**, routed through the same native interface, not a
   side channel.

---

## 1. Module hierarchy

```
hyperram_axi  (top: AXI4 slave  + HyperBus device pins)
hyperram_avalon (top: Avalon-MM slave + HyperBus device pins)
        │
        │  each top instantiates exactly three blocks:
        ▼
  ┌──────────────┐    native cmd/wr/rd    ┌──────────────┐   DDR-parallel PHY IF   ┌──────────────┐
  │ hyperbus_axi │ ─────────────────────► │ hyperbus_ctrl│ ──────────────────────► │ hyperbus_phy │ ═══► device pins
  │      or      │ ◄───────────────────── │  (protocol)  │ ◄────────────────────── │  (DDR SERDES)│ ◄═══
  │hyperbus_aval.│      read-data          └──────────────┘      recovered read      └──────────────┘
  └──────────────┘                                                                   (generic | intel | xilinx)
   (front-end / bus adapter)                                              hyperram_model  ⇦ sim only, wired
                                                                          to the same device pins in a TB
```

Only these seven modules have **frozen** public ports (`INTERFACES.md`). Sub-blocks *inside*
`hyperbus_ctrl` and `hyperbus_phy` (below) are implementation detail and may change.

### Block responsibilities

| Module | Role | Vendor prims? | Verilator? |
|---|---|---|---|
| `hyperbus_pkg` | params, typedefs, CA pack/unpack, latency & wrap tables | no | yes |
| `hyperbus_ctrl` | protocol engine: CA emission, latency counting, RWDS-gated read capture, write masking, burst chopping (tCSM), POR init + CR programming. Native slave ⇄ PHY master. | **no** | yes |
| `hyperbus_phy` | DDR SERDES + I/O. Serializes CA/write words to DQ edges, generates CK, recovers RWDS-strobed read words. One port list, several variants. | generic = **no**; intel/xilinx = yes | generic + xilinx (via shim) = yes; intel = no |
| `hyperbus_avalon` | Avalon-MM slave → native. Thin. | no | yes |
| `hyperbus_axi` | AXI4 slave → native. Thin. | no | yes |
| `hyperram_axi` | top = axi front-end + ctrl + phy | depends on PHY variant | yes (generic) |
| `hyperram_avalon` | top = avalon front-end + ctrl + phy | depends on PHY variant | yes (generic) |
| `hyperram_model` | behavioral device (memory + CR/ID + latency + row-gap) for simulation | no | yes |

### Optional internal sub-blocks (NOT frozen, listed for the file plan)

Inside `hyperbus_ctrl`: `hb_init` (POR delay + CR0/CR1 write sequencer), `hb_ca_fsm` (transaction
FSM), `hb_rd_align` (word assembly from recovered DDR pairs). Inside `hyperbus_phy_generic`:
`hb_ddr_out` (2:1 out), `hb_ddr_in` (RWDS-strobed 1:2 in). These are free to change; the **module
boundaries above are the contract**.

---

## 2. Clocking plan

The controller and both front-ends are **fully synchronous to one clock** `clk` with one synchronous,
active-high reset `rst` (Hyperflex-friendly: no async reset, no gating in the datapath). The DDR bus
runs at the *same* rate as `clk`: one DATA_WIDTH (16-bit) word per `clk` cycle = two DQ bytes per cycle
on the two DDR edges. So a 100 MHz `clk` ⇒ 100 MHz HyperBus CK ⇒ 200 MB/s per the DDR bus.

Clocks crossing into the PHY:

| Clock | Rate | Consumed by | Purpose |
|---|---|---|---|
| `clk` | f (e.g. 100–200 MHz) | ctrl, front-ends, phy TX/RX domain | system + bus word rate |
| `clk90` | f, +90° phase | phy TX | center-aligns CK to DQ; source for the CK output register (SPEC_DIGEST §9 write-data center alignment) |
| `clk_ref` *(optional)* | e.g. 200 MHz | phy RX (vendor variants only) | reference for input delay / SERDES calibration; **unused by the generic PHY** |

All clocks are expected to come from one PLL/MMCM (phase-related). The **generic** PHY uses only `clk`
and `clk90`; `clk_ref` is a tie-off input so the port list is identical across variants. The
controller itself needs **only `clk`** — it never sees `clk90`/`clk_ref`.

The one true clock-domain crossing in the system is the read path: DQ is launched by the device
source-synchronous to **RWDS**, and must be recovered into the `clk` domain. **That crossing lives
entirely inside `hyperbus_phy`** (an RWDS-clocked capture + elastic hand-off to `clk`). The controller
sees only already-synchronized, `clk`-domain read words plus a `valid`. Per project convention, no
functional module hand-rolls a synchronizer; the PHY is the designated CDC boundary.

---

## 3. Native controller ⇄ front-end interface (the frozen contract)

One command channel + one write-data channel + one read-data channel, each a simple
**valid/ready** (AXI-stream-like) handshake, all in the `clk` domain. Exact widths/names in
`INTERFACES.md §hyperbus_ctrl`.

- **Command** (`cmd_*`): `valid/ready`, plus `read` (R/W#), `reg` (address space: register vs memory),
  `wrap` (burst type), `addr` (WORD address), `len` (burst length in words, ≥1). One handshake starts
  one HyperBus transaction. Register/ID access is just `reg=1, len=1`.
- **Write data** (`wr_*`): `valid/ready`, `data` (one 16-bit word), `strb` (per-byte write-enable,
  `1`=write), `last`. The controller consumes exactly `len` words. `strb` maps to the HyperBus RWDS
  byte-mask (inverted internally: RWDS High = masked). For register/zero-latency writes, `strb` must
  be all-ones (full-word only) and is not sent on the wire (SPEC_DIGEST §6).
- **Read data** (`rd_*`): `valid/ready`, `data`, `last`. The controller produces exactly `len` words,
  `last` on the final one. `rd_ready` may back-pressure; the PHY's elastic buffer absorbs slack.

**Why word-granular native, not FIFO ports:** it makes AXI and Avalon front-ends trivial (each maps its
data beat 1:1 to a native word) and keeps the controller independent of any particular FIFO IP. A
front-end that needs a wider host data bus than 16 bits instantiates its own small gearbox; that is a
front-end concern, not the controller's.

### Address-space selection

`cmd_reg` chooses memory vs register space directly. The bus adapters expose this as the **top address
bit** of their address port (`ADDR[MSB]=1 ⇒ register space`), matching the common HyperRAM-controller
convention, and drive `cmd_reg` from it. This keeps a single flat address map on AXI/Avalon while still
reaching CR0/CR1/ID0/ID1 (addresses in `hyperbus_pkg`).

---

## 4. Controller ⇄ PHY interface

DDR-parallel, single `clk` domain, unidirectional split signals (no `inout` inside the IP — a board
wrapper adds the tristate). One HyperBus word = two DQ edges is presented to the PHY as a
`2*DQ_WIDTH`-bit vector; **`[2W-1:W]` is byte A (first / rising edge), `[W-1:0]` is byte B (second /
falling edge)** — the big-endian order from SPEC_DIGEST §4. Exact ports in `INTERFACES.md §hyperbus_phy`.

Transmit (ctrl → phy):
- `phy_cs_n`, `phy_rst_n` — chip-select and device reset (registered into IOB by the PHY).
- `phy_ck_en` — run CK this cycle. The controller de-asserts it while CS# is settling and asserts it
  through CA + latency + data so the PHY emits exactly the right CK pulses (CK idles Low between).
- `phy_dq_o[2W-1:0]`, `phy_dq_oe` — DDR output word + bus output-enable (CA bytes, write data).
- `phy_rwds_o[1:0]`, `phy_rwds_oe` — DDR RWDS output (write byte-mask; two phases) + enable.

Receive (phy → ctrl):
- `phy_dq_i[2W-1:0]`, `phy_dq_i_valid` — one recovered, `clk`-synchronized read word per pulse
  (already RWDS-strobed and deskewed by the PHY). The controller just counts these into the burst.
- `phy_rwds_i` — the synchronized RWDS *level*, used by the controller **only during CA** to pick
  1×/2× variable latency (RWDS High during CA ⇒ 2×, SPEC_DIGEST §3) and to detect a stalled read
  (RWDS held Low ≥ 32 clocks ⇒ timeout/abort, SPEC_DIGEST §4/§7).
- `phy_rd_arm` — ctrl → phy strobe telling the receiver a read-data phase is beginning (so it can
  enable RWDS-clocked capture and reset its elastic buffer).

**Division of labor:** the PHY owns all DDR serialization/deserialization, CK generation, input
deskew, and the RWDS→`clk` crossing. The controller owns all protocol semantics: CA content, latency
counting, word counting, write masking, burst chopping to satisfy tCSM, and POR init. This is what
lets the *generic* PHY be a thin, inferrable, Verilator-clean DDR wrapper while the vendor PHYs swap in
`ODDR`/`IDDR`/`ALTDDIO` + input-delay calibration behind the same ports.

---

## 5. Transaction flow (reference)

1. **POR init** (`hb_init`): hold `hb_rst_n` low ≥ tRPH, wait the configured POR delay (~150 µs
   modeled as a cycle count param), then, if `PROGRAM_CR` is set, issue a register-space write of CR0
   (latency code + fixed/variable + burst config from parameters) and optionally CR1. Assert
   `init_done`. User commands are gated until `init_done`.
2. **Command accept:** on `cmd_valid & cmd_ready`, latch read/reg/wrap/addr/len; build the 48-bit CA
   with `hb_pack_ca()`.
3. **CA phase:** drop `phy_cs_n`, run CK, drive the 6 CA bytes over 3 cycles (`phy_dq_o` word pairs).
   Sample `phy_rwds_i` during CA for variable-latency doubling.
4. **Latency:** wait `latency_clocks` (×2 if variable+doubled), unless it is a zero-latency write
   (register write / `ZLW` per space) in which case data follows CA immediately.
5. **Data phase:**
   - *Read:* assert `phy_rd_arm`; for each `phy_dq_i_valid`, push a word to `rd_*`, count to `len`,
     set `rd_last` on the last. RWDS-gating in the PHY means mid-burst row/page latency gaps are
     absorbed for free.
   - *Write:* for each word from `wr_*`, drive `phy_dq_o` and `phy_rwds_o` = inverted `strb` (mask).
     Stall (mask) the bus if `wr_valid` is low mid-burst.
6. **Burst chop:** if a linear burst would hold CS# low longer than `MAX_BURST_WORDS` (= tCSM/tCK),
   close the transaction (CS# high ≥ tCSHI, respect tRWR) and re-open at the next address
   transparently — the user still sees one logical burst.
7. **Close:** raise `phy_cs_n`, hold the tCSHI/tRWR recovery gap, return to idle.

---

## 6. File / module plan

```
rtl/
  hyperbus_pkg.sv          package: params, typedefs, CA/latency/wrap functions        [DONE]
  hyperbus_ctrl.sv         protocol engine (native slave <-> PHY master)               [frozen ports]
  hyperbus_phy.sv          PHY wrapper: selects variant by parameter PHY_VARIANT        [frozen ports]
  hyperbus_phy_generic.sv  inferrable-DDR variant (sim + generic FPGA)                  [internal]
  hyperbus_phy_intel.sv    ALTDDIO/Agilex DDR-IO variant (placeholder)                  [internal]
  hyperbus_phy_xilinx.sv   7-series ODDR/IDDR/IDELAYE2 variant (simulates via primitive shim) [internal]
  hyperbus_avalon.sv       Avalon-MM slave front-end                                    [frozen ports]
  hyperbus_axi.sv          AXI4 slave front-end                                         [frozen ports]
  hyperram_avalon.sv       top: avalon + ctrl + phy                                     [frozen ports]
  hyperram_axi.sv          top: axi + ctrl + phy                                        [frozen ports]
  common/                  (internal helpers: hb_init, hb_rd_align, hb_ddr_*, cdc)      [internal]
sim/
  hyperram_model.sv        behavioral device model                                      [frozen ports]
  tb_*                     self-checking Verilator testbenches per module
docs/
  SPEC_DIGEST.md  DESIGN.md  INTERFACES.md
examples/
  (integration wrappers: board tristate + PLL, per target)
```

**Parallel-implementation rule:** every module's ports are frozen in `INTERFACES.md`. Implementers may
change *internals* freely but must not touch a frozen port name/direction/width without a documented
interface-revision note. Everything imports `hyperbus_pkg`; no magic numbers.
