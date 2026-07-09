# HyperBus Master IP — FROZEN Interface Contract

This file is **normative for module boundaries**. Parallel implementers MUST NOT change any port
name, direction, or width below without a documented interface-revision note in this file. Internals
are free. All modules `import hyperbus_pkg::*` and use no magic numbers. Clocking/semantics:
`DESIGN.md`. Protocol: `SPEC_DIGEST.md`.

Legend: dir `I`=input, `O`=output. Widths use the module parameters defined at the top of each section.
All interfaces are synchronous to `clk`, reset by synchronous active-high `rst`, unless noted.

Shared handshake rule (all `*_valid`/`*_ready` channels): **transfer happens on the cycle where both
`valid` and `ready` are high**; `valid` must not depend combinationally on `ready`.

---

## Common parameters

Every module below declares these (defaults from `hyperbus_pkg`):

| Parameter | Default | Meaning |
|---|---|---|
| `DQ_WIDTH` | 8 | HyperBus DQ pins |
| `DATA_WIDTH` | 16 | native word = `2*DQ_WIDTH` (one HyperBus word) |
| `ADDR_WIDTH` | 32 | word-address width |
| `LEN_WIDTH` | 16 | burst-length counter width (words) |

`STRB_WIDTH` is always `DATA_WIDTH/8` (= 2). `PHYW` (PHY parallel width) is always `2*DQ_WIDTH` (= 16).

---

## `hyperbus_ctrl` — protocol engine (native slave ⇄ PHY master)

Additional parameters: `LATENCY_CLOCKS` (default `HB_LATENCY_CLOCKS_DEFAULT`=6),
`FIXED_LATENCY` (default 1), `MAX_BURST_WORDS` (default 0 = no chop; else tCSM/tCK),
`PROGRAM_CR` (default 1 = program CR0 at init), `POR_DELAY_CYCLES` (default 0 in sim),
`INIT_LATENCY_CODE` / `INIT_CR0` (CR image written at init).

| Port | Dir | Width | Notes |
|---|---|---|---|
| `clk` | I | 1 | system + bus word clock |
| `rst` | I | 1 | synchronous, active high |
| **— native command channel (slave) —** | | | |
| `cmd_valid` | I | 1 | |
| `cmd_ready` | O | 1 | |
| `cmd_read` | I | 1 | 1 = read, 0 = write (CA[47]) |
| `cmd_reg` | I | 1 | 1 = register space, 0 = memory (CA[46]) |
| `cmd_wrap` | I | 1 | 1 = wrapped burst, 0 = linear (CA[45] = ~wrap) |
| `cmd_addr` | I | `ADDR_WIDTH` | WORD address |
| `cmd_len` | I | `LEN_WIDTH` | burst length in words, ≥1 |
| **— native write-data channel (slave) —** | | | |
| `wr_valid` | I | 1 | |
| `wr_ready` | O | 1 | |
| `wr_data` | I | `DATA_WIDTH` | one word; byte A = `[DATA_WIDTH-1:DQ_WIDTH]` |
| `wr_strb` | I | `STRB_WIDTH` | per-byte write-enable, 1 = write (inverted to RWDS mask on wire) |
| `wr_last` | I | 1 | asserted with final word of the burst |
| **— native read-data channel (master) —** | | | |
| `rd_valid` | O | 1 | |
| `rd_ready` | I | 1 | back-pressure allowed |
| `rd_data` | O | `DATA_WIDTH` | one word; byte A = `[DATA_WIDTH-1:DQ_WIDTH]` |
| `rd_last` | O | 1 | asserted with final word of the burst |
| **— status —** | | | |
| `busy` | O | 1 | transaction in progress |
| `init_done` | O | 1 | POR init + CR programming complete; commands gated until high |
| `err_underrun` | O | 1 | write data not delivered in time (pulse) |
| `err_timeout` | O | 1 | read RWDS stalled ≥ 32 clks (pulse) |
| **— PHY interface (master) — TX —** | | | |
| `phy_cs_n` | O | 1 | chip select, active low |
| `phy_rst_n` | O | 1 | device reset, active low |
| `phy_ck_en` | O | 1 | run CK this cycle |
| `phy_dq_o` | O | `PHYW` | DDR out word; `[PHYW-1:DQ_WIDTH]`=byte A (1st edge), `[DQ_WIDTH-1:0]`=byte B |
| `phy_dq_oe` | O | 1 | DQ bus output enable |
| `phy_rwds_o` | O | 2 | DDR RWDS out (write mask); `[1]`=1st phase, `[0]`=2nd |
| `phy_rwds_oe` | O | 1 | RWDS output enable |
| `phy_rd_arm` | O | 1 | arm PHY receiver for a read-data phase |
| **— PHY interface (master) — RX —** | | | |
| `phy_dq_i` | I | `PHYW` | recovered, clk-synchronized read word (byte A in high half) |
| `phy_dq_i_valid` | I | 1 | one pulse per recovered read word |
| `phy_rwds_i` | I | 1 | synchronized RWDS level (CA latency-select + stall detect) |

---

## `hyperbus_phy` — DDR SERDES + I/O (ctrl-facing slave ⇄ device pins)

Parameters: common + `PHY_VARIANT` (string: `"GENERIC"` | `"SDR"` | `"INTEL"` | `"XILINX"`, default
`"GENERIC"`), `DIFF_CK` (default 1: drive `hb_ck_n`; 0 = single-ended, `hb_ck_n` tied). All variants
share this exact port list.

**`"SDR"` variant clock note (does not change the port list):** the portable single-clock-phase SDR
PHY (`hyperbus_phy_sdr.sv`, for FPGAs/fits where two clock phases cannot reach the I/O periphery)
**repurposes the `clk90` port as a 2× byte-serialisation clock** (same PLL, 0°) rather than a
90°-shifted `clk`. `clk` stays the CK-rate word clock (drives the controller); `clk90` = 2×`clk`
drives the SDR output/capture registers and generates `hb_ck = clk90/2`. Port names/directions/widths
are unchanged, so the frozen contract holds; only this variant's interpretation of `clk90` differs.

| Port | Dir | Width | Notes |
|---|---|---|---|
| `clk` | I | 1 | bus word clock |
| `clk90` | I | 1 | 90°-shifted clk; CK/write-data center alignment |
| `clk_ref` | I | 1 | delay/SERDES reference (vendor variants); tie-off for GENERIC |
| `rst` | I | 1 | synchronous, active high |
| **— ctrl-facing (slave, mirror of ctrl TX/RX) —** | | | |
| `phy_cs_n` | I | 1 | |
| `phy_rst_n` | I | 1 | |
| `phy_ck_en` | I | 1 | |
| `phy_dq_o` | I | `PHYW` | DDR out word from ctrl |
| `phy_dq_oe` | I | 1 | |
| `phy_rwds_o` | I | 2 | |
| `phy_rwds_oe` | I | 1 | |
| `phy_rd_arm` | I | 1 | |
| `phy_dq_i` | O | `PHYW` | recovered read word to ctrl |
| `phy_dq_i_valid` | O | 1 | |
| `phy_rwds_i` | O | 1 | synchronized RWDS level to ctrl |
| **— device pins (split; board wrapper adds tristate) —** | | | |
| `hb_ck` | O | 1 | HyperBus clock |
| `hb_ck_n` | O | 1 | complementary clock (valid only if `DIFF_CK`) |
| `hb_cs_n` | O | 1 | chip select |
| `hb_rst_n` | O | 1 | device reset |
| `hb_dq_o` | O | `DQ_WIDTH` | DQ output value |
| `hb_dq_oe` | O | 1 | DQ output enable (1 = master drives) |
| `hb_dq_i` | I | `DQ_WIDTH` | DQ input value |
| `hb_rwds_o` | O | 1 | RWDS output value |
| `hb_rwds_oe` | O | 1 | RWDS output enable |
| `hb_rwds_i` | I | 1 | RWDS input value |

Board integration (outside IP): `assign hb_dq = hb_dq_oe ? hb_dq_o : 'z; assign hb_dq_i = hb_dq;`
(and likewise RWDS) in the top wrapper / IOBUF instantiation.

---

## `hyperbus_avalon` — Avalon-MM slave → native master

Parameters: common. Avalon address is a WORD address; `avs_address[ADDR_WIDTH-1]` selects register
space (drives `cmd_reg`). Bursts are linear (`cmd_wrap=0`).

| Port | Dir | Width | Notes |
|---|---|---|---|
| `clk` | I | 1 | |
| `rst` | I | 1 | |
| **— Avalon-MM slave —** | | | |
| `avs_address` | I | `ADDR_WIDTH` | word address; MSB = register-space select |
| `avs_read` | I | 1 | |
| `avs_write` | I | 1 | |
| `avs_writedata` | I | `DATA_WIDTH` | |
| `avs_byteenable` | I | `STRB_WIDTH` | |
| `avs_burstcount` | I | `LEN_WIDTH` | words in burst |
| `avs_readdata` | O | `DATA_WIDTH` | |
| `avs_readdatavalid` | O | 1 | |
| `avs_waitrequest` | O | 1 | |
| **— native master (to `hyperbus_ctrl`) —** | | | |
| `cmd_valid` | O | 1 | |
| `cmd_ready` | I | 1 | |
| `cmd_read` | O | 1 | |
| `cmd_reg` | O | 1 | |
| `cmd_wrap` | O | 1 | tied 0 (linear) |
| `cmd_addr` | O | `ADDR_WIDTH` | |
| `cmd_len` | O | `LEN_WIDTH` | |
| `wr_valid` | O | 1 | |
| `wr_ready` | I | 1 | |
| `wr_data` | O | `DATA_WIDTH` | |
| `wr_strb` | O | `STRB_WIDTH` | |
| `wr_last` | O | 1 | |
| `rd_valid` | I | 1 | |
| `rd_ready` | O | 1 | |
| `rd_data` | I | `DATA_WIDTH` | |
| `rd_last` | I | 1 | |

---

## `hyperbus_axi` — AXI4 slave → native master

Parameters: common + `ID_WIDTH` (default 4), `AXI_DATA_WIDTH` (default `DATA_WIDTH`=16; if wider, the
front-end includes a gearbox), `AXI_ADDR_WIDTH` (default `ADDR_WIDTH+1`, byte address; low bit(s)
index the word, MSB = register-space select). AXI beats map 1:1 to native words when
`AXI_DATA_WIDTH==16`. Standard AXI4 signal semantics; only widths/names are frozen here.

| Port | Dir | Width | Notes |
|---|---|---|---|
| `clk` | I | 1 | `aclk` |
| `rst` | I | 1 | active-high synchronous (invert of `aresetn` at the wrapper) |
| **— AW —** | | | |
| `awid` | I | `ID_WIDTH` | |
| `awaddr` | I | `AXI_ADDR_WIDTH` | byte address; MSB = register space |
| `awlen` | I | 8 | AXI burst length − 1 |
| `awsize` | I | 3 | |
| `awburst` | I | 2 | INCR expected (WRAP → `cmd_wrap`) |
| `awvalid` | I | 1 | |
| `awready` | O | 1 | |
| **— W —** | | | |
| `wdata` | I | `AXI_DATA_WIDTH` | |
| `wstrb` | I | `AXI_DATA_WIDTH/8` | |
| `wlast` | I | 1 | |
| `wvalid` | I | 1 | |
| `wready` | O | 1 | |
| **— B —** | | | |
| `bid` | O | `ID_WIDTH` | |
| `bresp` | O | 2 | |
| `bvalid` | O | 1 | |
| `bready` | I | 1 | |
| **— AR —** | | | |
| `arid` | I | `ID_WIDTH` | |
| `araddr` | I | `AXI_ADDR_WIDTH` | byte address; MSB = register space |
| `arlen` | I | 8 | |
| `arsize` | I | 3 | |
| `arburst` | I | 2 | |
| `arvalid` | I | 1 | |
| `arready` | O | 1 | |
| **— R —** | | | |
| `rid` | O | `ID_WIDTH` | |
| `rdata` | O | `AXI_DATA_WIDTH` | |
| `rresp` | O | 2 | |
| `rlast` | O | 1 | |
| `rvalid` | O | 1 | |
| `rready` | I | 1 | |
| **— native master (to `hyperbus_ctrl`) —** | | | identical to the native master block in `hyperbus_avalon` |
| `cmd_valid`/`cmd_ready`/`cmd_read`/`cmd_reg`/`cmd_wrap`/`cmd_addr`/`cmd_len` | O/I/O… | — | as `hyperbus_ctrl` native command channel. `cmd_wrap` is tied 0: AXI WRAP/FIXED are reproduced as linear native segments in the front-end (see below). |
| `wr_valid`/`wr_ready`/`wr_data`/`wr_strb`/`wr_last` | O/I/O/O/O | — | as native write-data channel |
| `rd_valid`/`rd_ready`/`rd_data`/`rd_last` | I/O/I/I | — | as native read-data channel |
| **— controller error status (v2) —** | | | |
| `err_underrun` | I | 1 | from `hyperbus_ctrl`; latches a sticky SLVERR on the write response |
| `err_timeout` | I | 1 | from `hyperbus_ctrl`; latches a sticky SLVERR on the read response |

**AXI burst-type handling (v2).** The device wraps only at its CR0[1:0] group and has no repeat-address
mode, so the front-end does **not** forward AXI burst geometry to the device. It decomposes each burst
into linear native segments that reproduce AXI order exactly: **INCR** → one segment; **WRAP** → two
segments (start→region-top, region-base→start-1), region from `(AxLEN+1)*2^AxSIZE`, so any boundary and
WRAP2/WRAP4 work regardless of the device wrap group; **FIXED** → `N` single-word segments at the same
address. A narrow `AxSIZE` (≠ log2(DATA_WIDTH/8)) is accepted but flagged **SLVERR**. `RRESP/BRESP` are
SLVERR whenever a controller error (`err_timeout`/`err_underrun`) occurred in the transaction. AXI-facing
ready/valid outputs are held Low during reset (AXI4 A3.1.2).

---

## `hyperram_axi` — TOP (AXI4 slave + HyperBus device pins)

Parameters: union of `hyperbus_axi`, `hyperbus_ctrl`, `hyperbus_phy` (incl. `PHY_VARIANT`, `DIFF_CK`,
`LATENCY_CLOCKS`, `FIXED_LATENCY`, `MAX_BURST_WORDS`, `PROGRAM_CR`, `POR_DELAY_CYCLES`). Instantiates
`hyperbus_axi` + `hyperbus_ctrl` + `hyperbus_phy`.

Ports = { clocking } ∪ { full AXI4 slave bus of `hyperbus_axi` } ∪ { device pins of `hyperbus_phy` }:

| Port | Dir | Width | Notes |
|---|---|---|---|
| `clk` | I | 1 | |
| `clk90` | I | 1 | to PHY |
| `clk_ref` | I | 1 | to PHY (tie for GENERIC) |
| `rst` | I | 1 | |
| *AXI4 slave* | | | every `aw*/w*/b*/ar*/r*` port from `hyperbus_axi`, same names/widths |
| *device pins* | | | `hb_ck, hb_ck_n, hb_cs_n, hb_rst_n, hb_dq_o, hb_dq_oe, hb_dq_i, hb_rwds_o, hb_rwds_oe, hb_rwds_i` from `hyperbus_phy` |
| `init_done` | O | 1 | from ctrl |

---

## `hyperram_avalon` — TOP (Avalon-MM slave + HyperBus device pins)

Same structure as `hyperram_axi` but with the Avalon-MM slave of `hyperbus_avalon`.

| Port | Dir | Width | Notes |
|---|---|---|---|
| `clk` | I | 1 | |
| `clk90` | I | 1 | |
| `clk_ref` | I | 1 | |
| `rst` | I | 1 | |
| *Avalon-MM slave* | | | every `avs_*` port from `hyperbus_avalon`, same names/widths |
| *device pins* | | | same 10 device-pin ports as `hyperram_axi` |
| `init_done` | O | 1 | |
| `err_underrun` | O | 1 | pulse: controller write-data underrun (v4). Avalon has no SLVERR channel, so the controller's `err_underrun` is surfaced here as a top-level status strobe |

---

## `hyperram_model` — behavioral device model (simulation only)

Parameters: `DQ_WIDTH` (8), `MEM_WORDS` (e.g. 1<<16), `LATENCY_CLOCKS` (6), `FIXED_LATENCY` (1),
`ROW_WORDS` / `ROW_PENALTY` (mid-burst row-crossing gap), plus reset CR/ID images
(`HB_ID0_RESET` etc. from the package). Split-driver pins (Verilator-safe; the TB resolves the shared
bus, no `inout` inside the model). The model receives the master's driven bus and exposes its own.

| Port | Dir | Width | Notes |
|---|---|---|---|
| `hb_ck` | I | 1 | clock from master |
| `hb_ck_n` | I | 1 | complementary (may be tied if single-ended) |
| `hb_cs_n` | I | 1 | chip select |
| `hb_rst_n` | I | 1 | device reset |
| `hb_dq_i` | I | `DQ_WIDTH` | DQ driven BY MASTER (CA / write data) |
| `hb_dq_ie` | I | 1 | master DQ output-enable (bus turnaround awareness) |
| `hb_dq_o` | O | `DQ_WIDTH` | DQ driven BY MODEL (read data) |
| `hb_dq_oe` | O | 1 | model DQ output-enable |
| `hb_rwds_i` | I | 1 | RWDS driven BY MASTER (write mask) |
| `hb_rwds_ie` | I | 1 | master RWDS output-enable |
| `hb_rwds_o` | O | 1 | RWDS driven BY MODEL (CA latency indicator + read strobe) |
| `hb_rwds_oe` | O | 1 | model RWDS output-enable |

Testbench bus resolution (outside the model): `hb_dq_i(model) = hb_dq_o(phy)`,
`hb_dq_ie(model)=hb_dq_oe(phy)`; `hb_dq_i(phy)=hb_dq_oe(phy)?hb_dq_o(phy):hb_dq_o(model)` (single
active driver at a time, enforced by the protocol). RWDS analogous.

---

## Interface-revision log

- **v1 (frozen, 2026-07-07):** initial freeze of all seven module boundaries + native/PHY interfaces.
- **v2 (2026-07-07):** `hyperbus_axi` gains two inputs — `err_underrun`, `err_timeout` — wired from the
  existing `hyperbus_ctrl` error outputs through the `hyperram_axi` top, so the AXI front-end can map
  controller-detected errors to `RRESP`/`BRESP` = SLVERR (spec-conformance fix). No other port
  name/direction/width changed; `hyperbus_ctrl` and `hyperbus_phy` boundaries are unchanged. The
  `hyperbus_phy_generic` variant additionally exposes a non-frozen behavioural-only parameter
  `RX_STROBE_DELAY` (models the read-strobe eye-centring delay; realised by a primitive in the vendor
  PHY variants) — internal, defaulted, not part of the frozen port list.
- **v3 (2026-07-08):** new `hyperbus_phy` variant `PHY_VARIANT="SDR"` (`hyperbus_phy_sdr.sv`) — a
  portable, single-clock-phase, normal-I/O (no DDR/no primitives) PHY that unblocks the AXC3000 fit
  (Quartus 24403/24404: two IOPLL phases could not reach the Bank-3A I/O). **No frozen port name,
  direction, or width changes.** The SDR variant reinterprets the existing `clk90` input as a 2×`clk`
  byte clock (0°, same PLL) instead of a 90° phase — see the "SDR variant clock note" above. It adds
  a non-frozen, defaulted read-eye tuning parameter `CAPTURE_PHASE`. Controller, front-ends, model,
  and all other module boundaries are unchanged.
- **v4 (2026-07-09):** the `RD_PREAMBLE_SKIP` read-strobe-preamble parameter (added on `hyperbus_phy`
  at v3, then implemented only by the `SDR` variant) now also reaches the `GENERIC` PHY variant
  (`hyperbus_phy_generic.sv` — read-path preamble phantom-word skip, disarm FIFO flush, deeper RX
  FIFO) and is exposed on the `hyperram_axi` top, joining `hyperram_avalon` which already forwarded
  it. **Parameter-only change: no frozen port name, direction, or width changes.** `RD_PREAMBLE_SKIP`
  keeps its name/meaning and its default `0`, which is bit-exact for every existing caller (the skip
  branch never fires). `ALTERA`/`XILINX` variants still do not implement it — the wrapper simply never
  forwards it to their branches, as before. Controller, front-ends, model, and all other module
  boundaries are unchanged.
- **v5 (2026-07-09):** `hyperram_avalon` gains one output — `err_underrun` — wired straight from the
  existing `hyperbus_ctrl.err_underrun` pulse (previously left unconnected). The AXI top folds this
  error into `BRESP=SLVERR`, but Avalon-MM has no response channel, so an Avalon integrator otherwise
  had no visibility of a controller write-data underrun (issue #4, B6). This is an **additive output**;
  existing instantiations that leave it unconnected are unaffected (unconnected outputs are legal). No
  other port name/direction/width changed; `hyperbus_ctrl`, `hyperbus_avalon`, `hyperbus_phy`, and the
  `hyperram_axi` boundary are unchanged.
- **v6 (2026-07-09):** `PHY_VARIANT="XILINX"` (`hyperbus_phy_xilinx.sv`) is filled in as a real
  AMD/Xilinx 7-series DDR datapath (`ODDR`/`IDDR`/`IDELAYE2`/`IDELAYCTRL`/`BUFIO`/`BUFR`/`OBUF`/
  `OBUFDS`) that now **simulates** under Verilator through a compile-guarded shim
  (`sim/model/xilinx_prims_sim.sv`, `` `ifdef VERILATOR ``) driven by `sim/tb_xilinx.sv`. **No frozen
  port name, direction, or width changes.** It adds three **non-frozen, defaulted** parameters, so every
  existing instantiation is bit-identical: `RX_STROBE_DLY_TAPS` (IDELAYE2 FIXED tap), `RX_PAIR_SKEW`
  (IDDR byte-pairing escape hatch), and `RD_PREAMBLE_SKIP` (read-strobe preamble skip, mirroring the SDR
  variant). `RD_PREAMBLE_SKIP` was already a wrapper (`hyperbus_phy`) parameter (v3); it is now also
  threaded to the `g_xilinx` branch. `RX_STROBE_DLY_TAPS`/`RX_PAIR_SKEW` stay internal to the variant
  (the wrapper never overrides them, exactly as the Altera variant's `RX_STROBE_DLY_TAPS`/`RX_PAIR_SKEW`).
  Controller, front-ends, model, and all other module boundaries are unchanged.
