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
`BURST_BOUNDARY_WORDS` (default 0 = off; else a linear segment is chopped so it never crosses this
WORD-aligned boundary — the W957D8NB bus-release quirk), `WR_COMMIT_READ` (default 0 = off; when 1,
after each split memory-write segment the controller self-issues an internal commit-read spanning the
last written word — **documented-ineffective for write→write streams since v9/2026-07-10 silicon**;
kept for read-terminated traffic), `COMMIT_READ_WORDS` / `COMMIT_READ_MODE`
(`"SPAN_END"`|`"FULL_BURST"`|`"NEXT_ROW"`, shape of that internal read), `WR_COALESCE` (default 0;
when 1, a contiguous linear memory-write command arriving within `WR_COALESCE_WAIT` cycles of a
completing write is spliced onto the SAME open CS# burst — no CS# boundary at all), `WR_CHOP_REPLAY`
(default 0; v9 — when 1, every intra-command write chop reopens `min(WR_REPLAY_WORDS, words sent)`
words early and re-sends them from an internal shadow, re-writing the pending tail the device drops
at the chop; `WR_REPLAY_WORDS` default 4 = the W957D8NB pending depth), `PROGRAM_CR` (default 1 =
program CR0 at init), `POR_DELAY_CYCLES` (default 0 in sim), `INIT_LATENCY_CODE` / `INIT_CR0` (CR
image written at init). The device-quirk parameters default OFF so existing instantiations are
bit-identical; they are threaded (defaulted) through `hyperram_avalon` / `hyperram_axi` /
`hyperram_bw_top` (the coalesce/commit-shape/replay set through `hyperram_avalon`, and
replay additionally through `hyperram_axi`).

Spec-feature options (v8, all defaulted OFF = legacy behavior — no port changes):
`PROGRAM_CR1` (default 0) / `INIT_CR1` (default `HB_CR1_RESET`) — optional second zero-latency register
write of CR1 at init, after CR0 (§8.2). `CLK_FREQ_MHZ` (default 0 = legacy fixed counts) with
`T_RP_NS`/`T_RPH_NS`/`T_RH_NS` (200/400/200) and `T_VCS_US` (150) — POR/reset AC-timing derived as
`cycles = ceil(t/tCK)` when `CLK_FREQ_MHZ≠0` (else `RESET_CYCLES=8`, POR dwell = `POR_DELAY_CYCLES`;
Table 8.3). `SUPPORT_DPD` (default 0) / `TDPDOUT_CYCLES` (default 0) — Deep-Power-Down: snoop a host
CR0[15]=0 write, then guard the next command with a CS# wake pulse + tDPDOUT dwell (§5.2.1).
`ACTIVE_CLK_STOP` (default 0) — pause CK on word boundaries while the read FIFO is above its high-water
mark (caller back-pressure), instead of dropping words (§1).

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
`"GENERIC"`), `DIFF_CK` (default 1: drive `hb_ck_n`; 0 = single-ended, `hb_ck_n` tied), plus the
**read-eye POR-seed** parameters `RD_PREAMBLE_SKIP` (SDR/GENERIC, v4) and `CAPTURE_PHASE` (SDR, v9) —
each seeds the reset value of the matching runtime `cal_*` port and defaults to that variant's legacy
compile-time default. (The DDIO knobs `RX_STROBE_DLY_TAPS`/`RX_PAIR_SKEW` remain per-variant
compile-time parameters — ALTERA defaults 8/1, XILINX 16/0 — their `cal_*` ports are elaboration-only
tie-offs until the runtime paths land, #3.) All variants share this exact port list, **including the
four mandatory `cal_*` inputs** (v9) — they carry no SV default value (Verilator 5.020 rejects default
port values), so every instantiation must wire them explicitly.

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
| **— runtime read-eye calibration (mandatory, no default; quasi-static) (v9) —** | | | |
| `cal_capture_phase` | I | 1 | live read-capture edge (SDR); reset-seeds from `CAPTURE_PHASE` |
| `cal_preamble_skip` | I | `HB_CAL_PREAMBLE_SKIP_WIDTH` (=3) | live read-strobe preamble-skip (SDR); reset-seeds from `RD_PREAMBLE_SKIP` |
| `cal_rx_tap` | I | `HB_CAL_RX_TAP_WIDTH` (=5) | live RWDS eye-centre tap (DDIO variants; tie-off until #3) |
| `cal_pair_skew` | I | 1 | live byte-pairing select (DDIO variants; tie-off until #3) |
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
`LATENCY_CLOCKS`, `FIXED_LATENCY`, `MAX_BURST_WORDS`, `BURST_BOUNDARY_WORDS`, `WR_COMMIT_READ`,
`PROGRAM_CR`, `POR_DELAY_CYCLES`, and the v8 spec-feature options `PROGRAM_CR1`/`INIT_CR1`/
`CLK_FREQ_MHZ`/`T_RP_NS`/`T_RPH_NS`/`T_RH_NS`/`T_VCS_US`/`SUPPORT_DPD`/`TDPDOUT_CYCLES`/
`ACTIVE_CLK_STOP`, all defaulted through to `hyperbus_ctrl`, and the read-eye POR seeds
`RD_PREAMBLE_SKIP`/`CAPTURE_PHASE` (v9)). Instantiates `hyperbus_axi` + `hyperbus_ctrl` +
`hyperbus_phy`.

Ports = { clocking } ∪ { runtime cal } ∪ { full AXI4 slave bus of `hyperbus_axi` } ∪ { device pins }:

| Port | Dir | Width | Notes |
|---|---|---|---|
| `clk` | I | 1 | |
| `clk90` | I | 1 | to PHY |
| `clk_ref` | I | 1 | to PHY (tie for GENERIC) |
| `rst` | I | 1 | |
| `cal_capture_phase` / `cal_preamble_skip` / `cal_rx_tap` / `cal_pair_skew` (v9) | I | 1 / 3 / 5 / 1 | mandatory (no default); forwarded 1:1 to `hyperbus_phy` — tie to constants (POR-seed equivalents) or drive from a CSR |
| *AXI4 slave* | | | every `aw*/w*/b*/ar*/r*` port from `hyperbus_axi`, same names/widths |
| *device pins* | | | `hb_ck, hb_ck_n, hb_cs_n, hb_rst_n, hb_dq_o, hb_dq_oe, hb_dq_i, hb_rwds_o, hb_rwds_oe, hb_rwds_i` from `hyperbus_phy` |
| `init_done` | O | 1 | from ctrl |

---

## `hyperram_avalon` — TOP (Avalon-MM slave + HyperBus device pins)

Same structure as `hyperram_axi` but with the Avalon-MM slave of `hyperbus_avalon`. Same read-eye POR
seed parameters and mandatory `cal_*` inputs (v9).

| Port | Dir | Width | Notes |
|---|---|---|---|
| `clk` | I | 1 | |
| `clk90` | I | 1 | |
| `clk_ref` | I | 1 | |
| `rst` | I | 1 | |
| `cal_capture_phase` / `cal_preamble_skip` / `cal_rx_tap` / `cal_pair_skew` (v9) | I | 1 / 3 / 5 / 1 | mandatory (no default); forwarded 1:1 to `hyperbus_phy` |
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
- **v7 (2026-07-09):** `hyperbus_ctrl` gains two device-quirk parameters for the Winbond W957D8NB
  (AXC3000) split-multi-burst work-arounds — `BURST_BOUNDARY_WORDS` (default 0 = off; else chop a
  linear segment at that WORD boundary) and `WR_COMMIT_READ` (default 0 = off; else interpose an
  internal commit-read after each split memory-write segment). **Both default OFF, so every existing
  instantiation is bit-identical; no frozen port name, direction, or width changed.** They are
  threaded (defaulted) through `hyperram_avalon`, `hyperram_axi`, and `hyperram_bw_top`. The
  behavioural device model `hyperram_model` gains matching, defaulted, non-frozen knobs
  `WR_COMMIT_QUIRK` and `BURST_BOUNDARY_WORDS` (simulation-only); `hyperram_bw_test` adds the
  runtime CSR `REG_RBURSTW` (word 12 / 0x30) for an independent read-phase burst length. Regression:
  `sim/tb_commit.sv`.
- **v8 (2026-07-09):** unimplemented HyperBus spec features added to `hyperbus_ctrl` as **defaulted
  parameters only — no port name/direction/width changed on any module** (issue #5). New ctrl params
  (all default OFF = prior behavior): `PROGRAM_CR1`/`INIT_CR1` (A3, CR1 init write); `CLK_FREQ_MHZ` +
  `T_RP_NS`/`T_RPH_NS`/`T_RH_NS`/`T_VCS_US` (A4, ns-derived POR/reset AC-timing; `CLK_FREQ_MHZ=0`
  reproduces the legacy `RESET_CYCLES=8` / raw `POR_DELAY_CYCLES` exactly); `SUPPORT_DPD`/
  `TDPDOUT_CYCLES` (A1, Deep-Power-Down enter-detect + guarded wake); `ACTIVE_CLK_STOP` (A2, CK pause on
  read back-pressure). The two top wrappers `hyperram_axi` and `hyperram_avalon` forward all of them to
  `hyperbus_ctrl` (defaulted); the SIM model `hyperram_model` gains a defaulted `SUPPORT_DPD` (DPD
  device state) — its frozen pin list is unchanged. `hyperbus_phy`, both front-ends, and the
  FPGA-referenced `hyperram_bw_top` are untouched. Regression: `sim/tb_cr1init.sv`, `sim/tb_por_timing.sv`,
  `sim/tb_dpd.sv`, `sim/tb_clkstop.sv`.
- **v9 (2026-07-09):** `hyperbus_phy` (all four variants), `hyperram_avalon`, and `hyperram_axi` gain
  **four mandatory runtime read-eye calibration inputs** — `cal_capture_phase` (1b), `cal_preamble_skip`
  (`HB_CAL_PREAMBLE_SKIP_WIDTH`=3b), `cal_rx_tap` (`HB_CAL_RX_TAP_WIDTH`=5b), `cal_pair_skew` (1b) — so a
  host can retune the read eye (preamble-skip / capture edge / RWDS tap / byte-pairing) by a CSR write
  with **no recompile** (`REG_CAL`, word 13/0x34, in `hyperram_bw_test`; word 12 is v7's `REG_RBURSTW`).
  The formerly compile-time SDR knobs `CAPTURE_PHASE`/`RD_PREAMBLE_SKIP` are demoted to **POR reset-seed**
  values for the matching `cal_*` port (`CAPTURE_PHASE` is now also a wrapper/top parameter); every
  parameter default reproduces the prior behaviour with zero call-site VALUE changes. The ports are
  **mandatory (no SV default value)** because Verilator 5.020 hard-rejects default port values
  (`%Error-UNSUPPORTED: Default value on module input`); consequently **every** call site (all sim TBs,
  both tops, `hyperram_bw_top`, and the board `top.sv`) needs an explicit wiring edit — a forgotten one
  is only a non-fatal `%Warning-PINMISSING` that silently ties the input to 0, so a
  `grep PINMISSING sim/build/*/build.log` gate over the TB build logs guards it. The SDR variant feeds
  `cal_preamble_skip` through a reset-seeded 2-flop `clk90` synchroniser (resampled at each read disarm)
  and selects the capture edge with a reset-seeded registered mux (both sampling pipelines always live);
  the ALTERA and XILINX DDIO variants carry the ports as elaboration-only tie-offs until #3 unblocks
  their runtime paths (their `RX_STROBE_DLY_TAPS`/`RX_PAIR_SKEW` stay compile-time, per-variant).
  `hyperbus_ctrl`, the native/PHY data interfaces, and the model boundary are unchanged. Regression:
  `sim/tb_cal.sv` (live REG_CAL preamble-skip flip: ERR=LEN-1 ⇒ ERR=0 with no rebuild).
- **v10 (2026-07-09/10):** `hyperbus_ctrl` gains an EXPERIMENTAL, default-off write-boundary family —
  `WR_CHOP_REPLAY`/`WR_REPLAY_WORDS`/`WR_REPLAY_PEND`/`WR_REPLAY_ALIGN`/`WR_REPLAY_MASK_LEAD`
  (rollback replay: reopen a chopped write early, optionally row-aligned and mask-led, re-sending
  the tail from a shadow; also fires at contiguous write→write command accepts) and
  `WR_CHOP_PAUSE_CYCLES`/`WR_CHOP_PAUSE_CK` (post-write CS#-High dwell, optionally with CK
  toggling). **Defaulted parameters only; no frozen port name, direction, or width changed on any
  module.** Replay params threaded (defaulted) through `hyperram_avalon` and `hyperram_axi`. (This
  entry also records the previously-unlogged 2026-07-09 ctrl parameters `COMMIT_READ_WORDS`/
  `COMMIT_READ_MODE`/`WR_COALESCE`/`WR_COALESCE_WAIT`.)
  SILICON VERDICT (W957D8NBRA4I, 2026-07-09 read-only-probe ladder — the reason all of the above is
  default-OFF): the long-standing "write-commit quirk" story (burst tail held pending, discarded by
  the next write, committed by covering reads) is FALSE. Measured truth: (1) write-burst tails
  COMMIT to the array fine; (2) **any memory-space WRITE CS# opening at word address B wounds the
  array at [B-4, B)** — zeroed (B 8-aligned) or garbled (else) — standalone writes included, and no
  rollback (E-A), CS#-High pause (E-B), CK-toggling dwell (E-C), or RWDS-masked lead-in (E-D)
  suppresses it: rollback merely relocates the wound below the new base; (3) READ CAs do not wound;
  (4) ROW WRAP: linear bursts must NEVER cross the device's 1024-word row — writes WRAP back to the
  row start (LEN=1536 single-burst: word 0 read back gen(1024) — 512+ words aliased; the early
  "≥1024 hits tCSM" and interim "refresh starvation" readings were THIS, and the historic
  "releases the bus when a read crosses 16 KB" was this same law at coarser granularity, 0x2000
  being a row multiple); no tCSM effect was observed in range (5.9 µs bursts otherwise clean), and
  `WR_CHOP_PAUSE_CYCLES`/`_CK` turned out to have no beneficial role on this device;
  (5) END-AT-ROW GARBLE: a write burst ENDING exactly on a 1024-word row multiple garbles its own
  last 4 words (persistent, foreign values).
  OPTIMAL LEGAL CONFIGURATION (silicon-exact): row-aligned segments — `MAX_BURST_WORDS=1024` =
  `BURST_BOUNDARY_WORDS='h400` + `WR_COALESCE=1`. In-row transfers are loss-free (768/768: 341.1 W
  / 332.3 R MB/s @175 MHz, ERR=0, 25-run soak); every row TRANSITION costs exactly 4 words at
  [row·1024-4, row·1024) — the closing burst's end-garble and the next open's wound coincide
  (measured: 1024→4, 1536→4, 2048→8, 4096/256→16, stable across read-only re-probes).
  Consequences: `WR_COMMIT_READ` is documented-ineffective for write→write streams; the ONLY
  wound-free shape is chop avoidance (`WR_COALESCE` + `MAX_BURST_WORDS` ≤ the tCSM ceiling); streams
  longer than one CS# budget pay a deterministic 4-word wound at every chop point [C-4, C). The SIM
  model `hyperram_model` is rewritten (non-frozen, sim-only) from pending/discard to wound
  semantics: `WR_WOUND_WORDS` (wound on write-CA open), `WR_WOUND_MASK_SUPPRESS` (counterfactual
  knob, silicon says 0), `WR_BOUNDARY_END_GARBLE`. Regression: `sim/tb_commit.sv` rewritten against
  the wound model (no-fix / coalesce / plain-replay-relocation / mask-led / command-edge /
  boundary-end stacks). The replay/pause machinery is retained default-off: it documents the
  falsification trail in-code and may genuinely help OTHER HyperBus devices with true
  pending-discard semantics.
