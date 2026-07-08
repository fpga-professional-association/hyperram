# Integration Guide

How to drop the HyperBus / HyperRAM controller into a design: pick a top, connect
the clocks and reset, wire the split HyperBus pins to real bidirectional pads, and
constrain the interface. Module boundaries are frozen in
[`INTERFACES.md`](INTERFACES.md); architecture and clocking are in
[`DESIGN.md`](DESIGN.md).

> **Scope note.** The RTL is protocol-complete and simulation-validated with the
> **generic** PHY. It ships **no** device/board `.sdc` timing constraints — those
> are target-specific hardware work (see [`PHY_PORTING.md`](PHY_PORTING.md) and the
> `.sdc` section below).

---

## 1. Choose a top

Both tops instantiate exactly `front-end + hyperbus_ctrl + hyperbus_phy` and expose
their host bus, the clocks, the split HyperBus pins, and `init_done`.

| Top | Host interface |
|---|---|
| `hyperram_axi` | AXI4 slave |
| `hyperram_avalon` | Avalon-MM slave |

Pick by your fabric. If you need a different host bus, write a thin front-end that
targets the native command/write/read valid-ready interface of `hyperbus_ctrl`
(`INTERFACES.md §hyperbus_ctrl`) — do not add protocol logic there.

---

## 2. Clocking & reset

The controller and front-ends are fully synchronous to a single clock `clk`, with
one synchronous, active-high reset `rst`.

| Clock | Rate | Used by | Notes |
|---|---|---|---|
| `clk` | f (e.g. 100–200 MHz) | ctrl, front-ends, PHY TX/RX | system + bus **word** rate; 1 word = 2 DQ edges/cycle |
| `clk90` | f, +90° | PHY TX | centers `hb_ck` on the DQ eye; generic PHY uses it |
| `clk_ref` | e.g. 200 MHz | PHY RX (vendor variants) | delay/SERDES reference; **tie to `clk` for GENERIC** |

Generate `clk` and `clk90` from one PLL/MMCM so they are phase-related. The bus CK
runs at the `clk` rate: 100 MHz `clk` ⇒ 100 MHz `hb_ck` ⇒ 200 MB/s on a x8 DDR bus.

Reset: drive `rst = ~aresetn` (AXI) at the wrapper. Hold `rst` until the PLL is
locked. On hardware, set `POR_DELAY_CYCLES` to cover the device tVCS (~150 µs);
in simulation it defaults to 0.

```systemverilog
// rst generation example
logic rst;
always_ff @(posedge clk) rst <= ~(pll_locked & aresetn);
```

---

## 3. Instantiating `hyperram_axi`

```systemverilog
hyperram_axi #(
    // bus geometry (defaults shown)
    .DQ_WIDTH        (8),
    .DATA_WIDTH      (16),
    .ADDR_WIDTH      (32),
    .LEN_WIDTH       (16),
    // AXI front-end
    .ID_WIDTH        (4),
    .AXI_DATA_WIDTH  (16),          // 16 => AXI beat maps 1:1 to a HyperBus word
    .AXI_ADDR_WIDTH  (33),          // byte address; MSB = register-space select
    // controller
    .LATENCY_CLOCKS  (6),           // from the device datasheet at your CK rate
    .FIXED_LATENCY   (1),
    .MAX_BURST_WORDS (0),           // set = tCSM/tCK on hardware to bound CS# Low
    .PROGRAM_CR      (1),
    .POR_DELAY_CYCLES(15000),       // ~150 us at 100 MHz on hardware (0 in sim)
    .INIT_CR0        (16'h0008),
    // PHY
    .PHY_VARIANT     ("GENERIC"),   // "INTEL" | "XILINX" on a real board
    .DIFF_CK         (1'b1)         // 1.8 V I/O => differential CK
) u_hyperram (
    .clk (clk), .clk90 (clk90), .clk_ref (clk), .rst (rst),
    // AXI4 slave — connect your interconnect
    .awid(awid), .awaddr(awaddr), .awlen(awlen), .awsize(awsize),
    .awburst(awburst), .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
    .bid(bid), .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .arid(arid), .araddr(araddr), .arlen(arlen), .arsize(arsize),
    .arburst(arburst), .arvalid(arvalid), .arready(arready),
    .rid(rid), .rdata(rdata), .rresp(rresp), .rlast(rlast),
    .rvalid(rvalid), .rready(rready),
    // HyperBus device pins (split; tristate added below)
    .hb_ck(hb_ck), .hb_ck_n(hb_ck_n), .hb_cs_n(hb_cs_n), .hb_rst_n(hb_rst_n),
    .hb_dq_o(hb_dq_o), .hb_dq_oe(hb_dq_oe), .hb_dq_i(hb_dq_i),
    .hb_rwds_o(hb_rwds_o), .hb_rwds_oe(hb_rwds_oe), .hb_rwds_i(hb_rwds_i),
    .init_done(init_done)
);
```

**Address map.** `awaddr`/`araddr` are **byte** addresses; the MSB selects register
space (CR0/CR1/ID0/ID1), the rest index the 16-bit word. Memory space is the flat
low half; anything with the top address bit set goes to config/ID registers.

**Errors.** `bresp`/`rresp` = `SLVERR` (`2'b10`) when the controller reports a
read RWDS timeout or a write underrun, or on a narrow (`AxSIZE ≠ log2(DATA_WIDTH/8)`)
transfer. Otherwise `OKAY`.

## 3b. Instantiating `hyperram_avalon`

Same clocks/PHY/controller parameters, with the Avalon-MM slave in place of AXI:

```systemverilog
hyperram_avalon #( /* same params, no AXI_* / ID_WIDTH */ ) u_hyperram (
    .clk(clk), .clk90(clk90), .clk_ref(clk), .rst(rst),
    .avs_address(avs_address),          // word address; MSB = register space
    .avs_read(avs_read), .avs_write(avs_write),
    .avs_writedata(avs_writedata), .avs_byteenable(avs_byteenable),
    .avs_burstcount(avs_burstcount),    // words per burst (linear)
    .avs_readdata(avs_readdata), .avs_readdatavalid(avs_readdatavalid),
    .avs_waitrequest(avs_waitrequest),
    .hb_ck(hb_ck), .hb_ck_n(hb_ck_n), .hb_cs_n(hb_cs_n), .hb_rst_n(hb_rst_n),
    .hb_dq_o(hb_dq_o), .hb_dq_oe(hb_dq_oe), .hb_dq_i(hb_dq_i),
    .hb_rwds_o(hb_rwds_o), .hb_rwds_oe(hb_rwds_oe), .hb_rwds_i(hb_rwds_i),
    .init_done(init_done)
);
```

---

## 4. Connecting the HyperBus pins (tristate)

The IP keeps **no `inout` inside it** — DQ and RWDS are exposed as split
`_o` / `_oe` / `_i` triples so the design stays Verilator-clean and vendor-neutral.
The board wrapper adds the bidirectional pads. There are two common styles:

### Inferred tristate (portable)

```systemverilog
// Bidirectional DQ / RWDS pads
assign HB_DQ   = hb_dq_oe   ? hb_dq_o   : {DQ_WIDTH{1'bz}};
assign hb_dq_i = HB_DQ;
assign HB_RWDS   = hb_rwds_oe ? hb_rwds_o : 1'bz;
assign hb_rwds_i = HB_RWDS;

// Unidirectional master-driven pins
assign HB_CK    = hb_ck;
assign HB_CK_N  = hb_ck_n;   // leave unconnected / ignore if single-ended (DIFF_CK=0)
assign HB_CS_N  = hb_cs_n;
assign HB_RST_N = hb_rst_n;
```

### Explicit IOBUF (vendor)

- **AMD/Xilinx:** `IOBUF` (or `IOBUFDS` for differential CK) per DQ bit and RWDS,
  with `.T(~hb_dq_oe)`, `.I(hb_dq_o[i])`, `.O(hb_dq_i[i])`, `.IO(HB_DQ[i])`.
- **Intel/Altera:** the tri-state is part of the `altera_gpio` / DDIO-IO
  instantiation done in the vendor PHY (see [`PHY_PORTING.md`](PHY_PORTING.md)); the
  external pad connects directly.

Only one side drives DQ/RWDS at a time; the protocol (and the golden model)
enforce the turnaround, so a simple `oe`-muxed tristate is correct.

### Differential vs single-ended CK

`DIFF_CK = 1` (1.8 V I/O) drives `hb_ck_n = ~hb_ck`; wire both to a differential
pad pair. `DIFF_CK = 0` (3.0 V I/O) uses `hb_ck` only — leave `hb_ck_n` unconnected.

---

## 5. `.sdc` / constraints notes

The interface is **source-synchronous DDR**; timing closure is device- and
board-specific and must be done against the target device datasheet. The RTL does
**not** ship a closed `.sdc`. Key items your constraints file must cover:

1. **Clocks.** `create_clock` on the input reference; `create_generated_clock` for
   `clk90` (90° of `clk`) from the PLL/MMCM. Declare `hb_ck` as a generated clock so
   read capture is analyzed against it.
2. **Output (write/CA) paths — center-aligned.** `hb_dq_o` / `hb_rwds_o` are DDR,
   launched to be centered on `hb_ck` (that is the purpose of `clk90`). Constrain
   `set_output_delay -max/-min` (rise and `-clock_fall`) for both DDR edges relative
   to `hb_ck`, using the device tIS/tIH (Table 9.5).
3. **Input (read) paths — edge-aligned to RWDS.** Read DQ is source-synchronous to
   the device-driven **RWDS** strobe, *not* to `hb_ck`. On real hardware you delay
   RWDS ~90° (IDELAY / DPA / PLL phase — a PHY primitive) so it samples the DQ eye
   center, then treat RWDS as the capture clock. Constrain `set_input_delay` against
   the RWDS-derived capture clock, or use the vendor's DDR-input/DQS-group flow. The
   generic PHY models this delay with `RX_STROBE_DELAY` (behavioral, sim only) — a
   real board replaces it with the primitive.
4. **CDC.** The RWDS→`clk` elastic hand-off lives inside `hyperbus_phy_generic`
   (gray-coded FIFO) / the vendor input FIFO. Apply `set_false_path` /
   `set_max_delay -datapath_only` across that boundary per your vendor's CDC guidance;
   do not let STA try to close RWDS-domain flops against `clk` combinationally.
5. **CS# / reset.** `hb_cs_n` and `hb_rst_n` are registered outputs on `clk`;
   constrain with the same `set_output_delay` group. Ensure `MAX_BURST_WORDS` is set
   so CS# never stays Low past the device tCSM.

Put one `.sdc` per clock architecture under a project `constraints/` directory and
comment each exception. Timing numbers (tIS/tIH/tACC/tRWR/…) come from
[`SPEC_DIGEST.md §10`](SPEC_DIGEST.md) and, for the exact device, its datasheet.

---

## 6. Bring-up checklist

1. Simulate your wrapper against `sim/model/hyperram_model.sv` first (adapt a
   `tb_*` — swap in your front-end wiring). Confirm `init_done` rises and a
   write/read-back matches.
2. On hardware, start with a conservative `LATENCY_CLOCKS` and a low CK rate; read
   ID0/ID1 (register space) and check the manufacturer/device nibble before trusting
   the memory array.
3. Swap `PHY_VARIANT` to your vendor variant and fill in the primitives + RWDS
   strobe delay ([`PHY_PORTING.md`](PHY_PORTING.md)). Sweep the input-delay tap to
   center the read eye. Then close timing with the `.sdc` above.
