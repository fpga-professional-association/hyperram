# HyperBus Bandwidth-Test Harness

`rtl/bench/hyperram_bw_test.sv` + `rtl/bench/hyperram_bw_top.sv` implement a synthesizable
read/write throughput test for the HyperBus master IP. The engine streams a **WRITE phase** and then
a **READ phase** over `LEN` words starting at `BASE_ADDR`, using linear Avalon-MM bursts of a
parameterized size (`BURST_WORDS`, default 16). It counts the `clk` cycles each phase occupies and
checks every returned read word against a deterministic pattern, so a host can read back the raw
datapath bandwidth and a pass/fail integrity result over a slow control channel (JTAG on hardware).

- `hyperram_bw_test` — the traffic generator + scoreboard. Presents an **Avalon-MM CSR slave**
  (control + readback) and drives an **Avalon-MM master** whose widths match the `hyperram_avalon`
  slave exactly (`DATA_WIDTH=16`, `ADDR_WIDTH=32` word address, `LEN_WIDTH=16` burstcount).
- `hyperram_bw_top` — structural SIM/on-chip top: `hyperram_bw_test` master → `hyperram_avalon`
  slave (generic PHY), hoisting the clocks/reset, the CSR slave, and the split HyperBus device pins.
  A testbench (or board wrapper) resolves the shared DQ/RWDS bus against `hyperram_model` /
  the real device, exactly as `sim/tb_avalon.sv` does.

Both files are single-clock, synchronous active-high reset, contain no vendor primitives, and
simulate cleanly under `verilator --binary --timing`.

## CSR map

The CSR slave has eight 32-bit registers. `csr_address` is a **word address**: register `k` lives at
host **byte offset `4*k`** (a byte-addressed JTAG-to-Avalon master uses the byte offsets below).
`csr_waitrequest` is tied low (zero wait states); reads are combinational.

| Byte off | Word | Name                  | Access | Bits / meaning |
|----------|------|-----------------------|--------|----------------|
| `0x00`   | 0    | `CTRL` (write)        | W      | bit0 = `start` (self-clearing strobe; ignored while busy) |
|          |      | `STATUS` (read)       | R      | bit0 = `busy`, bit1 = `done`, bit2 = `error` |
| `0x04`   | 1    | `LEN`                 | R/W    | number of words to test (per phase) |
| `0x08`   | 2    | `BASE_ADDR`           | R/W    | starting **word** address (keep MSB = 0 ⇒ memory space) |
| `0x0C`   | 3    | `WR_CYCLES`           | R      | `clk` cycles of the WRITE phase |
| `0x10`   | 4    | `RD_CYCLES`           | R      | `clk` cycles of the READ phase |
| `0x14`   | 5    | `ERR_COUNT`           | R      | number of read words that mismatched |
| `0x18`   | 6    | `DATA_BYTES_PER_WORD` | R      | constant `= 2` |
| `0x1C`   | 7    | `VERSION` / `MAGIC`   | R      | constant identifier (default `0x48425754` = `"HBWT"`) |

### Control / status semantics

- Write `LEN` and `BASE_ADDR` first, then pulse `CTRL.start` (`0x00 <= 0x1`). `start` is a strobe:
  it is latched only when the engine is idle and self-clears — there is nothing to write back to 0.
- While a run is in flight `STATUS.busy = 1`. On completion `STATUS.busy` drops and `STATUS.done`
  rises; `done`/`error`/the cycle and error counters remain readable until the next `start`, which
  clears them and re-arms. `STATUS.error` (and `ERR_COUNT > 0`) latches if any read word mismatched.
- `LEN = 0` completes immediately with zero cycle counts and no error.

### Data pattern

Each word carries `gen_pattern(word_addr)` — an xorshift of the word address folded to 16 bits. It is
a pure function of the address, so the READ phase recomputes the expected value per word with no
stored expectation memory. `BASE_ADDR` seeds the sequence, so different base addresses exercise
different data.

## Cycle counting and the bandwidth formula

- `WR_CYCLES` counts every `clk` from the cycle the **first write command is asserted** on the Avalon
  master through the cycle the **final write beat is accepted** (`!waitrequest`). Inter-burst bubbles
  are included; the single leading command-setup cycle is excluded.
- `RD_CYCLES` counts every `clk` from the cycle the **first read command is asserted** through the
  cycle the **final read word is returned** (`readdatavalid`). Read latency and inter-burst bubbles
  are included.

The host computes bandwidth off-chip from a known `clk` frequency `f_clk`:

```
seconds   = cycles / f_clk
bytes     = LEN * DATA_BYTES_PER_WORD          # DATA_BYTES_PER_WORD = 2
MB_per_s  = bytes / seconds / 1e6
          = (LEN * DATA_BYTES_PER_WORD * f_clk) / (cycles * 1e6)
```

Report WRITE and READ bandwidth separately using `WR_CYCLES` and `RD_CYCLES`. Because the counters
measure the Avalon accept/return window (not the JTAG access time), the result reflects the true
on-chip datapath throughput and is independent of the slow control channel.

## Running it in simulation

`hyperram_bw_top` is driven by a testbench that instantiates it alongside `hyperram_model`, resolves
the shared split DQ/RWDS bus (single active driver at a time — see `sim/tb_avalon.sv` for the exact
wiring), and then, over the CSR slave: waits for `init_done`, writes `LEN` and `BASE_ADDR`, pulses
`CTRL.start`, polls `STATUS.done`, and reads back `WR_CYCLES`/`RD_CYCLES`/`ERR_COUNT`. A run with
`ERR_COUNT == 0`, `STATUS.error == 0`, and non-zero cycle counts is a pass.

Defaults on `hyperram_bw_top` mirror `sim/tb_avalon.sv` (`PHY_VARIANT="GENERIC"`,
`LATENCY_CLOCKS=6`, `FIXED_LATENCY=1`, `INIT_CR0=0x8F1F` so the programmed latency code `0001` = 6
clocks matches the controller and the model), so read-back data matches out of the box.

Build/run follows the same pattern as `sim/run.sh`:

```
verilator --binary --timing -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM \
  -Wno-PINCONNECTEMPTY -Wno-TIMESCALEMOD -Wno-INITIALDLY -Wno-fatal \
  -I rtl -I rtl/if -I rtl/phy -I rtl/bench \
  --top-module <tb> --Mdir build/<tb> -o <tb> \
  rtl/hyperbus_pkg.sv rtl/hyperbus_ctrl.sv rtl/if/hyperbus_avalon.sv rtl/if/hyperbus_axi.sv \
  rtl/phy/hyperbus_phy_generic.sv rtl/phy/hyperbus_phy_altera.sv rtl/phy/hyperbus_phy_xilinx.sv \
  rtl/phy/hyperbus_phy.sv rtl/hyperram_avalon.sv rtl/hyperram_axi.sv \
  rtl/bench/hyperram_bw_test.sv rtl/bench/hyperram_bw_top.sv sim/model/hyperram_model.sv <tb>.sv
```

## Reuse on hardware

The same RTL is the on-chip bandwidth measurement. On the AXC3000 board the CSR slave is driven by an
**Altera JTAG-to-Avalon-MM master bridge**: the host issues Avalon writes/reads over JTAG to program
`LEN`/`BASE_ADDR`, pulse `CTRL.start`, poll `STATUS`, and read the cycle/error counters. JTAG is used
**only as the control plane** — the measured `WR_CYCLES`/`RD_CYCLES` cover the on-chip Avalon
datapath and never the JTAG access time, so the reported MB/s is the real HyperBus throughput. The
board wrapper replaces the generic PHY with the Agilex-3 PHY variant and adds the DQ/RWDS IOBUF
tristate around the split `hb_*_o/oe/i` device pins.
