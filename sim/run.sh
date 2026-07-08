#!/usr/bin/env bash
# run.sh — build + run the HyperBus IP self-checking Verilator testbenches.
#
# One verilator --binary build+run per testbench (tb_avalon, tb_axi). Exits non-zero on any
# build, elaboration, or simulation failure (a TB signals failure with $fatal -> non-zero exit
# and prints "TB_RESULT: FAIL"; success prints "TB_RESULT: PASS" and $finish -> exit 0).
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RTL="$ROOT/rtl"
SIM="$ROOT/sim"
BUILD="$SIM/build"

# Source order: package first, then all RTL, PHY variants, model, TB.
COMMON_SRCS=(
  "$RTL/hyperbus_pkg.sv"
  "$RTL/hyperbus_ctrl.sv"
  "$RTL/if/hyperbus_avalon.sv"
  "$RTL/if/hyperbus_axi.sv"
  "$RTL/phy/hyperbus_phy_generic.sv"
  "$RTL/phy/hyperbus_phy_sdr.sv"
  "$RTL/phy/hyperbus_phy_altera.sv"
  "$RTL/phy/hyperbus_phy_xilinx.sv"
  "$RTL/phy/hyperbus_phy.sv"
  "$RTL/hyperram_avalon.sv"
  "$RTL/hyperram_axi.sv"
  "$SIM/model/hyperram_model.sv"
)

# Bandwidth-test harness RTL — only needed by tb_bw (bench engine + sim/board top).
BENCH_SRCS=(
  "$RTL/bench/hyperram_bw_test.sv"
  "$RTL/bench/hyperram_bw_top.sv"
)

# -Wall as required; a handful of benign lint classes are waived (vendor-PHY skeleton tie-offs,
# testbench-only unused status/ID signals, timescale-on-some-modules, empty status pin taps).
VFLAGS=(--binary --timing -Wall
        -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-PINCONNECTEMPTY
        -Wno-TIMESCALEMOD -Wno-INITIALDLY -Wno-fatal
        -I"$RTL" -I"$RTL/if" -I"$RTL/phy" -j 0)

overall=0

run_one() {
  local tb="$1" top="$2"
  shift 2
  local extra_srcs=("$@")       # optional extra RTL sources (e.g. bench harness for tb_bw)
  echo "=================================================================="
  echo "== Building $top"
  echo "=================================================================="
  local odir="$BUILD/$top"
  rm -rf "$odir"
  mkdir -p "$odir"
  if ! verilator "${VFLAGS[@]}" --top-module "$top" --Mdir "$odir" -o "$top" \
        "${COMMON_SRCS[@]}" "${extra_srcs[@]}" "$SIM/$tb" > "$odir/build.log" 2>&1; then
    echo "-- build FAILED; log follows --"
    cat "$odir/build.log"
    echo "TB_RESULT: FAIL ($top build error)"
    overall=1
    return
  fi
  echo "-- build ok"
  echo "== Running $top"
  if ! "$odir/$top"; then
    echo "TB_RESULT: FAIL ($top simulation error / non-zero exit)"
    overall=1
    return
  fi
}

run_one tb_avalon.sv   tb_avalon
run_one tb_sdr.sv      tb_sdr
run_one tb_axi.sv      tb_axi
run_one tb_fixed2x.sv  tb_fixed2x
run_one tb_timeout.sv  tb_timeout
run_one tb_preamble.sv tb_preamble
run_one tb_bw.sv       tb_bw         "${BENCH_SRCS[@]}"
run_one tb_multiburst.sv tb_multiburst "${BENCH_SRCS[@]}"

echo "=================================================================="
if [ "$overall" -eq 0 ]; then
  echo "ALL TESTBENCHES PASSED"
else
  echo "ONE OR MORE TESTBENCHES FAILED"
fi
exit "$overall"
