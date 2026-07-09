// xilinx_prims_sim — VERILATOR-ONLY behavioural shim for the AMD/Xilinx 7-series I/O primitives
// instantiated by rtl/phy/hyperbus_phy_xilinx.sv (ODDR / IDDR / IDELAYE2 / IDELAYCTRL / OBUF /
// OBUFDS / BUFIO / BUFR).
//
// WHY THIS FILE EXISTS
//   The real primitives live only in the Vivado `unisim` simulation/synthesis library and cannot be
//   compiled by Verilator. These behavioural stand-ins let `bash sim/run.sh` exercise the REAL
//   hyperbus_phy_xilinx datapath (same RTL that synthesises) end-to-end under `verilator --binary
//   --timing`, against both the ideal and the non-ideal (read-preamble + over-stream) device model.
//
// COMPILE GUARD (cannot collide with the real library)
//   The whole file is wrapped in `ifdef VERILATOR`. Verilator predefines `VERILATOR`, so these modules
//   exist ONLY for Verilator. Under Vivado (no `VERILATOR`) this file is empty and the genuine `unisim`
//   ODDR/IDDR/... are elaborated instead — hyperbus_phy_xilinx.sv is unchanged between the two flows.
//
//   Build wiring: this file is in sim/run.sh's XILINX_SIM_SRCS (tb_xilinx only), NOT COMMON_SRCS. Every
//   other testbench compiles hyperbus_phy_xilinx.sv but never selects PHY_VARIANT="XILINX", so its
//   primitive instances sit in an elaboration-dead generate branch (hyperbus_phy.sv g_xilinx) that
//   the simulator prunes — those TBs need no primitive definitions at all.
//
// FIDELITY / CAVEAT
//   These are FUNCTIONAL stand-ins (which byte lands where, and a representative fixed input delay), not
//   AC-timing or PVT models. Exact port/attribute names are reconstructed from the 7-series libraries
//   guide; run xvlog/xelab against the real unisim to catch any misremembered pin before synthesis.
//   The IDELAYE2 tap→delay scale here is a behavioural convenience (see TAP_NS); on silicon a tap is
//   ~78 ps at a 200 MHz REFCLK and the correct value is a hardware read-eye sweep result.
//
// UltraScale/UltraScale+ note (documented in hyperbus_phy_xilinx.sv): swap ODDR→ODDRE1, IDDR→IDDRE1,
// IDELAYE2→IDELAYE3 (+ODELAY/BITSLICE), BUFR→BUFGCE_DIV. Out of scope here (7-series only).

`ifdef VERILATOR
`timescale 1ns/1ps

/* verilator lint_off DECLFILENAME */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */

// ======================================================================================
//  ODDR — output DDR register (7-series). DDR_CLK_EDGE="SAME_EDGE": D1 and D2 are both sampled
//  off the SAME rising edge of C; D1 is launched on the C-high sub-phase, D2 on the C-low sub-
//  phase. The output is REGISTERED (Q changes only AT C edges), not a combinational C-mux — a
//  mux `Q = C ? d1 : d2` glitches when the sampled data changes on the same edge that flips the
//  mux select (harmless for DQ data, but a spurious CK EDGE for the clk90-clocked CK ODDR whose
//  enable de-asserts). Registering Q reproduces the real primitive's glitch-free DDR waveform.
// ======================================================================================
module ODDR #(
    parameter         DDR_CLK_EDGE = "SAME_EDGE",   // "SAME_EDGE" | "OPPOSITE_EDGE"
    parameter [0:0]   INIT         = 1'b0,
    parameter         SRTYPE       = "SYNC"
) (
    output logic Q,
    input  logic C,
    input  logic CE,
    input  logic D1,
    input  logic D2,
    input  logic R,
    input  logic S
);
  // Q is a DDR output register launched on BOTH edges of C. Driven from a SINGLE dual-edge block (one
  // driver → no MULTIDRIVEN, unlike a posedge+negedge pair) so the registered, glitch-free waveform
  // propagates cleanly to the pin net. SAME_EDGE: D1 and D2 are both sampled at the rising edge; D1 is
  // launched on the C-High sub-phase, the captured D2 on the C-Low sub-phase.
  logic d2_hold;
  always @(posedge C or negedge C) begin
    if (R) begin
      Q       <= INIT;
      d2_hold <= INIT;
    end else if (C) begin          // rising edge: launch byte A, capture byte B
      if (CE) begin
        Q       <= D1;
        d2_hold <= D2;
      end
    end else begin                 // falling edge: launch the captured byte B
      if (CE) Q <= d2_hold;
    end
  end
  wire _u = &{1'b0, S};
endmodule

// ======================================================================================
//  IDDR — input DDR register (7-series). Q1 = data captured on the C RISING edge, Q2 = data
//  captured on the C FALLING edge. (The real DDR_CLK_EDGE="SAME_EDGE_PIPELINED" additionally
//  retimes both onto the rising edge one cycle later; this functional stand-in exposes the two
//  half-samples directly so the PHY's byte-pairing FSM can consume Q1 on the OPPOSITE (falling)
//  edge — race-free and with no trailing-edge dependency, see hyperbus_phy_xilinx.sv RX notes.)
// ======================================================================================
module IDDR #(
    parameter       DDR_CLK_EDGE = "SAME_EDGE_PIPELINED",
    parameter [0:0] INIT_Q1      = 1'b0,
    parameter [0:0] INIT_Q2      = 1'b0,
    parameter       SRTYPE       = "SYNC"
) (
    output logic Q1,   // rising-edge sample  (HyperBus byte A)
    output logic Q2,   // falling-edge sample (HyperBus byte B)
    input  logic C,
    input  logic CE,
    input  logic D,
    input  logic R,
    input  logic S
);
  always_ff @(posedge C) begin
    if (R)       Q1 <= INIT_Q1;
    else if (CE) Q1 <= D;
  end
  always_ff @(negedge C) begin
    if (R)       Q2 <= INIT_Q2;
    else if (CE) Q2 <= D;
  end
  wire _u = &{1'b0, S};
endmodule

// ======================================================================================
//  IDELAYE2 — input delay line. FIXED mode: a static delay of IDELAY_VALUE taps on IDATAIN.
//  Behavioural: assign #(taps*TAP_NS) DATAOUT = IDATAIN (the same delayed-continuous-assign
//  technique proven at hyperbus_phy_generic.sv:162 under verilator --binary --timing). The tap
//  scale is a sim convenience so the default tap centres the read eye at the tb CK rate; on
//  silicon a tap is ~78 ps @200 MHz REFCLK and the value is a hardware sweep result.
// ======================================================================================
module IDELAYE2 #(
    parameter         IDELAY_TYPE           = "FIXED",
    parameter integer IDELAY_VALUE          = 0,
    parameter         HIGH_PERFORMANCE_MODE = "TRUE",
    parameter real    REFCLK_FREQUENCY      = 200.0,
    parameter         DELAY_SRC             = "IDATAIN",
    parameter         CINVCTRL_SEL          = "FALSE",
    parameter         PIPE_SEL              = "FALSE",
    parameter         SIGNAL_PATTERN        = "DATA"
) (
    output logic       DATAOUT,
    output logic [4:0] CNTVALUEOUT,
    input  logic       C,
    input  logic       CE,
    input  logic       INC,
    input  logic       LD,
    input  logic       LDPIPEEN,
    input  logic       CINVCTRL,
    input  logic       REGRST,
    input  logic       IDATAIN,
    input  logic       DATAIN,
    input  logic [4:0] CNTVALUEIN
);
  localparam real TAP_NS = 0.15625;                 // behavioural ns/tap (16 taps => 2.5 ns)
  localparam real DLY_NS = IDELAY_VALUE * TAP_NS;    // compile-time-constant delay
  assign #(DLY_NS) DATAOUT = IDATAIN;
  assign CNTVALUEOUT = 5'(IDELAY_VALUE);
  wire _u = &{1'b0, C, CE, INC, LD, LDPIPEEN, CINVCTRL, REGRST, DATAIN, CNTVALUEIN};
endmodule

// ======================================================================================
//  IDELAYCTRL — calibrates the IDELAY tap reference. RDY deasserts under RST and reasserts one
//  REFCLK edge later. Functionally inert for these fixed taps; instantiated for parity with the
//  real design (one per I/O bank group).
// ======================================================================================
module IDELAYCTRL (
    output logic RDY,
    input  logic REFCLK,
    input  logic RST
);
  always_ff @(posedge REFCLK or posedge RST) begin
    if (RST) RDY <= 1'b0;
    else     RDY <= 1'b1;
  end
endmodule

// ======================================================================================
//  OBUF / OBUFDS — output buffers. OBUF: O=I. OBUFDS: true/complementary pair (O=I, OB=~I).
// ======================================================================================
module OBUF #(
    parameter      DRIVE     = 12,
    parameter      IOSTANDARD = "DEFAULT",
    parameter      SLEW      = "SLOW"
) (
    output logic O,
    input  logic I
);
  assign O = I;
endmodule

module OBUFDS #(
    parameter      IOSTANDARD = "DEFAULT",
    parameter      SLEW      = "SLOW"
) (
    output logic O,
    output logic OB,
    input  logic I
);
  assign O  = I;
  assign OB = ~I;
endmodule

// ======================================================================================
//  BUFIO / BUFR — clock-capable buffers for the delayed RWDS strobe. BUFIO drives the I/O-column
//  clock (the IDDRs); BUFR drives the regional clock (the byte-pairing FSM / FIFO write side).
//  Both are pass-throughs functionally; the split exists so silicon P&R can reach both loads.
// ======================================================================================
module BUFIO (
    output logic O,
    input  logic I
);
  assign O = I;
endmodule

module BUFR #(
    parameter      BUFR_DIVIDE = "BYPASS",
    parameter      SIM_DEVICE  = "7SERIES"
) (
    output logic O,
    input  logic I,
    input  logic CE,
    input  logic CLR
);
  assign O = I;   // BYPASS: regional-clock pass-through
  wire _u = &{1'b0, CE, CLR};
endmodule

/* verilator lint_on UNUSEDPARAM */
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on DECLFILENAME */
`endif
