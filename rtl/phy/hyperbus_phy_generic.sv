// hyperbus_phy_generic — GENERIC inferrable DDR I/O PHY for the HyperBus master IP.
//
// Implements the `hyperbus_phy` frozen port contract (docs/INTERFACES.md) with NO vendor
// primitives, using plain clocked registers and behavioural DDR muxes so the whole design
// simulates under `verilator --binary` (5.020) and infers on any FPGA.
//
// Role (docs/DESIGN.md §4): the controller speaks an SDR, clk-domain interface where one HyperBus
// 16-bit word is presented per clk cycle as two DQ bytes (byte A = high half = first/CK-rising edge,
// byte B = low half = second/CK-falling edge, per SPEC_DIGEST §4). This PHY:
//
//   TX  * registers cs#/rst#/output-enables into the pin domain (uniform 1-clk pipeline so cs#,
//         CA/data and CK stay mutually aligned),
//       * serialises the DDR DQ / RWDS word onto the pins (behavioural ODDR: byte A during clk-high,
//         byte B during clk-low),
//       * generates the HyperBus clock from clk90 so its edges land in the CENTRE of each DQ byte
//         eye (SPEC_DIGEST §4 centre-aligned write/CA data), gated by phy_ck_en (CK idles Low).
//
//   RX  * recovers read data with an RWDS-clocked DDR capture (SPEC_DIGEST §4: read data is edge-
//         aligned to the slave's source-synchronous RWDS strobe; byte A after a rising edge, byte B
//         after a falling edge) and hands it across the RWDS→clk clock-domain boundary through a
//         small gray-coded elastic FIFO — the single true CDC of the system lives here (DESIGN §2),
//       * synchronises the RWDS level into clk for the controller's CA latency-select / stall watch.
//
// The RWDS-domain receiver is held cleared whenever a read is not armed (phy_rd_arm low): this is the
// designated CDC-boundary reset and is the only place an async clear appears — the synthesised
// controller/front-end datapath remains synchronous-reset / Hyperflex-clean per DESIGN §2.
//
// clk90 is used (TX CK centring); clk_ref is unused by the generic variant (tie-off, kept so all PHY
// variants share one port list). Board tristate lives outside the IP (split hb_dq_o/oe/i pins).

`timescale 1ns/1ps

/* verilator lint_off UNUSEDPARAM */
module hyperbus_phy_generic
  import hyperbus_pkg::*;
#(
    parameter int unsigned DQ_WIDTH    = HB_DQ_WIDTH_DEFAULT,   // HyperBus DQ pins
    parameter int unsigned DATA_WIDTH  = 2 * DQ_WIDTH,          // native word width (= PHYW)
    parameter int unsigned ADDR_WIDTH  = HB_ADDR_WIDTH,         // (unused here; contract parameter)
    parameter int unsigned LEN_WIDTH   = HB_LEN_WIDTH_DEFAULT,  // (unused here; contract parameter)
    parameter              PHY_VARIANT = "GENERIC",             // (this file is the GENERIC variant)
    parameter bit          DIFF_CK     = 1'b1,                  // 1: drive hb_ck_n = ~hb_ck; 0: tie High
    // Behavioural model of the read-strobe eye-centring delay (a ~90-degree / quarter-bit shift of the
    // slave-driven RWDS). In real silicon this is an IDELAY/DLL primitive in the vendor PHY variants;
    // the GENERIC variant models it with a delay element so read capture is source-synchronous to RWDS
    // and therefore TOLERANT of the DQ/RWDS round-trip flight delay (unlike a fixed local-clock phase).
    // Units follow the module timescale (ns); default 2.5 ns ~= tCK/4 at 100 MHz.
    parameter realtime     RX_STROBE_DELAY = 2.5
) (
    input  logic                clk,       // system + bus word clock
    input  logic                clk90,     // 90-degree shifted clk; centres CK on the DQ eye
    input  logic                clk_ref,   // delay/SERDES reference (vendor variants only); tie-off here
    input  logic                rst,       // synchronous, active high

    // ---- ctrl-facing (slave; mirror of hyperbus_ctrl TX/RX) ----
    input  logic                phy_cs_n,
    input  logic                phy_rst_n,
    input  logic                phy_ck_en,
    input  logic [2*DQ_WIDTH-1:0] phy_dq_o,   // DDR out word: [hi]=byte A (1st edge), [lo]=byte B
    input  logic                phy_dq_oe,
    input  logic [1:0]          phy_rwds_o,   // DDR RWDS out (write mask): [1]=1st phase, [0]=2nd
    input  logic                phy_rwds_oe,
    input  logic                phy_rd_arm,   // arm receiver for a read-data phase
    output logic [2*DQ_WIDTH-1:0] phy_dq_i,   // recovered, clk-synchronised read word (byte A hi half)
    output logic                phy_dq_i_valid,
    output logic                phy_rwds_i,   // synchronised RWDS level to ctrl

    // ---- device pins (split; board wrapper adds tristate) ----
    output logic                hb_ck,
    output logic                hb_ck_n,
    output logic                hb_cs_n,
    output logic                hb_rst_n,
    output logic [DQ_WIDTH-1:0] hb_dq_o,
    output logic                hb_dq_oe,     // 1 = master drives DQ
    input  logic [DQ_WIDTH-1:0] hb_dq_i,
    output logic                hb_rwds_o,
    output logic                hb_rwds_oe,
    input  logic                hb_rwds_i
);
  /* verilator lint_on UNUSEDPARAM */

  localparam int unsigned PHYW = 2 * DQ_WIDTH;  // PHY parallel width (one HyperBus word)

  // ------------------------------------------------------------------
  //  TX : chip-select / reset / output-enable pipeline
  //  One uniform clk of latency on every launched signal keeps cs#, CA/data bytes and CK aligned.
  //  Architectural state (enables, cs#, rst#, CK enable) gets the synchronous reset; the pure DDR
  //  data pipeline registers below are reset-less per the Hyperflex discipline.
  // ------------------------------------------------------------------
  logic ck_en_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      hb_cs_n    <= 1'b1;   // idle: chip deselected
      hb_rst_n   <= 1'b0;   // hold device in reset while core is reset
      hb_dq_oe   <= 1'b0;
      hb_rwds_oe <= 1'b0;
      ck_en_q    <= 1'b0;
    end else begin
      hb_cs_n    <= phy_cs_n;
      hb_rst_n   <= phy_rst_n;
      hb_dq_oe   <= phy_dq_oe;
      hb_rwds_oe <= phy_rwds_oe;
      ck_en_q    <= phy_ck_en;
    end
  end

  // ------------------------------------------------------------------
  //  TX : behavioural DDR serialisers (reset-less datapath pipeline)
  //  Both halves of controller word N are latched on the SAME clk rising edge and held for pin-cycle
  //  N+1; a clk-level mux then presents byte A during the clk-high phase and byte B during the clk-low
  //  phase of that one cycle. Registering both halves on one edge (rather than a split posedge/negedge
  //  ODDR) keeps byte A and byte B of the same word in the SAME pin cycle and aligned with the uniform
  //  1-clk cs#/oe/CK pipeline above.
  // ------------------------------------------------------------------
  logic [DQ_WIDTH-1:0] dq_a_r, dq_b_r;      // DQ DDR halves (byte A / byte B)
  logic                rwds_a_r, rwds_b_r;  // RWDS DDR halves (write byte-mask phases)

  always_ff @(posedge clk) begin
    dq_a_r   <= phy_dq_o[PHYW-1:DQ_WIDTH];  // byte A (1st / clk-high)
    dq_b_r   <= phy_dq_o[DQ_WIDTH-1:0];     // byte B (2nd / clk-low)
    rwds_a_r <= phy_rwds_o[1];              // 1st phase mask
    rwds_b_r <= phy_rwds_o[0];              // 2nd phase mask
  end

  assign hb_dq_o   = clk ? dq_a_r   : dq_b_r;    // behavioural DDR mux
  assign hb_rwds_o = clk ? rwds_a_r : rwds_b_r;

  // ------------------------------------------------------------------
  //  TX : CK generation from clk90 (centre-aligned to the DQ eye)
  //  clk90 lags clk by 90 deg, so its rising edge lands mid byte-A window (clk-High) and its falling
  //  edge mid byte-B window (clk-Low). Gate clk90 directly with the clk-domain enable ck_en_q: because
  //  ck_en_q only changes on a clk rising edge — a moment when clk90 is Low — enabling/disabling never
  //  chops clk90 mid-pulse, so CK is exactly one centred pulse per enabled cycle with NO runt and no
  //  extra trailing edge, and idles Low (SPEC_DIGEST §1). (An earlier clk90-domain resample of the
  //  enable produced a spurious trailing half-pulse; gating the level directly is glitch-free here.)
  // ------------------------------------------------------------------
  assign hb_ck   = ck_en_q ? clk90 : 1'b0;
  assign hb_ck_n = DIFF_CK ? ~hb_ck : 1'b1;    // idle CK Low => idle CK# High

  // ------------------------------------------------------------------
  //  RX : RWDS-STROBED read capture + gray-coded elastic FIFO  (the one true CDC, DESIGN §2)
  //  Read data is source-synchronous and edge-aligned to the slave-driven RWDS strobe (SPEC_DIGEST §4):
  //  byte A follows an RWDS rising edge, byte B an RWDS falling edge. Because DQ and RWDS are launched
  //  together by the device and travel back with the SAME round-trip flight delay, the ONLY delay-
  //  tolerant way to sample DQ is off RWDS itself, phase-shifted ~90 degrees so its edges land in the
  //  CENTRE of each DQ byte eye. A fixed local-clock phase (the previous scheme) only works at zero
  //  round-trip delay and corrupts/mis-pairs bytes on real hardware. Here `rwds_dly` is RWDS delayed
  //  by RX_STROBE_DELAY (the behavioural stand-in for the vendor IDELAY/DLL); byte A is latched on its
  //  rising edge, byte B on its falling edge, and the assembled 16-bit word is written into a small
  //  FIFO in the RWDS-strobe domain. The read side, in the clk domain, drains it via a gray-coded
  //  pointer 2-flop-synchronised across the boundary and presents one clk-synchronous word + valid per
  //  entry to the controller. Capture is gated by phy_rd_arm (High only across read latency + data,
  //  when RWDS is a real strobe); during CA the slave-driven RWDS latency indicator is NOT captured.
  //  Reset of the strobe-domain write pointer is the single designated async clear (DESIGN §2).
  // ------------------------------------------------------------------
  localparam int unsigned RXF_DEPTH = 8;
  localparam int unsigned RXF_AW    = $clog2(RXF_DEPTH);

  // Eye-centring: RWDS delayed ~90 degrees (quarter bit). Behavioural; vendor variants use a primitive.
  wire rwds_dly;
  assign #(RX_STROBE_DELAY) rwds_dly = hb_rwds_i;

  // FIFO storage + pointers. Write pointer lives in the RWDS-strobe domain; read pointer in clk.
  logic [PHYW-1:0]     rxf_mem [RXF_DEPTH];
  logic [DQ_WIDTH-1:0] rx_byte_a;                 // byte A, held between strobe edges
  logic [RXF_AW:0]     wptr_bin;                  // strobe-domain binary write pointer (extra MSB)
  logic [RXF_AW:0]     rptr_bin;                  // clk-domain binary read pointer

  // Byte A on the delayed-strobe RISING edge (mid byte-A eye).
  always_ff @(posedge rwds_dly) begin
    if (phy_rd_arm) rx_byte_a <= hb_dq_i;
  end

  // Byte B on the delayed-strobe FALLING edge; assemble {A,B} and push. The write pointer takes the
  // ONE designated async clear of the system (DESIGN §2: the CDC boundary is the only place an async
  // reset appears); `rst` is otherwise synchronous everywhere, so the intentional sync+async use of
  // `rst` here is expected (waive SYNCASYNCNET locally).
  /* verilator lint_off SYNCASYNCNET */
  always_ff @(negedge rwds_dly or posedge rst) begin
    if (rst) begin
      wptr_bin <= '0;
    end else if (phy_rd_arm) begin
      rxf_mem[wptr_bin[RXF_AW-1:0]] <= {rx_byte_a, hb_dq_i};  // {byte A (hi), byte B (lo)}
      wptr_bin                       <= wptr_bin + 1'b1;
    end
  end
  /* verilator lint_on SYNCASYNCNET */

  // Gray-code the write pointer and 2-flop-synchronise it into the clk domain.
  wire  [RXF_AW:0] wptr_gray = wptr_bin ^ (wptr_bin >> 1);
  logic [RXF_AW:0] wgray_s1, wgray_s2;
  always_ff @(posedge clk) begin
    if (rst) begin
      wgray_s1 <= '0;
      wgray_s2 <= '0;
    end else begin
      wgray_s1 <= wptr_gray;
      wgray_s2 <= wgray_s1;
    end
  end

  // Convert the synchronised gray write pointer back to binary for the empty test.
  function automatic logic [RXF_AW:0] gray2bin(input logic [RXF_AW:0] g);
    logic [RXF_AW:0] b;
    for (int i = RXF_AW; i >= 0; i--)
      b[i] = (i == RXF_AW) ? g[RXF_AW] : (b[i+1] ^ g[i]);
    return b;
  endfunction
  wire [RXF_AW:0] wptr_bin_s = gray2bin(wgray_s2);
  wire            rxf_empty  = (rptr_bin == wptr_bin_s);

  // Read side (clk domain): one recovered word + valid per FIFO entry.
  always_ff @(posedge clk) begin
    if (rst) begin
      rptr_bin       <= '0;
      phy_dq_i       <= '0;
      phy_dq_i_valid <= 1'b0;
    end else begin
      phy_dq_i_valid <= 1'b0;
      if (!rxf_empty) begin
        phy_dq_i       <= rxf_mem[rptr_bin[RXF_AW-1:0]];
        phy_dq_i_valid <= 1'b1;
        rptr_bin       <= rptr_bin + 1'b1;
      end
    end
  end

  // ------------------------------------------------------------------
  //  RX : RWDS level synchroniser (clk domain)
  //  Two-flop sync of the raw RWDS pin level for the controller's CA latency-select (High during CA
  //  => 2x latency) and read-stall detection (SPEC_DIGEST §3/§4).
  // ------------------------------------------------------------------
  logic rwds_s1;
  always_ff @(posedge clk) begin
    if (rst) begin
      rwds_s1    <= 1'b0;
      phy_rwds_i <= 1'b0;
    end else begin
      rwds_s1    <= hb_rwds_i;
      phy_rwds_i <= rwds_s1;
    end
  end

  // clk_ref / ADDR_WIDTH / LEN_WIDTH / PHY_VARIANT are contract-only for the generic variant.
  logic _unused_ok;
  /* verilator lint_off UNUSEDSIGNAL */
  assign _unused_ok = &{1'b0, clk_ref};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
