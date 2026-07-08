// hyperbus_capture — on-chip logic-analyzer / capture buffer for the AXC3000 HyperBus bring-up.
//
// BOARD-DEBUG ONLY (fpga/axc3000/): snoops the SDR PHY's pin-side signals plus the bench<->IP
// Avalon handshake so a host can SEE, over the JTAG-Avalon control plane, what actually happens on
// the HyperBus wires during one transaction (why writes "complete" in constant cycles and reads
// hang — see docs/BW_TEST.md and sysconsole/bw_diag.tcl).
//
// Operation:
//   * Runs on the 100 MHz SDR byte clock (`cap_clk` = top.sv clk2x) — one sample per 10 ns.
//   * The host ARMs it over the CSR (CTRL.arm = 1). Once armed it waits for the first cycle with
//     hb_cs_n == 0 (transaction start), then records one sample per cap_clk cycle until DEPTH
//     samples are stored, then raises STATUS.done and disarms.
//   * Samples land in an inferred simple-dual-port RAM (write @ cap_clk, read @ clk -> M20K).
//
// Sample word bit fields (CAP_WIDTH = 26 significant bits, zero-padded to 64 for readback):
//   [0]     hb_cs_n            [19]  hb_rwds_oe
//   [1]     hb_ck              [20]  hb_rwds_o
//   [2]     hb_dq_oe           [21]  hb_rwds_i
//   [10:3]  hb_dq_o[7:0]       [22]  av_read
//   [18:11] hb_dq_i[7:0]       [23]  av_write
//                              [24]  av_waitrequest
//                              [25]  av_readdatavalid
//   [63:26] zero (pad)
//
// CSR slave (clk domain, word-addressed: byte offset = 4 * csr_address, waitrequest tied low,
// reads combinational — same contract as the bw_test CSR so the top-level JTAG adapter is shared):
//   0x00  CTRL (w): bit0 = arm (1 = arm and clear a previous capture; 0 = disarm)
//         STATUS (r): bit0 = armed, bit1 = done, bits[31:16] = fill count (samples stored)
//   0x04  RDADDR (w/r): sample index for readback
//   0x08  DATA_LO (r): sample[31:0]  at RDADDR
//   0x0C  DATA_HI (r): sample[63:32] at RDADDR (always 0 with CAP_WIDTH = 26)
//
// Clocking / CDC: clk (50 MHz) and cap_clk (100 MHz) are 0-deg outputs of ONE IOPLL (top.sv clock
// plan), so all crossings here are timed synchronous paths; the arm/done levels still go through
// 2-flop stages for a clean release. hb_dq_i / hb_rwds_i are raw pad inputs — they get one input
// register in the cap_clk domain (exactly how the PHY itself samples them). Debug-grade by design.

`timescale 1ns/1ps

module hyperbus_capture #(
    parameter int unsigned DEPTH = 1024            // samples per capture
) (
    input  logic        clk,          // CSR / word clock (50 MHz, JTAG-Avalon master domain)
    input  logic        rst,          // synchronous, active-high (clk domain)
    input  logic        cap_clk,      // capture clock (100 MHz SDR byte clock, top.sv clk2x)

    // ---- probes (cap_clk-domain PHY outputs + raw pad inputs + clk-domain Avalon handshake) ----
    input  logic        hb_cs_n,
    input  logic        hb_ck,
    input  logic        hb_dq_oe,
    input  logic [7:0]  hb_dq_o,
    input  logic [7:0]  hb_dq_i,
    input  logic        hb_rwds_oe,
    input  logic        hb_rwds_o,
    input  logic        hb_rwds_i,
    input  logic        av_read,
    input  logic        av_write,
    input  logic        av_waitrequest,
    input  logic        av_readdatavalid,

    // ---- CSR slave (clk domain) ----
    input  logic [1:0]  csr_address,
    input  logic        csr_read,
    output logic [31:0] csr_readdata,
    input  logic        csr_write,
    input  logic [31:0] csr_writedata,
    output logic        csr_waitrequest
);

  localparam int unsigned CAP_WIDTH = 26;            // significant sample bits (see header map)
  localparam int unsigned AW        = $clog2(DEPTH); // sample index width

  assign csr_waitrequest = 1'b0;                     // single-cycle slave, zero wait states

  // ==================================================================================================
  //  clk-domain control: arm level + read address. `armed` is set by CTRL.arm, cleared when the
  //  capture engine reports done (or by writing arm = 0).
  // ==================================================================================================
  logic          armed;
  logic [AW-1:0] rdaddr;

  // done, 2-flop synchronised from the cap_clk domain (see below)
  logic done2x;
  logic done_m, done_s, done_s_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      done_m   <= 1'b0;
      done_s   <= 1'b0;
      done_s_q <= 1'b0;
    end else begin
      done_m   <= done2x;
      done_s   <= done_m;
      done_s_q <= done_s;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      armed  <= 1'b0;
      rdaddr <= '0;
    end else begin
      // Edge-triggered auto-disarm: only the RISING edge of done clears `armed`, so a re-arm issued
      // while the previous done level is still draining through the synchronisers is not swallowed.
      if (done_s & ~done_s_q) armed <= 1'b0;
      if (csr_write) begin
        unique case (csr_address)
          2'd0:    armed  <= csr_writedata[0];         // CTRL.arm (host write wins over auto-disarm)
          2'd1:    rdaddr <= csr_writedata[AW-1:0];    // RDADDR
          default: ;
        endcase
      end
    end
  end

  // ==================================================================================================
  //  cap_clk-domain capture engine. Reset + arm level brought over with 2-flop stages.
  // ==================================================================================================
  logic rst2x_meta, rst2x;
  always_ff @(posedge cap_clk) begin
    rst2x_meta <= rst;
    rst2x      <= rst2x_meta;
  end

  logic arm_meta, arm_s, arm_q;
  always_ff @(posedge cap_clk) begin
    if (rst2x) begin
      arm_meta <= 1'b0;
      arm_s    <= 1'b0;
      arm_q    <= 1'b0;
    end else begin
      arm_meta <= armed;
      arm_s    <= arm_meta;
      arm_q    <= arm_s;
    end
  end
  wire arm_pulse = arm_s & ~arm_q;                     // host just armed: (re)start a capture

  // One input register stage: aligns every probed field onto the same cap_clk sample grid and
  // synchronises the raw hb_dq_i / hb_rwds_i pad inputs (same sampling the PHY itself performs).
  logic [CAP_WIDTH-1:0] samp_q;
  always_ff @(posedge cap_clk) begin
    samp_q <= {av_readdatavalid, av_waitrequest, av_write, av_read,   // [25:22]
               hb_rwds_i, hb_rwds_o, hb_rwds_oe,                      // [21:19]
               hb_dq_i,                                               // [18:11]
               hb_dq_o,                                               // [10:3]
               hb_dq_oe, hb_ck, hb_cs_n};                             // [2:0]
  end

  typedef enum logic [1:0] { S_IDLE, S_ARMED, S_RUN, S_DONE } state_t;
  state_t        st;
  logic [AW:0]   wptr;                                 // fill count (0..DEPTH), extra MSB
  wire           trig = ~samp_q[0];                    // hb_cs_n == 0 on the registered sample

  wire cap_we = (st == S_ARMED && trig) || (st == S_RUN);

  always_ff @(posedge cap_clk) begin
    if (rst2x) begin
      st     <= S_IDLE;
      wptr   <= '0;
      done2x <= 1'b0;
    end else if (arm_pulse) begin
      st     <= S_ARMED;                               // arm: clear any previous capture
      wptr   <= '0;
      done2x <= 1'b0;
    end else begin
      unique case (st)
        S_ARMED: begin
          if (trig)        begin st <= S_RUN; wptr <= wptr + 1'b1; end   // first sample stored
          else if (!arm_s)       st <= S_IDLE;                            // host disarmed
        end
        S_RUN: begin
          wptr <= wptr + 1'b1;
          if (wptr == (AW+1)'(DEPTH - 1)) begin
            st     <= S_DONE;
            done2x <= 1'b1;
          end
        end
        default: ;                                     // S_IDLE / S_DONE: wait for arm_pulse
      endcase
    end
  end

  // ==================================================================================================
  //  Sample store: inferred simple-dual-port RAM, write @ cap_clk / read @ clk (M20K).
  // ==================================================================================================
  (* ramstyle = "M20K" *) logic [CAP_WIDTH-1:0] mem [DEPTH];

  always_ff @(posedge cap_clk) begin
    if (cap_we) mem[wptr[AW-1:0]] <= samp_q;
  end

  // Registered read port in the clk domain. RDADDR is programmed by one JTAG transaction and the
  // data read by a later one (many clk cycles apart), so the 1-cycle read latency is invisible to
  // the combinational CSR readback below.
  logic [CAP_WIDTH-1:0] ram_q;
  always_ff @(posedge clk) begin
    ram_q <= mem[rdaddr];
  end

  // ==================================================================================================
  //  CSR readback (clk domain, combinational — mirrors the bw_test CSR contract).
  // ==================================================================================================
  // Fill count into the clk domain. clk/cap_clk are same-PLL synchronous, so this registered copy
  // is a timed path; it is exact once done, monotonic (debug-grade) while running.
  logic [AW:0] fill_m, fill_s;
  always_ff @(posedge clk) begin
    fill_m <= wptr;
    fill_s <= fill_m;
  end

  wire [63:0] sample64 = 64'(ram_q);

  always_comb begin
    unique case (csr_address)
      2'd0:    csr_readdata = {16'(fill_s), 14'd0, done_s, armed};   // STATUS
      2'd1:    csr_readdata = 32'(rdaddr);                           // RDADDR
      2'd2:    csr_readdata = sample64[31:0];                        // DATA_LO
      default: csr_readdata = sample64[63:32];                       // DATA_HI (0x0C)
    endcase
  end

  // csr_read is part of the shared CSR contract but unused here (reads have no side effects).
  logic _unused_ok;
  /* verilator lint_off UNUSEDSIGNAL */
  assign _unused_ok = &{1'b0, csr_read, csr_writedata[31:1]};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
