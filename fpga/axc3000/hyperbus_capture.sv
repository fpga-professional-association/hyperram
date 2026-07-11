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
// Sample word bit fields (issue #13: CAP_WIDTH = 74; hb_dq_o/hb_dq_i widened 8->16 to carry the full
// fabric TX/RX word, so DATA_HI now carries real data). hb_dq_i STRADDLES bit 32 (an inherent
// consequence of widening in place): LO[31:19] = hb_dq_i[12:0], HI[2:0] = hb_dq_i[15:13].
//   [0]     hb_cs_n            [35]     hb_rwds_oe
//   [1]     hb_ck (phy_ck_en)  [36]     hb_rwds_o (1st-phase mask)
//   [2]     hb_dq_oe           [37]     hb_rwds_i
//   [18:3]  hb_dq_o[15:0]      [38]     av_read
//   [34:19] hb_dq_i[15:0]      [39]     av_write
//                              [40]     av_waitrequest
//                              [41]     av_readdatavalid
//                              [73:42]  dbg_bus[31:0] (Avalon read-stream: bits [16:0] meaningful ->
//                                       land at [58:42], all inside the readable low 64; [73:59]
//                                       are the always-0 dbg pad, dropped by the 64-bit readback)
//
// CSR slave (clk domain, word-addressed: byte offset = 4 * csr_address, waitrequest tied low,
// reads combinational — same contract as the bw_test CSR so the top-level JTAG adapter is shared):
//   0x00  CTRL (w): bit0 = arm (1 = arm and clear a previous capture; 0 = disarm)
//         STATUS (r): bit0 = armed, bit1 = done, bits[31:16] = fill count (samples stored)
//   0x04  RDADDR (w/r): sample index for readback
//   0x08  DATA_LO (r): sample[31:0]  at RDADDR
//   0x0C  DATA_HI (r): sample[63:32] at RDADDR (issue #13: now carries hb_dq_i[15:13]/rwds/av/dbg)
//   0x10  CAPCFG (w/r): [15:0] = N, the 1-based Nth hb_cs_n FALLING edge to trigger on after arming
//                       (reset 1 = the legacy first-edge behavior). Program before arming.
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
    input  logic [15:0] hb_dq_o,       // issue #13: 16-bit fabric TX word ([hi]=byte A, [lo]=byte B)
    input  logic [15:0] hb_dq_i,       // issue #13: 16-bit fabric recovered RX word
    input  logic        hb_rwds_oe,
    input  logic        hb_rwds_o,
    input  logic        hb_rwds_i,
    input  logic        av_read,
    input  logic        av_write,
    input  logic        av_waitrequest,
    input  logic        av_readdatavalid,
    input  logic [31:0] dbg_bus,        // DEBUG: ctrl/front-end/FIFO taps (hyperram_avalon dbg_bus)

    // ---- CSR slave (clk domain) ----
    input  logic [2:0]  csr_address,   // issue #13: widened 2->3 for REG_CAPCFG at word 4 (0x110)
    input  logic        csr_read,
    output logic [31:0] csr_readdata,
    input  logic        csr_write,
    input  logic [31:0] csr_writedata,
    output logic        csr_waitrequest
);

  localparam int unsigned CAP_WIDTH = 74;            // 42 pin/av bits + 32 debug-bus bits (issue #13: 16-bit DQ; see map)
  localparam int unsigned AW        = $clog2(DEPTH); // sample index width

  assign csr_waitrequest = 1'b0;                     // single-cycle slave, zero wait states

  // ==================================================================================================
  //  clk-domain control: arm level + read address. `armed` is set by CTRL.arm, cleared when the
  //  capture engine reports done (or by writing arm = 0).
  // ==================================================================================================
  logic          armed;
  logic [AW-1:0] rdaddr;
  logic [15:0]   capcfg;   // REG_CAPCFG: 1-based Nth CS#-fall to trigger on (reset 1 = legacy first-edge)

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
      capcfg <= 16'd1;                                 // legacy: trigger on the FIRST CS# fall
    end else begin
      // Edge-triggered auto-disarm: only the RISING edge of done clears `armed`, so a re-arm issued
      // while the previous done level is still draining through the synchronisers is not swallowed.
      if (done_s & ~done_s_q) armed <= 1'b0;
      if (csr_write) begin
        unique case (csr_address)
          3'd0:    armed  <= csr_writedata[0];         // CTRL.arm (host write wins over auto-disarm)
          3'd1:    rdaddr <= csr_writedata[AW-1:0];    // RDADDR
          3'd4:    capcfg <= csr_writedata[15:0];      // REG_CAPCFG (issue #13); program BEFORE arming
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
    samp_q <= {dbg_bus,                                               // [73:42]
               av_readdatavalid, av_waitrequest, av_write, av_read,   // [41:38]
               hb_rwds_i, hb_rwds_o, hb_rwds_oe,                      // [37:35]
               hb_dq_i,                                               // [34:19]  (16-bit, issue #13)
               hb_dq_o,                                               // [18:3]   (16-bit, issue #13)
               hb_dq_oe, hb_ck, hb_cs_n};                             // [2:0]
  end

  typedef enum logic [1:0] { S_IDLE, S_ARMED, S_RUN, S_DONE } state_t;
  state_t        st;
  logic [AW:0]   wptr;                                 // fill count (0..DEPTH), extra MSB

  // Nth CS#-fall trigger (issue #13 REG_CAPCFG). samp_q[0] = hb_cs_n on the registered sample grid;
  // cs_prev idles HIGH so the FIRST low sample reads as a genuine 1->0 fall. With N_cap==1 (POR) the
  // first fall fires the trigger — bit-identical to the pre-issue-13 level trigger, because the host
  // always arms while cs_n is high (arm capture, THEN start the bus run). N_cap is latched from the
  // clk-domain capcfg at arm_pulse; capcfg is quasi-static (host sets it well before arming), so the
  // clk->cap_clk read is a safe debug-grade crossing, same philosophy as the arm/rst 2-flop stages.
  logic        cs_prev;
  always_ff @(posedge cap_clk) begin
    if (rst2x) cs_prev <= 1'b1;
    else       cs_prev <= samp_q[0];
  end
  wire cs_fall = cs_prev & ~samp_q[0];

  // Round-2 timing rework: the original up-counter compared (fall_cnt == N_cap - 1) — a 16-bit
  // subtract+compare in the trig_now->wptr/RAM-enable cone that missed 350 MHz setup by ~190 ps.
  // A DOWN-counter loads capcfg-1 once at arm (slow host event) so the hot cone is just a
  // zero-detect: same Nth-fall semantics, falls_left==0 on the fall that fires.
  logic [15:0] falls_left;                             // CS# falls remaining BEFORE the trigger fall
  wire trig_now = (st == S_ARMED) && cs_fall && (falls_left == 16'd0);   // the Nth fall
  wire cap_we   = trig_now || (st == S_RUN);

  always_ff @(posedge cap_clk) begin
    if (rst2x) begin
      st         <= S_IDLE;
      wptr       <= '0;
      done2x     <= 1'b0;
      falls_left <= '0;
    end else if (arm_pulse) begin
      st         <= S_ARMED;                           // arm: clear any previous capture
      wptr       <= '0;
      done2x     <= 1'b0;
      falls_left <= (capcfg == 16'd0) ? 16'd0 : capcfg - 16'd1;  // latch REG_CAPCFG (min 1) for this capture
    end else begin
      unique case (st)
        S_ARMED: begin
          if (cs_fall && falls_left != 16'd0) falls_left <= falls_left - 16'd1;  // count down the falls
          if (trig_now) begin st <= S_RUN; wptr <= wptr + 1'b1; end      // Nth fall: first sample stored
          else if (!arm_s)    st <= S_IDLE;                              // host disarmed
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
      3'd0:    csr_readdata = {16'(fill_s), 14'd0, done_s, armed};   // STATUS
      3'd1:    csr_readdata = 32'(rdaddr);                           // RDADDR
      3'd2:    csr_readdata = sample64[31:0];                        // DATA_LO
      3'd3:    csr_readdata = sample64[63:32];                       // DATA_HI (0x0C)
      3'd4:    csr_readdata = 32'(capcfg);                           // REG_CAPCFG (0x110, issue #13)
      default: csr_readdata = 32'd0;
    endcase
  end

  // csr_read is part of the shared CSR contract but unused here (reads have no side effects).
  logic _unused_ok;
  /* verilator lint_off UNUSEDSIGNAL */
  assign _unused_ok = &{1'b0, csr_read, csr_writedata[31:16]};   // [15:0] now used (armed/rdaddr/capcfg)
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
