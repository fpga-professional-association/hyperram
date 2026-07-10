// tb_local1x — phase-sweep regression for the QUAD1X RX pairing in fpga/axc3000/hyperbus_gpio_io.sv
// (module/file names kept as "local1x" for continuity; the DUT's RX now samples FOUR phases per
// word cycle — clk pos/neg (0/180 deg) AND clk_smp pos/neg (90/270 deg) — not two).
//
// The claim under test: edge-detect pairing over a 4-phase (0/90/180/270 deg), all-1x-clocked
// sample stream recovers the read byte stream for ANY flight-delay phase AND absorbs the brief
// device output-timing wobble at page crossings (see WOBBLE_* below) — the property the
// source-synchronous SDR PHY demonstrated on silicon, now at quarter-cycle resolution with no 2x
// clock anywhere (the min-pulse-limited 2x clock is out of bounds for a 200 MHz build). The
// reusable local1x_harness below instantiates hyperbus_gpio_io (with the behavioral atom stubs
// from sim/model/gpio_io_sim_stubs.sv), drives a synthetic device read burst — optional preamble
// cycle, N data words (one of which wobbles), over-stream stragglers — onto hb_dq/hb_rwds with a
// swept launch phase PHI in [0, T), and checks the recovered words for every phase step. The
// controller re-arms per burst; the disarm between phases must flush the over-stream words so the
// next burst starts clean.
//
// tb_local1x drives the harness in TWO configurations from a single run:
//   * h_hostile : ARM_DELAY_CYCLES=16, HOSTILE window (latency indicator + float-coupling RWDS
//                 runts) — the 200 MHz silicon hazard the arm-delay must blind.
//   * h_benign  : ARM_DELAY_CYCLES=0  (arm immediately), benign window (no float runts) — the
//                 parameter-sweep sanity check that the pairing + arm/disarm path is correct even
//                 with zero blind delay.
// Both configurations also carry the page-crossing wobble (WOBBLE_* parameters, default period 16
// / offset 7 / 0.6 ns) on top of their own hazard, since the wobble is a device-side effect
// independent of the arm-delay/float-window receiver-side hazard.
//
// Runs under: verilator --binary --timing (5.020).

`timescale 1ns/1ps

// ------------------------------------------------------------------------------------------------
//  Reusable phase-sweep harness.
// ------------------------------------------------------------------------------------------------
module local1x_harness #(
    parameter int unsigned DQW       = 8,
    parameter realtime     T_CK      = 5.0,    // 200 MHz
    parameter int unsigned N_WORDS   = 16,
    parameter int unsigned N_OVER    = 5,      // over-stream words after the master count
    parameter int unsigned PREAMBLE  = 1,      // device preamble CK cycles
    parameter int unsigned N_PHASES  = 20,     // PHI sweep steps across one CK period
    parameter int unsigned ARM_DELAY = 16,     // hyperbus_gpio_io ARM_DELAY_CYCLES
    parameter bit          HOSTILE   = 1'b1,   // 1 = latency indicator + float runts; 0 = benign
    // Device page-crossing output wobble (silicon finding: the W957D8NB's 32-byte internal page =
    // 16 words; right at the crossing the device's output launch edge lands ~0.6 ns late for ONE
    // edge, then SNAPS BACK onto the normal CK grid for the very next edge — a brief output-timing
    // shift, not data corruption, and not a cumulative phase shift into the rest of the stream. A
    // fixed 2-sample/cycle grid clips it (one bit flip on silicon); the QUAD1X 4-phase edge-detect
    // pairing must follow it like the source-synchronous SDR PHY did. Silicon-observed period 16,
    // first-hit offset 7 (words 7,23,39,55,... on a burst starting at word 0).
    parameter int unsigned WOBBLE_PERIOD = 16,
    parameter int unsigned WOBBLE_OFFSET = 7,
    parameter realtime     WOBBLE_NS     = 0.6
) (
    input  logic        clk,
    input  logic        clk_smp,
    input  logic        rst,
    output int unsigned fails,
    output logic        done
);

  // DUT ctrl-side
  logic        rd_arm;
  logic [15:0] dq_i;
  logic        dq_i_valid;
  wire  [DQW-1:0] hb_dq;
  wire            hb_rwds;

  // device-side drivers (tri-state onto the buses)
  logic [DQW-1:0] dev_dq;
  logic           dev_dq_en;
  logic           dev_rwds;
  logic           dev_rwds_en;
  assign hb_dq   = dev_dq_en   ? dev_dq   : {DQW{1'bz}};
  assign hb_rwds = dev_rwds_en ? dev_rwds : 1'bz;

  hyperbus_gpio_io #(
    .DQ_WIDTH         (DQW),
    .RD_PREAMBLE_SKIP (PREAMBLE),
    .TX_B_DLY         (1'b1),
    .CK_DIN_HI        (1'b1),
    .ARM_DELAY_CYCLES (ARM_DELAY)
  ) dut (
    .clk            (clk),
    .clk_smp        (clk_smp),
    .rst            (rst),
    .phy_cs_n       (1'b1),
    .phy_rst_n      (1'b1),
    .phy_ck_en      (1'b0),
    .phy_dq_o       (16'h0000),
    .phy_dq_oe      (1'b0),
    .phy_rwds_o     (2'b00),
    .phy_rwds_oe    (1'b0),
    .phy_rd_arm     (rd_arm),
    .phy_dq_i       (dq_i),
    .phy_dq_i_valid (dq_i_valid),
    .phy_rwds_i     (),
    .hb_ck          (),
    .hb_cs_n        (),
    .hb_rst_n       (),
    .hb_dq          (hb_dq),
    .hb_rwds        (hb_rwds)
  );

  function automatic logic [15:0] pat(input int unsigned k);
    logic [31:0] x;
    x = 32'(k) + 32'h1;
    x = x ^ (x << 7); x = x ^ (x >> 9); x = x ^ (x << 8);
    return x[15:0];
  endfunction

  // Page-crossing wobble extra delay for word k's byte-A launch (0 for every word except the
  // periodic crossing hit) — see WOBBLE_* parameters above.
  function automatic realtime wobble_of(input int unsigned k);
    return ((k % WOBBLE_PERIOD) == WOBBLE_OFFSET) ? WOBBLE_NS : 0.0;
  endfunction

  // Drive one device read burst launched at absolute phase offset phi (ns) after a clk edge.
  task automatic drive_burst(input realtime phi);
    if (HOSTILE) begin
      // -------- hostile: latency indicator + float window with CK-coupling runts --------
      dev_dq_en   = 1'b0;
      dev_rwds_en = 1'b0;
      dev_rwds    = 1'b0;
      @(posedge clk); #phi;
      // latency indicator: device drives RWDS HIGH for a few CK, then releases
      dev_rwds_en = 1'b1;
      dev_rwds    = 1'b1;
      #(4*T_CK);
      dev_rwds_en = 1'b0;   // release -> float window
      // float window with CK-coupling runts on RWDS and junk on DQ (the 200 MHz silicon hazard)
      dev_dq_en = 1'b1;
      dev_dq    = 8'haa;
      repeat (10) begin
        dev_rwds_en = 1'b1; dev_rwds = 1'b1; #(T_CK/2.0);
        dev_rwds    = 1'b0; #(T_CK/2.0);
        dev_rwds_en = 1'b0;
      end
      dev_dq_en = 1'b0;
      #(2*T_CK);
      // preamble: RWDS toggles, DQ driven 0 (device turnaround)
      dev_rwds_en = 1'b1;
      dev_dq_en   = 1'b1;
      dev_dq      = 8'h00;
      repeat (PREAMBLE) begin
        dev_rwds = 1'b1; #(T_CK/2.0);
        dev_rwds = 1'b0; #(T_CK/2.0);
      end
      // data words + over-stream, byte A during RWDS high, byte B during low. Page-crossing wobble:
      // word k's byte-A rise is delayed wob (0 or WOBBLE_NS), then the byte-A window is shortened
      // by the same amount so the falling edge (and every edge after it) lands back on the normal
      // CK grid — a brief, self-correcting timing shift, not a data error and not a phase drift.
      for (int unsigned k = 0; k < N_WORDS + N_OVER; k++) begin
        logic [15:0] w;
        realtime     wob;
        w   = pat(k);
        wob = wobble_of(k);
        #(wob);
        dev_dq   = w[15:8];  dev_rwds = 1'b1; #(T_CK/2.0 - wob);
        dev_dq   = w[7:0];   dev_rwds = 1'b0; #(T_CK/2.0);
      end
      dev_rwds_en = 1'b0;
      dev_dq_en   = 1'b0;
    end else begin
      // -------- benign: clean window, no float runts, receiver arms immediately --------
      // dev_*_en are held asserted (see initial) so the bus is always driven — with ARM_DELAY=0
      // the receiver arms ~1 CK in and must never sample a floating strobe.
      dev_rwds = 1'b0;
      dev_dq   = 8'h00;
      @(posedge clk); #phi;
      #(2*T_CK);            // brief driven-idle turnaround; the (1 CK) arm delay elapses here
      // preamble: RWDS toggles, DQ driven 0 (device turnaround)
      dev_dq = 8'h00;
      repeat (PREAMBLE) begin
        dev_rwds = 1'b1; #(T_CK/2.0);
        dev_rwds = 1'b0; #(T_CK/2.0);
      end
      // data words + over-stream, byte A during RWDS high, byte B during low (page-crossing wobble
      // applied the same way as the hostile branch — see the comment there).
      for (int unsigned k = 0; k < N_WORDS + N_OVER; k++) begin
        logic [15:0] w;
        realtime     wob;
        w   = pat(k);
        wob = wobble_of(k);
        #(wob);
        dev_dq   = w[15:8];  dev_rwds = 1'b1; #(T_CK/2.0 - wob);
        dev_dq   = w[7:0];   dev_rwds = 1'b0; #(T_CK/2.0);
      end
      dev_rwds = 1'b0;      // back to driven idle
      dev_dq   = 8'h00;
    end
  endtask

  int unsigned got_cnt;
  logic [15:0] got [N_WORDS + N_OVER + 4];
  always @(posedge clk) if (dq_i_valid && got_cnt < N_WORDS + N_OVER + 4) begin
    got[got_cnt] = dq_i;
    got_cnt     += 1;
  end

  initial begin
    fails       = 0;
    done        = 1'b0;
    rd_arm      = 1'b0;
    // hostile drives the bus only during a burst (float window is part of the hazard); benign
    // keeps the bus driven at all times so the immediate arm never samples a floating strobe.
    dev_dq_en   = ~HOSTILE;
    dev_rwds_en = ~HOSTILE;
    dev_dq      = '0;
    dev_rwds    = 1'b0;
    @(negedge rst);
    repeat (4) @(posedge clk);

    for (int unsigned p = 0; p < N_PHASES; p++) begin
      realtime phi;
      phi     = (T_CK * p) / N_PHASES;
      got_cnt = 0;
      // arm ahead of the burst (controller arms during latency), then drive the whole burst to
      // completion BEFORE checking — bursts must not overlap.
      rd_arm  = 1'b1;
      drive_burst(phi);
      repeat (8) @(posedge clk);   // drain the FIFO
      // check the first N_WORDS recovered words (over-stream may add extras — the controller
      // word-counts; here we only require the leading words to be exact and in order)
      if (got_cnt < N_WORDS) begin
        $display("[%s] PHASE %0d (phi=%0.2f ns): FAIL — only %0d/%0d words recovered (wptr=%0d)",
                 HOSTILE ? "hostile" : "benign", p, phi, got_cnt, N_WORDS, dut.wptr_bin);
        fails++;
      end else begin
        for (int unsigned k = 0; k < N_WORDS; k++) begin
          if (got[k] !== pat(k)) begin
            $display("[%s] PHASE %0d (phi=%0.2f ns): FAIL — word[%0d] got=%04x exp=%04x",
                     HOSTILE ? "hostile" : "benign", p, phi, k, got[k], pat(k));
            fails++;
            break;
          end
        end
      end
      rd_arm = 1'b0;               // disarm flushes for the next phase
      repeat (6) @(posedge clk);
    end
    done = 1'b1;
  end

endmodule

// ------------------------------------------------------------------------------------------------
//  Top: shared clocks + reset, two harness configurations, aggregate result.
// ------------------------------------------------------------------------------------------------
module tb_local1x;

  localparam realtime     T_CK     = 5.0;   // 200 MHz
  localparam int unsigned N_PHASES = 20;

  logic clk = 0, clk_smp, rst = 1;
  always #(T_CK/2.0) clk = ~clk;
  // +90 deg sampling clock, same frequency
  initial begin
    clk_smp = 0;
    #(T_CK/4.0);
    forever #(T_CK/2.0) clk_smp = ~clk_smp;
  end

  int unsigned fails_h, fails_b;
  logic        done_h,  done_b;

  // hostile window, arm-delay blinds the float runts
  local1x_harness #(.N_PHASES(N_PHASES), .ARM_DELAY(16), .HOSTILE(1'b1)) h_hostile (
    .clk(clk), .clk_smp(clk_smp), .rst(rst), .fails(fails_h), .done(done_h)
  );
  // benign window, arm immediately (parameter-sweep sanity)
  local1x_harness #(.N_PHASES(N_PHASES), .ARM_DELAY(0), .HOSTILE(1'b0)) h_benign (
    .clk(clk), .clk_smp(clk_smp), .rst(rst), .fails(fails_b), .done(done_b)
  );

  initial begin
    rst = 1'b1;
    repeat (8) @(posedge clk);
    rst = 1'b0;
    wait (done_h && done_b);
    if (fails_h == 0 && fails_b == 0)
      $display("TB_RESULT: PASS (hostile ARM=16 + benign ARM=0, all %0d phases each)", N_PHASES);
    else
      $display("TB_RESULT: FAIL (hostile %0d/%0d + benign %0d/%0d phases failed)",
               fails_h, N_PHASES, fails_b, N_PHASES);
    if (fails_h != 0 || fails_b != 0) $fatal(1, "tb_local1x failed");
    $finish;
  end

endmodule
