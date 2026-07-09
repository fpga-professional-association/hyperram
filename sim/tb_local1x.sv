// tb_local1x — phase-sweep regression for the LOCAL1X RX pairing in fpga/axc3000/hyperbus_gpio_io.sv.
//
// The LOCAL1X claim: both-edge fabric sampling on a phase-shifted 1x clock + RWDS edge-detect
// pairing recovers the read byte stream for ANY flight-delay phase (the property the SDR PHY's
// 2x scheme demonstrated on silicon). This TB instantiates hyperbus_gpio_io (with the behavioral
// atom stubs from sim/model/gpio_io_sim_stubs.sv), drives a synthetic device read burst — optional
// preamble cycle, N data words, over-stream stragglers — onto hb_dq/hb_rwds with a swept launch
// phase PHI in [0, T), and checks the recovered words for every phase step.
//
// Runs under: verilator --binary --timing (5.020).

`timescale 1ns/1ps
module tb_local1x;

  localparam int unsigned DQW      = 8;
  localparam realtime     T_CK     = 5.0;   // 200 MHz
  localparam int unsigned N_WORDS  = 16;
  localparam int unsigned N_OVER   = 5;     // over-stream words after the master count
  localparam int unsigned PREAMBLE = 1;     // device preamble CK cycles
  localparam int unsigned N_PHASES = 20;    // PHI sweep steps across one CK period

  logic clk = 0, clk_smp, rst = 1;
  always #(T_CK/2.0) clk = ~clk;
  // +90 deg sampling clock, same frequency
  initial begin
    clk_smp = 0;
    #(T_CK/4.0);
    forever #(T_CK/2.0) clk_smp = ~clk_smp;
  end

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
  assign hb_dq   = dev_dq_en   ? dev_dq   : 8'hzz;
  assign hb_rwds = dev_rwds_en ? dev_rwds : 1'bz;

  hyperbus_gpio_io #(
    .DQ_WIDTH         (DQW),
    .RD_PREAMBLE_SKIP (PREAMBLE),
    .TX_B_DLY         (1'b1),
    .CK_DIN_HI        (1'b1)
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

  // Drive one device read burst launched at absolute phase offset phi (ns) after a clk edge.
  task automatic drive_burst(input realtime phi);
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
    // data words + over-stream, byte A during RWDS high, byte B during low
    for (int unsigned k = 0; k < N_WORDS + N_OVER; k++) begin
      logic [15:0] w;
      w        = pat(k);
      dev_dq   = w[15:8];  dev_rwds = 1'b1; #(T_CK/2.0);
      dev_dq   = w[7:0];   dev_rwds = 1'b0; #(T_CK/2.0);
    end
    dev_rwds_en = 1'b0;
    dev_dq_en   = 1'b0;
  endtask

  int unsigned got_cnt;
  logic [15:0] got [N_WORDS + N_OVER + 4];
  always @(posedge clk) if (dq_i_valid && got_cnt < N_WORDS + N_OVER + 4) begin
    got[got_cnt] = dq_i;
    got_cnt     += 1;
  end

  int unsigned fails = 0;
  int unsigned vcount;
  always @(posedge clk) if (dq_i_valid) vcount++;
  initial begin
    forever begin
      @(posedge dut.rd_arm_eff);
      $display("[%0t] rd_arm_eff ROSE (vcount=%0d)", $time, vcount);
    end
  end

  initial begin
    rd_arm = 1'b0;
    repeat (8) @(posedge clk);
    rst = 1'b0;
    repeat (4) @(posedge clk);

    for (int unsigned p = 0; p < N_PHASES; p++) begin
      realtime phi;
      phi = (T_CK * p) / N_PHASES;
      got_cnt = 0;
      // arm ahead of the burst (controller arms during latency)
      rd_arm  = 1'b1;
      fork
        drive_burst(phi);
      join_none
      if (p == 0) begin
        repeat (20) @(posedge clk);
        $display("DBG p0: rst=%b rd_arm=%b arm_cnt=%0d rd_arm_eff=%b rwds_s0=%b rwds_s1=%b",
                 rst, rd_arm, dut.arm_cnt, dut.rd_arm_eff, dut.rwds_s0_q, dut.rwds_s1_q);
      end
      // (the hostile window inside drive_burst spans ~20 CK; ARM_DELAY_CYCLES=16 must blind it)
      repeat (8) @(posedge clk);   // drain
      // check the first N_WORDS recovered words (over-stream may add extras — the controller
      // word-counts; here we only require the leading words to be exact and in order)
      if (got_cnt < N_WORDS) begin
        $display("PHASE %0d (phi=%0.2f ns): FAIL — only %0d/%0d words recovered (vcount=%0d wptr=%0d)",
                 p, phi, got_cnt, N_WORDS, vcount, dut.wptr_bin);
        fails++;
      end else begin
        for (int unsigned k = 0; k < N_WORDS; k++) begin
          if (got[k] !== pat(k)) begin
            $display("PHASE %0d (phi=%0.2f ns): FAIL — word[%0d] got=%04x exp=%04x", p, phi, k, got[k], pat(k));
            fails++;
            break;
          end
        end
      end
      rd_arm = 1'b0;               // disarm flushes for the next phase
      repeat (6) @(posedge clk);
    end

    if (fails == 0) $display("TB_RESULT: PASS (all %0d phases clean)", N_PHASES);
    else            $display("TB_RESULT: FAIL (%0d/%0d phases failed)", fails, N_PHASES);
    if (fails != 0) $fatal(1, "tb_local1x failed");
    $finish;
  end

endmodule
