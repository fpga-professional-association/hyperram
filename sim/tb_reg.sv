// tb_reg — self-checking Verilator TB for register/init/clock paths untested elsewhere (issue #4):
//   * B7 — CR1 / ID1 register access (SPEC_DIGEST §8.2/§8.3). Every other TB touches only CR0 (rw)
//     and ID0 (r). Here CR1 is written + read back and ID1 is read (a distinctive non-zero ID1_RESET
//     is programmed into the model so the aliased decode + big-endian readback are genuinely checked).
//   * B8 — non-zero POR_DELAY_CYCLES (the ST_POR dwell, a no-op at 0 in all other TBs) and a RUNTIME
//     reset toggle that drives hb_rst_n Low and exercises the model's reset register-restore
//     (hyperram_model:207-222): CR1 (which the controller does NOT reprogram at init) is written to a
//     distinctive value, reset is pulsed, and after re-init CR1 must read back its reset value.
//   * B9 — DIFF_CK complementary clock. A DIFF_CK=1 top must drive hb_ck_n as ~hb_ck (it goes Low
//     while hb_ck is High); a DIFF_CK=0 top must hold hb_ck_n High even while hb_ck toggles.
//   * B6 (Avalon connection) — the newly exposed hyperram_avalon.err_underrun output must stay Low
//     across all these well-formed transactions (no spurious underrun).
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_reg;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2
  localparam int unsigned REG_MSB    = ADDR_WIDTH - 1;

  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam int unsigned POR_DELAY   = 20;                    // exercised ST_POR dwell
  // Distinctive read-only ID1 + config CR1 reset images so the register decode is meaningfully checked.
  localparam logic [15:0] TB_ID1      = 16'h1D01;
  localparam logic [15:0] TB_CR1      = 16'h0000;

  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;    end
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90 = ~clk90;  end
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  // -------------------- Primary DUT (DIFF_CK=1, POR_DELAY>0) --------------------
  logic [ADDR_WIDTH-1:0]   avs_address;
  logic                    avs_read, avs_write;
  logic [DATA_WIDTH-1:0]   avs_writedata;
  logic [STRB_WIDTH-1:0]   avs_byteenable;
  logic [LEN_WIDTH-1:0]    avs_burstcount;
  logic [DATA_WIDTH-1:0]   avs_readdata;
  logic                    avs_readdatavalid, avs_waitrequest, init_done, err_underrun;

  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  phy_dq_o;   logic phy_dq_oe;  logic phy_rwds_o, phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;  logic mdl_rwds_o, mdl_rwds_oe;
  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);
  localparam realtime RTT = 3.0;
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  hyperram_avalon #(
    .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1), .MAX_BURST_WORDS (0),
    .PROGRAM_CR (1'b1), .POR_DELAY_CYCLES (POR_DELAY), .INIT_CR0 (TB_INIT_CR0),
    .PHY_VARIANT ("GENERIC"), .DIFF_CK (1'b1)
  ) dut (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // runtime read-eye calibration tied to POR-equivalent constants (reproduces pre-cal behaviour)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd0), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .avs_address (avs_address), .avs_read (avs_read), .avs_write (avs_write),
    .avs_writedata (avs_writedata), .avs_byteenable (avs_byteenable), .avs_burstcount (avs_burstcount),
    .avs_readdata (avs_readdata), .avs_readdatavalid (avs_readdatavalid), .avs_waitrequest (avs_waitrequest),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (phy_dq_o), .hb_dq_oe (phy_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (phy_rwds_o), .hb_rwds_oe (phy_rwds_oe), .hb_rwds_i (rwds_line_dly),
    .init_done (init_done), .err_underrun (err_underrun), .dbg_bus (),
    // issue #13: new hyperram_avalon debug bundle + wrap_en tied to per-instance legacy (A1).
    .dbg_wr_lat_trim (4'd0), .dbg_lat_clocks (4'd6), .dbg_cr0_reprog (1'b0),
    .dbg_prewin_drive (1'b0), .dbg_prewin_n (3'd0), .dbg_prewin_marker (1'b0),
    .dbg_postwin_hold (1'b0), .dbg_prewin_contig (1'b0), .dbg_end_cwrite (1'b0), .wrap_en (1'b0)
  );

  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0), .ID1_RESET (TB_ID1), .CR1_RESET (TB_CR1)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i (dq_line), .hb_dq_ie (phy_dq_oe), .hb_dq_o (mdl_dq_o), .hb_dq_oe (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (phy_rwds_oe), .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );

  // -------------------- Secondary DUT (DIFF_CK=0), init-only, for the B9 single-ended check --------------------
  logic [ADDR_WIDTH-1:0]   avs2_address;
  logic                    avs2_read, avs2_write;
  logic [DATA_WIDTH-1:0]   avs2_writedata;
  logic [STRB_WIDTH-1:0]   avs2_byteenable;
  logic [LEN_WIDTH-1:0]    avs2_burstcount;
  logic [DATA_WIDTH-1:0]   avs2_readdata;
  logic                    avs2_readdatavalid, avs2_waitrequest, init_done2, err_underrun2;
  logic                 hb_ck2, hb_ck_n2, hb_cs_n2, hb_rst_n2;
  logic [DQ_WIDTH-1:0]  phy2_dq_o;  logic phy2_dq_oe;  logic phy2_rwds_o, phy2_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl2_dq_o;  logic mdl2_dq_oe;  logic mdl2_rwds_o, mdl2_rwds_oe;
  wire [DQ_WIDTH-1:0] dq2_line   = mdl2_dq_oe   ? mdl2_dq_o   : (phy2_dq_oe   ? phy2_dq_o   : '0);
  wire                rwds2_line = mdl2_rwds_oe ? mdl2_rwds_o : (phy2_rwds_oe ? phy2_rwds_o : 1'b0);
  wire [DQ_WIDTH-1:0] dq2_line_dly;   assign #RTT dq2_line_dly   = dq2_line;
  wire                rwds2_line_dly; assign #RTT rwds2_line_dly = rwds2_line;

  hyperram_avalon #(
    .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1), .MAX_BURST_WORDS (0),
    .PROGRAM_CR (1'b1), .POR_DELAY_CYCLES (0), .INIT_CR0 (TB_INIT_CR0),
    .PHY_VARIANT ("GENERIC"), .DIFF_CK (1'b0)                       // single-ended CK
  ) dut2 (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // runtime read-eye calibration tied to POR-equivalent constants (reproduces pre-cal behaviour)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd0), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .avs_address (avs2_address), .avs_read (avs2_read), .avs_write (avs2_write),
    .avs_writedata (avs2_writedata), .avs_byteenable (avs2_byteenable), .avs_burstcount (avs2_burstcount),
    .avs_readdata (avs2_readdata), .avs_readdatavalid (avs2_readdatavalid), .avs_waitrequest (avs2_waitrequest),
    .hb_ck (hb_ck2), .hb_ck_n (hb_ck_n2), .hb_cs_n (hb_cs_n2), .hb_rst_n (hb_rst_n2),
    .hb_dq_o (phy2_dq_o), .hb_dq_oe (phy2_dq_oe), .hb_dq_i (dq2_line_dly),
    .hb_rwds_o (phy2_rwds_o), .hb_rwds_oe (phy2_rwds_oe), .hb_rwds_i (rwds2_line_dly),
    .init_done (init_done2), .err_underrun (err_underrun2), .dbg_bus (),
    // issue #13: new hyperram_avalon debug bundle + wrap_en tied to per-instance legacy (A1).
    .dbg_wr_lat_trim (4'd0), .dbg_lat_clocks (4'd6), .dbg_cr0_reprog (1'b0),
    .dbg_prewin_drive (1'b0), .dbg_prewin_n (3'd0), .dbg_prewin_marker (1'b0),
    .dbg_postwin_hold (1'b0), .dbg_prewin_contig (1'b0), .dbg_end_cwrite (1'b0), .wrap_en (1'b0)
  );

  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0)
  ) model2 (
    .hb_ck (hb_ck2), .hb_ck_n (hb_ck_n2), .hb_cs_n (hb_cs_n2), .hb_rst_n (hb_rst_n2),
    .hb_dq_i (dq2_line), .hb_dq_ie (phy2_dq_oe), .hb_dq_o (mdl2_dq_o), .hb_dq_oe (mdl2_dq_oe),
    .hb_rwds_i (rwds2_line), .hb_rwds_ie (phy2_rwds_oe), .hb_rwds_o (mdl2_rwds_o), .hb_rwds_oe (mdl2_rwds_oe)
  );

  // --------------------------------------------------------------------
  // Scoreboard + capture + monitors
  // --------------------------------------------------------------------
  int unsigned errors = 0, checks = 0;
  localparam int unsigned CAP_MAX = 16;
  logic [DATA_WIDTH-1:0] cap [CAP_MAX];
  int unsigned           cap_n;
  logic                  capturing;
  logic                  saw_underrun = 1'b0;

  // B9 monitors (accumulate steady-state observations; race-free vs. the exact CK edge).
  logic saw_ck1_hi = 1'b0, saw_ckn1_lo = 1'b0;   // DIFF_CK=1: hb_ck toggles, hb_ck_n goes Low
  logic saw_ck2_hi = 1'b0, ckn2_low_bad = 1'b0;  // DIFF_CK=0: hb_ck toggles, hb_ck_n must stay High

  always @(posedge clk) begin
    if (capturing && avs_readdatavalid) begin
      if (cap_n < CAP_MAX) cap[cap_n] <= avs_readdata;
      cap_n <= cap_n + 1;
    end
    if (err_underrun || err_underrun2) saw_underrun <= 1'b1;
  end

  // Samples the async device CK pins in a clk_ref-synchronous monitor (TB-only observation): that
  // dual use is intentional, so waive SYNCASYNCNET locally (matching the PHY/model house style).
  /* verilator lint_off SYNCASYNCNET */
  always @(posedge clk_ref) begin
    if (hb_ck)     saw_ck1_hi   <= 1'b1;
    if (!hb_ck_n)  saw_ckn1_lo  <= 1'b1;
    if (hb_ck2)    saw_ck2_hi   <= 1'b1;
    if (!hb_ck_n2) ckn2_low_bad <= 1'b1;
  end
  /* verilator lint_on SYNCASYNCNET */

  // --------------------------------------------------------------------
  // Avalon tasks (register-capable; from tb_avalon's skeleton)
  // --------------------------------------------------------------------
  task automatic avs_idle();
    @(negedge clk);
    avs_address='0; avs_read=1'b0; avs_write=1'b0; avs_writedata='0; avs_byteenable='1; avs_burstcount='0;
  endtask

  task automatic do_write1(input logic [ADDR_WIDTH-1:0] addr, input logic reg_space,
                           input logic [DATA_WIDTH-1:0] data);
    int unsigned g; logic [ADDR_WIDTH-1:0] a_full;
    a_full=addr; a_full[REG_MSB]=reg_space;
    @(negedge clk);
    avs_write=1'b1; avs_read=1'b0; avs_address=a_full; avs_burstcount=LEN_WIDTH'(1);
    avs_byteenable='1; avs_writedata=data;
    g=0; forever begin @(posedge clk); g=g+1;
      if (g>5000) begin $display("[%0t] HANG do_write1 @0x%08x", $time, addr); errors=errors+1; break; end
      if (!avs_waitrequest) break; end
    avs_idle();
  endtask

  task automatic do_read1(input logic [ADDR_WIDTH-1:0] addr, input logic reg_space);
    int unsigned guard; logic [ADDR_WIDTH-1:0] a_full;
    a_full=addr; a_full[REG_MSB]=reg_space;
    cap_n=0; capturing=1'b1;
    @(negedge clk);
    avs_read=1'b1; avs_write=1'b0; avs_address=a_full; avs_burstcount=LEN_WIDTH'(1);
    guard=0; forever begin @(posedge clk); guard=guard+1;
      if (guard>5000) begin $display("[%0t] HANG do_read1 @0x%08x", $time, addr); errors=errors+1; break; end
      if (!avs_waitrequest) break; end
    avs_idle();
    guard=0; while (cap_n<1 && guard<5000) begin @(posedge clk); guard=guard+1; end
    @(posedge clk); capturing=1'b0;
    if (cap_n<1) begin $display("[%0t] ERROR read @0x%08x got none", $time, addr); errors=errors+1; end
  endtask

  task automatic chk(input logic [DATA_WIDTH-1:0] got, input logic [DATA_WIDTH-1:0] exp,
                     input string tag);
    checks=checks+1;
    if (got!==exp) begin $display("[%0t] ERROR (%s): got 0x%04x exp 0x%04x", $time, tag, got, exp);
                        errors=errors+1; end
    else $display("[%0t] %s ok (0x%04x)", $time, tag, got);
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  int unsigned guard, por_window;
  initial begin
    avs_address='0; avs_read=1'b0; avs_write=1'b0; avs_writedata='0; avs_byteenable='1; avs_burstcount='0;
    // secondary DUT is init-only: tie its Avalon inputs idle for the whole sim.
    avs2_address='0; avs2_read=1'b0; avs2_write=1'b0; avs2_writedata='0; avs2_byteenable='1; avs2_burstcount='0;
    capturing=1'b0; cap_n=0; rst=1'b1;
    repeat (5) @(posedge clk); @(negedge clk); rst=1'b0;

    // ---- B8 part 1: measure the ST_POR dwell (hb_rst_n rising -> first hb_cs_n Low) ----
    guard=0; while (!hb_rst_n && guard<5000) begin @(posedge clk); guard=guard+1; end
    por_window=0; while (hb_cs_n && por_window<5000) begin @(posedge clk); por_window=por_window+1; end
    checks=checks+1;
    if (por_window < POR_DELAY) begin
      $display("[%0t] ERROR: POR window %0d cycles < POR_DELAY_CYCLES %0d (ST_POR dwell not honored)",
               $time, por_window, POR_DELAY);
      errors=errors+1;
    end else
      $display("[%0t] POR dwell honored: %0d cycles from hb_rst_n high to first CS# (POR_DELAY=%0d)",
               $time, por_window, POR_DELAY);

    guard=0; while (!init_done && guard<100000) begin @(posedge clk); guard=guard+1; end
    if (!init_done) begin $display("[%0t] FATAL init_done", $time); errors=errors+1; end
    else $display("[%0t] init_done asserted (POR_DELAY=%0d)", $time, POR_DELAY);
    repeat (4) @(posedge clk);

    // ---- B7: CR0/ID0 (re-confirm) + CR1 write/read + ID1 read ----
    do_read1(HB_REG_CR0[ADDR_WIDTH-1:0], 1'b1); chk(cap[0], TB_INIT_CR0, "CR0 readback");
    do_read1(HB_REG_ID0[ADDR_WIDTH-1:0], 1'b1); chk(cap[0], HB_ID0_RESET, "ID0 read");
    do_write1(HB_REG_CR1[ADDR_WIDTH-1:0], 1'b1, 16'hABCD);
    do_read1 (HB_REG_CR1[ADDR_WIDTH-1:0], 1'b1); chk(cap[0], 16'hABCD, "CR1 write+readback");
    do_read1 (HB_REG_ID1[ADDR_WIDTH-1:0], 1'b1); chk(cap[0], TB_ID1, "ID1 read");
    // ID1 is read-only: a write must be ignored (value unchanged).
    do_write1(HB_REG_ID1[ADDR_WIDTH-1:0], 1'b1, 16'hFFFF);
    do_read1 (HB_REG_ID1[ADDR_WIDTH-1:0], 1'b1); chk(cap[0], TB_ID1, "ID1 read-only (write ignored)");

    // ---- B8 part 2: runtime reset -> model restores config registers ----
    // Set CR1 to a distinctive value the controller never reprograms, then pulse reset. After re-init
    // the model must have restored CR1 to its reset image (proves the model reset register-restore).
    do_write1(HB_REG_CR1[ADDR_WIDTH-1:0], 1'b1, 16'hBEEF);
    do_read1 (HB_REG_CR1[ADDR_WIDTH-1:0], 1'b1); chk(cap[0], 16'hBEEF, "CR1 pre-reset value");
    $display("[%0t] pulsing runtime reset...", $time);
    @(negedge clk); rst=1'b1;
    repeat (6) @(posedge clk); @(negedge clk); rst=1'b0;
    guard=0; while (!init_done && guard<100000) begin @(posedge clk); guard=guard+1; end
    if (!init_done) begin $display("[%0t] FATAL init_done (post-reset)", $time); errors=errors+1; end
    else $display("[%0t] re-init after runtime reset complete", $time);
    repeat (4) @(posedge clk);
    do_read1(HB_REG_CR1[ADDR_WIDTH-1:0], 1'b1); chk(cap[0], TB_CR1, "CR1 restored to reset by hb_rst_n");
    // CR0 was reprogrammed by the controller at re-init, so it is back to the INIT image (not reset).
    do_read1(HB_REG_CR0[ADDR_WIDTH-1:0], 1'b1); chk(cap[0], TB_INIT_CR0, "CR0 reprogrammed at re-init");

    // ---- B9: DIFF_CK complementary vs single-ended clock ----
    // Let the secondary (DIFF_CK=0) DUT finish its own init so its CK has toggled.
    guard=0; while (!init_done2 && guard<100000) begin @(posedge clk); guard=guard+1; end
    repeat (8) @(posedge clk);
    checks=checks+1;
    if (!(saw_ck1_hi && saw_ckn1_lo)) begin
      $display("[%0t] ERROR (DIFF_CK=1): hb_ck_hi=%0b hb_ck_n_lo=%0b (expected complementary toggling)",
               $time, saw_ck1_hi, saw_ckn1_lo);
      errors=errors+1;
    end else $display("[%0t] DIFF_CK=1: hb_ck_n toggles complementary to hb_ck (ok)", $time);
    checks=checks+1;
    if (!saw_ck2_hi || ckn2_low_bad) begin
      $display("[%0t] ERROR (DIFF_CK=0): ck_toggled=%0b ck_n_went_low=%0b (expected ck_n held High)",
               $time, saw_ck2_hi, ckn2_low_bad);
      errors=errors+1;
    end else $display("[%0t] DIFF_CK=0: hb_ck_n held High while hb_ck toggled (ok)", $time);

    // ---- B6 (Avalon connection): err_underrun stayed Low throughout well-formed traffic ----
    checks=checks+1;
    if (saw_underrun) begin
      $display("[%0t] ERROR: err_underrun asserted during well-formed traffic (spurious)", $time);
      errors=errors+1;
    end else $display("[%0t] err_underrun stayed Low across all normal transactions (ok)", $time);

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_reg done: %0d checks, %0d errors", $time, checks, errors);
    if (errors==0) begin $display("TB_RESULT: PASS"); $finish; end
    else begin $display("TB_RESULT: FAIL"); $fatal(1, "tb_reg: %0d errors", errors); end
  end

  initial begin
    #4_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_reg: global timeout");
  end

endmodule
