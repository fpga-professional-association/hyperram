// tb_varlat — self-checking Verilator TB for TRUE variable initial latency (issue #4, B5;
// SPEC_DIGEST §3.2/§5.2.4): a device that ALTERNATES 1x and 2x initial latency per transaction.
//
// tb_fixed2x only covers a device that is CONSTANTLY 2x. Here the model runs in VARIABLE mode
// (CR0[3]=0) with REFRESH_EVERY=2, so it inserts the additional latency count on every other
// transaction (a refresh collision) and drives RWDS High during CA only for those. The controller
// must re-decode the slave-driven RWDS level and re-latch rwds_hi PER TRANSACTION: a sticky rwds_hi
// (never cleared between transactions) would carry a 2x decision into a following 1x transaction (or
// vice-versa) and misalign read/write data by one latency count — caught here by byte-exact
// read-back across an alternating stream of writes and reads.
//
// The top is built with FIXED_LATENCY=0 and INIT_CR0 = 0x8F17 (latency code 6, VARIABLE, legacy wrap,
// 32 B) so the programmed device is variable; the controller's latency doubling is exercised live.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_varlat;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2

  // VARIABLE latency image: latency code 0001 (=6), CR0[3]=0 (variable), legacy wrap, 32 B group.
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b0, 1'b1, 2'b11}; // 0x8F17
  localparam int unsigned REG_MSB = ADDR_WIDTH - 1;

  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;    end
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90 = ~clk90;  end
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  logic [ADDR_WIDTH-1:0]   avs_address;
  logic                    avs_read, avs_write;
  logic [DATA_WIDTH-1:0]   avs_writedata;
  logic [STRB_WIDTH-1:0]   avs_byteenable;
  logic [LEN_WIDTH-1:0]    avs_burstcount;
  logic [DATA_WIDTH-1:0]   avs_readdata;
  logic                    avs_readdatavalid;
  logic                    avs_waitrequest;
  logic                    init_done;

  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  phy_dq_o;   logic phy_dq_oe;
  logic                 phy_rwds_o; logic phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;
  logic                 mdl_rwds_o; logic mdl_rwds_oe;
  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);
  localparam realtime RTT = 3.0;
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  // DUT: variable-latency Avalon top.
  hyperram_avalon #(
    .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b0), .MAX_BURST_WORDS (0),
    .PROGRAM_CR (1'b1), .POR_DELAY_CYCLES (0), .INIT_CR0 (TB_INIT_CR0),
    .PHY_VARIANT ("GENERIC"), .DIFF_CK (1'b1)
  ) dut (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // runtime read-eye calibration tied to POR-equivalent constants (reproduces pre-cal behaviour)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd0), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .avs_address (avs_address), .avs_read (avs_read), .avs_write (avs_write),
    .avs_writedata (avs_writedata), .avs_byteenable (avs_byteenable),
    .avs_burstcount (avs_burstcount), .avs_readdata (avs_readdata),
    .avs_readdatavalid (avs_readdatavalid), .avs_waitrequest (avs_waitrequest),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (phy_dq_o), .hb_dq_oe (phy_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (phy_rwds_o), .hb_rwds_oe (phy_rwds_oe), .hb_rwds_i (rwds_line_dly),
    .init_done (init_done), .err_underrun (/* unused */), .dbg_bus (),
    // issue #13: new hyperram_avalon debug bundle + wrap_en tied to per-instance legacy (A1).
    .dbg_wr_lat_trim (4'd0), .dbg_lat_clocks (4'd6), .dbg_cr0_reprog (1'b0),
    .dbg_prewin_drive (1'b0), .dbg_prewin_n (3'd0), .dbg_prewin_marker (1'b0),
    .dbg_postwin_hold (1'b0), .dbg_prewin_contig (1'b0), .dbg_end_cwrite (1'b0), .wrap_en (1'b0)
  );

  // Model: VARIABLE latency, collision every 2nd transaction (alternating 1x/2x).
  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b0),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (2)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i (dq_line), .hb_dq_ie (phy_dq_oe), .hb_dq_o (mdl_dq_o), .hb_dq_oe (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (phy_rwds_oe), .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );

  int unsigned errors = 0, checks = 0;
  localparam int unsigned CAP_MAX = 64;
  logic [DATA_WIDTH-1:0] cap [CAP_MAX];
  int unsigned           cap_n;
  logic                  capturing;

  // 2x-latency witness: during the CA phase the master drives DQ (phy_dq_oe) while the DEVICE owns
  // RWDS as the latency indicator (mdl_rwds_oe). A High there (mdl_rwds_o) is the device asking for the
  // additional (2x) latency count. Counting it proves the collision/2x path actually fired (the test
  // is not trivially all-1x) — and since all data still checks out, both counts are handled.
  int unsigned lat2x_cycles = 0;
  // The witness samples the async device bus (hb_cs_n) in a clk-synchronous monitor; that dual use is
  // intentional and TB-only (waive SYNCASYNCNET locally, matching the house style in the PHY/model).
  /* verilator lint_off SYNCASYNCNET */
  always @(posedge clk) begin
    if (capturing && avs_readdatavalid) begin
      if (cap_n < CAP_MAX) cap[cap_n] <= avs_readdata;
      cap_n <= cap_n + 1;
    end
    if (!hb_cs_n && phy_dq_oe && mdl_rwds_oe && mdl_rwds_o) lat2x_cycles <= lat2x_cycles + 1;
  end
  /* verilator lint_on SYNCASYNCNET */

  function automatic logic [DATA_WIDTH-1:0] genword(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction

  task automatic avs_idle();
    @(negedge clk);
    avs_address='0; avs_read=1'b0; avs_write=1'b0; avs_writedata='0; avs_byteenable='1; avs_burstcount='0;
  endtask

  task automatic do_write(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                          input logic [DATA_WIDTH-1:0] data [$]);
    int unsigned idx, g;
    idx=0; @(negedge clk);
    avs_write=1'b1; avs_read=1'b0; avs_address={1'b0, addr[ADDR_WIDTH-2:0]};
    avs_burstcount=LEN_WIDTH'(n); avs_byteenable='1; avs_writedata=data[0];
    g=0;
    forever begin @(posedge clk); g=g+1;
      if (g>5000) begin $display("[%0t] HANG do_write @0x%08x", $time, addr); errors=errors+1; break; end
      if (!avs_waitrequest) begin idx=idx+1; if (idx==n) break; @(negedge clk); avs_writedata=data[idx]; end
    end
    avs_idle();
  endtask

  task automatic do_read(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n);
    int unsigned guard;
    cap_n=0; capturing=1'b1; @(negedge clk);
    avs_read=1'b1; avs_write=1'b0; avs_address={1'b0, addr[ADDR_WIDTH-2:0]}; avs_burstcount=LEN_WIDTH'(n);
    guard=0;
    forever begin @(posedge clk); guard=guard+1;
      if (guard>5000) begin $display("[%0t] HANG do_read @0x%08x", $time, addr); errors=errors+1; break; end
      if (!avs_waitrequest) break; end
    avs_idle();
    guard=0; while (cap_n<n && guard<8000) begin @(posedge clk); guard=guard+1; end
    @(posedge clk); capturing=1'b0;
    if (cap_n<n) begin $display("[%0t] ERROR read %0d @0x%08x got %0d", $time, n, addr, cap_n);
                       errors=errors+1; end
  endtask

  task automatic wr_rd_check(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n, input string tag);
    logic [DATA_WIDTH-1:0] wdata [$]; logic [DATA_WIDTH-1:0] exp; int unsigned i;
    wdata = {}; for (i=0;i<n;i++) wdata.push_back(genword(addr+i));
    do_write(addr, n, wdata);
    do_read (addr, n);
    for (i=0;i<n;i++) begin exp=genword(addr+i); checks=checks+1;
      if (cap[i]!==exp) begin
        $display("[%0t] ERROR (%s): @0x%08x word %0d got 0x%04x exp 0x%04x", $time, tag, addr, i, cap[i], exp);
        errors=errors+1; end end
    $display("[%0t] %s: %0d-word wr/rd @0x%08x done (errors so far %0d)", $time, tag, n, addr, errors);
  endtask

  int unsigned guard;
  initial begin
    avs_address='0; avs_read=1'b0; avs_write=1'b0; avs_writedata='0; avs_byteenable='1; avs_burstcount='0;
    capturing=1'b0; cap_n=0; rst=1'b1;
    repeat (5) @(posedge clk); @(negedge clk); rst=1'b0;
    guard=0; while (!init_done && guard<100000) begin @(posedge clk); guard=guard+1; end
    if (!init_done) begin $display("[%0t] FATAL init_done", $time); errors=errors+1; end
    else $display("[%0t] init_done asserted (variable-latency device, REFRESH_EVERY=2)", $time);
    repeat (4) @(posedge clk);

    // A long alternating stream of writes+reads. With REFRESH_EVERY=2 the device flips 1x<->2x every
    // transaction; every write and every read must stay byte-aligned regardless of which count it got.
    wr_rd_check(32'h0000_0000, 1,  "single0");
    wr_rd_check(32'h0000_0021, 1,  "single1");
    wr_rd_check(32'h0000_0042, 1,  "single2");
    wr_rd_check(32'h0000_0063, 1,  "single3");
    wr_rd_check(32'h0000_0100, 4,  "burst4");
    wr_rd_check(32'h0000_0200, 8,  "burst8");
    wr_rd_check(32'h0000_0300, 3,  "burst3");
    wr_rd_check(32'h0000_1000, 16, "burst16");
    wr_rd_check(32'h0000_1100, 5,  "burst5");
    wr_rd_check(32'h0000_2000, 2,  "burst2");

    // Confirm the device actually exercised the 2x (collision) latency path at least once.
    checks = checks + 1;
    if (lat2x_cycles == 0) begin
      $display("[%0t] ERROR: device never requested 2x latency — variable path not exercised", $time);
      errors = errors + 1;
    end else
      $display("[%0t] 2x-latency (collision) CA cycles observed: %0d — alternation exercised", $time, lat2x_cycles);

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_varlat done: %0d checks, %0d errors", $time, checks, errors);
    if (errors==0) begin $display("TB_RESULT: PASS"); $finish; end
    else begin $display("TB_RESULT: FAIL"); $fatal(1, "tb_varlat: %0d errors", errors); end
  end

  initial begin
    #4_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_varlat: global timeout");
  end

endmodule
