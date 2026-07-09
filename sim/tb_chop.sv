// tb_chop — self-checking Verilator TB for tCSM burst chopping (issue #4, B1; SPEC_DIGEST §6).
//
// De-blinds the chop/re-open FSM in hyperbus_ctrl (seg_size / per-segment cur_addr advance /
// ST_RECOVER -> ST_CS). Every other TB builds with MAX_BURST_WORDS=0, so seg_size returns the whole
// length and the multi-segment re-open branch never runs. Here the Avalon top is built with a SMALL
// MAX_BURST_WORDS and driven with linear bursts LONGER than it, so a single caller burst is chopped
// into several back-to-back device CS# segments (a new CA at the advanced address each time),
// transparently to the Avalon master. Byte-exact read-back proves the per-segment address advance and
// re-open are correct; a controller that mis-advanced the segment address would corrupt later words.
//
// The chop is a controller behavior shared by both front-ends, so exercising it through the Avalon
// top fully covers the FSM. Cases: LEN == MAX (one segment, no re-open), LEN == MAX+1 (first re-open),
// and LEN >> MAX (many segments), at several base addresses.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_chop;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2

  // Chop threshold under test. Small so short bursts already span several CS# segments.
  localparam int unsigned MAX_BURST = 4;

  // CR0 image: latency code 0001 (=6), fixed, legacy wrap, 32 B group (matches the model latency).
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F

  localparam int unsigned REG_MSB = ADDR_WIDTH - 1;

  // --------------------------------------------------------------------
  // Clocking / reset
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;    end   // 100 MHz
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90 = ~clk90;  end // +90 deg
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  // --------------------------------------------------------------------
  // Avalon-MM slave signals
  // --------------------------------------------------------------------
  logic [ADDR_WIDTH-1:0]   avs_address;
  logic                    avs_read, avs_write;
  logic [DATA_WIDTH-1:0]   avs_writedata;
  logic [STRB_WIDTH-1:0]   avs_byteenable;
  logic [LEN_WIDTH-1:0]    avs_burstcount;
  logic [DATA_WIDTH-1:0]   avs_readdata;
  logic                    avs_readdatavalid;
  logic                    avs_waitrequest;
  logic                    init_done;

  // --------------------------------------------------------------------
  // HyperBus device pins + bus resolution (identical scheme to tb_avalon)
  // --------------------------------------------------------------------
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  phy_dq_o;   logic phy_dq_oe;
  logic                 phy_rwds_o; logic phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;
  logic                 mdl_rwds_o; logic mdl_rwds_oe;

  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);

  localparam realtime RTT = 3.0;    // ns device->master flight delay (read path)
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  // --------------------------------------------------------------------
  // DUT: Avalon top with SMALL MAX_BURST_WORDS (chopping enabled)
  // --------------------------------------------------------------------
  hyperram_avalon #(
    .LATENCY_CLOCKS   (6),
    .FIXED_LATENCY    (1'b1),
    .MAX_BURST_WORDS  (MAX_BURST),
    .PROGRAM_CR       (1'b1),
    .POR_DELAY_CYCLES (0),
    .INIT_CR0         (TB_INIT_CR0),
    .PHY_VARIANT      ("GENERIC"),
    .DIFF_CK          (1'b1)
  ) dut (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    .avs_address       (avs_address),
    .avs_read          (avs_read),
    .avs_write         (avs_write),
    .avs_writedata     (avs_writedata),
    .avs_byteenable    (avs_byteenable),
    .avs_burstcount    (avs_burstcount),
    .avs_readdata      (avs_readdata),
    .avs_readdatavalid (avs_readdatavalid),
    .avs_waitrequest   (avs_waitrequest),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (phy_dq_o), .hb_dq_oe (phy_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (phy_rwds_o), .hb_rwds_oe (phy_rwds_oe), .hb_rwds_i (rwds_line_dly),
    .init_done (init_done), .err_underrun (/* unused */)
  );

  // --------------------------------------------------------------------
  // Golden device model
  // --------------------------------------------------------------------
  hyperram_model #(
    .DQ_WIDTH       (DQ_WIDTH),
    .MEM_WORDS      (1 << 16),
    .LATENCY_CLOCKS (6),
    .FIXED_LATENCY  (1'b1),
    .ROW_WORDS      (0),
    .ROW_PENALTY    (4),
    .REFRESH_EVERY  (0)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i  (dq_line),  .hb_dq_ie  (phy_dq_oe),
    .hb_dq_o  (mdl_dq_o), .hb_dq_oe  (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (phy_rwds_oe),
    .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );

  // --------------------------------------------------------------------
  // Scoreboard + capture (same skeleton as tb_avalon)
  // --------------------------------------------------------------------
  int unsigned errors = 0;
  int unsigned checks = 0;

  localparam int unsigned CAP_MAX = 256;
  logic [DATA_WIDTH-1:0] cap [CAP_MAX];
  int unsigned           cap_n;
  logic                  capturing;

  always @(posedge clk) begin
    if (capturing && avs_readdatavalid) begin
      if (cap_n < CAP_MAX) cap[cap_n] <= avs_readdata;
      cap_n <= cap_n + 1;
    end
  end

  function automatic logic [DATA_WIDTH-1:0] genword(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction

  // --------------------------------------------------------------------
  // Avalon transaction tasks (copied from tb_avalon's proven skeleton)
  // --------------------------------------------------------------------
  task automatic avs_idle();
    @(negedge clk);
    avs_address    = '0;
    avs_read       = 1'b0;
    avs_write      = 1'b0;
    avs_writedata  = '0;
    avs_byteenable = '1;
    avs_burstcount = '0;
  endtask

  task automatic do_write(input logic [ADDR_WIDTH-1:0] addr,
                          input int unsigned            n,
                          input logic                   reg_space,
                          input logic [DATA_WIDTH-1:0]  data [$]);
    int unsigned idx, g;
    logic [ADDR_WIDTH-1:0] a_full;
    a_full = addr;
    a_full[REG_MSB] = reg_space;
    idx = 0;
    @(negedge clk);
    avs_write      = 1'b1;
    avs_read       = 1'b0;
    avs_address    = a_full;
    avs_burstcount = LEN_WIDTH'(n);
    avs_byteenable = '1;
    avs_writedata  = data[0];
    g = 0;
    forever begin
      @(posedge clk);
      g = g + 1;
      if (g > 5000) begin
        $display("[%0t] HANG do_write @0x%08x idx=%0d/%0d", $time, addr, idx, n);
        errors = errors + 1; break;
      end
      if (!avs_waitrequest) begin
        idx = idx + 1;
        if (idx == n) break;
        @(negedge clk);
        avs_writedata = data[idx];
      end
    end
    avs_idle();
  endtask

  task automatic do_read(input logic [ADDR_WIDTH-1:0] addr,
                         input int unsigned            n,
                         input logic                   reg_space);
    logic [ADDR_WIDTH-1:0] a_full;
    int unsigned guard;
    a_full = addr;
    a_full[REG_MSB] = reg_space;
    cap_n     = 0;
    capturing = 1'b1;
    @(negedge clk);
    avs_read       = 1'b1;
    avs_write      = 1'b0;
    avs_address    = a_full;
    avs_burstcount = LEN_WIDTH'(n);
    guard = 0;
    forever begin
      @(posedge clk);
      guard = guard + 1;
      if (guard > 5000) begin
        $display("[%0t] HANG do_read accept @0x%08x", $time, addr);
        errors = errors + 1; break;
      end
      if (!avs_waitrequest) break;
    end
    avs_idle();
    guard = 0;
    while (cap_n < n && guard < 8000) begin
      @(posedge clk);
      guard = guard + 1;
    end
    @(posedge clk);
    capturing = 1'b0;
    if (cap_n < n) begin
      $display("[%0t] ERROR: read of %0d words at 0x%08x returned only %0d", $time, n, addr, cap_n);
      errors = errors + 1;
    end
  endtask

  // Write then read back a linear memory burst, checking each word. LEN may exceed MAX_BURST so the
  // controller chops both the write and the read into segments.
  task automatic wr_rd_check(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                             input string tag);
    logic [DATA_WIDTH-1:0] wdata [$];
    logic [DATA_WIDTH-1:0] exp;
    int unsigned i;
    wdata = {};
    for (i = 0; i < n; i++) wdata.push_back(genword(addr + i));
    do_write(addr, n, 1'b0, wdata);
    do_read (addr, n, 1'b0);
    for (i = 0; i < n; i++) begin
      exp = genword(addr + i);
      checks = checks + 1;
      if (cap[i] !== exp) begin
        $display("[%0t] ERROR (%s): addr 0x%08x word %0d got 0x%04x exp 0x%04x",
                 $time, tag, addr, i, cap[i], exp);
        errors = errors + 1;
      end
    end
    $display("[%0t] %s: %0d-word wr/rd @0x%08x done (MAX_BURST=%0d, errors so far %0d)",
             $time, tag, n, addr, MAX_BURST, errors);
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  int unsigned guard;

  initial begin
    avs_address    = '0;
    avs_read       = 1'b0;
    avs_write      = 1'b0;
    avs_writedata  = '0;
    avs_byteenable = '1;
    avs_burstcount = '0;
    capturing = 1'b0;
    cap_n     = 0;
    rst       = 1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk);
    rst = 1'b0;

    guard = 0;
    while (!init_done && guard < 100000) begin @(posedge clk); guard = guard + 1; end
    if (!init_done) begin
      $display("[%0t] FATAL: init_done never asserted", $time);
      errors = errors + 1;
    end else begin
      $display("[%0t] init_done asserted (chop MAX_BURST=%0d)", $time, MAX_BURST);
    end
    repeat (4) @(posedge clk);

    // Edge cases around the chop threshold.
    wr_rd_check(32'h0000_0000, MAX_BURST,     "len==max");   // single segment, no re-open
    wr_rd_check(32'h0000_0010, MAX_BURST + 1, "len==max+1"); // first re-open (segments 4 + 1)
    wr_rd_check(32'h0000_0020, MAX_BURST - 1, "len<max");    // below threshold, no chop

    // Multi-segment bursts at several base addresses / lengths.
    wr_rd_check(32'h0000_0100, 5,  "chop5");    // 4 + 1
    wr_rd_check(32'h0000_0200, 8,  "chop8");    // 4 + 4
    wr_rd_check(32'h0000_0333, 10, "chop10");   // 4 + 4 + 2, unaligned base
    wr_rd_check(32'h0000_0400, 16, "chop16");   // 4 segments
    wr_rd_check(32'h0000_1abc, 20, "chop20");   // 5 segments, unaligned base
    wr_rd_check(32'h0000_2000, 7,  "chop7");    // 4 + 3

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_chop done: %0d checks, %0d errors", $time, checks, errors);
    if (errors == 0) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_chop: %0d errors", errors);
    end
  end

  initial begin
    #4_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_chop: global timeout");
  end

endmodule
