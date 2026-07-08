// tb_fixed2x — self-checking Verilator TB: controller against a FIXED-latency HyperRAM that drives
// RWDS HIGH during CA and uses TWO latency counts (SPEC_DIGEST §5.2.4; the common Cypress/Winbond
// W957D8NB behaviour). De-blinds the fixed-latency findings: the controller must ALWAYS decode the
// slave-driven RWDS level during CA and double the initial latency, even though FIXED_LATENCY=1.
//
// The DUT (hyperram_axi) is built with FIXED_LATENCY=1, LATENCY_CLOCKS=6. The golden model is built
// with FIXED_2X=1, so it drives RWDS high during CA and delivers/captures data only after 2x=12
// latency clocks. A controller that hard-coded 1x (the pre-fix bug) would misalign every read AND
// write; this TB checks byte-exact read-back of both, so it fails against the buggy controller.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_fixed2x;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH       = HB_DQ_WIDTH_DEFAULT;
  localparam int unsigned DATA_WIDTH     = 2 * DQ_WIDTH;
  localparam int unsigned ADDR_WIDTH     = HB_ADDR_WIDTH;
  localparam int unsigned LEN_WIDTH      = HB_LEN_WIDTH_DEFAULT;
  localparam int unsigned ID_WIDTH       = 4;
  localparam int unsigned AXI_DATA_WIDTH = DATA_WIDTH;
  localparam int unsigned AXI_ADDR_WIDTH = ADDR_WIDTH + 1;
  localparam int unsigned AXI_STRB_WIDTH = AXI_DATA_WIDTH / 8;

  localparam logic [1:0] AXI_INCR = 2'b01;
  localparam logic [1:0] AXI_OKAY = 2'b00;

  // CR0 image: latency code 0001 (=6), fixed-latency, legacy wrap, 32B group.
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F

  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;   end
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90 = ~clk90; end
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  logic [ID_WIDTH-1:0]       awid;   logic [AXI_ADDR_WIDTH-1:0] awaddr;
  logic [7:0] awlen;  logic [2:0] awsize;  logic [1:0] awburst;  logic awvalid, awready;
  logic [AXI_DATA_WIDTH-1:0] wdata;  logic [AXI_STRB_WIDTH-1:0] wstrb;  logic wlast, wvalid, wready;
  logic [ID_WIDTH-1:0] bid;  logic [1:0] bresp;  logic bvalid, bready;
  logic [ID_WIDTH-1:0] arid;  logic [AXI_ADDR_WIDTH-1:0] araddr;
  logic [7:0] arlen;  logic [2:0] arsize;  logic [1:0] arburst;  logic arvalid, arready;
  logic [ID_WIDTH-1:0] rid;  logic [AXI_DATA_WIDTH-1:0] rdata;  logic [1:0] rresp;
  logic rlast, rvalid, rready;  logic init_done;

  logic hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0] phy_dq_o;  logic phy_dq_oe;  logic phy_rwds_o, phy_rwds_oe;
  logic [DQ_WIDTH-1:0] mdl_dq_o;  logic mdl_dq_oe;  logic mdl_rwds_o, mdl_rwds_oe;
  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);
  localparam realtime RTT = 3.0;    // ns round-trip flight delay on the read path (finding #4)
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  hyperram_axi #(
    .ID_WIDTH (ID_WIDTH), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1), .MAX_BURST_WORDS (0),
    .PROGRAM_CR (1'b1), .POR_DELAY_CYCLES (0), .INIT_CR0 (TB_INIT_CR0),
    .PHY_VARIANT ("GENERIC"), .DIFF_CK (1'b1)
  ) dut (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    .awid (awid), .awaddr (awaddr), .awlen (awlen), .awsize (awsize), .awburst (awburst),
    .awvalid (awvalid), .awready (awready),
    .wdata (wdata), .wstrb (wstrb), .wlast (wlast), .wvalid (wvalid), .wready (wready),
    .bid (bid), .bresp (bresp), .bvalid (bvalid), .bready (bready),
    .arid (arid), .araddr (araddr), .arlen (arlen), .arsize (arsize), .arburst (arburst),
    .arvalid (arvalid), .arready (arready),
    .rid (rid), .rdata (rdata), .rresp (rresp), .rlast (rlast), .rvalid (rvalid), .rready (rready),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (phy_dq_o), .hb_dq_oe (phy_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (phy_rwds_o), .hb_rwds_oe (phy_rwds_oe), .hb_rwds_i (rwds_line_dly),
    .init_done (init_done)
  );

  // Golden model: FIXED latency, but the RWDS-high / 2x variant.
  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0), .FIXED_2X (1'b1)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i (dq_line), .hb_dq_ie (phy_dq_oe), .hb_dq_o (mdl_dq_o), .hb_dq_oe (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (phy_rwds_oe), .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );

  int unsigned errors = 0, checks = 0;
  localparam int unsigned CAP_MAX = 64;
  logic [DATA_WIDTH-1:0] cap [CAP_MAX];  int unsigned cap_n;

  function automatic logic [DATA_WIDTH-1:0] genword(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction
  function automatic logic [AXI_ADDR_WIDTH-1:0] byte_addr(input logic [ADDR_WIDTH-1:0] wa);
    logic [AXI_ADDR_WIDTH-1:0] b; b = '0; b[ADDR_WIDTH-1:1] = wa[ADDR_WIDTH-2:0]; return b;
  endfunction

  task automatic axi_idle();
    @(negedge clk);
    awid='0; awaddr='0; awlen='0; awsize=3'd1; awburst=AXI_INCR; awvalid=1'b0;
    wdata='0; wstrb='1; wlast=1'b0; wvalid=1'b0; bready=1'b0;
    arid='0; araddr='0; arlen='0; arsize=3'd1; arburst=AXI_INCR; arvalid=1'b0; rready=1'b0;
  endtask

  task automatic axi_write(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                           input logic [DATA_WIDTH-1:0] data [$]);
    int unsigned idx, g;
    @(negedge clk);
    awid=4'h5; awaddr=byte_addr(addr); awlen=8'(n-1); awsize=3'd1; awburst=AXI_INCR; awvalid=1'b1;
    g=0; forever begin @(posedge clk); g=g+1; if (awready) break;
      if (g>3000) begin $display("[%0t] HANG AW", $time); errors=errors+1; break; end end
    @(negedge clk); awvalid=1'b0;
    idx=0; @(negedge clk); wvalid=1'b1; wstrb='1; wdata=data[0]; wlast=(n==1);
    g=0; forever begin @(posedge clk); g=g+1;
      if (g>6000) begin $display("[%0t] HANG W", $time); errors=errors+1; break; end
      if (wready) begin idx=idx+1; if (idx==n) break; @(negedge clk); wdata=data[idx]; wlast=(idx==n-1); end
    end
    @(negedge clk); wvalid=1'b0; wlast=1'b0;
    @(negedge clk); bready=1'b1;
    g=0; forever begin @(posedge clk); g=g+1;
      if (bvalid) begin checks=checks+1;
        if (bresp!==AXI_OKAY) begin $display("[%0t] ERROR bresp=0x%0x", $time, bresp); errors=errors+1; end break; end
      if (g>3000) begin $display("[%0t] HANG B", $time); errors=errors+1; break; end end
    @(negedge clk); bready=1'b0; axi_idle();
  endtask

  task automatic axi_read(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n);
    int unsigned g; cap_n=0;
    @(negedge clk);
    arid=4'h5; araddr=byte_addr(addr); arlen=8'(n-1); arsize=3'd1; arburst=AXI_INCR; arvalid=1'b1;
    g=0; forever begin @(posedge clk); g=g+1; if (arready) break;
      if (g>3000) begin $display("[%0t] HANG AR", $time); errors=errors+1; break; end end
    @(negedge clk); arvalid=1'b0;
    @(negedge clk); rready=1'b1;
    g=0; forever begin @(posedge clk); g=g+1;
      if (rvalid) begin if (cap_n<CAP_MAX) cap[cap_n]=rdata; checks=checks+1;
        if (rresp!==AXI_OKAY) begin $display("[%0t] ERROR rresp=0x%0x", $time, rresp); errors=errors+1; end
        if ((cap_n==n-1) && !rlast) begin $display("[%0t] ERROR rlast missing", $time); errors=errors+1; end
        cap_n=cap_n+1; if (cap_n==n) break; end
      if (g>6000) begin $display("[%0t] HANG R got %0d/%0d", $time, cap_n, n); errors=errors+1; break; end end
    @(negedge clk); rready=1'b0; axi_idle();
  endtask

  task automatic wr_rd(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n, input string tag);
    logic [DATA_WIDTH-1:0] wdat [$]; logic [DATA_WIDTH-1:0] exp; int unsigned i;
    wdat = {}; for (i=0;i<n;i++) wdat.push_back(genword(addr+i));
    axi_write(addr, n, wdat);
    axi_read (addr, n);
    for (i=0;i<n;i++) begin exp=genword(addr+i); checks=checks+1;
      if (cap[i]!==exp) begin
        $display("[%0t] ERROR (%s): @0x%08x beat %0d got 0x%04x exp 0x%04x", $time, tag, addr, i, cap[i], exp);
        errors=errors+1; end end
    $display("[%0t] %s: %0d-beat wr/rd @0x%08x done (errors so far %0d)", $time, tag, n, addr, errors);
  endtask

  initial begin
    awid='0; awaddr='0; awlen='0; awsize=3'd1; awburst=AXI_INCR; awvalid=1'b0;
    wdata='0; wstrb='1; wlast=1'b0; wvalid=1'b0; bready=1'b0;
    arid='0; araddr='0; arlen='0; arsize=3'd1; arburst=AXI_INCR; arvalid=1'b0; rready=1'b0;
    cap_n=0; rst=1'b1;
    repeat (5) @(posedge clk); @(negedge clk); rst=1'b0;
    begin int unsigned g; g=0; while (!init_done && g<100000) begin @(posedge clk); g=g+1; end end
    if (!init_done) begin $display("[%0t] FATAL: init_done never asserted", $time); errors=errors+1; end
    else $display("[%0t] init_done asserted (fixed-2x device)", $time);
    repeat (4) @(posedge clk);

    wr_rd(32'h0000_0000, 1,  "single");
    wr_rd(32'h0000_0033, 1,  "single");
    wr_rd(32'h0000_0040, 4,  "burst4");
    wr_rd(32'h0000_0200, 8,  "burst8");
    wr_rd(32'h0000_1000, 16, "burst16");

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_fixed2x done: %0d checks, %0d errors", $time, checks, errors);
    if (errors==0) begin $display("TB_RESULT: PASS"); $finish; end
    else begin $display("TB_RESULT: FAIL"); $fatal(1, "tb_fixed2x: %0d errors", errors); end
  end

  initial begin #3_000_000; $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_fixed2x: global timeout"); end
endmodule
