// tb_axi_wrap — self-checking Verilator TB for AXI front-end paths untested elsewhere (issue #4):
//   * B4 — AXI WRAP *write* decomposition (hyperbus_axi ~255-259 seg setup, ~306-321 walk). tb_axi
//     only issues AXI_WRAP via axi_read; every axi_write there is INCR/FIXED, so the WRAP-write
//     second segment (region-base .. start-1) is never driven. Here a WRAP write is decomposed into
//     seg0+seg1 and read back (both WRAP and INCR) to prove every wrapped address was written.
//   * B10 — AR/AW round-robin arbiter (hyperbus_axi:200-203). tb_axi is strictly sequential, so AR
//     and AW are never simultaneously valid and the last_was_write toggle never runs. Here both
//     address channels are driven valid at once; the granted order is checked against the round-robin
//     prediction (write-first when the previous grant was a read, read-first when it was a write) and
//     both transactions must complete with correct data.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_axi_wrap;
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
  localparam logic [1:0] AXI_WRAP = 2'b10;
  localparam logic [1:0] AXI_OKAY = 2'b00;

  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F

  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;    end
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90 = ~clk90;  end
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  logic [ID_WIDTH-1:0]       awid;   logic [AXI_ADDR_WIDTH-1:0] awaddr;
  logic [7:0] awlen; logic [2:0] awsize; logic [1:0] awburst; logic awvalid, awready;
  logic [AXI_DATA_WIDTH-1:0] wdata; logic [AXI_STRB_WIDTH-1:0] wstrb; logic wlast, wvalid, wready;
  logic [ID_WIDTH-1:0] bid; logic [1:0] bresp; logic bvalid, bready;
  logic [ID_WIDTH-1:0] arid; logic [AXI_ADDR_WIDTH-1:0] araddr;
  logic [7:0] arlen; logic [2:0] arsize; logic [1:0] arburst; logic arvalid, arready;
  logic [ID_WIDTH-1:0] rid; logic [AXI_DATA_WIDTH-1:0] rdata; logic [1:0] rresp;
  logic rlast, rvalid, rready; logic init_done;

  logic hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0] phy_dq_o; logic phy_dq_oe; logic phy_rwds_o, phy_rwds_oe;
  logic [DQ_WIDTH-1:0] mdl_dq_o; logic mdl_dq_oe; logic mdl_rwds_o, mdl_rwds_oe;
  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);
  localparam realtime RTT = 3.0;
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

  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i (dq_line), .hb_dq_ie (phy_dq_oe), .hb_dq_o (mdl_dq_o), .hb_dq_oe (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (phy_rwds_oe), .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );

  int unsigned errors = 0, checks = 0;
  localparam int unsigned CAP_MAX = 64;
  logic [DATA_WIDTH-1:0] cap [CAP_MAX];
  int unsigned cap_n;

  function automatic logic [DATA_WIDTH-1:0] genA(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction
  function automatic logic [DATA_WIDTH-1:0] genB(input int unsigned k);
    return (16'(k) * 16'h2545) ^ 16'hC0DE;
  endfunction
  function automatic logic [AXI_ADDR_WIDTH-1:0] byte_addr(input logic [ADDR_WIDTH-1:0] wa,
                                                          input logic reg_space);
    logic [AXI_ADDR_WIDTH-1:0] b; b = '0;
    b[AXI_ADDR_WIDTH-1] = reg_space; b[ADDR_WIDTH-1:1] = wa[ADDR_WIDTH-2:0]; return b;
  endfunction

  task automatic axi_idle();
    @(negedge clk);
    awid='0; awaddr='0; awlen='0; awsize=3'd1; awburst=AXI_INCR; awvalid=1'b0;
    wdata='0; wstrb='1; wlast=1'b0; wvalid=1'b0; bready=1'b0;
    arid='0; araddr='0; arlen='0; arsize=3'd1; arburst=AXI_INCR; arvalid=1'b0; rready=1'b0;
  endtask

  // --- address-channel present + accept (leaves valid deasserted after the accepting edge) ---
  task automatic aw_present(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                            input logic reg_space, input logic [1:0] burst);
    int unsigned g;
    @(negedge clk);
    awid=4'h5; awaddr=byte_addr(addr,reg_space); awlen=8'(n-1); awsize=3'd1; awburst=burst; awvalid=1'b1;
    g=0; forever begin @(posedge clk); g=g+1; if (awready) break;
      if (g>3000) begin $display("[%0t] HANG AW @0x%08x", $time, addr); errors=errors+1; break; end end
    @(negedge clk); awvalid=1'b0;
  endtask

  task automatic ar_present(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                            input logic reg_space, input logic [1:0] burst);
    int unsigned g;
    @(negedge clk);
    arid=4'h6; araddr=byte_addr(addr,reg_space); arlen=8'(n-1); arsize=3'd1; arburst=burst; arvalid=1'b1;
    g=0; forever begin @(posedge clk); g=g+1; if (arready) break;
      if (g>3000) begin $display("[%0t] HANG AR @0x%08x", $time, addr); errors=errors+1; break; end end
    @(negedge clk); arvalid=1'b0;
  endtask

  // --- data-phase tails (address channel already accepted) ---
  task automatic w_burst(input int unsigned n, input logic [DATA_WIDTH-1:0] data [$]);
    int unsigned idx, g;
    idx=0; @(negedge clk); wvalid=1'b1; wstrb='1; wdata=data[0]; wlast=(n==1);
    g=0; forever begin @(posedge clk); g=g+1;
      if (g>5000) begin $display("[%0t] HANG W idx=%0d", $time, idx); errors=errors+1; break; end
      if (wready) begin idx=idx+1; if (idx==n) break; @(negedge clk); wdata=data[idx]; wlast=(idx==n-1); end end
    @(negedge clk); wvalid=1'b0; wlast=1'b0;
    @(negedge clk); bready=1'b1;
    g=0; forever begin @(posedge clk); g=g+1;
      if (bvalid) begin checks=checks+1;
        if (bresp!==AXI_OKAY) begin $display("[%0t] ERROR bresp=0x%0x", $time, bresp); errors=errors+1; end break; end
      if (g>3000) begin $display("[%0t] HANG B", $time); errors=errors+1; break; end end
    @(negedge clk); bready=1'b0;
  endtask

  task automatic r_burst(input int unsigned n);
    int unsigned g;
    cap_n=0; @(negedge clk); rready=1'b1;
    g=0; forever begin @(posedge clk); g=g+1;
      if (rvalid) begin if (cap_n<CAP_MAX) cap[cap_n]=rdata; checks=checks+1;
        if (rresp!==AXI_OKAY) begin $display("[%0t] ERROR rresp=0x%0x beat %0d", $time, rresp, cap_n); errors=errors+1; end
        if ((cap_n==n-1) && !rlast) begin $display("[%0t] ERROR rlast missing beat %0d", $time, cap_n); errors=errors+1; end
        cap_n=cap_n+1; if (cap_n==n) break; end
      if (g>5000) begin $display("[%0t] HANG R got %0d/%0d", $time, cap_n, n); errors=errors+1; break; end end
    @(negedge clk); rready=1'b0;
  endtask

  task automatic axi_write(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n, input logic rg,
                           input logic [1:0] burst, input logic [DATA_WIDTH-1:0] data [$]);
    aw_present(addr, n, rg, burst); w_burst(n, data); axi_idle();
  endtask
  task automatic axi_read(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n, input logic rg,
                          input logic [1:0] burst);
    ar_present(addr, n, rg, burst); r_burst(n); axi_idle();
  endtask

  // B4 — WRAP write, then read back (WRAP and INCR) to prove seg0+seg1 wrote every wrapped address.
  task automatic wrap_write_verify(input logic [ADDR_WIDTH-1:0] rbase, input int unsigned n,
                                   input int unsigned off, input string tag);
    logic [DATA_WIDTH-1:0] pa [$]; logic [DATA_WIDTH-1:0] wb [$]; logic [DATA_WIDTH-1:0] exp;
    logic [ADDR_WIDTH-1:0] start;
    int unsigned i, kk;
    start = rbase + ADDR_WIDTH'(off);
    pa = {}; for (i=0;i<n;i++) pa.push_back(genA(rbase+i));
    axi_write(rbase, n, 1'b0, AXI_INCR, pa);                 // INCR pre-fill (pattern A)
    wb = {}; for (i=0;i<n;i++) wb.push_back(genB(i));
    axi_write(start, n, 1'b0, AXI_WRAP, wb);                 // WRAP write (pattern B, seg0+seg1)
    // INCR read-back of the whole region: address rbase+j was written by beat (j-off) mod n.
    axi_read(rbase, n, 1'b0, AXI_INCR);
    for (i=0;i<n;i++) begin
      kk  = (i + n - off) % n;
      exp = genB(kk);
      checks=checks+1;
      if (cap[i]!==exp) begin
        $display("[%0t] ERROR (%s WRAP-wr): region word %0d (0x%08x) got 0x%04x exp 0x%04x (beat %0d)",
                 $time, tag, i, rbase+i, cap[i], exp, kk);
        errors=errors+1;
      end
    end
    // WRAP read at start must return the written beats in order (D_0..D_{n-1}).
    axi_read(start, n, 1'b0, AXI_WRAP);
    for (i=0;i<n;i++) begin
      exp = genB(i);
      checks=checks+1;
      if (cap[i]!==exp) begin
        $display("[%0t] ERROR (%s WRAP-rd): beat %0d got 0x%04x exp 0x%04x", $time, tag, i, cap[i], exp);
        errors=errors+1;
      end
    end
    $display("[%0t] %s: WRAP write rbase=0x%08x n=%0d off=%0d verified (errs %0d)",
             $time, tag, rbase, n, off, errors);
  endtask

  // B10 — present AW+AR simultaneously; check which is granted first, complete both, verify data.
  // waddr gets pattern B written; raddr must already hold genA() (pre-filled by the caller).
  task automatic simul_rw(input logic [ADDR_WIDTH-1:0] waddr, input logic [ADDR_WIDTH-1:0] raddr,
                          input int unsigned n, input logic exp_write_first, input string tag);
    logic [DATA_WIDTH-1:0] wd [$]; logic wfirst; int unsigned g, i;
    wd = {}; for (i=0;i<n;i++) wd.push_back(genB(i));
    // Present BOTH address channels valid on the same edge.
    @(negedge clk);
    awid=4'h1; awaddr=byte_addr(waddr,1'b0); awlen=8'(n-1); awsize=3'd1; awburst=AXI_INCR; awvalid=1'b1;
    arid=4'h2; araddr=byte_addr(raddr,1'b0); arlen=8'(n-1); arsize=3'd1; arburst=AXI_INCR; arvalid=1'b1;
    // Detect which channel the arbiter grants first (exactly one ready asserts in S_IDLE).
    wfirst=1'bx; g=0;
    forever begin
      @(posedge clk); g=g+1;
      if (awready) begin wfirst=1'b1; break; end
      if (arready) begin wfirst=1'b0; break; end
      if (g>3000) begin $display("[%0t] HANG simul grant", $time); errors=errors+1; break; end
    end
    checks=checks+1;
    if (wfirst !== exp_write_first) begin
      $display("[%0t] ERROR (%s): arbiter granted %s first, expected %s first", $time, tag,
               wfirst ? "WRITE" : "READ", exp_write_first ? "WRITE" : "READ");
      errors=errors+1;
    end else
      $display("[%0t] %s: arbiter granted %s first (round-robin ok)", $time, tag, wfirst ? "WRITE":"READ");

    if (wfirst) begin
      @(negedge clk); awvalid=1'b0;           // AW taken; AR still valid
      w_burst(n, wd);
      // read second: AR still asserted, wait its grant
      g=0; forever begin @(posedge clk); g=g+1; if (arready) break;
        if (g>3000) begin $display("[%0t] HANG simul AR2", $time); errors=errors+1; break; end end
      @(negedge clk); arvalid=1'b0;
      r_burst(n);
    end else begin
      @(negedge clk); arvalid=1'b0;           // AR taken; AW still valid
      r_burst(n);
      g=0; forever begin @(posedge clk); g=g+1; if (awready) break;
        if (g>3000) begin $display("[%0t] HANG simul AW2", $time); errors=errors+1; break; end end
      @(negedge clk); awvalid=1'b0;
      w_burst(n, wd);
    end
    axi_idle();

    // Verify the read returned the pre-filled pattern A.
    for (i=0;i<n;i++) begin
      logic [DATA_WIDTH-1:0] exp; exp=genA(raddr+i); checks=checks+1;
      if (cap[i]!==exp) begin
        $display("[%0t] ERROR (%s read): beat %0d got 0x%04x exp 0x%04x", $time, tag, i, cap[i], exp);
        errors=errors+1;
      end
    end
    // Verify the write landed (INCR read-back of waddr).
    axi_read(waddr, n, 1'b0, AXI_INCR);
    for (i=0;i<n;i++) begin
      logic [DATA_WIDTH-1:0] exp; exp=genB(i); checks=checks+1;
      if (cap[i]!==exp) begin
        $display("[%0t] ERROR (%s write): beat %0d got 0x%04x exp 0x%04x", $time, tag, i, cap[i], exp);
        errors=errors+1;
      end
    end
    $display("[%0t] %s: simultaneous AR+AW completed + verified (errs %0d)", $time, tag, errors);
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  logic [DATA_WIDTH-1:0] pf [$];
  int unsigned guard, i;
  initial begin
    axi_idle();
    cap_n=0; rst=1'b1;
    repeat (5) @(posedge clk); @(negedge clk); rst=1'b0;
    guard=0; while (!init_done && guard<100000) begin @(posedge clk); guard=guard+1; end
    if (!init_done) begin $display("[%0t] FATAL init_done", $time); errors=errors+1; end
    else $display("[%0t] init_done asserted (AXI wrap/arbiter harness)", $time);
    repeat (4) @(posedge clk);

    // ---- B4: AXI WRAP writes (drives the seg1 = region-base..start-1 walk) ----
    wrap_write_verify(32'h0000_0040, 4,  2,  "wrap4-wr");   // 4-word region, start mid-region
    wrap_write_verify(32'h0000_0080, 8,  5,  "wrap8-wr");   // 8-word region
    wrap_write_verify(32'h0000_0100, 16, 6,  "wrap16-wr");  // 16-word region
    wrap_write_verify(32'h0000_0200, 2,  1,  "wrap2-wr");   // minimal region

    // ---- B10: AR/AW round-robin arbiter (both valid at once) ----
    // Pre-fill the read targets with pattern A.
    pf = {}; for (i=0;i<4;i++) pf.push_back(genA(32'h0000_0500 + i));
    axi_write(32'h0000_0500, 4, 1'b0, AXI_INCR, pf);
    pf = {}; for (i=0;i<4;i++) pf.push_back(genA(32'h0000_0600 + i));
    axi_write(32'h0000_0600, 4, 1'b0, AXI_INCR, pf);

    // Standalone READ -> last_was_write=0, so a following simultaneous pair grants WRITE first.
    axi_read(32'h0000_0500, 4, 1'b0, AXI_INCR);
    simul_rw(32'h0000_0700, 32'h0000_0500, 4, 1'b1, "simul-wfirst");

    // Standalone WRITE -> last_was_write=1, so the next simultaneous pair grants READ first.
    pf = {}; for (i=0;i<4;i++) pf.push_back(genB(i));
    axi_write(32'h0000_0710, 4, 1'b0, AXI_INCR, pf);
    simul_rw(32'h0000_0720, 32'h0000_0600, 4, 1'b0, "simul-rfirst");

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_axi_wrap done: %0d checks, %0d errors", $time, checks, errors);
    if (errors==0) begin $display("TB_RESULT: PASS"); $finish; end
    else begin $display("TB_RESULT: FAIL"); $fatal(1, "tb_axi_wrap: %0d errors", errors); end
  end

  initial begin
    #4_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_axi_wrap: global timeout");
  end

endmodule
