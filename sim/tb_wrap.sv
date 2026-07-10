// tb_wrap — self-checking Verilator TB for native wrapped / legacy / hybrid bursts (issue #4, B2;
// SPEC_DIGEST §7). This is the ONLY TB that drives a WRAPPED HyperBus CA (CA[45]=0).
//
// Both front-ends tie cmd_wrap=0, so the controller's wrapped path and the model's legacy/hybrid
// next_addr generator are dead in every top-level TB. This TB therefore instantiates the controller
// DIRECTLY (native command channel + generic PHY + golden model) and drives cmd_wrap=1, so a real
// wrapped CA reaches the device. The device then returns/accepts words in wrap order; a LINEAR CA
// would return linear order and fail the wrapped-order expectation below — so a passing check is
// itself proof that CA[45]=0 was emitted and decoded.
//
// Coverage:
//   * all four CR0[1:0] wrap sizes: 128 B/64 words, 64 B/32, 16 B/8, 32 B/16 (hb_wrap_words),
//   * legacy wrap (CR0[2]=1: wrap-in-group forever) AND hybrid wrap (CR0[2]=0: one traversal then
//     linear tail) — distinguished with bursts LONGER than the group (L>W),
//   * wrapped READ (order of returned words) and wrapped WRITE (order of written addresses).
// CR0 is reprogrammed between sub-tests via a native register write, so all configs run in one sim.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_wrap;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2
  localparam int unsigned PHYW       = 2 * DQ_WIDTH;          // 16

  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F

  // --------------------------------------------------------------------
  // Clocking / reset
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;    end
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90 = ~clk90;  end
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  // --------------------------------------------------------------------
  // Native command / write / read channels (TB drives the controller directly)
  // --------------------------------------------------------------------
  logic                    cmd_valid, cmd_ready, cmd_read, cmd_reg, cmd_wrap;
  logic [ADDR_WIDTH-1:0]   cmd_addr;
  logic [LEN_WIDTH-1:0]    cmd_len;
  logic                    wr_valid, wr_ready, wr_last;
  logic [DATA_WIDTH-1:0]   wr_data;
  logic [STRB_WIDTH-1:0]   wr_strb;
  logic                    rd_valid, rd_ready, rd_last;
  logic [DATA_WIDTH-1:0]   rd_data;
  logic                    busy, init_done, err_underrun, err_timeout;

  // --------------------------------------------------------------------
  // ctrl <-> phy DDR-parallel interface
  // --------------------------------------------------------------------
  logic                    phy_cs_n, phy_rst_n, phy_ck_en, phy_dq_oe, phy_rwds_oe, phy_rd_arm;
  logic [PHYW-1:0]         phy_dq_o, phy_dq_i;
  logic [1:0]              phy_rwds_o;
  logic                    phy_dq_i_valid, phy_rwds_i;

  // --------------------------------------------------------------------
  // HyperBus device pins + bus resolution
  // --------------------------------------------------------------------
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  hbp_dq_o;   logic hbp_dq_oe;
  logic                 hbp_rwds_o; logic hbp_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;
  logic                 mdl_rwds_o; logic mdl_rwds_oe;

  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (hbp_dq_oe   ? hbp_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (hbp_rwds_oe ? hbp_rwds_o : 1'b0);
  localparam realtime RTT = 3.0;
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  // --------------------------------------------------------------------
  // DUT: controller directly (native cmd channel exposes cmd_wrap)
  // --------------------------------------------------------------------
  hyperbus_ctrl #(
    .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1), .MAX_BURST_WORDS (0),
    .PROGRAM_CR (1'b1), .POR_DELAY_CYCLES (0), .INIT_CR0 (TB_INIT_CR0)
  ) u_ctrl (
    .clk (clk), .rst (rst),
    .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_read (cmd_read), .cmd_reg (cmd_reg),
    .cmd_wrap (cmd_wrap), .cmd_addr (cmd_addr), .cmd_len (cmd_len),
    .wr_valid (wr_valid), .wr_ready (wr_ready), .wr_data (wr_data), .wr_strb (wr_strb),
    .wr_last (wr_last),
    .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_data (rd_data), .rd_last (rd_last),
    .busy (busy), .init_done (init_done), .err_underrun (err_underrun), .err_timeout (err_timeout),
    .phy_cs_n (phy_cs_n), .phy_rst_n (phy_rst_n), .phy_ck_en (phy_ck_en),
    .phy_dq_o (phy_dq_o), .phy_dq_oe (phy_dq_oe), .phy_rwds_o (phy_rwds_o),
    .phy_rwds_oe (phy_rwds_oe), .phy_rd_arm (phy_rd_arm),
    .phy_dq_i (phy_dq_i), .phy_dq_i_valid (phy_dq_i_valid), .phy_rwds_i (phy_rwds_i),
    .dbg_state (/* unused */), .dbg_rd_wptr (/* unused */), .dbg_rd_rptr (/* unused */)
  );

  hyperbus_phy #(
    .PHY_VARIANT ("GENERIC"), .DIFF_CK (1'b1)
  ) u_phy (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // runtime read-eye calibration tied to POR-equivalent constants (reproduces pre-cal behaviour)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd0), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .phy_cs_n (phy_cs_n), .phy_rst_n (phy_rst_n), .phy_ck_en (phy_ck_en),
    .phy_dq_o (phy_dq_o), .phy_dq_oe (phy_dq_oe), .phy_rwds_o (phy_rwds_o),
    .phy_rwds_oe (phy_rwds_oe), .phy_rd_arm (phy_rd_arm),
    .phy_dq_i (phy_dq_i), .phy_dq_i_valid (phy_dq_i_valid), .phy_rwds_i (phy_rwds_i),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (hbp_dq_o), .hb_dq_oe (hbp_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (hbp_rwds_o), .hb_rwds_oe (hbp_rwds_oe), .hb_rwds_i (rwds_line_dly)
  );

  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i (dq_line), .hb_dq_ie (hbp_dq_oe), .hb_dq_o (mdl_dq_o), .hb_dq_oe (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (hbp_rwds_oe), .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );

  // --------------------------------------------------------------------
  // Scoreboard + capture
  // --------------------------------------------------------------------
  int unsigned errors = 0;
  int unsigned checks = 0;
  localparam int unsigned CAP_MAX = 256;
  logic [DATA_WIDTH-1:0] cap [CAP_MAX];
  int unsigned           cap_n;
  logic                  capturing;

  always @(posedge clk) begin
    if (capturing && rd_valid && rd_ready) begin
      if (cap_n < CAP_MAX) cap[cap_n] <= rd_data;
      cap_n <= cap_n + 1;
    end
  end

  function automatic logic [DATA_WIDTH-1:0] genword(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction
  function automatic logic [DATA_WIDTH-1:0] genwordB(input int unsigned k);
    return (16'(k) * 16'h2545) ^ 16'hC0DE;
  endfunction

  // CR0 image with a given legacy bit + wrap-size code, keeping latency code 6 + fixed.
  function automatic logic [15:0] cr0_img(input logic legacy, input logic [1:0] size);
    return {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, legacy, size};
  endfunction

  // Absolute word address of the k-th delivered word of a wrapped burst (independent reference model
  // of hyperram_model.next_addr). base = group base, off = start offset in group, W = group words.
  function automatic logic [ADDR_WIDTH-1:0] wrap_addr(input logic [ADDR_WIDTH-1:0] base,
                                                      input int unsigned off, input int unsigned k,
                                                      input int unsigned W, input logic legacy);
    if (legacy || (k < W)) return base + ADDR_WIDTH'((off + k) % W); // wrap-in-group rotation
    else                   return base + ADDR_WIDTH'(k);              // hybrid linear tail (base+W..)
  endfunction

  // --------------------------------------------------------------------
  // Native transaction tasks
  // --------------------------------------------------------------------
  task automatic nat_idle();
    @(negedge clk);
    cmd_valid = 1'b0; cmd_read = 1'b0; cmd_reg = 1'b0; cmd_wrap = 1'b0;
    cmd_addr = '0; cmd_len = '0;
    wr_valid = 1'b0; wr_data = '0; wr_strb = '1; wr_last = 1'b0;
  endtask

  task automatic nat_cmd(input logic rd, input logic rg, input logic wr,
                         input logic [ADDR_WIDTH-1:0] addr, input int unsigned n);
    int unsigned g;
    @(negedge clk);
    cmd_valid = 1'b1; cmd_read = rd; cmd_reg = rg; cmd_wrap = wr;
    cmd_addr = addr; cmd_len = LEN_WIDTH'(n);
    g = 0;
    forever begin
      @(posedge clk); g = g + 1;
      if (cmd_ready) break;
      if (g > 20000) begin $display("[%0t] HANG nat_cmd @0x%08x", $time, addr); errors=errors+1; break; end
    end
    @(negedge clk); cmd_valid = 1'b0;
  endtask

  // Streaming native write of n words (full strobe). Once ST_WRITE begins the controller consumes one
  // word per cycle (non-stalling), so present each next word on the negedge after its accept.
  task automatic nat_write(input logic [ADDR_WIDTH-1:0] addr, input logic rg, input logic wr,
                           input int unsigned n, input logic [DATA_WIDTH-1:0] data [$]);
    int unsigned idx, g;
    nat_cmd(1'b0, rg, wr, addr, n);
    idx = 0;
    @(negedge clk);
    wr_valid = 1'b1; wr_strb = '1; wr_data = data[0]; wr_last = (n == 1);
    g = 0;
    forever begin
      @(posedge clk); g = g + 1;
      if (g > 8000) begin $display("[%0t] HANG nat_write @0x%08x idx=%0d/%0d", $time, addr, idx, n);
                          errors=errors+1; break; end
      if (wr_ready) begin
        idx = idx + 1;
        if (idx == n) break;
        @(negedge clk); wr_data = data[idx]; wr_last = (idx == n-1);
      end
    end
    @(negedge clk); wr_valid = 1'b0; wr_last = 1'b0;
  endtask

  // Native read of n words into cap[0..n-1] (rd_ready held high globally).
  task automatic nat_read(input logic [ADDR_WIDTH-1:0] addr, input logic rg, input logic wr,
                          input int unsigned n);
    int unsigned g;
    nat_cmd(1'b1, rg, wr, addr, n);
    cap_n = 0; capturing = 1'b1;
    g = 0;
    while (cap_n < n && g < 30000) begin @(posedge clk); g = g + 1; end
    @(posedge clk); capturing = 1'b0;
    if (cap_n < n) begin
      $display("[%0t] ERROR: wrapped read of %0d words @0x%08x got only %0d", $time, n, addr, cap_n);
      errors = errors + 1;
    end
  endtask

  task automatic set_cr0(input logic legacy, input logic [1:0] size);
    logic [DATA_WIDTH-1:0] one [$];
    one = {}; one.push_back(cr0_img(legacy, size));
    nat_write(HB_REG_CR0[ADDR_WIDTH-1:0], 1'b1, 1'b0, 1, one);
    repeat (3) @(posedge clk);
  endtask

  // Pre-fill [base .. base+m-1] linearly, then wrapped-READ n words at base+off and check order.
  task automatic wrap_read_check(input logic legacy, input logic [1:0] size, input int unsigned W,
                                 input logic [ADDR_WIDTH-1:0] base, input int unsigned off,
                                 input int unsigned n, input string tag);
    logic [DATA_WIDTH-1:0] fill [$];
    logic [DATA_WIDTH-1:0] exp;
    logic [ADDR_WIDTH-1:0] a;
    int unsigned m, i;
    m = (n > W) ? n : W;                          // cover both legacy revisits and hybrid tail
    set_cr0(legacy, size);
    fill = {}; for (i = 0; i < m; i++) fill.push_back(genword(base + i));
    nat_write(base, 1'b0, 1'b0, m, fill);         // linear pre-fill
    nat_read (base + ADDR_WIDTH'(off), 1'b0, 1'b1, n);   // WRAPPED read
    for (i = 0; i < n; i++) begin
      a   = wrap_addr(base, off, i, W, legacy);
      exp = genword(a);
      checks = checks + 1;
      if (cap[i] !== exp) begin
        $display("[%0t] ERROR (%s legacy=%0b W=%0d): word %0d exp addr 0x%08x=0x%04x got 0x%04x",
                 $time, tag, legacy, W, i, a, exp, cap[i]);
        errors = errors + 1;
      end
    end
    $display("[%0t] %s: wrapped read legacy=%0b W=%0d base=0x%08x off=%0d n=%0d ok (errs %0d)",
             $time, tag, legacy, W, base, off, n, errors);
  endtask

  // Wrapped WRITE of n (<=W) words at base+off, then linear read-back of the group to prove each
  // wrapped address was written (in particular the wrap-around portion). n<=W => a clean permutation.
  task automatic wrap_write_check(input logic legacy, input logic [1:0] size, input int unsigned W,
                                  input logic [ADDR_WIDTH-1:0] base, input int unsigned off,
                                  input int unsigned n, input string tag);
    logic [DATA_WIDTH-1:0] pre [$];
    logic [DATA_WIDTH-1:0] wd  [$];
    logic [DATA_WIDTH-1:0] exp;
    int unsigned i, kk;
    set_cr0(legacy, size);
    pre = {}; for (i = 0; i < W; i++) pre.push_back(genword(base + i));
    nat_write(base, 1'b0, 1'b0, W, pre);          // linear pre-fill of whole group (pattern A)
    wd = {}; for (i = 0; i < n; i++) wd.push_back(genwordB(i));
    nat_write(base + ADDR_WIDTH'(off), 1'b0, 1'b1, n, wd);  // WRAPPED write (pattern B)
    nat_read (base, 1'b0, 1'b0, W);               // linear read-back of the group
    for (i = 0; i < W; i++) begin
      kk  = (i + W - off) % W;                    // beat index that maps to group word i (if <n)
      exp = (kk < n) ? genwordB(kk) : genword(base + i);
      checks = checks + 1;
      if (cap[i] !== exp) begin
        $display("[%0t] ERROR (%s wrapped-write legacy=%0b W=%0d): group word %0d (0x%08x) exp 0x%04x got 0x%04x",
                 $time, tag, legacy, W, i, base + i, exp, cap[i]);
        errors = errors + 1;
      end
    end
    $display("[%0t] %s: wrapped write legacy=%0b W=%0d base=0x%08x off=%0d n=%0d ok (errs %0d)",
             $time, tag, legacy, W, base, off, n, errors);
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  int unsigned guard;
  initial begin
    nat_idle();
    rd_ready  = 1'b1;               // greedy read consumer
    capturing = 1'b0; cap_n = 0;
    rst = 1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk); rst = 1'b0;

    guard = 0;
    while (!init_done && guard < 100000) begin @(posedge clk); guard = guard + 1; end
    if (!init_done) begin $display("[%0t] FATAL: init_done never asserted", $time); errors=errors+1; end
    else $display("[%0t] init_done asserted (native wrap harness)", $time);
    repeat (4) @(posedge clk);

    // ---- legacy vs hybrid distinction (L > W): small groups so the tail is visible ----
    // W = 8 (CR0[1:0]=10, 16 B)
    wrap_read_check(1'b1, 2'b10, 8,  32'h0000_0040, 5, 12, "w8-legacy");
    wrap_read_check(1'b0, 2'b10, 8,  32'h0000_0080, 5, 12, "w8-hybrid");
    // W = 16 (CR0[1:0]=11, 32 B, default)
    wrap_read_check(1'b1, 2'b11, 16, 32'h0000_0100, 10, 20, "w16-legacy");
    wrap_read_check(1'b0, 2'b11, 16, 32'h0000_0140, 10, 20, "w16-hybrid");

    // ---- larger wrap sizes: exercise hb_wrap_words 01/00 with a wrap-crossing L<=W ----
    // W = 32 (CR0[1:0]=01, 64 B)
    wrap_read_check(1'b1, 2'b01, 32, 32'h0000_0200, 28, 10, "w32-legacy");
    wrap_read_check(1'b0, 2'b01, 32, 32'h0000_0240, 28, 10, "w32-hybrid");
    // W = 64 (CR0[1:0]=00, 128 B)
    wrap_read_check(1'b1, 2'b00, 64, 32'h0000_0300, 60, 12, "w64-legacy");
    wrap_read_check(1'b0, 2'b00, 64, 32'h0000_0380, 60, 12, "w64-hybrid");

    // ---- wrapped WRITE (proves CA[45]=0 write + wrapped write addressing incl. wrap-around) ----
    wrap_write_check(1'b1, 2'b10, 8,  32'h0000_1000, 5,  6, "w8-wr");
    wrap_write_check(1'b1, 2'b11, 16, 32'h0000_1100, 12, 10, "w16-wr");
    wrap_write_check(1'b0, 2'b01, 32, 32'h0000_1200, 30, 8, "w32-wr");

    // ---- restore the default CR0 so we leave the device in a sane state ----
    set_cr0(1'b1, 2'b11);

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_wrap done: %0d checks, %0d errors", $time, checks, errors);
    if (errors == 0) begin $display("TB_RESULT: PASS"); $finish; end
    else begin $display("TB_RESULT: FAIL"); $fatal(1, "tb_wrap: %0d errors", errors); end
  end

  initial begin
    #8_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_wrap: global timeout");
  end

endmodule
