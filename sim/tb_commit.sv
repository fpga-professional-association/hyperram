// tb_commit — regression for the W957D8NB SPLIT-WRITE commit quirk (issue #1), the 0x2000-word
// boundary-release quirk, and the independent read-burst-size CSR (issue #2). It drives the EXACT
// board stack — bench engine (hyperram_bw_test) -> hyperram_avalon (SDR PHY, RD_PREAMBLE_SKIP=1) ->
// golden model — but the model now reproduces the two device quirks (WR_COMMIT_QUIRK,
// BURST_BOUNDARY_WORDS) and the controller fixes them (WR_COMMIT_READ, BURST_BOUNDARY_WORDS).
//
// Six independent stacks (each = engine + controller + model, differing only in the DUT/model
// parameters below) let one testbench prove every direction at once:
//
//   idx  DUT WR_COMMIT_READ  DUT boundary  model WR_QUIRK  model boundary  model over-stream  proves
//   ---  -----------------  ------------  -------------  --------------  ----------------  ----------------
//    0        0 (off)            0            1 (on)          0                0            commit: FAIL (n-1)
//    1        1 (on)             0            1 (on)          0                0            commit: PASS (0)
//    2        0                  0            0               0x2000           0            boundary: FAIL
//    3        0                  0x2000       0               0x2000           0            boundary: PASS
//    4        1                  0x2000       1               0x2000           0            compose: PASS
//    5        0                  0            0               0                9            read-split: PASS
//
//   - idx 0 reproduces the silicon signature ERR_COUNT == n_write_bursts-1 for a split multi-burst
//     write (LEN=32/burst16 ->1, LEN=64/16 ->3, LEN=256/64 ->3); idx 1 is the SAME sweep with the
//     commit-read fix on -> ERR_COUNT=0.
//   - idx 2/3: a single burst crossing a 0x2000-word boundary. With the DUT boundary chop OFF (2) the
//     boundary-release model corrupts the read-back (ERR>0, proving the model models it); with the
//     chop ON (3) the controller never crosses the boundary -> ERR_COUNT=0.
//   - idx 4: the real-board config (both quirks + both fixes); the write crossing is chopped AND each
//     split segment gets a commit-read (the intended composition) -> ERR_COUNT=0.
//   - idx 5: write a single (correct) burst, split the READ into many via REG_RBURSTW, against a clean
//     model that over-streams like the silicon -> ERR_COUNT=0 (isolates the multi-burst READ path).
//
// Every poll loop is BOUNDED, so a hang shows up as a FAIL (bounded timeout) not an infinite loop.
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps

// ---------------------------------------------------------------------------------------------------
// One complete board stack: bench engine (Avalon master + CSR slave) -> hyperram_avalon (SDR PHY) ->
// golden device model, with the shared DQ/RWDS bus resolved and a round-trip flight delay, exactly as
// sim/tb_multiburst.sv wires it. The CSR slave is hoisted to the top so the testbench can drive it.
// ---------------------------------------------------------------------------------------------------
module commit_stack
  import hyperbus_pkg::*;
#(
    parameter int unsigned CSR_ADDR_WIDTH = 4,
    parameter bit          WR_COMMIT_READ = 1'b0,   // DUT: interpose commit-read after split writes
    parameter int unsigned DUT_BOUNDARY   = 0,      // DUT: chop at this WORD boundary (0 = off)
    parameter bit          MDL_WR_QUIRK   = 1'b0,   // model: split-write commit quirk
    parameter int unsigned MDL_BOUNDARY   = 0,      // model: 0x2000-word boundary release quirk
    parameter int unsigned MDL_OS         = 0       // model: read over-stream words
) (
    input  logic                        clk,
    input  logic                        clk90,
    input  logic                        clk_ref,
    input  logic                        rst,
    input  logic [CSR_ADDR_WIDTH-1:0]   csr_address,
    input  logic                        csr_read,
    input  logic                        csr_write,
    input  logic [31:0]                 csr_writedata,
    output logic [31:0]                 csr_readdata,
    output logic                        csr_waitrequest,
    output logic                        init_done
);
  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;    // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;           // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;          // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;   // 16
  localparam int unsigned BURST_WORDS = HB_BURST_WORDS_DEFAULT;// 16
  // CR0 image: latency code 0001 (=6), fixed-latency, legacy wrap, 32B group (matches the board).
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam logic [31:0] TB_MAGIC    = 32'h4842_5754;         // "HBWT"

  // HyperBus device pins + split-driver resolution (single active driver at a time).
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  phy_dq_o;   logic phy_dq_oe;
  logic                 phy_rwds_o; logic phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;
  logic                 mdl_rwds_o; logic mdl_rwds_oe;
  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);
  localparam realtime RTT = 3.0;    // ns device->master flight delay
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  // Avalon-MM link: bench master <-> hyperram_avalon slave.
  logic [ADDR_WIDTH-1:0] av_address;
  logic [LEN_WIDTH-1:0]  av_burstcount;
  logic                  av_read, av_write;
  logic [DATA_WIDTH-1:0] av_writedata, av_readdata;
  logic                  av_readdatavalid, av_waitrequest;

  hyperram_bw_test #(
    .DATA_WIDTH     (DATA_WIDTH),
    .ADDR_WIDTH     (ADDR_WIDTH),
    .LEN_WIDTH      (LEN_WIDTH),
    .BURST_WORDS    (BURST_WORDS),
    .CSR_ADDR_WIDTH (CSR_ADDR_WIDTH),
    .VERSION_MAGIC  (TB_MAGIC)
  ) u_bw (
    .clk (clk), .rst (rst),
    .csr_address (csr_address), .csr_read (csr_read), .csr_readdata (csr_readdata),
    .csr_write (csr_write), .csr_writedata (csr_writedata), .csr_waitrequest (csr_waitrequest),
    .m_address (av_address), .m_burstcount (av_burstcount), .m_read (av_read), .m_write (av_write),
    .m_writedata (av_writedata), .m_readdata (av_readdata),
    .m_readdatavalid (av_readdatavalid), .m_waitrequest (av_waitrequest)
  );

  hyperram_avalon #(
    .DQ_WIDTH         (DQ_WIDTH),
    .DATA_WIDTH       (DATA_WIDTH),
    .ADDR_WIDTH       (ADDR_WIDTH),
    .LEN_WIDTH        (LEN_WIDTH),
    .LATENCY_CLOCKS   (6),
    .FIXED_LATENCY    (1'b1),
    .MAX_BURST_WORDS  (0),
    .BURST_BOUNDARY_WORDS (DUT_BOUNDARY),
    .WR_COMMIT_READ   (WR_COMMIT_READ),
    .PROGRAM_CR       (1'b1),
    .POR_DELAY_CYCLES (0),
    .INIT_CR0         (TB_INIT_CR0),
    .PHY_VARIANT      ("SDR"),
    .DIFF_CK          (1'b0),
    .RD_PREAMBLE_SKIP (1)
  ) u_hyperram (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    .avs_address (av_address), .avs_read (av_read), .avs_write (av_write),
    .avs_writedata (av_writedata), .avs_byteenable (2'b11), .avs_burstcount (av_burstcount),
    .avs_readdata (av_readdata), .avs_readdatavalid (av_readdatavalid), .avs_waitrequest (av_waitrequest),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (phy_dq_o), .hb_dq_oe (phy_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (phy_rwds_o), .hb_rwds_oe (phy_rwds_oe), .hb_rwds_i (rwds_line_dly),
    .init_done (init_done)
  );

  hyperram_model #(
    .DQ_WIDTH             (DQ_WIDTH),
    .MEM_WORDS            (1 << 16),
    .LATENCY_CLOCKS       (6),
    .FIXED_LATENCY        (1'b1),
    .ROW_WORDS            (0),
    .REFRESH_EVERY        (0),
    .RD_PREAMBLE_CLOCKS   (1),               // matches RD_PREAMBLE_SKIP=1 on the SDR PHY
    .RD_OVERSTREAM_WORDS  (MDL_OS),
    .WR_COMMIT_QUIRK      (MDL_WR_QUIRK),
    .BURST_BOUNDARY_WORDS (MDL_BOUNDARY)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i (dq_line), .hb_dq_ie (phy_dq_oe), .hb_dq_o (mdl_dq_o), .hb_dq_oe (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (phy_rwds_oe), .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );
endmodule


module tb_commit;
  import hyperbus_pkg::*;

  localparam int unsigned CSR_ADDR_WIDTH = 4;                  // 16 word-regs (REG_RBURSTW = word 12)

  // CSR word-register indices (byte offset >> 2) — must match hyperram_bw_test.
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_CTRL    = 4'd0;    // W: CTRL / R: STATUS
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_LEN     = 4'd1;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_BASE    = 4'd2;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRCNT  = 4'd5;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRADDR = 4'd8;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRGOT  = 4'd9;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERREXP  = 4'd10;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_BURSTW  = 4'd11;   // WRITE-phase burst length
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_RBURSTW = 4'd12;   // READ-phase  burst length

  // --------------------------------------------------------------------
  // Clocking / reset — SDR arrangement (as tb_sdr / tb_multiburst / the board).
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk     = 1'b0; forever #10.0 clk     = ~clk;     end   // 50 MHz
  initial begin clk90   = 1'b0; forever #5.0  clk90   = ~clk90;   end   // 100 MHz
  initial begin clk_ref = 1'b0; forever #5.0  clk_ref = ~clk_ref; end

  // --------------------------------------------------------------------
  // The six stacks. Per-stack parameters (see the header table) selected by genvar index.
  // --------------------------------------------------------------------
  localparam int NSTACK = 6;
  localparam bit STK_WCR  [NSTACK] = '{1'b0, 1'b1, 1'b0, 1'b0,  1'b1,  1'b0};   // DUT WR_COMMIT_READ
  localparam int STK_DBND [NSTACK] = '{0,    0,    0,    8192,  8192,  0};       // DUT boundary chop
  localparam bit STK_MWCQ [NSTACK] = '{1'b1, 1'b1, 1'b0, 1'b0,  1'b1,  1'b0};   // model WR_COMMIT_QUIRK
  localparam int STK_MBND [NSTACK] = '{0,    0,    8192, 8192,  8192,  0};       // model boundary release
  localparam int STK_MOS  [NSTACK] = '{0,    0,    0,    0,     0,     9};       // model read over-stream

  logic [CSR_ADDR_WIDTH-1:0] s_addr  [NSTACK];
  logic                      s_read  [NSTACK];
  logic                      s_write [NSTACK];
  logic [31:0]               s_wdata [NSTACK];
  logic [31:0]               s_rdata [NSTACK];
  logic                      s_wait  [NSTACK];
  logic                      s_initd [NSTACK];

  generate
    for (genvar i = 0; i < NSTACK; i++) begin : g_stk
      commit_stack #(
        .CSR_ADDR_WIDTH (CSR_ADDR_WIDTH),
        .WR_COMMIT_READ (STK_WCR[i]),
        .DUT_BOUNDARY   (STK_DBND[i]),
        .MDL_WR_QUIRK   (STK_MWCQ[i]),
        .MDL_BOUNDARY   (STK_MBND[i]),
        .MDL_OS         (STK_MOS[i])
      ) u (
        .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
        .csr_address (s_addr[i]), .csr_read (s_read[i]), .csr_write (s_write[i]),
        .csr_writedata (s_wdata[i]), .csr_readdata (s_rdata[i]),
        .csr_waitrequest (s_wait[i]), .init_done (s_initd[i])
      );
    end
  endgenerate

  // --------------------------------------------------------------------
  // Scoreboard
  // --------------------------------------------------------------------
  int unsigned errors = 0;
  task automatic check(input logic cond, input string msg);
    if (!cond) begin
      $display("[%0t] ERROR: %s", $time, msg);
      errors = errors + 1;
    end
  endtask

  // --------------------------------------------------------------------
  // Per-stack CSR access (drive on the falling edge; waitrequest tied low).
  // --------------------------------------------------------------------
  task automatic csr_wr(input int idx, input logic [CSR_ADDR_WIDTH-1:0] addr, input logic [31:0] data);
    @(negedge clk);
    s_addr[idx]  = addr;
    s_wdata[idx] = data;
    s_write[idx] = 1'b1;
    s_read[idx]  = 1'b0;
    @(negedge clk);
    s_write[idx] = 1'b0;
    s_wdata[idx] = '0;
    s_addr[idx]  = '0;
  endtask

  // Hold the (combinational) read address stable for a FULL clock and sample at the next negedge, so
  // s_rdata is guaranteed settled. (A within-cycle `#1` sample races the model's 1 ns over-stream
  // watchdog across the array-connected readdata and returns stale data.)
  task automatic csr_rd(input int idx, input logic [CSR_ADDR_WIDTH-1:0] addr, output logic [31:0] data);
    @(negedge clk);
    s_addr[idx] = addr;
    s_read[idx] = 1'b1;
    s_write[idx]= 1'b0;
    @(negedge clk);
    data        = s_rdata[idx];
    s_read[idx] = 1'b0;
    s_addr[idx] = '0;
  endtask

  // Program one stack and run a single write+read pass; return completion + counters (BOUNDED poll).
  task automatic run_one(input int idx,
                         input logic [31:0] len, input logic [31:0] base,
                         input logic [31:0] wburst, input logic [31:0] rburst,
                         output logic done, output logic [31:0] status, output logic [31:0] err);
    int unsigned guard;
    csr_wr(idx, REG_LEN,     len);
    csr_wr(idx, REG_BASE,    base);
    csr_wr(idx, REG_BURSTW,  wburst);
    csr_wr(idx, REG_RBURSTW, rburst);
    csr_wr(idx, REG_CTRL,    32'h0000_0001);   // pulse start
    guard = 0;
    do begin
      csr_rd(idx, REG_CTRL, status);
      guard = guard + 1;
    end while (!status[1] && guard < 200000);
    done = status[1];
    csr_rd(idx, REG_ERRCNT, err);
    if (err != 0) begin
      logic [31:0] ea, eg, ee;
      csr_rd(idx, REG_ERRADDR, ea);
      csr_rd(idx, REG_ERRGOT,  eg);
      csr_rd(idx, REG_ERREXP,  ee);
      $display("        first-err: addr=0x%08x got=0x%04x exp=0x%04x", ea, eg[15:0], ee[15:0]);
    end
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  logic [31:0] status, err;
  logic        done;
  int unsigned guard;

  // commit-quirk sweep: expected nofix ERR = n_write_bursts - 1 (silicon signature).
  localparam int NSW = 3;
  int sw_len   [NSW] = '{32, 64, 256};
  int sw_burst [NSW] = '{16, 16, 64};
  int sw_nm1   [NSW] = '{1,  3,  3};

  initial begin
    for (int k = 0; k < NSTACK; k++) begin
      s_addr[k] = '0; s_read[k] = 1'b0; s_write[k] = 1'b0; s_wdata[k] = '0;
    end
    rst = 1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk);
    rst = 1'b0;

    // Wait for every stack's POR + CR0 programming.
    guard = 0;
    while (guard < 100000) begin
      logic all_done;
      all_done = 1'b1;
      for (int k = 0; k < NSTACK; k++) all_done &= s_initd[k];
      if (all_done) break;
      @(posedge clk); guard = guard + 1;
    end
    for (int k = 0; k < NSTACK; k++) check(s_initd[k], $sformatf("stack %0d init_done never asserted", k));
    repeat (4) @(posedge clk);

    $display("==================================================================");
    $display("tb_commit: split-write commit quirk + 0x2000 boundary + REG_RBURSTW");
    $display("==================================================================");

    // ---- (A) Split-write commit quirk (issue #1) ----
    // idx0 = no fix -> must reproduce ERR == n_write_bursts-1 ; idx1 = fix -> must be 0.
    $display("-- (A) split-write commit quirk (base=0x100) --");
    for (int i = 0; i < NSW; i++) begin
      run_one(0, sw_len[i][31:0], 32'h0000_0100, sw_burst[i][31:0], sw_burst[i][31:0], done, status, err);
      $display("   [nofix] LEN=%0d wburst=%0d done=%0b ERR=%0d (expect %0d)",
               sw_len[i], sw_burst[i], done, err, sw_nm1[i]);
      check(done, $sformatf("nofix LEN=%0d did not complete (STATUS=0x%08x)", sw_len[i], status));
      check(err == 32'(sw_nm1[i]),
            $sformatf("nofix LEN=%0d ERR=%0d expected %0d (silicon signature n_bursts-1)",
                      sw_len[i], err, sw_nm1[i]));

      run_one(1, sw_len[i][31:0], 32'h0000_0100, sw_burst[i][31:0], sw_burst[i][31:0], done, status, err);
      $display("   [ fix ] LEN=%0d wburst=%0d done=%0b ERR=%0d (expect 0)",
               sw_len[i], sw_burst[i], done, err);
      check(done, $sformatf("fix LEN=%0d did not complete (STATUS=0x%08x)", sw_len[i], status));
      check(err == 32'd0, $sformatf("fix LEN=%0d ERR=%0d expected 0 (commit-read)", sw_len[i], err));
    end

    // ---- (B) 0x2000-word boundary chop ----
    // A single 16-word burst @0x1FF8 crosses the 0x2000-word boundary.
    $display("-- (B) 0x2000-word boundary (base=0x1FF8, LEN=16, burst=16 -> crosses) --");
    run_one(2, 32'd16, 32'h0000_1FF8, 32'd16, 32'd16, done, status, err);   // DUT chop OFF
    $display("   [chop off] done=%0b ERR=%0d STATUS=0x%08x (expect ERR>0)", done, err, status);
    check(done, "boundary chop-off did not complete (hang)");
    check(err != 32'd0, "boundary chop-off ERR=0 — model boundary-release not modelled");

    run_one(3, 32'd16, 32'h0000_1FF8, 32'd16, 32'd16, done, status, err);   // DUT chop ON
    $display("   [chop on ] done=%0b ERR=%0d STATUS=0x%08x (expect ERR=0)", done, err, status);
    check(done, "boundary chop-on did not complete");
    check(err == 32'd0, $sformatf("boundary chop-on ERR=%0d expected 0", err));

    // ---- (C) Composition: boundary chop of a WRITE + commit-read (real-board config) ----
    $display("-- (C) compose: boundary chop + commit-read (base=0x1FF8, LEN=16, burst=16) --");
    run_one(4, 32'd16, 32'h0000_1FF8, 32'd16, 32'd16, done, status, err);
    $display("   [compose] done=%0b ERR=%0d STATUS=0x%08x (expect ERR=0)", done, err, status);
    check(done, "compose did not complete");
    check(err == 32'd0, $sformatf("compose ERR=%0d expected 0", err));

    // ---- (D) Independent read-burst-size CSR (issue #2) ----
    // Write ONE burst (correct memory), split the READ into many; clean model over-streams like silicon.
    $display("-- (D) REG_RBURSTW: write single burst, split read (base=0x100, LEN=64, wburst=64, rburst=16) --");
    run_one(5, 32'd64, 32'h0000_0100, 32'd64, 32'd16, done, status, err);
    $display("   [rsplit ] done=%0b ERR=%0d STATUS=0x%08x (expect ERR=0)", done, err, status);
    check(done, "read-split did not complete (multi-burst read HANG)");
    check(err == 32'd0, $sformatf("read-split ERR=%0d expected 0 (multi-burst READ path)", err));

    $display("==================================================================");
    $display("[%0t] tb_commit done: %0d errors", $time, errors);
    if (errors == 0) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_commit: %0d errors", errors);
    end
  end

  // Global watchdog — a true infinite hang (should never happen; every poll is bounded).
  initial begin
    #40_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_commit: global timeout");
  end

endmodule
