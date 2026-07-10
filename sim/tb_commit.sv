// tb_commit — regression for the W957D8NB SPLIT-WRITE commit quirk (issue #1), the 0x2000-word
// boundary-release quirk, the independent read-burst-size CSR (issue #2), and (2026-07-09 silicon
// evidence) the WR_PENDING_WORDS=4 remedy iteration: CS#-coalescing (WR_COALESCE, direction 4) and the
// FULL_BURST commit-read shape (COMMIT_READ_MODE, direction 1 re-shaped). It drives the EXACT board
// stack — bench engine (hyperram_bw_test) -> hyperram_avalon (SDR PHY, RD_PREAMBLE_SKIP=1) -> golden
// model — but the model now reproduces the two device quirks (WR_COMMIT_QUIRK, BURST_BOUNDARY_WORDS)
// and the controller fixes them (WR_COMMIT_READ, BURST_BOUNDARY_WORDS, WR_COALESCE).
//
// Ten independent stacks (each = engine + controller + model, differing only in the DUT/model
// parameters below) let one testbench prove every direction at once:
//
//   idx  DUT WR_COMMIT_READ  DUT boundary  DUT COALESCE  model WR_QUIRK  model boundary  model OS  model PEND  proves
//   ---  -----------------  ------------  ------------  -------------  --------------  --------  ----------  ----------------
//    0        0 (off)            0             0             1 (on)          0             0          1        commit: FAIL (n-1)
//    1        1 (on)             0             0             1 (on)          0             0          1        commit: PASS (0)
//    2        0                  0             0             0               0x2000        0          1        boundary: FAIL
//    3        0                  0x2000        0             0               0x2000        0          1        boundary: PASS
//    4        1                  0x2000        0             1               0x2000        0          1        compose: PASS
//    5        0                  0             0             0               0              9          1        read-split: PASS
//    6        0                  0             1             1               0              0          4        depth4 COALESCE: PASS (0)
//    7        1 (SPAN_END)       0             0             1               0              0          4        depth4 SPAN_END: FAIL (4*(n-1))
//    8        0                  0             0             1               0              0          4        depth4 nofix: FAIL (4*(n-1))
//    9*       1 (FULL_BURST)     0             0             1               0              0          4        depth4 FULL_BURST: PASS (0)
//   * idx 9 is a standalone instance below (not generate-for-driven), so its string-typed
//     COMMIT_READ_MODE="FULL_BURST" override is a plain per-instance literal.
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
//   - idx 6/7/8/9 (2026-07-09 silicon: the real W957D8NB holds FOUR pending words, not one) re-run the
//     SAME split-write sweep at WR_PENDING_WORDS=4: idx 8 (no fix) and idx 7 (the OLD SPAN_END
//     commit-read shape) both now reproduce ERR = 4*(n_bursts-1) — the 4-word span-end read exactly
//     covers the 4 pending words but no longer STARTS before them, so it never satisfies the "covers
//     the whole pending range" commit trigger (same failure class as issue #1 attempt #3); idx 6
//     (WR_COALESCE alone) and idx 9 (COMMIT_READ_MODE=FULL_BURST alone) both restore ERR_COUNT=0.
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
    parameter int unsigned MDL_OS         = 0,      // model: read over-stream words
    // -- issue #1 remedy A/B iteration (2026-07-09 silicon: WR_PENDING_WORDS=4 on the real device) --
    parameter bit          WR_COALESCE      = 1'b0, // DUT: CS#-coalescing (direction 4)
    parameter int unsigned WR_COALESCE_WAIT = 8,     // DUT: cycles to await a splice command
    parameter              COMMIT_READ_MODE = "SPAN_END",  // DUT: SPAN_END|FULL_BURST|NEXT_ROW
    parameter int unsigned COMMIT_READ_WORDS= 4,     // DUT: fixed-shape commit-read length
    parameter int unsigned MDL_PEND_WORDS   = 1      // model: pending-write delay-line depth
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
    .m_readdatavalid (av_readdatavalid), .m_waitrequest (av_waitrequest),
    // REG_CAL outputs unused here — u_hyperram's cal is tied to constants below (empty = PINCONNECTEMPTY)
    .cal_capture_phase (), .cal_preamble_skip (), .cal_rx_tap (), .cal_pair_skew ()
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
    .COMMIT_READ_WORDS(COMMIT_READ_WORDS),
    .COMMIT_READ_MODE (COMMIT_READ_MODE),
    .WR_COALESCE      (WR_COALESCE),
    .WR_COALESCE_WAIT (WR_COALESCE_WAIT),
    .PROGRAM_CR       (1'b1),
    .POR_DELAY_CYCLES (0),
    .INIT_CR0         (TB_INIT_CR0),
    .PHY_VARIANT      ("SDR"),
    .DIFF_CK          (1'b0),
    .RD_PREAMBLE_SKIP (1)
  ) u_hyperram (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // cal tied to constants reproducing RD_PREAMBLE_SKIP=1 (board setting; preamble edge discarded)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd1), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .avs_address (av_address), .avs_read (av_read), .avs_write (av_write),
    .avs_writedata (av_writedata), .avs_byteenable (2'b11), .avs_burstcount (av_burstcount),
    .avs_readdata (av_readdata), .avs_readdatavalid (av_readdatavalid), .avs_waitrequest (av_waitrequest),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (phy_dq_o), .hb_dq_oe (phy_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (phy_rwds_o), .hb_rwds_oe (phy_rwds_oe), .hb_rwds_i (rwds_line_dly),
    .init_done (init_done), .err_underrun (), .dbg_bus ()
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
    .WR_PENDING_WORDS     (MDL_PEND_WORDS),
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
  // The stacks. Per-stack parameters (see the header table) selected by genvar index.
  //
  // idx 0-5: the original six (issue #1 commit-read + boundary + REG_RBURSTW), unchanged.
  // idx 6-8: 2026-07-09 silicon evidence — the W957D8NB actually holds WR_PENDING_WORDS=4 words
  //   pending (not 1), so the split-write sweep is re-run at that depth:
  //     6  WR_COALESCE=1 alone (issue #1 direction 4)      -> expect ERR=0   (single-CS# by construction)
  //     7  WR_COMMIT_READ=1, COMMIT_READ_MODE=SPAN_END(default) -> expect ERR=4*(n-1) (same class as
  //        issue #1 attempt #3 — a 4-word span read no longer triggers a 4-deep pending commit; this
  //        is the exact regression the 2026-07-09 silicon run found)
  //     8  defaults off (no fix)                            -> expect ERR=4*(n-1) (silicon signature)
  // idx 9 (standalone, below): WR_COMMIT_READ=1, COMMIT_READ_MODE=FULL_BURST -> expect ERR=0 (direction 1
  //   re-shaped: re-read the ENTIRE just-closed write segment from its base, not just its span-end).
  // --------------------------------------------------------------------
  localparam int GEN_N  = 9;                                    // idx 0-8: array/generate-for driven
  localparam int NSTACK = 10;                                   // + idx 9: standalone (FULL_BURST)
  localparam bit STK_WCR   [GEN_N] = '{1'b0, 1'b1, 1'b0, 1'b0,  1'b1,  1'b0, 1'b0, 1'b1, 1'b0};  // DUT WR_COMMIT_READ
  localparam int STK_DBND  [GEN_N] = '{0,    0,    0,    8192,  8192,  0,    0,    0,    0};      // DUT boundary chop
  localparam bit STK_MWCQ  [GEN_N] = '{1'b1, 1'b1, 1'b0, 1'b0,  1'b1,  1'b0, 1'b1, 1'b1, 1'b1};  // model WR_COMMIT_QUIRK
  localparam int STK_MBND  [GEN_N] = '{0,    0,    8192, 8192,  8192,  0,    0,    0,    0};      // model boundary release
  localparam int STK_MOS   [GEN_N] = '{0,    0,    0,    0,     0,     9,    0,    0,    0};      // model read over-stream
  localparam bit STK_WCOAL [GEN_N] = '{1'b0, 1'b0, 1'b0, 1'b0,  1'b0,  1'b0, 1'b1, 1'b0, 1'b0};  // DUT WR_COALESCE
  localparam int STK_MPEND [GEN_N] = '{1,    1,    1,    1,     1,     1,    4,    4,    4};      // model WR_PENDING_WORDS

  logic [CSR_ADDR_WIDTH-1:0] s_addr  [NSTACK];
  logic                      s_read  [NSTACK];
  logic                      s_write [NSTACK];
  logic [31:0]               s_wdata [NSTACK];
  logic [31:0]               s_rdata [NSTACK];
  logic                      s_wait  [NSTACK];
  logic                      s_initd [NSTACK];

  generate
    for (genvar i = 0; i < GEN_N; i++) begin : g_stk
      commit_stack #(
        .CSR_ADDR_WIDTH (CSR_ADDR_WIDTH),
        .WR_COMMIT_READ (STK_WCR[i]),
        .DUT_BOUNDARY   (STK_DBND[i]),
        .MDL_WR_QUIRK   (STK_MWCQ[i]),
        .MDL_BOUNDARY   (STK_MBND[i]),
        .MDL_OS         (STK_MOS[i]),
        .WR_COALESCE    (STK_WCOAL[i]),
        .MDL_PEND_WORDS (STK_MPEND[i])
      ) u (
        .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
        .csr_address (s_addr[i]), .csr_read (s_read[i]), .csr_write (s_write[i]),
        .csr_writedata (s_wdata[i]), .csr_readdata (s_rdata[i]),
        .csr_waitrequest (s_wait[i]), .init_done (s_initd[i])
      );
    end
  endgenerate

  // idx 9 (standalone — not generate-for-driven, so the string-typed COMMIT_READ_MODE override can be
  // a plain literal): WR_COMMIT_READ + COMMIT_READ_MODE="FULL_BURST" against WR_PENDING_WORDS=4.
  commit_stack #(
    .CSR_ADDR_WIDTH    (CSR_ADDR_WIDTH),
    .WR_COMMIT_READ    (1'b1),
    .DUT_BOUNDARY      (0),
    .MDL_WR_QUIRK      (1'b1),
    .MDL_BOUNDARY      (0),
    .MDL_OS            (0),
    .WR_COALESCE       (1'b0),
    .COMMIT_READ_MODE  ("FULL_BURST"),
    .MDL_PEND_WORDS    (4)
  ) u_stk9 (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    .csr_address (s_addr[9]), .csr_read (s_read[9]), .csr_write (s_write[9]),
    .csr_writedata (s_wdata[9]), .csr_readdata (s_rdata[9]),
    .csr_waitrequest (s_wait[9]), .init_done (s_initd[9])
  );

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

    // ---- (E) 2026-07-09 silicon: WR_PENDING_WORDS=4 (the real W957D8NB holds FOUR words pending,
    // not one) — re-run the SAME split-write sweep against both remedies (A: WR_COALESCE, B: commit-read
    // reshaped to FULL_BURST), plus proof that the OLD SPAN_END shape and no-fix both now reproduce the
    // silicon fingerprint ERR = 4*(n_bursts-1) (4 words lost per non-final boundary, not 1).
    $display("-- (E) WR_PENDING_WORDS=4 split-write sweep (base=0x100) --");
    for (int i = 0; i < NSW; i++) begin
      run_one(8, sw_len[i][31:0], 32'h0000_0100, sw_burst[i][31:0], sw_burst[i][31:0], done, status, err);
      $display("   [depth4 nofix    ] LEN=%0d wburst=%0d done=%0b ERR=%0d (expect %0d)",
               sw_len[i], sw_burst[i], done, err, 4 * sw_nm1[i]);
      check(done, $sformatf("depth4 nofix LEN=%0d did not complete (STATUS=0x%08x)", sw_len[i], status));
      check(err == 32'(4 * sw_nm1[i]),
            $sformatf("depth4 nofix LEN=%0d ERR=%0d expected %0d (silicon signature 4*(n_bursts-1))",
                      sw_len[i], err, 4 * sw_nm1[i]));

      run_one(7, sw_len[i][31:0], 32'h0000_0100, sw_burst[i][31:0], sw_burst[i][31:0], done, status, err);
      $display("   [depth4 SPAN_END ] LEN=%0d wburst=%0d done=%0b ERR=%0d (expect %0d — attempt #3 class)",
               sw_len[i], sw_burst[i], done, err, 4 * sw_nm1[i]);
      check(done, $sformatf("depth4 SPAN_END LEN=%0d did not complete (STATUS=0x%08x)", sw_len[i], status));
      check(err == 32'(4 * sw_nm1[i]),
            $sformatf("depth4 SPAN_END LEN=%0d ERR=%0d expected %0d (a 4-word span-end read no longer spans PAST the 4-deep pending region, so it must NOT commit)",
                      sw_len[i], err, 4 * sw_nm1[i]));

      run_one(6, sw_len[i][31:0], 32'h0000_0100, sw_burst[i][31:0], sw_burst[i][31:0], done, status, err);
      $display("   [depth4 COALESCE ] LEN=%0d wburst=%0d done=%0b ERR=%0d (expect 0)",
               sw_len[i], sw_burst[i], done, err);
      check(done, $sformatf("depth4 COALESCE LEN=%0d did not complete (STATUS=0x%08x)", sw_len[i], status));
      check(err == 32'd0,
            $sformatf("depth4 COALESCE LEN=%0d ERR=%0d expected 0 (single-CS# by construction)",
                      sw_len[i], err));

      run_one(9, sw_len[i][31:0], 32'h0000_0100, sw_burst[i][31:0], sw_burst[i][31:0], done, status, err);
      $display("   [depth4 FULL_BURST] LEN=%0d wburst=%0d done=%0b ERR=%0d (expect 0)",
               sw_len[i], sw_burst[i], done, err);
      check(done, $sformatf("depth4 FULL_BURST LEN=%0d did not complete (STATUS=0x%08x)", sw_len[i], status));
      check(err == 32'd0,
            $sformatf("depth4 FULL_BURST LEN=%0d ERR=%0d expected 0 (re-reads the whole just-closed segment)",
                      sw_len[i], err));
    end

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
