// tb_commit — regression for the W957D8NB SPLIT-WRITE commit quirk (issue #1), the 0x2000-word
// boundary-release quirk, the independent read-burst-size CSR (issue #2), the WR_PENDING_WORDS=4
// remedy iteration (2026-07-09: CS#-coalescing / FULL_BURST commit-read), and the 2026-07-10
// WR_CHOP_REPLAY fix. It drives the EXACT board stack — bench engine (hyperram_bw_test) ->
// hyperram_avalon (SDR PHY, RD_PREAMBLE_SKIP=1) -> golden model — the model reproduces the device
// quirks (WR_COMMIT_QUIRK, BURST_BOUNDARY_WORDS) and the controller mitigates them.
//
// 2026-07-10 SILICON CORRECTION: the model's old covering-read-commits trigger is FALSIFIED — on
// the real device NO read-shaped interpose preserves the pending words when another WRITE follows
// (pend serves reads but is discarded by the next memory-write CS#; see hyperram_model's
// WR_COMMIT_QUIRK note). Consequently every commit-read PASS expectation below flipped to the
// no-fix signature (the mode is documented-ineffective for write->write; RTL kept), and the
// working chop-boundary remedy is WR_CHOP_REPLAY (re-send the dropped words).
//
// Twelve independent stacks (each = engine + controller + model, differing only in the DUT/model
// parameters below) let one testbench prove every direction at once:
//
//   idx  DUT WR_COMMIT_READ  DUT boundary  DUT maxburst  DUT COALESCE  DUT REPLAY  model WR_QUIRK  model boundary  model OS  model PEND  proves
//   ---  -----------------  ------------  ------------  ------------  ----------  -------------  --------------  --------  ----------  ----------------
//    0        0 (off)            0             0             0            0           1 (on)          0             0          1        commit quirk: n-1
//    1        1 (on)             0             0             0            0           1 (on)          0             0          1        commit-read ineffective: n-1
//    2        0                  0             0             0            0           0               0x2000        0          1        boundary: FAIL
//    3        0                  0x2000        0             0            0           0               0x2000        0          1        boundary: PASS
//    4        1                  0x2000        0             0            0           1               0x2000        0          1        compose: commit-read ineffective: 1
//    5        0                  0             0             0            0           0               0              9          1        read-split: PASS
//    6        0                  0             0             1            0           1               0              0          4        depth4 COALESCE: PASS (0)
//    7        1 (SPAN_END)       0             0             0            0           1               0              0          4        depth4 SPAN_END: 4*(n-1)
//    8        0                  0             0             0            0           1               0              0          4        depth4 nofix: 4*(n-1)
//    9*       1 (FULL_BURST)     0             0             0            0           1               0              0          4        depth4 FULL_BURST ineffective: 4*(n-1)
//   10*       0                  0x2000        64            1            1           1               0              0          4        depth4 chop REPLAY: PASS (0)
//   11*       0                  0x2000        64            1            0           1               0              0          4        depth4 chop no-replay: 4-per-chop
//   * idx 9/10/11 are standalone instances below (not generate-for-driven: idx 9 needs a
//     string-typed COMMIT_READ_MODE="FULL_BURST" literal; 10/11 keep the generate arrays intact).
//
//   - idx 0 reproduces the silicon signature ERR_COUNT == n_write_bursts-1 for a split multi-burst
//     write (LEN=32/burst16 ->1, LEN=64/16 ->3, LEN=256/64 ->3); idx 1 is the SAME sweep with the
//     commit-read interpose on -> the SAME n-1 signature (2026-07-10: a read between two writes
//     commits nothing on silicon; the corrected model reproduces that).
//   - idx 2/3: a single burst crossing a 0x2000-word boundary. With the DUT boundary chop OFF (2) the
//     boundary-release model corrupts the read-back (ERR>0, proving the model models it); with the
//     chop ON (3) the controller never crosses the boundary -> ERR_COUNT=0.
//   - idx 4: boundary chop + commit-read at the chop: the chop itself is fixed but the interposed
//     commit-read does NOT save the chopped segment's pending tail word -> ERR=1 (the old model
//     wrongly showed 0 here).
//   - idx 5: write a single (correct) burst, split the READ into many via REG_RBURSTW, against a clean
//     model that over-streams like the silicon -> ERR_COUNT=0 (isolates the multi-burst READ path).
//   - idx 6/7/8/9 (2026-07-09 silicon: the real W957D8NB holds FOUR pending words, not one) re-run the
//     SAME split-write sweep at WR_PENDING_WORDS=4: idx 8 (no fix), idx 7 (SPAN_END) and — with the
//     corrected model — also idx 9 (FULL_BURST, falsified on 2026-07-10 silicon) all reproduce
//     ERR = 4*(n_bursts-1); only idx 6 (WR_COALESCE: no CS# boundary at all) restores ERR_COUNT=0
//     among the pre-replay remedies.
//   - idx 10/11 (WR_CHOP_REPLAY, 2026-07-10): forced intra-command chops (MAX_BURST_WORDS=64, plus a
//     BURST_BOUNDARY-crossing case) with WR_COALESCE=1 against the depth-4 model. With replay ON
//     every chop reopens 4 words early and re-sends the words the device drops -> ERR=0 for a
//     chopped single command (LEN=768/wburst=768), a coalesced+chopped multi-command stream
//     (LEN=4096/wburst=256), and a boundary-crossing write; replay OFF (idx 11) reproduces the
//     4-per-chop signature (44 for the 11-chop LEN=768 case, 4 for the single boundary chop).
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
    parameter int unsigned MDL_PEND_WORDS   = 1,     // model: pending-write delay-line depth
    // -- issue #1 direction 5 (2026-07-10 silicon: replay is the chop-boundary fix) --
    parameter int unsigned DUT_MAXBURST     = 0,     // DUT: MAX_BURST_WORDS tCSM chop (0 = off)
    parameter bit          WR_CHOP_REPLAY   = 1'b0,  // DUT: re-send the dropped words at write chops
    parameter int unsigned WR_REPLAY_WORDS  = 4      // DUT: replay depth (= device pending depth)
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
    .MAX_BURST_WORDS  (DUT_MAXBURST),
    .BURST_BOUNDARY_WORDS (DUT_BOUNDARY),
    .WR_COMMIT_READ   (WR_COMMIT_READ),
    .COMMIT_READ_WORDS(COMMIT_READ_WORDS),
    .COMMIT_READ_MODE (COMMIT_READ_MODE),
    .WR_COALESCE      (WR_COALESCE),
    .WR_COALESCE_WAIT (WR_COALESCE_WAIT),
    .WR_CHOP_REPLAY   (WR_CHOP_REPLAY),
    .WR_REPLAY_WORDS  (WR_REPLAY_WORDS),
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
  // idx 0-5: the original six (issue #1 commit-read + boundary + REG_RBURSTW); the commit-read PASS
  //   expectations (1, 4) flipped to the no-fix signature under the 2026-07-10 corrected model.
  // idx 6-8: 2026-07-09 silicon evidence — the W957D8NB actually holds WR_PENDING_WORDS=4 words
  //   pending (not 1), so the split-write sweep is re-run at that depth:
  //     6  WR_COALESCE=1 alone (issue #1 direction 4)      -> expect ERR=0   (single-CS# by construction)
  //     7  WR_COMMIT_READ=1, COMMIT_READ_MODE=SPAN_END(default) -> expect ERR=4*(n-1)
  //     8  defaults off (no fix)                            -> expect ERR=4*(n-1) (silicon signature)
  // idx 9 (standalone, below): WR_COMMIT_READ=1, COMMIT_READ_MODE=FULL_BURST -> expect ERR=4*(n-1)
  //   (2026-07-10: falsified on silicon; the corrected model no longer lets ANY read flush pend).
  // idx 10/11 (standalone, below): WR_CHOP_REPLAY on/off against forced chops (issue #1 direction 5).
  // --------------------------------------------------------------------
  localparam int GEN_N  = 9;                                    // idx 0-8: array/generate-for driven
  localparam int NSTACK = 12;                                   // + idx 9 (FULL_BURST), 10/11 (REPLAY)
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

  // idx 10/11 (WR_CHOP_REPLAY, issue #1 direction 5): WR_COALESCE=1 + MAX_BURST_WORDS=64 (forces
  // intra-command chops) + the 0x2000-word boundary chop, against the WR_PENDING_WORDS=4 model with
  // NO model boundary-release (MDL_BOUNDARY=0): these stacks isolate the corrected COMMIT quirk at
  // the chop boundaries — including the boundary-chop case, where the replayed reopen deliberately
  // starts 4 words BEFORE the boundary (see hyperbus_ctrl's WR_CHOP_REPLAY BURST_BOUNDARY note).
  // idx 10 = replay ON (expect ERR=0 everywhere), idx 11 = replay OFF (expect 4-per-chop).
  commit_stack #(
    .CSR_ADDR_WIDTH    (CSR_ADDR_WIDTH),
    .WR_COMMIT_READ    (1'b0),
    .DUT_BOUNDARY      (8192),
    .DUT_MAXBURST      (64),
    .MDL_WR_QUIRK      (1'b1),
    .MDL_BOUNDARY      (0),
    .MDL_OS            (0),
    .WR_COALESCE       (1'b1),
    .WR_CHOP_REPLAY    (1'b1),
    .MDL_PEND_WORDS    (4)
  ) u_stk10 (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    .csr_address (s_addr[10]), .csr_read (s_read[10]), .csr_write (s_write[10]),
    .csr_writedata (s_wdata[10]), .csr_readdata (s_rdata[10]),
    .csr_waitrequest (s_wait[10]), .init_done (s_initd[10])
  );

  commit_stack #(
    .CSR_ADDR_WIDTH    (CSR_ADDR_WIDTH),
    .WR_COMMIT_READ    (1'b0),
    .DUT_BOUNDARY      (8192),
    .DUT_MAXBURST      (64),
    .MDL_WR_QUIRK      (1'b1),
    .MDL_BOUNDARY      (0),
    .MDL_OS            (0),
    .WR_COALESCE       (1'b1),
    .WR_CHOP_REPLAY    (1'b0),
    .MDL_PEND_WORDS    (4)
  ) u_stk11 (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    .csr_address (s_addr[11]), .csr_read (s_read[11]), .csr_write (s_write[11]),
    .csr_writedata (s_wdata[11]), .csr_readdata (s_rdata[11]),
    .csr_waitrequest (s_wait[11]), .init_done (s_initd[11])
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
    // idx0 = no fix -> must reproduce ERR == n_write_bursts-1; idx1 = commit-read interpose -> the
    // SAME n-1 signature (2026-07-10: NO read-shaped interpose commits when another write follows —
    // the corrected model's pend is discarded by the next write CS# regardless of the read between).
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
      $display("   [cread] LEN=%0d wburst=%0d done=%0b ERR=%0d (expect %0d — interpose ineffective)",
               sw_len[i], sw_burst[i], done, err, sw_nm1[i]);
      check(done, $sformatf("commit-read LEN=%0d did not complete (STATUS=0x%08x)", sw_len[i], status));
      check(err == 32'(sw_nm1[i]),
            $sformatf("commit-read LEN=%0d ERR=%0d expected %0d (2026-07-10: a read interpose commits nothing when a write follows)",
                      sw_len[i], err, sw_nm1[i]));
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

    // ---- (C) Composition: boundary chop of a WRITE + commit-read at the chop ----
    // The boundary chop itself works (no crossing), but the interposed commit-read does NOT save
    // the first segment's pending tail word from the resumed write (2026-07-10) -> exactly 1 error
    // (the pre-boundary segment's last word; PEND=1 here). The old model wrongly showed 0.
    $display("-- (C) compose: boundary chop + commit-read (base=0x1FF8, LEN=16, burst=16) --");
    run_one(4, 32'd16, 32'h0000_1FF8, 32'd16, 32'd16, done, status, err);
    $display("   [compose] done=%0b ERR=%0d STATUS=0x%08x (expect ERR=1 — chop fixed, commit-read ineffective)",
             done, err, status);
    check(done, "compose did not complete");
    check(err == 32'd1,
          $sformatf("compose ERR=%0d expected 1 (chopped segment's tail word lost despite the commit-read)", err));

    // ---- (D) Independent read-burst-size CSR (issue #2) ----
    // Write ONE burst (correct memory), split the READ into many; clean model over-streams like silicon.
    $display("-- (D) REG_RBURSTW: write single burst, split read (base=0x100, LEN=64, wburst=64, rburst=16) --");
    run_one(5, 32'd64, 32'h0000_0100, 32'd64, 32'd16, done, status, err);
    $display("   [rsplit ] done=%0b ERR=%0d STATUS=0x%08x (expect ERR=0)", done, err, status);
    check(done, "read-split did not complete (multi-burst read HANG)");
    check(err == 32'd0, $sformatf("read-split ERR=%0d expected 0 (multi-burst READ path)", err));

    // ---- (E) 2026-07-09 silicon: WR_PENDING_WORDS=4 (the real W957D8NB holds FOUR words pending,
    // not one) — re-run the SAME split-write sweep. no-fix, SPAN_END and (2026-07-10 corrected)
    // FULL_BURST all reproduce the silicon fingerprint ERR = 4*(n_bursts-1); only WR_COALESCE
    // (no CS# boundary at all) restores ERR=0 among the pre-replay remedies.
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
      $display("   [depth4 FULL_BURST] LEN=%0d wburst=%0d done=%0b ERR=%0d (expect %0d — falsified on silicon)",
               sw_len[i], sw_burst[i], done, err, 4 * sw_nm1[i]);
      check(done, $sformatf("depth4 FULL_BURST LEN=%0d did not complete (STATUS=0x%08x)", sw_len[i], status));
      check(err == 32'(4 * sw_nm1[i]),
            $sformatf("depth4 FULL_BURST LEN=%0d ERR=%0d expected %0d (2026-07-10 silicon: even a full re-read of the closed segment commits nothing when a write follows)",
                      sw_len[i], err, 4 * sw_nm1[i]));
    end

    // ---- (F) WR_CHOP_REPLAY (issue #1 direction 5, 2026-07-10) ----
    // Forced intra-command chops (MAX_BURST_WORDS=64) + WR_COALESCE against the depth-4 model.
    // Replay ON (idx 10): every chop reopens 4 words early and re-sends the words the device is
    // about to discard -> ERR=0. Replay OFF (idx 11): 4 words lost per chop.
    $display("-- (F) WR_CHOP_REPLAY: depth-4 model, WR_COALESCE=1, MAX_BURST_WORDS=64 --");
    // (F1) one native command chopped intra-command: LEN=768 = 12 segments = 11 chops.
    run_one(10, 32'd768, 32'h0000_0100, 32'd768, 32'd768, done, status, err);
    $display("   [replay  LEN=768/768 ] done=%0b ERR=%0d (expect 0)", done, err);
    check(done, "replay LEN=768 did not complete");
    check(err == 32'd0, $sformatf("replay LEN=768 ERR=%0d expected 0 (chops replayed)", err));

    run_one(11, 32'd768, 32'h0000_0100, 32'd768, 32'd768, done, status, err);
    $display("   [noreplay LEN=768/768] done=%0b ERR=%0d (expect 44 = 4 per chop x 11)", done, err);
    check(done, "no-replay LEN=768 did not complete");
    check(err == 32'd44,
          $sformatf("no-replay LEN=768 ERR=%0d expected 44 (11 chops x 4 pending words)", err));

    // (F2) multi-command stream: coalescing splices the command boundaries onto the same CS#, the
    // hw-cap chops the chain every 64 words, replay covers every chop -> ERR=0 end to end.
    run_one(10, 32'd4096, 32'h0000_0100, 32'd256, 32'd256, done, status, err);
    $display("   [replay  LEN=4096/256] done=%0b ERR=%0d (expect 0)", done, err);
    check(done, "replay LEN=4096 did not complete");
    check(err == 32'd0,
          $sformatf("replay LEN=4096 ERR=%0d expected 0 (coalesce + chop + replay compose)", err));

    // (F3) BURST_BOUNDARY-crossing write: base=0x1FE0, LEN=64 crosses the 0x2000-word boundary at
    // word 32 -> one boundary chop. The replayed reopen starts at 0x1FFC (4 words BEFORE the
    // boundary — the only way the pre-boundary tail can be re-written; model boundary-release is
    // OFF in these stacks, see the stack comment) and runs through 0x201F.
    run_one(10, 32'd64, 32'h0000_1FE0, 32'd64, 32'd64, done, status, err);
    $display("   [replay  bound cross ] done=%0b ERR=%0d (expect 0)", done, err);
    check(done, "replay boundary-cross did not complete");
    check(err == 32'd0, $sformatf("replay boundary-cross ERR=%0d expected 0", err));

    run_one(11, 32'd64, 32'h0000_1FE0, 32'd64, 32'd64, done, status, err);
    $display("   [noreplay bound cross] done=%0b ERR=%0d (expect 4 = one chop)", done, err);
    check(done, "no-replay boundary-cross did not complete");
    check(err == 32'd4,
          $sformatf("no-replay boundary-cross ERR=%0d expected 4 (pre-boundary tail lost)", err));

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
