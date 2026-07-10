// tb_multiburst — TEST-FIRST regression for the AXC3000 MULTI-BURST read over-stream hang.
//
// Reproduces the on-silicon multi-burst hang and proves the fix. On the AXC3000 a single 16-word
// (= BURST_WORDS) read completes, but any LEN > BURST_WORDS (which the bench engine issues as several
// back-to-back 16-word Avalon reads) HANGS: burst 1 delivers its 16 words, the bench requests read
// burst 2 (av_read=1), but the controller never accepts it (av_waitrequest stuck High, CS# never
// re-asserts, STATUS = busy). ROOT CAUSE: after the master stops CK at 16 words the real W957D8NB
// OVER-STREAMS a few extra read words (its read output pipeline drains past the master's CK-stop), so
// stray words linger in the read FIFOs; hyperbus_ctrl's ST_IDLE gates cmd_ready on rd_fifo_empty and
// never sees an empty FIFO, so it never accepts the next burst.
//
// This TB drives the EXACT board stack — bench engine (hyperram_bw_test) -> hyperram_avalon
// (PHY_VARIANT="SDR", RD_PREAMBLE_SKIP=1) -> golden model — but the model now emits the read
// over-stream (hyperram_model RD_OVERSTREAM_WORDS = OS_WORDS) AND the read preamble
// (RD_PREAMBLE_CLOCKS=1), exactly as the silicon does. It runs the bench for LEN = 16 (single burst,
// must already pass), then LEN = 32 / 64 / 256 (multi-burst) and asserts each run COMPLETES
// (STATUS.done) with ERR_COUNT==0 and STATUS.error==0. Every poll loop is BOUNDED, so the hang shows
// up as a FAIL (bounded timeout) rather than an infinite loop.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_multiburst;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH       = HB_DQ_WIDTH_DEFAULT;    // 8
  localparam int unsigned DATA_WIDTH     = 2 * DQ_WIDTH;           // 16
  localparam int unsigned ADDR_WIDTH     = HB_ADDR_WIDTH;          // 32
  localparam int unsigned LEN_WIDTH      = HB_LEN_WIDTH_DEFAULT;   // 16
  localparam int unsigned CSR_ADDR_WIDTH = 3;
  localparam int unsigned BURST_WORDS    = HB_BURST_WORDS_DEFAULT; // 16 (board Avalon burst size)

  // Number of EXTRA words the device over-streams after the master stops CK (silicon CK-stop pipeline
  // latency) — the AXC3000 saw ~7 extra ("~23 words for a 16-word request"). In this deterministic
  // 2-state sim a stray tail only contaminates the NEXT burst once it outlasts the natural inter-burst
  // gap (>= 8 words here); a real device's metastable RXF-flush hazard can bleed at fewer. 9 reliably
  // reproduces the corruption on the UNFIXED design; the fix must be robust to a VARIABLE count (the
  // sweep below also runs longer tails through the fixed design — see sim/run.sh notes).
  localparam int unsigned OS_WORDS = 9;

  // CR0 image: latency code 0001 (=6), fixed-latency, legacy wrap, 32B group (matches the board).
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam logic [31:0] TB_MAGIC    = 32'h4842_5754;             // "HBWT"

  // CSR word-register indices (byte offset >> 2).
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_CTRL   = 3'd0;   // W: CTRL / R: STATUS
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_LEN    = 3'd1;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_BASE   = 3'd2;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_WRCYC  = 3'd3;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_RDCYC  = 3'd4;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRCNT = 3'd5;

  localparam logic [ADDR_WIDTH-1:0] TB_BASE = 32'h0000_0100;       // memory space (MSB=0), in range
  localparam real                   F_CLK   = 50.0e6;              // word-clock (CK) rate for MB/s

  // --------------------------------------------------------------------
  // Clocking / reset — SDR arrangement (as tb_sdr / the board):
  //   clk   = 50 MHz  (CK-rate word clock; controller + bench)
  //   clk90 = 100 MHz (2x byte clock to the SDR PHY; phase-aligned)
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk     = 1'b0; forever #10.0 clk     = ~clk;     end   // 50 MHz
  initial begin clk90   = 1'b0; forever #5.0  clk90   = ~clk90;   end   // 100 MHz
  initial begin clk_ref = 1'b0; forever #5.0  clk_ref = ~clk_ref; end   // (tie-off)

  // --------------------------------------------------------------------
  // Bench CSR slave signals
  // --------------------------------------------------------------------
  logic [CSR_ADDR_WIDTH-1:0] csr_address;
  logic                      csr_read, csr_write;
  logic [31:0]               csr_writedata;
  logic [31:0]               csr_readdata;
  logic                      csr_waitrequest;
  logic                      init_done;

  // --------------------------------------------------------------------
  // HyperBus device pins: master (PHY) side + device (model) side + resolution
  // --------------------------------------------------------------------
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  phy_dq_o;   logic phy_dq_oe;
  logic                 phy_rwds_o; logic phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;
  logic                 mdl_rwds_o; logic mdl_rwds_oe;

  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);

  // Round-trip DQ/RWDS flight delay (device -> master), as in tb_sdr.
  localparam realtime RTT = 3.0;    // ns
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  // --------------------------------------------------------------------
  // Avalon-MM link: bench master <-> hyperram_avalon slave (inlined exactly as fpga/axc3000/top.sv,
  // so RD_PREAMBLE_SKIP can be set — hyperram_bw_top does not expose it).
  // --------------------------------------------------------------------
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
    .clk             (clk),
    .rst             (rst),
    .csr_address     (csr_address),
    .csr_read        (csr_read),
    .csr_readdata    (csr_readdata),
    .csr_write       (csr_write),
    .csr_writedata   (csr_writedata),
    .csr_waitrequest (csr_waitrequest),
    .m_address       (av_address),
    .m_burstcount    (av_burstcount),
    .m_read          (av_read),
    .m_write         (av_write),
    .m_writedata     (av_writedata),
    .m_readdata      (av_readdata),
    .m_readdatavalid (av_readdatavalid),
    .m_waitrequest   (av_waitrequest),
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
    .PROGRAM_CR       (1'b1),
    .POR_DELAY_CYCLES (0),
    .INIT_CR0         (TB_INIT_CR0),
    .PHY_VARIANT      ("SDR"),
    .DIFF_CK          (1'b0),
    .RD_PREAMBLE_SKIP (1)              // board setting: discard the W957D8NB read-strobe preamble edge
  ) u_hyperram (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // cal tied to constants reproducing RD_PREAMBLE_SKIP=1 (board setting; preamble edge discarded)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd1), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .avs_address       (av_address),
    .avs_read          (av_read),
    .avs_write         (av_write),
    .avs_writedata     (av_writedata),
    .avs_byteenable    (2'b11),
    .avs_burstcount    (av_burstcount),
    .avs_readdata      (av_readdata),
    .avs_readdatavalid (av_readdatavalid),
    .avs_waitrequest   (av_waitrequest),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (phy_dq_o), .hb_dq_oe (phy_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (phy_rwds_o), .hb_rwds_oe (phy_rwds_oe), .hb_rwds_i (rwds_line_dly),
    .init_done (init_done), .err_underrun (), .dbg_bus ()
  );

  // --------------------------------------------------------------------
  // Golden device model — now emits the read preamble AND the read over-stream tail (silicon).
  // --------------------------------------------------------------------
  hyperram_model #(
    .DQ_WIDTH             (DQ_WIDTH),
    .MEM_WORDS            (1 << 16),
    .LATENCY_CLOCKS       (6),
    .FIXED_LATENCY        (1'b1),
    .ROW_WORDS            (0),
    .ROW_PENALTY          (4),
    .REFRESH_EVERY        (0),
    .RD_PREAMBLE_CLOCKS   (1),         // W957D8NB read-strobe preamble (as the board)
    .RD_OVERSTREAM_WORDS  (OS_WORDS)   // <-- silicon CK-stop over-stream (the multi-burst hang cause)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i  (dq_line),  .hb_dq_ie  (phy_dq_oe),
    .hb_dq_o  (mdl_dq_o), .hb_dq_oe  (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (phy_rwds_oe),
    .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
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
  // CSR access tasks (drive on the falling edge; single-cycle, waitrequest tied low).
  // --------------------------------------------------------------------
  task automatic csr_wr(input logic [CSR_ADDR_WIDTH-1:0] addr, input logic [31:0] data);
    @(negedge clk);
    csr_address   = addr;
    csr_writedata = data;
    csr_write     = 1'b1;
    csr_read      = 1'b0;
    @(negedge clk);
    csr_write     = 1'b0;
    csr_writedata = '0;
    csr_address   = '0;
  endtask

  task automatic csr_rd(input logic [CSR_ADDR_WIDTH-1:0] addr, output logic [31:0] data);
    @(negedge clk);
    csr_address = addr;
    csr_read    = 1'b1;
    csr_write   = 1'b0;
    #1;
    data        = csr_readdata;
    @(negedge clk);
    csr_read    = 1'b0;
    csr_address = '0;
  endtask

  // Run one bench write+read pass at LEN words. Returns completion + the counters. The done-poll is
  // BOUNDED: a multi-burst hang leaves done=0 (surfaces as FAIL, not an infinite loop).
  task automatic run_len(input logic [31:0] len,
                         output logic       done,
                         output logic [31:0] status,
                         output logic [31:0] err_count,
                         output logic [31:0] wr_cycles,
                         output logic [31:0] rd_cycles);
    int unsigned guard;
    csr_wr(REG_LEN,  len);
    csr_wr(REG_BASE, TB_BASE);
    csr_wr(REG_CTRL, 32'h0000_0001);        // pulse start
    guard = 0;
    do begin
      csr_rd(REG_CTRL, status);
      guard = guard + 1;
    end while (!status[1] && guard < 200000);
    done = status[1];
    csr_rd(REG_ERRCNT, err_count);
    csr_rd(REG_WRCYC,  wr_cycles);
    csr_rd(REG_RDCYC,  rd_cycles);
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  logic [31:0] status, err_count, wr_cycles, rd_cycles;
  logic        done;
  int unsigned guard;
  real         rd_mbps, wr_mbps;

  // burst-length sweep (single burst + several multi-burst cases)
  localparam int NLEN = 4;
  int unsigned lens [NLEN] = '{16, 32, 64, 256};

  initial begin
    csr_address   = '0; csr_read = 1'b0; csr_write = 1'b0; csr_writedata = '0;
    rst           = 1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk);
    rst = 1'b0;

    guard = 0;
    while (!init_done && guard < 100000) begin @(posedge clk); guard = guard + 1; end
    check(init_done, "init_done never asserted");
    if (init_done) $display("[%0t] init_done asserted (OS_WORDS=%0d)", $time, OS_WORDS);
    repeat (4) @(posedge clk);

    $display("==================================================================");
    $display("tb_multiburst: burst-length sweep (BURST_WORDS=%0d, over-stream=%0d words)",
             BURST_WORDS, OS_WORDS);
    $display("==================================================================");

    for (int i = 0; i < NLEN; i++) begin
      run_len(lens[i][31:0], done, status, err_count, wr_cycles, rd_cycles);
      wr_mbps = (wr_cycles != 0) ? (real'(lens[i]) * 2.0 * F_CLK / (real'(wr_cycles) * 1.0e6)) : 0.0;
      rd_mbps = (rd_cycles != 0) ? (real'(lens[i]) * 2.0 * F_CLK / (real'(rd_cycles) * 1.0e6)) : 0.0;
      $display("  LEN=%0d  done=%0b STATUS=0x%08x ERR_COUNT=%0d  WR_CYCLES=%0d (%.2f MB/s)  RD_CYCLES=%0d (%.2f MB/s)",
               lens[i], done, status, err_count, wr_cycles, wr_mbps, rd_cycles, rd_mbps);
      check(done,             $sformatf("LEN=%0d did not complete (STATUS=0x%08x) — multi-burst HANG",
                                        lens[i], status));
      check(status[2] === 1'b0, $sformatf("LEN=%0d STATUS.error set (STATUS=0x%08x)", lens[i], status));
      check(err_count == 32'd0, $sformatf("LEN=%0d ERR_COUNT=%0d (expected 0)", lens[i], err_count));
    end

    $display("==================================================================");
    $display("[%0t] tb_multiburst done: %0d errors", $time, errors);
    if (errors == 0) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_multiburst: %0d errors", errors);
    end
  end

  // Global watchdog — a true infinite hang (should never happen; every poll is bounded).
  initial begin
    #20_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_multiburst: global timeout");
  end

endmodule
