// tb_cal — self-checking Verilator TB: RUNTIME PHY read-eye calibration (REG_CAL), no recompile.
//
// Proves the new cal_* path end to end: a live REG_CAL CSR write reprograms the SDR PHY's read-strobe
// preamble-skip mid-session and fixes a read that was mis-aligned at power-on — with NO rebuild. This
// is the software half of what the AXC3000 needs (sweep cal on hardware instead of recompiling Quartus).
//
// Setup (mirrors sim/tb_multiburst.sv / tb_preamble.sv): hyperram_bw_top (PHY_VARIANT="SDR") driving a
// hyperram_model that emits the real Winbond W957D8NB read-strobe PREAMBLE (RD_PREAMBLE_CLOCKS=1, one
// RWDS pulse with DQ Hi-Z=0x00 before the first real read byte). CAL_RESET=0 => POR cal_preamble_skip=0,
// so out of reset the PHY pairs that phantom {0x00,0x00} preamble word into the read stream and shifts
// every word by one — the exact on-silicon bring-up mis-read.
//
// Geometry is load-bearing: BASE_ADDR=0, LEN=16 (= BURST_WORDS => a SINGLE Avalon burst, so the
// over-stream/multi-burst hang is out of scope — that's tb_multiburst's job). gen_pattern(0)==0
// (hyperram_bw_test xorshift), so the phantom 0x0000 preamble word coincidentally MATCHES the expected
// word-0 pattern; the remaining 15 words are each shifted => a deterministic ERR_COUNT == LEN-1 == 15
// (not 16). See tb_preamble.sv for the same phantom-word mechanism.
//
//   Run 1 (CAL_RESET default, cal_preamble_skip=0): pulse start, poll done, ERR_COUNT must be 15.
//   Live CSR write REG_CAL <= 0x2 (bits[3:1]=1 => cal_preamble_skip=1), no recompile.
//   Run 2 (re-pulse start; S_IDLE re-arms a full write-then-read cycle): ERR_COUNT must be 0.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_cal;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH       = HB_DQ_WIDTH_DEFAULT;    // 8
  localparam int unsigned DATA_WIDTH     = 2 * DQ_WIDTH;           // 16
  localparam int unsigned ADDR_WIDTH     = HB_ADDR_WIDTH;          // 32
  localparam int unsigned LEN_WIDTH      = HB_LEN_WIDTH_DEFAULT;   // 16
  localparam int unsigned CSR_ADDR_WIDTH = 5;                      // 32 regs (issue #13) — REG_CAL is word 13; width 5 avoids REG_PAT/WRAP/EMAP aliasing
  localparam int unsigned BURST_WORDS    = HB_BURST_WORDS_DEFAULT; // 16 (single-burst LEN)

  // CR0 image: latency code 0001 (=6), fixed-latency, legacy wrap, 32B group (matches the board/model).
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam logic [31:0] TB_MAGIC    = 32'h4842_5754;             // "HBWT"

  // CSR word-register indices (byte offset >> 2).
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_CTRL   = 4'd0;   // W: CTRL / R: STATUS
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_LEN    = 4'd1;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_BASE   = 4'd2;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRCNT = 4'd5;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_CAL    = 4'd13;  // R/W: live PHY read-eye calibration

  // REG_CAL bit map: [0]=cal_capture_phase [3:1]=cal_preamble_skip [8:4]=cal_rx_tap [9]=cal_pair_skew.
  localparam logic [31:0] CAL_SKIP1 = 32'h0000_0002;         // cal_preamble_skip=1, all other knobs 0

  localparam logic [31:0]           TB_LEN  = 32'd16;        // = BURST_WORDS => a single Avalon burst
  localparam logic [ADDR_WIDTH-1:0] TB_BASE = 32'h0000_0000; // BASE_ADDR=0 (required for ERR_COUNT==LEN-1)
  localparam logic [31:0]           EXP_ERR = TB_LEN - 32'd1;// 15 phantom-shifted words, word 0 aliases

  // --------------------------------------------------------------------
  // Clocking / reset — SDR arrangement (as tb_sdr / tb_multiburst / the board):
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
  // HyperBus device pins: master (PHY) side + device (model) side + resolution (as tb_bw / tb_sdr)
  // --------------------------------------------------------------------
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  phy_dq_o;   logic phy_dq_oe;
  logic                 phy_rwds_o; logic phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;
  logic                 mdl_rwds_o; logic mdl_rwds_oe;

  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);

  // Round-trip DQ/RWDS flight delay (device -> master), as in tb_sdr / tb_multiburst.
  localparam realtime RTT = 3.0;    // ns
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  // --------------------------------------------------------------------
  // DUT: bandwidth-test top (bench engine + hyperram_avalon SDR IP). CAL_RESET=0 => POR
  // cal_preamble_skip=0 (the buggy state); RD_PREAMBLE_SKIP=0 seeds the PHY to match.
  // --------------------------------------------------------------------
  hyperram_bw_top #(
    .DQ_WIDTH         (DQ_WIDTH),
    .DATA_WIDTH       (DATA_WIDTH),
    .ADDR_WIDTH       (ADDR_WIDTH),
    .LEN_WIDTH        (LEN_WIDTH),
    .BURST_WORDS      (BURST_WORDS),
    .CSR_ADDR_WIDTH   (CSR_ADDR_WIDTH),
    .VERSION_MAGIC    (TB_MAGIC),
    .LATENCY_CLOCKS   (6),
    .FIXED_LATENCY    (1'b1),
    .MAX_BURST_WORDS  (0),
    .PROGRAM_CR       (1'b1),
    .POR_DELAY_CYCLES (0),
    .INIT_CR0         (TB_INIT_CR0),
    .PHY_VARIANT      ("SDR"),
    .DIFF_CK          (1'b1),
    .RD_PREAMBLE_SKIP (0),          // POR seed; cal_preamble_skip (from CAL_RESET=0) also 0 => buggy
    .CAL_RESET        (32'h0)       // REG_CAL POR image: cal_preamble_skip=0 for Run 1
  ) dut (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    .csr_address     (csr_address),
    .csr_read        (csr_read),
    .csr_readdata    (csr_readdata),
    .csr_write       (csr_write),
    .csr_writedata   (csr_writedata),
    .csr_waitrequest (csr_waitrequest),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (phy_dq_o), .hb_dq_oe (phy_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (phy_rwds_o), .hb_rwds_oe (phy_rwds_oe), .hb_rwds_i (rwds_line_dly),
    .init_done (init_done)
  );

  // --------------------------------------------------------------------
  // Golden device model — emits the read PREAMBLE (as the board), no over-stream (single burst).
  // --------------------------------------------------------------------
  hyperram_model #(
    .DQ_WIDTH            (DQ_WIDTH),
    .MEM_WORDS           (1 << 16),
    .LATENCY_CLOCKS      (6),
    .FIXED_LATENCY       (1'b1),
    .ROW_WORDS           (0),
    .ROW_PENALTY         (4),
    .REFRESH_EVERY       (0),
    .RD_PREAMBLE_CLOCKS  (1)          // W957D8NB read-strobe preamble (the on-silicon behaviour)
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
  // CSR access tasks (drive on the falling edge; single-cycle, waitrequest tied low) — as tb_bw.
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

  // One bench write+read pass over LEN/BASE. The done-poll is BOUNDED (a hang surfaces as FAIL).
  task automatic run_pass(output logic done, output logic [31:0] status, output logic [31:0] err_count);
    int unsigned guard;
    csr_wr(REG_CTRL, 32'h0000_0001);        // pulse start
    guard = 0;
    do begin
      csr_rd(REG_CTRL, status);
      guard = guard + 1;
    end while (!status[1] && guard < 200000);
    done = status[1];
    csr_rd(REG_ERRCNT, err_count);
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  logic [31:0] status, err_count, rb;
  logic        done;
  int unsigned guard;

  initial begin
    csr_address   = '0; csr_read = 1'b0; csr_write = 1'b0; csr_writedata = '0;
    rst           = 1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk);
    rst = 1'b0;

    // ---- wait for POR init + CR0 programming ----
    guard = 0;
    while (!init_done && guard < 100000) begin @(posedge clk); guard = guard + 1; end
    check(init_done, "init_done never asserted");
    if (init_done) $display("[%0t] init_done asserted", $time);
    repeat (4) @(posedge clk);

    // ---- program LEN and BASE_ADDR (single 16-word burst at address 0) ----
    csr_wr(REG_LEN,  TB_LEN);
    csr_wr(REG_BASE, TB_BASE);

    // ================= RUN 1: POR cal (cal_preamble_skip=0) — phantom preamble mis-read ==========
    run_pass(done, status, err_count);
    $display("[%0t] RUN 1 (cal_preamble_skip=0): done=%0b STATUS=0x%08x ERR_COUNT=%0d (expect %0d)",
             $time, done, status, err_count, EXP_ERR);
    check(done, "RUN 1 did not complete (STATUS.done never asserted)");
    check(err_count == EXP_ERR,
          $sformatf("RUN 1 ERR_COUNT=%0d, expected %0d (=LEN-1; phantom-preamble mis-read at BASE=0)",
                    err_count, EXP_ERR));

    // ================= LIVE REG_CAL WRITE: cal_preamble_skip=1 — NO RECOMPILE ====================
    csr_wr(REG_CAL, CAL_SKIP1);
    csr_rd(REG_CAL, rb);
    check(rb == CAL_SKIP1, $sformatf("REG_CAL readback 0x%08x exp 0x%08x", rb, CAL_SKIP1));
    $display("[%0t] REG_CAL <= 0x%08x (cal_preamble_skip=1) — retuned live, no recompile", $time, CAL_SKIP1);

    // ================= RUN 2: retuned cal (cal_preamble_skip=1) — clean, exact read-back =========
    run_pass(done, status, err_count);
    $display("[%0t] RUN 2 (cal_preamble_skip=1): done=%0b STATUS=0x%08x ERR_COUNT=%0d (expect 0)",
             $time, done, status, err_count);
    check(done, "RUN 2 did not complete (STATUS.done never asserted)");
    check(status[2] === 1'b0, $sformatf("RUN 2 STATUS.error set (STATUS=0x%08x)", status));
    check(err_count == 32'd0,
          $sformatf("RUN 2 ERR_COUNT=%0d, expected 0 (live REG_CAL preamble-skip should fix the read)",
                    err_count));

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_cal done: %0d errors  (Run1 ERR=%0d->fixed to Run2 ERR=%0d via live REG_CAL)",
             $time, errors, EXP_ERR, err_count);
    if (errors == 0) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_cal: %0d errors", errors);
    end
  end

  // Global watchdog.
  initial begin
    #10_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_cal: global timeout");
  end

endmodule
