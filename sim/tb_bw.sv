// tb_bw — self-checking Verilator testbench for the bandwidth-test harness.
//
// Instantiates hyperram_bw_top (bench traffic generator + hyperram_avalon master IP) and the golden
// hyperram_model device, resolving the shared split HyperBus bus (DQ/RWDS) exactly as sim/tb_avalon.sv
// does. It then drives the bench CSR slave the way a JTAG-to-Avalon host would:
//   * waits for POR init (init_done),
//   * programs LEN and BASE_ADDR,
//   * pulses CTRL.start,
//   * polls STATUS.done,
//   * reads back WR_CYCLES / RD_CYCLES / ERR_COUNT / DATA_BYTES_PER_WORD / MAGIC.
//
// The bench engine writes a LEN-word address-seeded pattern through the REAL controller+model, then
// reads it back and self-checks. This TB asserts the integrity is clean (ERR_COUNT==0 and
// STATUS.error==0 — the harness read back exactly what it wrote), that the run completed (done), and
// that WR_CYCLES>0 and RD_CYCLES>0 and both sit in a sane range for LEN (>= LEN, not absurdly large).
// It prints the measured cycle counts and the implied bytes moved. Any failure -> $fatal (non-zero).
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_bw;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH       = HB_DQ_WIDTH_DEFAULT;    // 8
  localparam int unsigned DATA_WIDTH     = 2 * DQ_WIDTH;           // 16
  localparam int unsigned ADDR_WIDTH     = HB_ADDR_WIDTH;          // 32
  localparam int unsigned LEN_WIDTH      = HB_LEN_WIDTH_DEFAULT;   // 16
  localparam int unsigned CSR_ADDR_WIDTH = 3;
  localparam int unsigned BURST_WORDS    = HB_BURST_WORDS_DEFAULT; // 16

  // CR0 image: latency code 0001 (=6), fixed-latency, legacy wrap, 32B group (matches tb_avalon).
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam logic [31:0] TB_MAGIC    = 32'h4842_5754;             // "HBWT"

  // CSR word-register indices (byte offset >> 2).
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_CTRL   = 3'd0;   // W: CTRL / R: STATUS
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_LEN    = 3'd1;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_BASE   = 3'd2;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_WRCYC  = 3'd3;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_RDCYC  = 3'd4;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRCNT = 3'd5;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_BYTES  = 3'd6;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_MAGIC  = 3'd7;

  // Test geometry.
  localparam logic [31:0]           TB_LEN  = 32'd64;              // 4 bursts of BURST_WORDS(16)
  localparam logic [ADDR_WIDTH-1:0] TB_BASE = 32'h0000_0100;      // memory space (MSB=0), in range

  // --------------------------------------------------------------------
  // Clocking / reset (mirror tb_avalon)
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk     = 1'b0; forever #5.0 clk     = ~clk;     end   // 100 MHz
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90  = ~clk90;  end // +90 deg
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end   // (tie-off for GENERIC)

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
  // (identical scheme to sim/tb_avalon.sv)
  // --------------------------------------------------------------------
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  phy_dq_o;   logic phy_dq_oe;
  logic                 phy_rwds_o; logic phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;
  logic                 mdl_rwds_o; logic mdl_rwds_oe;

  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);

  // Round-trip flight delay on the read path (device -> master), as in tb_avalon.
  localparam realtime RTT = 3.0;    // ns
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  // --------------------------------------------------------------------
  // DUT: bandwidth-test top (bench engine + hyperram_avalon master IP)
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
    .PHY_VARIANT      ("GENERIC"),
    .DIFF_CK          (1'b1)
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
  // Golden device model (same config as tb_avalon)
  // --------------------------------------------------------------------
  hyperram_model #(
    .DQ_WIDTH       (DQ_WIDTH),
    .MEM_WORDS      (1 << 16),
    .LATENCY_CLOCKS (6),
    .FIXED_LATENCY  (1'b1),
    .ROW_WORDS      (0),          // disable mid-burst row-crossing gaps for this TB
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
  // CSR access tasks. Drive on the falling edge so inputs are stable across the
  // rising edge the CSR slave/FSM samples. csr_waitrequest is tied low (single-
  // cycle accesses); reads are combinational on csr_address.
  // --------------------------------------------------------------------
  task automatic csr_wr(input logic [CSR_ADDR_WIDTH-1:0] addr, input logic [31:0] data);
    @(negedge clk);
    csr_address   = addr;
    csr_writedata = data;
    csr_write     = 1'b1;
    csr_read      = 1'b0;
    @(negedge clk);              // one rising edge sampled the write
    csr_write     = 1'b0;
    csr_writedata = '0;
    csr_address   = '0;
  endtask

  task automatic csr_rd(input logic [CSR_ADDR_WIDTH-1:0] addr, output logic [31:0] data);
    @(negedge clk);
    csr_address = addr;
    csr_read    = 1'b1;
    csr_write   = 1'b0;
    #1;                          // combinational readback settles
    data        = csr_readdata;
    @(negedge clk);
    csr_read    = 1'b0;
    csr_address = '0;
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  logic [31:0] status, wr_cycles, rd_cycles, err_count, bytes_per_word, magic, rb;
  longint unsigned implied_bytes, lo, hi;
  int unsigned guard;

  initial begin
    csr_address   = '0;
    csr_read      = 1'b0;
    csr_write     = 1'b0;
    csr_writedata = '0;
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

    // ---- sanity: constant/identity CSRs ----
    csr_rd(REG_MAGIC, magic);
    check(magic === TB_MAGIC, $sformatf("MAGIC 0x%08x exp 0x%08x", magic, TB_MAGIC));
    csr_rd(REG_BYTES, bytes_per_word);
    check(bytes_per_word == 32'd2, $sformatf("DATA_BYTES_PER_WORD %0d exp 2", bytes_per_word));

    // ---- program LEN and BASE_ADDR, verify read-back ----
    csr_wr(REG_LEN,  TB_LEN);
    csr_wr(REG_BASE, TB_BASE);
    csr_rd(REG_LEN,  rb);
    check(rb == TB_LEN, $sformatf("LEN readback %0d exp %0d", rb, TB_LEN));
    csr_rd(REG_BASE, rb);
    check(rb == 32'(TB_BASE), $sformatf("BASE readback 0x%08x exp 0x%08x", rb, TB_BASE));

    // ---- pulse CTRL.start ----
    csr_wr(REG_CTRL, 32'h0000_0001);

    // Engine should go busy; confirm it left IDLE.
    csr_rd(REG_CTRL, status);
    check(status[0] === 1'b1 || status[1] === 1'b1,
          $sformatf("engine did not go busy after start (STATUS=0x%08x)", status));

    // ---- poll STATUS.done ----
    guard = 0;
    do begin
      csr_rd(REG_CTRL, status);
      guard = guard + 1;
    end while (!status[1] && guard < 200000);
    check(status[1] === 1'b1, $sformatf("STATUS.done never asserted (STATUS=0x%08x)", status));
    check(status[0] === 1'b0, $sformatf("STATUS.busy still set at done (STATUS=0x%08x)", status));
    check(status[2] === 1'b0, $sformatf("STATUS.error set (STATUS=0x%08x)", status));

    // ---- read the measured counters ----
    csr_rd(REG_WRCYC,  wr_cycles);
    csr_rd(REG_RDCYC,  rd_cycles);
    csr_rd(REG_ERRCNT, err_count);

    // ---- integrity: exact read-back through the real controller+model ----
    check(err_count == 32'd0, $sformatf("ERR_COUNT=%0d (expected 0)", err_count));

    // ---- cycle-count sanity: nonzero and within a sane range for LEN ----
    // Lower bound: each of LEN words occupies at least one counted clk in its beat state,
    // so both phases must count >= LEN cycles.
    // Upper bound: even with per-burst command setup + full read latency + bubbles, a phase
    // cannot plausibly exceed a few dozen cycles per word; 64x LEN is a generous ceiling that
    // still rejects a runaway/absurd count.
    lo = TB_LEN;
    hi = longint'(TB_LEN) * 64;
    check(wr_cycles > 0,  "WR_CYCLES is zero");
    check(rd_cycles > 0,  "RD_CYCLES is zero");
    check(wr_cycles >= lo && wr_cycles <= hi,
          $sformatf("WR_CYCLES=%0d out of sane range [%0d..%0d]", wr_cycles, lo, hi));
    check(rd_cycles >= lo && rd_cycles <= hi,
          $sformatf("RD_CYCLES=%0d out of sane range [%0d..%0d]", rd_cycles, lo, hi));

    // ---- report ----
    implied_bytes = longint'(TB_LEN) * longint'(bytes_per_word);
    $display("==================================================================");
    $display("[%0t] tb_bw measurement:", $time);
    $display("    LEN                = %0d words", TB_LEN);
    $display("    BASE_ADDR          = 0x%08x", TB_BASE);
    $display("    DATA_BYTES_PER_WORD= %0d", bytes_per_word);
    $display("    implied bytes/phase= %0d bytes", implied_bytes);
    $display("    WR_CYCLES          = %0d clk", wr_cycles);
    $display("    RD_CYCLES          = %0d clk", rd_cycles);
    $display("    ERR_COUNT          = %0d", err_count);
    $display("    STATUS             = 0x%08x (busy=%0b done=%0b error=%0b)",
             status, status[0], status[1], status[2]);
    $display("==================================================================");
    $display("[%0t] tb_bw done: %0d errors", $time, errors);
    if (errors == 0) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_bw: %0d errors", errors);
    end
  end

  // Global watchdog.
  initial begin
    #5_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_bw: global timeout");
  end

endmodule
