// tb_avalon — self-checking Verilator testbench for hyperram_avalon + hyperram_model.
//
// Instantiates the Avalon-MM top and the golden device model, resolves the shared split HyperBus
// bus (DQ/RWDS) between master (PHY) and device (model), and exercises:
//   * POR init + CR0 programming (waits for init_done),
//   * single-word memory write-then-read-back at several addresses,
//   * multi-word (burst) memory write-then-read-back at several addresses/lengths,
//   * a burst spanning a wrap-group boundary (linear front-end; verifies address advance),
//   * a CR0 register write + read-back,
//   * an ID0 register read (expects the reset device-ID from the package).
// Data is address-derived (genword) so every readback word is checked against an independent
// expectation. Any mismatch -> error count > 0 -> $fatal (non-zero exit). Success -> $finish.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_avalon;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2

  // Latency 6, fixed. INIT_CR0 image programmed at init must carry latency code 0001 (=6 clocks),
  // fixed-latency bit, so the model's post-CR0 latency matches the controller's LATENCY_CLOCKS.
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F

  localparam int unsigned REG_MSB = ADDR_WIDTH - 1;           // Avalon addr MSB selects register space

  // --------------------------------------------------------------------
  // Clocking / reset
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;    end   // 100 MHz
  initial begin #2.5; clk90  = 1'b0; forever #5.0 clk90  = ~clk90;  end // +90 deg
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end   // (tie-off for GENERIC)

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
  // HyperBus device pins: master (PHY) side + device (model) side + resolution
  // --------------------------------------------------------------------
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  phy_dq_o;   logic phy_dq_oe;
  logic                 phy_rwds_o; logic phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;
  logic                 mdl_rwds_o; logic mdl_rwds_oe;

  // Shared, resolved bus lines (single active driver at a time, enforced by protocol).
  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);

  // Round-trip DQ/RWDS flight delay (device -> master). Delaying the read path proves the PHY recovers
  // read data source-synchronously to RWDS (finding #4), independent of round-trip delay.
  localparam realtime RTT = 3.0;    // ns
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  // --------------------------------------------------------------------
  // DUT: Avalon top
  // --------------------------------------------------------------------
  hyperram_avalon #(
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
    // runtime read-eye calibration tied to POR-equivalent constants (reproduces pre-cal behaviour)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd0), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
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
    .init_done (init_done), .err_underrun (/* unused */), .dbg_bus (),
    // issue #13: new hyperram_avalon debug bundle + wrap_en tied to per-instance legacy (A1).
    .dbg_wr_lat_trim (4'd0), .dbg_lat_clocks (4'd6), .dbg_cr0_reprog (1'b0),
    .dbg_prewin_drive (1'b0), .dbg_prewin_n (3'd0), .dbg_prewin_marker (1'b0),
    .dbg_postwin_hold (1'b0), .dbg_prewin_contig (1'b0), .dbg_end_cread (1'b0), .wrap_en (1'b0)
  );

  // --------------------------------------------------------------------
  // Golden device model
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
  // Scoreboard bookkeeping
  // --------------------------------------------------------------------
  int unsigned errors = 0;
  int unsigned checks = 0;

  // Read-data capture: a small collection buffer filled by readdatavalid.
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

  // Address-derived data pattern (independent of the model's power-on fill).
  function automatic logic [DATA_WIDTH-1:0] genword(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction

  // --------------------------------------------------------------------
  // Avalon transaction tasks.
  //
  // All stimulus is driven on the FALLING edge (blocking assignments) so every DUT input is stable
  // across the rising edge where the front-end/controller sample it — this removes the
  // driver-vs-sampler delta race that would otherwise corrupt the valid/ready handshake. Status
  // (waitrequest / readdatavalid) is sampled on the rising edge.
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

  // Linear write burst of `n` words at word `addr` (reg=1 => register space via MSB).
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
    // A word is accepted on each rising edge waitrequest is low (only happens in WR_DATA).
    g = 0;
    forever begin
      @(posedge clk);
      g = g + 1;
      if (g > 5000) begin
        $display("[%0t] HANG do_write @0x%08x reg=%0b idx=%0d/%0d wait=%0b",
                 $time, addr, reg_space, idx, n, avs_waitrequest);
        errors = errors + 1; break;
      end
      if (!avs_waitrequest) begin
        idx = idx + 1;
        if (idx == n) break;             // last beat accepted this edge
        @(negedge clk);
        avs_writedata = data[idx];       // present next word for the following cycle
      end
    end
    avs_idle();                          // deassert write AFTER the accepting edge
  endtask

  // Linear read burst of `n` words at word `addr`; results land in cap[0..n-1].
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
    // Command accepted when waitrequest drops on a rising edge (IDLE: waitrequest = ~cmd_ready).
    guard = 0;
    forever begin
      @(posedge clk);
      guard = guard + 1;
      if (guard > 3000) begin
        $display("[%0t] HANG do_read accept @0x%08x wait=%0b", $time, addr, avs_waitrequest);
        errors = errors + 1; break;
      end
      if (!avs_waitrequest) break;
    end
    avs_idle();                          // deassert read AFTER the accepting edge
    // Wait for all n words (with a generous guard against a hang).
    guard = 0;
    while (cap_n < n && guard < 3000) begin
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

  // Write then read back a linear memory burst, checking each word.
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
    $display("[%0t] %s: %0d-word wr/rd @0x%08x done (errors so far %0d)", $time, tag, n, addr, errors);
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  logic [DATA_WIDTH-1:0] one [$];
  logic [DATA_WIDTH-1:0] cr0_rb, id0_rb;
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

    // Wait for POR init + CR0 programming to complete.
    guard = 0;
    while (!init_done && guard < 100000) begin @(posedge clk); guard = guard + 1; end
    if (!init_done) begin
      $display("[%0t] FATAL: init_done never asserted", $time);
      errors = errors + 1;
    end else begin
      $display("[%0t] init_done asserted", $time);
    end
    repeat (4) @(posedge clk);

    // ---- single-word writes/reads at several addresses ----
    wr_rd_check(32'h0000_0000, 1, "single");
    wr_rd_check(32'h0000_0041, 1, "single");
    wr_rd_check(32'h0000_1234, 1, "single");

    // ---- multi-word (burst) writes/reads at several addresses/lengths ----
    wr_rd_check(32'h0000_0010, 4,  "burst4");
    wr_rd_check(32'h0000_0100, 8,  "burst8");
    wr_rd_check(32'h0000_2000, 16, "burst16");
    // burst crossing a 16-word (32B) wrap-group boundary, linear:
    wr_rd_check(32'h0000_0038, 20, "cross");

    // ---- CR0 register write + read-back (value keeps latency/fixed unchanged) ----
    one = {}; one.push_back(TB_INIT_CR0);
    do_write(HB_REG_CR0[ADDR_WIDTH-1:0], 1, 1'b1, one);
    do_read (HB_REG_CR0[ADDR_WIDTH-1:0], 1, 1'b1);
    cr0_rb = cap[0];
    checks = checks + 1;
    if (cr0_rb !== TB_INIT_CR0) begin
      $display("[%0t] ERROR: CR0 readback 0x%04x exp 0x%04x", $time, cr0_rb, TB_INIT_CR0);
      errors = errors + 1;
    end else $display("[%0t] CR0 write+readback ok (0x%04x)", $time, cr0_rb);

    // ---- ID0 register read (read-only reset value from the package) ----
    do_read(HB_REG_ID0[ADDR_WIDTH-1:0], 1, 1'b1);
    id0_rb = cap[0];
    checks = checks + 1;
    if (id0_rb !== HB_ID0_RESET) begin
      $display("[%0t] ERROR: ID0 readback 0x%04x exp 0x%04x", $time, id0_rb, HB_ID0_RESET);
      errors = errors + 1;
    end else $display("[%0t] ID0 read ok (0x%04x)", $time, id0_rb);

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_avalon done: %0d checks, %0d errors", $time, checks, errors);
    if (errors == 0) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_avalon: %0d errors", errors);
    end
  end

  // Global watchdog.
  initial begin
    #2_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_avalon: global timeout");
  end

endmodule
