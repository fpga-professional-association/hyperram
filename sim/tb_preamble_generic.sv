// tb_preamble_generic — TEST-FIRST regression for the GENERIC-PHY read-strobe PREAMBLE bug.
//
// GENERIC-variant parity for tb_preamble (which proves the fix on the SDR PHY). hyperbus_phy_generic
// is the DEFAULT, vendor-free reference PHY and had the SAME read-strobe preamble bug: the real
// Winbond W957D8NB drives a short RWDS PREAMBLE — RWDS toggles like CK with DQ Hi-Z (=0x00) for one CK
// cycle BEFORE the first real read byte — and the GENERIC PHY paired those preamble edges into a
// PHANTOM {0x00,0x00} word that shifted the whole read by one (fingerprint ERR_COUNT = LEN-1); on
// hardware STATUS never reached `done`.
//
// This TB drives the GENERIC datapath (hyperram_avalon, PHY_VARIANT="GENERIC") against a device model
// that now emits that preamble (hyperram_model RD_PREAMBLE_CLOCKS=1), using a REAL 90-degree clk/clk90
// pair (as tb_avalon) — NOT the SDR 2x byte clock — because the GENERIC PHY recovers read data source-
// synchronously off a ~90-degree-shifted RWDS strobe. It instantiates the SAME datapath TWICE:
//   * DUT_BUG : GENERIC PHY RD_PREAMBLE_SKIP=0  -> must MIS-READ (the reproduction; wrong/short read)
//   * DUT_FIX : GENERIC PHY RD_PREAMBLE_SKIP=1  -> must read back exactly what was written, and complete
// The TB PASSES iff the bug is reproduced on DUT_BUG *and* DUT_FIX is clean — i.e. the preamble skip
// both (a) is necessary and (b) is sufficient. DUT_FIX's very FIRST transaction (fix_mism1, the single-
// word read) must independently be 0 — the exact reset-safety case that broke without the explicit
// pre_skip/have_a initializers in hyperbus_phy_generic.sv (that variant's RX is clocked directly off
// rwds_dly with no free-running fallback, so a missed first async edge persists). Reads are bounded
// (guarded), so a reproduction that hangs on hardware surfaces here as a short/incomplete read rather
// than an infinite loop.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_preamble_generic;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2

  // Latency 6, fixed — matches tb_sdr so the model's post-CR0 latency lines up with the controller.
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F

  // Device read PREAMBLE length (CK cycles) and the matching PHY skip. One CK cycle == the AXC3000
  // capture (1 preamble RWDS pulse, cap idx85/86, ahead of the first real byte at idx89).
  localparam int unsigned PREAMBLE_CLOCKS = 1;
  localparam int unsigned FIX_SKIP        = 1;   // DUT_FIX: skip exactly the preamble pulse

  // --------------------------------------------------------------------
  // Clocking / reset: REAL 90-degree pair (as tb_avalon) — the GENERIC PHY needs a true quarter-cycle
  // phase, NOT the SDR 2x byte clock. clk = 100 MHz word clock; clk90 = clk shifted +90 deg (+2.5 ns).
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;    end   // 100 MHz
  initial begin #2.5; clk90  = 1'b0; forever #5.0 clk90  = ~clk90;  end // +90 deg
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end   // (tie-off for GENERIC)

  // --------------------------------------------------------------------
  // Shared Avalon-MM stimulus (driven by the tasks). Fanned to both DUTs; the active DUT is picked
  // by `sel` so only one datapath runs a transaction at a time (the other stays idle).
  // --------------------------------------------------------------------
  logic                    sel;               // 0 = DUT_BUG, 1 = DUT_FIX
  logic [ADDR_WIDTH-1:0]   s_address;
  logic                    s_read, s_write;
  logic [DATA_WIDTH-1:0]   s_writedata;
  logic [STRB_WIDTH-1:0]   s_byteenable;
  logic [LEN_WIDTH-1:0]    s_burstcount;

  // Muxed observation back to the tasks.
  logic [DATA_WIDTH-1:0]   m_readdata;
  logic                    m_readdatavalid;
  logic                    m_waitrequest;

  // ==================================================================================================
  //  Helper to build one GENERIC datapath + golden model with the read preamble.  (Written out twice —
  //  once per RD_PREAMBLE_SKIP — because the parameter differs; a task cannot span two instances.)
  // ==================================================================================================
  // ---- DUT_BUG (RD_PREAMBLE_SKIP = 0) ----
  logic                 b_hb_ck, b_hb_ck_n, b_hb_cs_n, b_hb_rst_n;
  logic [DQ_WIDTH-1:0]  b_phy_dq_o;   logic b_phy_dq_oe;
  logic                 b_phy_rwds_o; logic b_phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  b_mdl_dq_o;   logic b_mdl_dq_oe;
  logic                 b_mdl_rwds_o; logic b_mdl_rwds_oe;
  logic [DATA_WIDTH-1:0] b_readdata;  logic b_readdatavalid, b_waitrequest, b_init_done;

  wire [DQ_WIDTH-1:0] b_dq_line   = b_mdl_dq_oe   ? b_mdl_dq_o   : (b_phy_dq_oe   ? b_phy_dq_o   : '0);
  wire                b_rwds_line = b_mdl_rwds_oe ? b_mdl_rwds_o : (b_phy_rwds_oe ? b_phy_rwds_o : 1'b0);
  localparam realtime RTT = 3.0;    // ns round-trip flight delay (as tb_avalon)
  wire [DQ_WIDTH-1:0] b_dq_dly;   assign #RTT b_dq_dly   = b_dq_line;
  wire                b_rwds_dly; assign #RTT b_rwds_dly = b_rwds_line;

  hyperram_avalon #(
    .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1), .MAX_BURST_WORDS (0), .PROGRAM_CR (1'b1),
    .POR_DELAY_CYCLES (0), .INIT_CR0 (TB_INIT_CR0), .PHY_VARIANT ("GENERIC"), .DIFF_CK (1'b1),
    .RD_PREAMBLE_SKIP (0)                            // <-- unfixed PHY
  ) dut_bug (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // cal tied to constants reproducing RD_PREAMBLE_SKIP=0 (the unfixed/buggy PHY; tie-off in GENERIC)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd0), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .avs_address (s_address), .avs_read (s_read & ~sel), .avs_write (s_write & ~sel),
    .avs_writedata (s_writedata), .avs_byteenable (s_byteenable), .avs_burstcount (s_burstcount),
    .avs_readdata (b_readdata), .avs_readdatavalid (b_readdatavalid), .avs_waitrequest (b_waitrequest),
    .hb_ck (b_hb_ck), .hb_ck_n (b_hb_ck_n), .hb_cs_n (b_hb_cs_n), .hb_rst_n (b_hb_rst_n),
    .hb_dq_o (b_phy_dq_o), .hb_dq_oe (b_phy_dq_oe), .hb_dq_i (b_dq_dly),
    .hb_rwds_o (b_phy_rwds_o), .hb_rwds_oe (b_phy_rwds_oe), .hb_rwds_i (b_rwds_dly),
    .init_done (b_init_done), .err_underrun (), .dbg_bus (),
    // issue #13: new hyperram_avalon debug bundle + wrap_en tied to per-instance legacy (A1).
    .dbg_wr_lat_trim (4'd0), .dbg_lat_clocks (4'd6), .dbg_cr0_reprog (1'b0),
    .dbg_prewin_drive (1'b0), .dbg_prewin_n (3'd0), .dbg_prewin_marker (1'b0),
    .dbg_postwin_hold (1'b0), .dbg_prewin_contig (1'b0), .dbg_end_cread (1'b0), .wrap_en (1'b0)
  );
  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0),
    .RD_PREAMBLE_CLOCKS (PREAMBLE_CLOCKS)           // <-- device emits the read preamble
  ) model_bug (
    .hb_ck (b_hb_ck), .hb_ck_n (b_hb_ck_n), .hb_cs_n (b_hb_cs_n), .hb_rst_n (b_hb_rst_n),
    .hb_dq_i (b_dq_line), .hb_dq_ie (b_phy_dq_oe), .hb_dq_o (b_mdl_dq_o), .hb_dq_oe (b_mdl_dq_oe),
    .hb_rwds_i (b_rwds_line), .hb_rwds_ie (b_phy_rwds_oe),
    .hb_rwds_o (b_mdl_rwds_o), .hb_rwds_oe (b_mdl_rwds_oe)
  );

  // ---- DUT_FIX (RD_PREAMBLE_SKIP = FIX_SKIP) ----
  logic                 f_hb_ck, f_hb_ck_n, f_hb_cs_n, f_hb_rst_n;
  logic [DQ_WIDTH-1:0]  f_phy_dq_o;   logic f_phy_dq_oe;
  logic                 f_phy_rwds_o; logic f_phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  f_mdl_dq_o;   logic f_mdl_dq_oe;
  logic                 f_mdl_rwds_o; logic f_mdl_rwds_oe;
  logic [DATA_WIDTH-1:0] f_readdata;  logic f_readdatavalid, f_waitrequest, f_init_done;

  wire [DQ_WIDTH-1:0] f_dq_line   = f_mdl_dq_oe   ? f_mdl_dq_o   : (f_phy_dq_oe   ? f_phy_dq_o   : '0);
  wire                f_rwds_line = f_mdl_rwds_oe ? f_mdl_rwds_o : (f_phy_rwds_oe ? f_phy_rwds_o : 1'b0);
  wire [DQ_WIDTH-1:0] f_dq_dly;   assign #RTT f_dq_dly   = f_dq_line;
  wire                f_rwds_dly; assign #RTT f_rwds_dly = f_rwds_line;

  hyperram_avalon #(
    .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1), .MAX_BURST_WORDS (0), .PROGRAM_CR (1'b1),
    .POR_DELAY_CYCLES (0), .INIT_CR0 (TB_INIT_CR0), .PHY_VARIANT ("GENERIC"), .DIFF_CK (1'b1),
    .RD_PREAMBLE_SKIP (FIX_SKIP)                     // <-- fixed PHY
  ) dut_fix (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // cal tied to constants reproducing RD_PREAMBLE_SKIP=FIX_SKIP=1 (the fixed PHY; tie-off in GENERIC)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd1), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .avs_address (s_address), .avs_read (s_read & sel), .avs_write (s_write & sel),
    .avs_writedata (s_writedata), .avs_byteenable (s_byteenable), .avs_burstcount (s_burstcount),
    .avs_readdata (f_readdata), .avs_readdatavalid (f_readdatavalid), .avs_waitrequest (f_waitrequest),
    .hb_ck (f_hb_ck), .hb_ck_n (f_hb_ck_n), .hb_cs_n (f_hb_cs_n), .hb_rst_n (f_hb_rst_n),
    .hb_dq_o (f_phy_dq_o), .hb_dq_oe (f_phy_dq_oe), .hb_dq_i (f_dq_dly),
    .hb_rwds_o (f_phy_rwds_o), .hb_rwds_oe (f_phy_rwds_oe), .hb_rwds_i (f_rwds_dly),
    .init_done (f_init_done), .err_underrun (), .dbg_bus (),
    // issue #13: new hyperram_avalon debug bundle + wrap_en tied to per-instance legacy (A1).
    .dbg_wr_lat_trim (4'd0), .dbg_lat_clocks (4'd6), .dbg_cr0_reprog (1'b0),
    .dbg_prewin_drive (1'b0), .dbg_prewin_n (3'd0), .dbg_prewin_marker (1'b0),
    .dbg_postwin_hold (1'b0), .dbg_prewin_contig (1'b0), .dbg_end_cread (1'b0), .wrap_en (1'b0)
  );
  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0),
    .RD_PREAMBLE_CLOCKS (PREAMBLE_CLOCKS)
  ) model_fix (
    .hb_ck (f_hb_ck), .hb_ck_n (f_hb_ck_n), .hb_cs_n (f_hb_cs_n), .hb_rst_n (f_hb_rst_n),
    .hb_dq_i (f_dq_line), .hb_dq_ie (f_phy_dq_oe), .hb_dq_o (f_mdl_dq_o), .hb_dq_oe (f_mdl_dq_oe),
    .hb_rwds_i (f_rwds_line), .hb_rwds_ie (f_phy_rwds_oe),
    .hb_rwds_o (f_mdl_rwds_o), .hb_rwds_oe (f_mdl_rwds_oe)
  );

  // Muxed observation.
  always_comb begin
    m_readdata      = sel ? f_readdata      : b_readdata;
    m_readdatavalid = sel ? f_readdatavalid : b_readdatavalid;
    m_waitrequest   = sel ? f_waitrequest   : b_waitrequest;
  end

  // --------------------------------------------------------------------
  // Scoreboard capture (of the selected DUT).
  // --------------------------------------------------------------------
  localparam int unsigned CAP_MAX = 64;
  logic [DATA_WIDTH-1:0] cap [CAP_MAX];
  int unsigned           cap_n;
  logic                  capturing;

  always @(posedge clk) begin
    if (capturing && m_readdatavalid) begin
      if (cap_n < CAP_MAX) cap[cap_n] <= m_readdata;
      cap_n <= cap_n + 1;
    end
  end

  function automatic logic [DATA_WIDTH-1:0] genword(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction

  // --------------------------------------------------------------------
  // Avalon transaction tasks (operate on the muxed s_*/m_* signals).
  // --------------------------------------------------------------------
  task automatic avs_idle();
    @(negedge clk);
    s_address    = '0; s_read = 1'b0; s_write = 1'b0;
    s_writedata  = '0; s_byteenable = '1; s_burstcount = '0;
  endtask

  task automatic do_write(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                          input logic [DATA_WIDTH-1:0] data [$]);
    int unsigned idx, g;
    idx = 0;
    @(negedge clk);
    s_write = 1'b1; s_read = 1'b0; s_address = addr;
    s_burstcount = LEN_WIDTH'(n); s_byteenable = '1; s_writedata = data[0];
    g = 0;
    forever begin
      @(posedge clk);
      g = g + 1;
      if (g > 5000) begin
        $display("[%0t] HANG do_write @0x%08x idx=%0d/%0d", $time, addr, idx, n);
        break;
      end
      if (!m_waitrequest) begin
        idx = idx + 1;
        if (idx == n) break;
        @(negedge clk);
        s_writedata = data[idx];
      end
    end
    avs_idle();
  endtask

  // Read n words; returns the number actually returned in `got` (bounded — a hang shows as got<n).
  task automatic do_read(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                         output int unsigned got);
    int unsigned guard;
    cap_n = 0; capturing = 1'b1;
    @(negedge clk);
    s_read = 1'b1; s_write = 1'b0; s_address = addr; s_burstcount = LEN_WIDTH'(n);
    guard = 0;
    forever begin
      @(posedge clk);
      guard = guard + 1;
      if (guard > 3000) begin
        $display("[%0t] do_read command never accepted @0x%08x (bug DUT stuck)", $time, addr);
        break;
      end
      if (!m_waitrequest) break;
    end
    avs_idle();
    guard = 0;
    while (cap_n < n && guard < 3000) begin @(posedge clk); guard = guard + 1; end
    @(posedge clk);
    capturing = 1'b0;
    got = cap_n;
  endtask

  // Write a pattern then read it back; report mismatches + whether the read completed.
  task automatic wr_rd(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                       output int unsigned mism, output logic complete);
    logic [DATA_WIDTH-1:0] wdata [$];
    int unsigned i, got;
    wdata = {};
    for (i = 0; i < n; i++) wdata.push_back(genword(addr + i));
    do_write(addr, n, wdata);
    do_read (addr, n, got);
    complete = (got >= n);
    mism = 0;
    for (i = 0; i < n; i++) begin
      if (i >= got) begin mism = mism + 1; continue; end     // missing word = mismatch
      if (cap[i] !== genword(addr + i)) mism = mism + 1;
    end
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  int unsigned guard;
  int unsigned bug_mism1, bug_mism2, fix_mism1, fix_mism2;
  logic        bug_cplt1, bug_cplt2, fix_cplt1, fix_cplt2;
  logic        bug_reproduced, fix_clean;
  int unsigned errors;

  initial begin
    errors = 0;
    sel = 1'b0;
    s_address = '0; s_read = 1'b0; s_write = 1'b0;
    s_writedata = '0; s_byteenable = '1; s_burstcount = '0;
    capturing = 1'b0; cap_n = 0;
    rst = 1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk); rst = 1'b0;

    guard = 0;
    while (!(b_init_done && f_init_done) && guard < 200000) begin @(posedge clk); guard = guard + 1; end
    if (!(b_init_done && f_init_done)) begin
      $display("[%0t] FATAL: init_done never asserted (bug=%0b fix=%0b)", $time, b_init_done, f_init_done);
      errors = errors + 1;
    end else $display("[%0t] both DUTs init_done", $time);
    repeat (4) @(posedge clk);

    // ================= PHASE 1: BUGGY PHY (RD_PREAMBLE_SKIP=0) — expect the read to break =========
    sel = 1'b0;
    wr_rd(32'h0000_0000, 1, bug_mism1, bug_cplt1);
    wr_rd(32'h0000_0020, 4, bug_mism2, bug_cplt2);
    bug_reproduced = (bug_mism1 > 0) || (bug_mism2 > 0) || !bug_cplt1 || !bug_cplt2;
    $display("[%0t] BUG DUT (skip=0): single mism=%0d complete=%0b ; burst4 mism=%0d complete=%0b",
             $time, bug_mism1, bug_cplt1, bug_mism2, bug_cplt2);
    if (!bug_reproduced) begin
      $display("[%0t] ERROR: preamble bug did NOT reproduce on the unfixed PHY — test is not proving the fix",
               $time);
      errors = errors + 1;
    end else
      $display("[%0t] preamble bug REPRODUCED on the unfixed PHY (phantom-word read corruption)", $time);

    // ================= PHASE 2: FIXED PHY (RD_PREAMBLE_SKIP=1) — expect a clean, complete read =====
    sel = 1'b1;
    fix_mism1 = 0; fix_cplt1 = 1'b0; fix_mism2 = 0; fix_cplt2 = 1'b0;
    wr_rd(32'h0000_0000, 1,  fix_mism1, fix_cplt1);
    wr_rd(32'h0000_0020, 4,  fix_mism2, fix_cplt2);
    // extra, longer bursts to exercise repeated preamble skips across bursts, incl. a FULL 16-word
    // burst (= BURST_WORDS on the board) and a 20-word burst — the on-silicon hang appeared for
    // LEN >= 16, so these are the load-bearing regression cases.
    begin
      int unsigned m3, m4, m5; logic c3, c4, c5;
      wr_rd(32'h0000_0100, 8,  m3, c3);
      wr_rd(32'h0000_0200, 16, m4, c4);
      wr_rd(32'h0000_0300, 20, m5, c5);
      fix_clean = (fix_mism1 == 0) && (fix_mism2 == 0) && (m3 == 0) && (m4 == 0) && (m5 == 0) &&
                  fix_cplt1 && fix_cplt2 && c3 && c4 && c5;
      $display("[%0t] FIX DUT (skip=1): single m=%0d/c=%0b b4 m=%0d/c=%0b b8 m=%0d/c=%0b b16 m=%0d/c=%0b b20 m=%0d/c=%0b",
               $time, fix_mism1, fix_cplt1, fix_mism2, fix_cplt2, m3, c3, m4, c4, m5, c5);
    end
    if (!fix_clean) begin
      $display("[%0t] ERROR: FIX DUT (skip=%0d) did not read back cleanly — preamble skip is wrong",
               $time, FIX_SKIP);
      errors = errors + 1;
    end else
      $display("[%0t] FIX DUT clean: preamble skip=%0d recovers exact read data", $time, FIX_SKIP);

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_preamble_generic: bug_reproduced=%0b fix_clean=%0b errors=%0d",
             $time, bug_reproduced, fix_clean, errors);
    if (errors == 0 && bug_reproduced && fix_clean) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_preamble_generic: %0d errors (bug_reproduced=%0b fix_clean=%0b)",
             errors, bug_reproduced, fix_clean);
    end
  end

  // Global watchdog.
  initial begin
    #6_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_preamble_generic: global timeout");
  end

endmodule
