// tb_por_timing — self-checking Verilator TB for A4: POR / reset AC-timing as real time guarantees.
//
// The controller's RST# low pulse and the post-reset gap before the first CS# used to be a hardcoded
// RESET_CYCLES=8 / raw POR_DELAY_CYCLES with NO relationship to the spec's ns/µs timings, so at a low
// CK a device could come up before tRP/tRPH/tRH/tVCS were met. hyperbus_ctrl now DERIVES those cycle
// counts from the spec timings once CLK_FREQ_MHZ != 0 (cycles = ceil(t / tCK), SPEC_DIGEST §9 Table 8.3).
//
// This TB drives the controller directly (so it can watch phy_rst_n / phy_cs_n at the controller pins,
// with no PHY pipeline offset) at a KNOWN CK frequency and asserts:
//   * RST# is held Low >= ceil(tRP / tCK)  AND exactly the derived RESET_CYCLES,
//   * the gap RST#-release -> first CS# Low >= ceil(tRH / tCK), and RST#-fall->first-CS# >= tRPH,
//   * that gap also covers tVCS (VCC-valid -> first access),
//   * init still completes (init_done) and a memory write/read is byte-exact (timing didn't break fn).
// The ns->cycle mapping is RE-COMPUTED here from the same spec inputs and checked end-to-end.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_por_timing;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2

  // ---- device / controller timing config under test -------------------------------------------
  localparam int unsigned CLK_FREQ_MHZ = 100;   // CK = 100 MHz => tCK = 10 ns
  localparam int unsigned T_RP_NS      = 200;   // spec tRP  >= 200 ns
  localparam int unsigned T_RPH_NS     = 400;   // spec tRPH >= 400 ns
  localparam int unsigned T_RH_NS      = 200;   // spec tRH  >= 200 ns
  localparam int unsigned T_VCS_US     = 2;     // shortened tVCS for sim speed (200 cycles @100 MHz)

  // ---- expected cycle counts, re-computed here from the same spec inputs ----------------------
  function automatic int unsigned ceil_div(input int unsigned a, input int unsigned b);
    return (a + b - 1) / b;
  endfunction
  localparam int unsigned EXP_RP_CYC   = ceil_div(T_RP_NS  * CLK_FREQ_MHZ, 1000);   // = 20
  localparam int unsigned EXP_RPH_CYC  = ceil_div(T_RPH_NS * CLK_FREQ_MHZ, 1000);   // = 40
  localparam int unsigned EXP_RH_CYC   = ceil_div(T_RH_NS  * CLK_FREQ_MHZ, 1000);   // = 20
  localparam int unsigned EXP_VCS_CYC  = T_VCS_US * CLK_FREQ_MHZ;                    // = 200
  localparam int unsigned EXP_RESET    = EXP_RP_CYC;                                 // RESET_CYCLES
  localparam int unsigned EXP_RH_GAP   = (EXP_RH_CYC > (EXP_RPH_CYC - EXP_RP_CYC)) ? EXP_RH_CYC
                                                                                   : (EXP_RPH_CYC - EXP_RP_CYC);
  localparam int unsigned EXP_POR      = (EXP_RH_GAP > EXP_VCS_CYC) ? EXP_RH_GAP : EXP_VCS_CYC; // POR_CYCLES

  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F (lat 6, fixed)

  // --------------------------------------------------------------------
  // Clocking: clk = 100 MHz word clock, clk90 = 90-deg, matches the generic-PHY TBs.
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;   end   // 100 MHz (tCK = 10 ns)
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90 = ~clk90; end
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  // --------------------------------------------------------------------
  // Controller native channel.
  // --------------------------------------------------------------------
  logic                    cmd_valid, cmd_ready, cmd_read, cmd_reg, cmd_wrap;
  logic [ADDR_WIDTH-1:0]   cmd_addr;
  logic [LEN_WIDTH-1:0]    cmd_len;
  logic                    wr_valid, wr_ready;
  logic [DATA_WIDTH-1:0]   wr_data;
  logic [STRB_WIDTH-1:0]   wr_strb;
  logic                    wr_last;
  logic                    rd_valid, rd_ready;
  logic [DATA_WIDTH-1:0]   rd_data;
  logic                    rd_last;
  logic                    busy, init_done, err_underrun, err_timeout;

  // ctrl <-> PHY parallel interface.
  logic                    phy_cs_n, phy_rst_n, phy_ck_en;
  logic [DATA_WIDTH-1:0]   phy_dq_o;   logic phy_dq_oe;
  logic [1:0]              phy_rwds_o; logic phy_rwds_oe, phy_rd_arm;
  logic [DATA_WIDTH-1:0]   phy_dq_i;   logic phy_dq_i_valid, phy_rwds_i;

  // PHY device pins <-> model (split bus, RTT flight delay as the other generic-PHY TBs).
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  phy_pin_dq_o;  logic phy_pin_dq_oe;
  logic                 phy_pin_rwds_o; logic phy_pin_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;      logic mdl_dq_oe;
  logic                 mdl_rwds_o;    logic mdl_rwds_oe;
  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_pin_dq_oe   ? phy_pin_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_pin_rwds_oe ? phy_pin_rwds_o : 1'b0);
  localparam realtime RTT = 3.0;
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  hyperbus_ctrl #(
    .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1), .MAX_BURST_WORDS (0),
    .PROGRAM_CR (1'b1), .POR_DELAY_CYCLES (0), .INIT_CR0 (TB_INIT_CR0),
    .CLK_FREQ_MHZ (CLK_FREQ_MHZ), .T_RP_NS (T_RP_NS), .T_RPH_NS (T_RPH_NS),
    .T_RH_NS (T_RH_NS), .T_VCS_US (T_VCS_US)
  ) u_ctrl (
    .clk (clk), .rst (rst),
    .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_read (cmd_read), .cmd_reg (cmd_reg),
    .cmd_wrap (cmd_wrap), .cmd_addr (cmd_addr), .cmd_len (cmd_len),
    .wr_valid (wr_valid), .wr_ready (wr_ready), .wr_data (wr_data), .wr_strb (wr_strb), .wr_last (wr_last),
    .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_data (rd_data), .rd_last (rd_last),
    .busy (busy), .init_done (init_done), .err_underrun (err_underrun), .err_timeout (err_timeout),
    .phy_cs_n (phy_cs_n), .phy_rst_n (phy_rst_n), .phy_ck_en (phy_ck_en),
    .phy_dq_o (phy_dq_o), .phy_dq_oe (phy_dq_oe), .phy_rwds_o (phy_rwds_o), .phy_rwds_oe (phy_rwds_oe),
    .phy_rd_arm (phy_rd_arm), .phy_dq_i (phy_dq_i), .phy_dq_i_valid (phy_dq_i_valid), .phy_rwds_i (phy_rwds_i),
    .dbg_state (), .dbg_rd_wptr (), .dbg_rd_rptr (),
    // issue #13: new hyperbus_ctrl debug bundle tied to per-instance legacy (A1; no wrap_en on ctrl).
    .dbg_wr_lat_trim (4'd0), .dbg_lat_clocks (4'd6), .dbg_cr0_reprog (1'b0),
    .dbg_prewin_drive (1'b0), .dbg_prewin_n (3'd0), .dbg_prewin_marker (1'b0), .dbg_postwin_hold (1'b0), .dbg_prewin_contig (1'b0), .dbg_end_cwrite (1'b0), .dbg_spray_defuse (1'b0)
  );

  hyperbus_phy_generic #(
    .DQ_WIDTH (DQ_WIDTH), .DATA_WIDTH (DATA_WIDTH), .DIFF_CK (1'b1)
  ) u_phy (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // runtime read-eye calibration tied to POR-equivalent constants (reproduces pre-cal behaviour)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd0), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .phy_cs_n (phy_cs_n), .phy_rst_n (phy_rst_n), .phy_ck_en (phy_ck_en),
    .phy_dq_o (phy_dq_o), .phy_dq_oe (phy_dq_oe), .phy_rwds_o (phy_rwds_o), .phy_rwds_oe (phy_rwds_oe),
    .phy_rd_arm (phy_rd_arm), .phy_dq_i (phy_dq_i), .phy_dq_i_valid (phy_dq_i_valid), .phy_rwds_i (phy_rwds_i),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (phy_pin_dq_o), .hb_dq_oe (phy_pin_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (phy_pin_rwds_o), .hb_rwds_oe (phy_pin_rwds_oe), .hb_rwds_i (rwds_line_dly)
  );

  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0)
  ) u_model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i (dq_line), .hb_dq_ie (phy_pin_dq_oe), .hb_dq_o (mdl_dq_o), .hb_dq_oe (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (phy_pin_rwds_oe), .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );

  // --------------------------------------------------------------------
  // Measurement: count RST# Low cycles after rst release, then the gap to first CS# Low.
  // --------------------------------------------------------------------
  int unsigned reset_low_cyc;   // clk cycles phy_rst_n stays Low after rst is released
  int unsigned gap_cyc;         // clk cycles from phy_rst_n rising to first phy_cs_n falling
  logic        rst_released;

  int unsigned errors = 0, checks = 0;
  function automatic logic [DATA_WIDTH-1:0] genword(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction

  task automatic chk(input logic cond, input string msg);
    checks = checks + 1;
    if (!cond) begin $display("[%0t] ERROR: %s", $time, msg); errors = errors + 1; end
  endtask

  // Native single-word memory write then read-back (proves timing config didn't break function).
  task automatic mem_write1(input logic [ADDR_WIDTH-1:0] a, input logic [DATA_WIDTH-1:0] d);
    int unsigned g;
    @(negedge clk); cmd_valid=1'b1; cmd_read=1'b0; cmd_reg=1'b0; cmd_wrap=1'b0; cmd_addr=a; cmd_len=LEN_WIDTH'(1);
    g=0; forever begin @(posedge clk); g=g+1; if (cmd_ready) break; if (g>2000) begin errors=errors+1; break; end end
    @(negedge clk); cmd_valid=1'b0;
    wr_valid=1'b1; wr_data=d; wr_strb='1; wr_last=1'b1;
    g=0; forever begin @(posedge clk); g=g+1; if (wr_ready) break; if (g>2000) begin errors=errors+1; break; end end
    @(negedge clk); wr_valid=1'b0; wr_last=1'b0;
    repeat (30) @(posedge clk);
  endtask

  task automatic mem_read1(input logic [ADDR_WIDTH-1:0] a, output logic [DATA_WIDTH-1:0] d);
    int unsigned g;
    @(negedge clk); cmd_valid=1'b1; cmd_read=1'b1; cmd_reg=1'b0; cmd_wrap=1'b0; cmd_addr=a; cmd_len=LEN_WIDTH'(1);
    g=0; forever begin @(posedge clk); g=g+1; if (cmd_ready) break; if (g>2000) begin errors=errors+1; break; end end
    @(negedge clk); cmd_valid=1'b0; rd_ready=1'b1;
    d='0; g=0; forever begin @(posedge clk); g=g+1; if (rd_valid) begin d=rd_data; break; end
      if (g>4000) begin $display("[%0t] read timeout", $time); errors=errors+1; break; end end
    @(negedge clk); rd_ready=1'b0;
  endtask

  logic [DATA_WIDTH-1:0] rback;
  initial begin
    cmd_valid=0; cmd_read=0; cmd_reg=0; cmd_wrap=0; cmd_addr=0; cmd_len=0;
    wr_valid=0; wr_data=0; wr_strb='1; wr_last=0; rd_ready=0;
    reset_low_cyc=0; gap_cyc=0; rst_released=1'b0;
    rst=1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk); rst=1'b0; rst_released=1'b1;

    // --- measure RST# Low pulse (cycles phy_rst_n==0 after rst release) ---
    reset_low_cyc = 0;
    while (phy_rst_n == 1'b0) begin @(posedge clk); reset_low_cyc = reset_low_cyc + 1; end
    // phy_rst_n just went High. Now count until the first CS# Low.
    gap_cyc = 0;
    while (phy_cs_n == 1'b1) begin @(posedge clk); gap_cyc = gap_cyc + 1; end

    $display("[%0t] measured: RST# low = %0d cyc (exp RESET_CYCLES=%0d), gap RST->CS# = %0d cyc (exp POR_CYCLES=%0d)",
             $time, reset_low_cyc, EXP_RESET, gap_cyc, EXP_POR);
    $display("[%0t] spec minima @%0d MHz: tRP=%0d cyc tRPH=%0d cyc tRH=%0d cyc tVCS=%0d cyc",
             $time, CLK_FREQ_MHZ, EXP_RP_CYC, EXP_RPH_CYC, EXP_RH_CYC, EXP_VCS_CYC);

    // --- A4 assertions ---
    // The spec timings are MINIMUMS (tRP/tRPH/tRH) or a device-ready MAX the host must wait out (tVCS);
    // the hard guarantees are the >= checks below. The tight upper bounds additionally prove the counts
    // are DERIVED from the ns/µs inputs (a couple of cycles of FSM/sampling convention), not the legacy
    // fixed constant. (Legacy CLK_FREQ_MHZ=0 => RESET_CYCLES=8 is covered by every other TB.)
    chk(reset_low_cyc >= EXP_RP_CYC,               "RST# low pulse shorter than tRP");
    chk(reset_low_cyc <= EXP_RESET + 1,            "RST# low pulse not tight to derived RESET_CYCLES");
    chk(reset_low_cyc > 8,                         "derivation inactive: RST# pulse is the legacy 8 cycles");
    chk(gap_cyc >= EXP_RH_CYC,                     "gap RST#-release->CS# shorter than tRH");
    chk((reset_low_cyc + gap_cyc) >= EXP_RPH_CYC,  "RST#-fall->first-CS# shorter than tRPH");
    chk(gap_cyc >= EXP_VCS_CYC,                    "gap does not cover tVCS");
    chk(gap_cyc >= EXP_POR,                        "gap shorter than derived POR_CYCLES");
    chk(gap_cyc <= EXP_POR + 4,                    "gap not tight to derived POR_CYCLES");

    // --- function still works ---
    begin int unsigned g; g=0; while (!init_done && g<100000) begin @(posedge clk); g=g+1; end end
    chk(init_done, "init_done never asserted");
    repeat (4) @(posedge clk);
    mem_write1(32'h0000_0040, genword(32'h40));
    mem_read1 (32'h0000_0040, rback);
    chk(rback === genword(32'h40), "memory read-back mismatch after POR-timed init");

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_por_timing done: %0d checks, %0d errors", $time, checks, errors);
    if (errors==0) begin $display("TB_RESULT: PASS"); $finish; end
    else begin $display("TB_RESULT: FAIL"); $fatal(1, "tb_por_timing: %0d errors", errors); end
  end

  initial begin #200_000; $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_por_timing: global timeout"); end
endmodule
