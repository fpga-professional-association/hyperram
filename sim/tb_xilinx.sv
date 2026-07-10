// tb_xilinx — self-checking Verilator testbench for hyperram_avalon with PHY_VARIANT="XILINX".
//
// Exercises the REAL 7-series datapath (rtl/phy/hyperbus_phy_xilinx.sv: ODDR/IDDR/IDELAYE2/IDELAYCTRL/
// BUFIO/BUFR/OBUF/OBUFDS) through the Verilator-only primitive shim (sim/model/xilinx_prims_sim.sv).
// Same rigor as tb_sdr (POR init + CR0, single/burst write-then-read-back, a wrap-group-crossing burst,
// CR0 write+readback, ID0 read), but with the true 90°-phase clocking the Xilinx variant expects.
//
// TWO scenarios, each logged DISTINCTLY (not folded into one aggregate), instantiated as two parallel
// datapaths whose shared Avalon stimulus is steered by `sel` (the tb_preamble dual-DUT pattern):
//
//   * DUT_IDEAL    — golden model with NO read preamble, NO over-stream; PHY RD_PREAMBLE_SKIP=0.
//                    Proves the datapath against a spec-ideal device (the tb_sdr coverage).
//   * DUT_NONIDEAL — golden model WITH a read-strobe preamble (RD_PREAMBLE_CLOCKS>0) AND a CK-stop
//                    over-stream tail (RD_OVERSTREAM_WORDS>0), matched with PHY RD_PREAMBLE_SKIP>0.
//                    Proves step-6's async-clear flush + step-7's disarm reset actually work: reads
//                    must be aligned (0 mismatches) AND no over-streamed word may leak into the next
//                    burst (a dedicated back-to-back read-A / read-B leakage probe). Without this a
//                    no-op flush would still pass the ideal scenario.
//
// The TB PASSES iff BOTH scenarios are clean (and the leakage probe finds no leak). Every read is
// bounded/guarded, so a flush/CDC regression surfaces as a short or corrupted read, not a hang.
//
// CLOCKING (the Xilinx variant, unlike SDR, uses a true 90° phase):
//   clk    = 100 MHz  — HyperBus word clock (controller + TX/RX-FIFO clk side). hb_ck runs at this rate.
//   clk90  = 100 MHz, +90° (tb_avalon's #2.5 offset) — centres hb_ck on the DQ eye.
//   clk_ref= 200 MHz  — IDELAYCTRL reference for the RWDS IDELAYE2 tap.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_xilinx;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2

  // Latency 6, fixed — matches tb_sdr so the model's post-CR0 latency lines up with the controller.
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam int unsigned REG_MSB     = ADDR_WIDTH - 1;       // Avalon addr MSB selects register space

  // Non-ideal device knobs (as the real Winbond W957D8NB captured on the AXC3000).
  localparam int unsigned PREAMBLE_CLOCKS = 1;   // read-strobe preamble (CK cycles)
  localparam int unsigned FIX_SKIP        = 1;   // matching PHY skip
  localparam int unsigned OS_WORDS        = 9;   // CK-stop over-stream tail (extra source-synced words)

  // --------------------------------------------------------------------
  // Clocking / reset — Xilinx arrangement: 100 MHz word clk, +90° clk90, 200 MHz clk_ref.
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin       clk     = 1'b0; forever #5.0  clk     = ~clk;     end   // 100 MHz  (CK word clock)
  initial begin #2.5; clk90   = 1'b0; forever #5.0  clk90   = ~clk90;   end   // +90 deg
  initial begin       clk_ref = 1'b0; forever #2.5  clk_ref = ~clk_ref; end   // 200 MHz (IDELAYCTRL ref)

  // --------------------------------------------------------------------
  // Shared Avalon-MM stimulus; the active datapath is picked by `sel` so only one runs a transaction
  // at a time (the other stays idle, its model quiescent).
  // --------------------------------------------------------------------
  logic                    sel;               // 0 = DUT_IDEAL, 1 = DUT_NONIDEAL
  logic [ADDR_WIDTH-1:0]   s_address;
  logic                    s_read, s_write;
  logic [DATA_WIDTH-1:0]   s_writedata;
  logic [STRB_WIDTH-1:0]   s_byteenable;
  logic [LEN_WIDTH-1:0]    s_burstcount;

  // Muxed observation back to the tasks.
  logic [DATA_WIDTH-1:0]   m_readdata;
  logic                    m_readdatavalid;
  logic                    m_waitrequest;

  localparam realtime RTT = 3.0;   // ns round-trip DQ/RWDS flight delay (as tb_sdr) — stresses capture

  // ==================================================================================================
  //  DUT_IDEAL (RD_PREAMBLE_SKIP = 0, spec-ideal model)
  // ==================================================================================================
  logic                 a_hb_ck, a_hb_ck_n, a_hb_cs_n, a_hb_rst_n;
  logic [DQ_WIDTH-1:0]  a_phy_dq_o;   logic a_phy_dq_oe;
  logic                 a_phy_rwds_o; logic a_phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  a_mdl_dq_o;   logic a_mdl_dq_oe;
  logic                 a_mdl_rwds_o; logic a_mdl_rwds_oe;
  logic [DATA_WIDTH-1:0] a_readdata;  logic a_readdatavalid, a_waitrequest, a_init_done;

  wire [DQ_WIDTH-1:0] a_dq_line   = a_mdl_dq_oe   ? a_mdl_dq_o   : (a_phy_dq_oe   ? a_phy_dq_o   : '0);
  wire                a_rwds_line = a_mdl_rwds_oe ? a_mdl_rwds_o : (a_phy_rwds_oe ? a_phy_rwds_o : 1'b0);
  wire [DQ_WIDTH-1:0] a_dq_dly;   assign #RTT a_dq_dly   = a_dq_line;
  wire                a_rwds_dly; assign #RTT a_rwds_dly = a_rwds_line;

  hyperram_avalon #(
    .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1), .MAX_BURST_WORDS (0), .PROGRAM_CR (1'b1),
    .POR_DELAY_CYCLES (0), .INIT_CR0 (TB_INIT_CR0), .PHY_VARIANT ("XILINX"), .DIFF_CK (1'b1),
    .RD_PREAMBLE_SKIP (0)
  ) dut_ideal (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // cal tied to constants reproducing RD_PREAMBLE_SKIP=0 (tie-off in the XILINX variant)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd0), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .avs_address (s_address), .avs_read (s_read & ~sel), .avs_write (s_write & ~sel),
    .avs_writedata (s_writedata), .avs_byteenable (s_byteenable), .avs_burstcount (s_burstcount),
    .avs_readdata (a_readdata), .avs_readdatavalid (a_readdatavalid), .avs_waitrequest (a_waitrequest),
    .hb_ck (a_hb_ck), .hb_ck_n (a_hb_ck_n), .hb_cs_n (a_hb_cs_n), .hb_rst_n (a_hb_rst_n),
    .hb_dq_o (a_phy_dq_o), .hb_dq_oe (a_phy_dq_oe), .hb_dq_i (a_dq_dly),
    .hb_rwds_o (a_phy_rwds_o), .hb_rwds_oe (a_phy_rwds_oe), .hb_rwds_i (a_rwds_dly),
    .init_done (a_init_done), .err_underrun (), .dbg_bus ()
  );
  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0)
  ) model_ideal (
    .hb_ck (a_hb_ck), .hb_ck_n (a_hb_ck_n), .hb_cs_n (a_hb_cs_n), .hb_rst_n (a_hb_rst_n),
    .hb_dq_i (a_dq_line), .hb_dq_ie (a_phy_dq_oe), .hb_dq_o (a_mdl_dq_o), .hb_dq_oe (a_mdl_dq_oe),
    .hb_rwds_i (a_rwds_line), .hb_rwds_ie (a_phy_rwds_oe),
    .hb_rwds_o (a_mdl_rwds_o), .hb_rwds_oe (a_mdl_rwds_oe)
  );

  // ==================================================================================================
  //  DUT_NONIDEAL (RD_PREAMBLE_SKIP = FIX_SKIP; model emits read preamble + CK-stop over-stream)
  // ==================================================================================================
  logic                 n_hb_ck, n_hb_ck_n, n_hb_cs_n, n_hb_rst_n;
  logic [DQ_WIDTH-1:0]  n_phy_dq_o;   logic n_phy_dq_oe;
  logic                 n_phy_rwds_o; logic n_phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  n_mdl_dq_o;   logic n_mdl_dq_oe;
  logic                 n_mdl_rwds_o; logic n_mdl_rwds_oe;
  logic [DATA_WIDTH-1:0] n_readdata;  logic n_readdatavalid, n_waitrequest, n_init_done;

  wire [DQ_WIDTH-1:0] n_dq_line   = n_mdl_dq_oe   ? n_mdl_dq_o   : (n_phy_dq_oe   ? n_phy_dq_o   : '0);
  wire                n_rwds_line = n_mdl_rwds_oe ? n_mdl_rwds_o : (n_phy_rwds_oe ? n_phy_rwds_o : 1'b0);
  wire [DQ_WIDTH-1:0] n_dq_dly;   assign #RTT n_dq_dly   = n_dq_line;
  wire                n_rwds_dly; assign #RTT n_rwds_dly = n_rwds_line;

  hyperram_avalon #(
    .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1), .MAX_BURST_WORDS (0), .PROGRAM_CR (1'b1),
    .POR_DELAY_CYCLES (0), .INIT_CR0 (TB_INIT_CR0), .PHY_VARIANT ("XILINX"), .DIFF_CK (1'b1),
    .RD_PREAMBLE_SKIP (FIX_SKIP)
  ) dut_nonideal (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // cal tied to constants reproducing RD_PREAMBLE_SKIP=FIX_SKIP=1 (tie-off in the XILINX variant)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd1), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .avs_address (s_address), .avs_read (s_read & sel), .avs_write (s_write & sel),
    .avs_writedata (s_writedata), .avs_byteenable (s_byteenable), .avs_burstcount (s_burstcount),
    .avs_readdata (n_readdata), .avs_readdatavalid (n_readdatavalid), .avs_waitrequest (n_waitrequest),
    .hb_ck (n_hb_ck), .hb_ck_n (n_hb_ck_n), .hb_cs_n (n_hb_cs_n), .hb_rst_n (n_hb_rst_n),
    .hb_dq_o (n_phy_dq_o), .hb_dq_oe (n_phy_dq_oe), .hb_dq_i (n_dq_dly),
    .hb_rwds_o (n_phy_rwds_o), .hb_rwds_oe (n_phy_rwds_oe), .hb_rwds_i (n_rwds_dly),
    .init_done (n_init_done), .err_underrun (), .dbg_bus ()
  );
  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0),
    .RD_PREAMBLE_CLOCKS (PREAMBLE_CLOCKS),      // device emits the read-strobe preamble
    .RD_OVERSTREAM_WORDS (OS_WORDS)             // device over-streams past the master's CK-stop
  ) model_nonideal (
    .hb_ck (n_hb_ck), .hb_ck_n (n_hb_ck_n), .hb_cs_n (n_hb_cs_n), .hb_rst_n (n_hb_rst_n),
    .hb_dq_i (n_dq_line), .hb_dq_ie (n_phy_dq_oe), .hb_dq_o (n_mdl_dq_o), .hb_dq_oe (n_mdl_dq_oe),
    .hb_rwds_i (n_rwds_line), .hb_rwds_ie (n_phy_rwds_oe),
    .hb_rwds_o (n_mdl_rwds_o), .hb_rwds_oe (n_mdl_rwds_oe)
  );

  // Muxed observation.
  always_comb begin
    m_readdata      = sel ? n_readdata      : a_readdata;
    m_readdatavalid = sel ? n_readdatavalid : a_readdatavalid;
    m_waitrequest   = sel ? n_waitrequest   : a_waitrequest;
  end

  // --------------------------------------------------------------------
  // Scoreboard capture (of the selected DUT).
  // --------------------------------------------------------------------
  localparam int unsigned CAP_MAX = 256;
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
                          input logic reg_space, input logic [DATA_WIDTH-1:0] data [$]);
    int unsigned idx, g;
    logic [ADDR_WIDTH-1:0] a_full;
    a_full = addr; a_full[REG_MSB] = reg_space;
    idx = 0;
    @(negedge clk);
    s_write = 1'b1; s_read = 1'b0; s_address = a_full;
    s_burstcount = LEN_WIDTH'(n); s_byteenable = '1; s_writedata = data[0];
    g = 0;
    forever begin
      @(posedge clk);
      g = g + 1;
      if (g > 5000) begin
        $display("[%0t] HANG do_write @0x%08x reg=%0b idx=%0d/%0d", $time, addr, reg_space, idx, n);
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

  // Read n words; capture into cap[]. Returns the number actually returned (bounded — hang => got<n).
  task automatic do_read(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                         input logic reg_space, output int unsigned got);
    logic [ADDR_WIDTH-1:0] a_full;
    int unsigned guard;
    a_full = addr; a_full[REG_MSB] = reg_space;
    cap_n = 0; capturing = 1'b1;
    @(negedge clk);
    s_read = 1'b1; s_write = 1'b0; s_address = a_full; s_burstcount = LEN_WIDTH'(n);
    guard = 0;
    forever begin
      @(posedge clk);
      guard = guard + 1;
      if (guard > 3000) begin
        $display("[%0t] HANG do_read accept @0x%08x", $time, addr);
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
    do_write(addr, n, 1'b0, wdata);
    do_read (addr, n, 1'b0, got);
    complete = (got >= n);
    mism = 0;
    for (i = 0; i < n; i++) begin
      if (i >= got) begin mism = mism + 1; continue; end
      if (cap[i] !== genword(addr + i)) mism = mism + 1;
    end
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  int unsigned errors;
  int unsigned guard;
  logic        ideal_clean, nonideal_clean, no_leak;
  logic [DATA_WIDTH-1:0] one [$];
  logic [DATA_WIDTH-1:0] cr0_rb, id0_rb;

  // scratch for the multi-case scenarios
  int unsigned m0, m1, m2, m3, m4;
  logic        c0, c1, c2, c3, c4;

  initial begin
    errors = 0; sel = 1'b0;
    s_address = '0; s_read = 1'b0; s_write = 1'b0;
    s_writedata = '0; s_byteenable = '1; s_burstcount = '0;
    capturing = 1'b0; cap_n = 0;
    ideal_clean = 1'b0; nonideal_clean = 1'b0; no_leak = 1'b0;
    rst = 1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk); rst = 1'b0;

    guard = 0;
    while (!(a_init_done && n_init_done) && guard < 200000) begin @(posedge clk); guard = guard + 1; end
    if (!(a_init_done && n_init_done)) begin
      $display("[%0t] FATAL: init_done never asserted (ideal=%0b nonideal=%0b)",
               $time, a_init_done, n_init_done);
      errors = errors + 1;
    end else $display("[%0t] both DUTs init_done", $time);
    repeat (4) @(posedge clk);

    // ================= SCENARIO 1: IDEAL device (skip=0, no preamble, no over-stream) =============
    sel = 1'b0;
    begin
      int unsigned ms, ma; logic cs, ca;               // single + several bursts, aggregated
      int unsigned msingle2, mburst8; logic csingle2, cburst8;
      wr_rd(32'h0000_0000, 1,  ms,       cs);
      wr_rd(32'h0000_0041, 1,  msingle2, csingle2);
      wr_rd(32'h0000_0010, 4,  m1,       c1);
      wr_rd(32'h0000_0100, 8,  mburst8,  cburst8);
      wr_rd(32'h0000_2000, 16, m2,       c2);
      wr_rd(32'h0000_0038, 20, ma,       ca);          // wrap-group-crossing linear burst
      // CR0 register write + read-back
      one = {}; one.push_back(TB_INIT_CR0);
      do_write(HB_REG_CR0[ADDR_WIDTH-1:0], 1, 1'b1, one);
      do_read (HB_REG_CR0[ADDR_WIDTH-1:0], 1, 1'b1, m3);
      cr0_rb = cap[0];
      // ID0 register read
      do_read (HB_REG_ID0[ADDR_WIDTH-1:0], 1, 1'b1, m4);
      id0_rb = cap[0];
      ideal_clean = (ms==0)&&(msingle2==0)&&(m1==0)&&(mburst8==0)&&(m2==0)&&(ma==0) &&
                    cs&&csingle2&&c1&&cburst8&&c2&&ca &&
                    (cr0_rb === TB_INIT_CR0) && (id0_rb === HB_ID0_RESET);
      $display("[%0t] IDEAL   (skip=0, preamble=0, over-stream=0): single m=%0d/c=%0b burst4 m=%0d/c=%0b burst8 m=%0d/c=%0b burst16 m=%0d/c=%0b cross m=%0d/c=%0b CR0=0x%04x ID0=0x%04x -> clean=%0b",
               $time, ms, cs, m1, c1, mburst8, cburst8, m2, c2, ma, ca, cr0_rb, id0_rb, ideal_clean);
    end
    if (!ideal_clean) begin
      $display("[%0t] ERROR: IDEAL scenario did not read back cleanly", $time);
      errors = errors + 1;
    end

    // ============ SCENARIO 2: NON-IDEAL device (skip>0, read preamble + CK-stop over-stream) ======
    // Read alignment across single + several bursts, plus a back-to-back read-A / read-B LEAKAGE probe
    // (no intervening write): DUT_NONIDEAL's over-stream tail after read A must be flushed so read B at
    // a DIFFERENT address returns its own data — a no-op flush would corrupt read B.
    sel = 1'b1;
    begin
      wr_rd(32'h0000_0000, 1,  m0, c0);
      wr_rd(32'h0000_0100, 8,  m1, c1);
      wr_rd(32'h0000_0200, 16, m2, c2);                 // full board burst — the on-silicon hang length
      wr_rd(32'h0000_0300, 20, m3, c3);
      nonideal_clean = (m0==0)&&(m1==0)&&(m2==0)&&(m3==0) && c0&&c1&&c2&&c3;
      $display("[%0t] NONIDEAL(skip=%0d, preamble=%0d, over-stream=%0d): single m=%0d/c=%0b burst8 m=%0d/c=%0b burst16 m=%0d/c=%0b burst20 m=%0d/c=%0b -> clean=%0b",
               $time, FIX_SKIP, PREAMBLE_CLOCKS, OS_WORDS, m0, c0, m1, c1, m2, c2, m3, c3, nonideal_clean);

      // ---- leakage probe: pre-write two DISJOINT regions, then read them back-to-back ----
      begin
        logic [DATA_WIDTH-1:0] wa [$], wb [$];
        int unsigned gota, gotb, mia, mib, i;
        wa = {}; wb = {};
        for (i = 0; i < 16; i++) begin
          wa.push_back(genword(32'h0000_0800 + i));
          wb.push_back(genword(32'h0000_0C00 + i));
        end
        do_write(32'h0000_0800, 16, 1'b0, wa);
        do_write(32'h0000_0C00, 16, 1'b0, wb);
        do_read (32'h0000_0800, 16, 1'b0, gota);
        // capture region A, then read region B WITHOUT any write between (tightest leakage test)
        mia = 0;
        for (i = 0; i < 16; i++)
          if (i >= gota || cap[i] !== genword(32'h0000_0800 + i)) mia = mia + 1;
        do_read (32'h0000_0C00, 16, 1'b0, gotb);
        mib = 0;
        for (i = 0; i < 16; i++)
          if (i >= gotb || cap[i] !== genword(32'h0000_0C00 + i)) mib = mib + 1;
        no_leak = (mia == 0) && (mib == 0) && (gota == 16) && (gotb == 16);
        $display("[%0t] NONIDEAL leakage probe: readA got=%0d mism=%0d ; readB got=%0d mism=%0d -> no_leak=%0b",
                 $time, gota, mia, gotb, mib, no_leak);
      end
    end
    if (!nonideal_clean) begin
      $display("[%0t] ERROR: NON-IDEAL scenario mis-read (preamble skip / capture wrong)", $time);
      errors = errors + 1;
    end
    if (!no_leak) begin
      $display("[%0t] ERROR: NON-IDEAL over-stream LEAKED into the next burst (flush/CDC broken)", $time);
      errors = errors + 1;
    end

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_xilinx done: ideal_clean=%0b nonideal_clean=%0b no_leak=%0b errors=%0d",
             $time, ideal_clean, nonideal_clean, no_leak, errors);
    if (errors == 0 && ideal_clean && nonideal_clean && no_leak) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_xilinx: errors=%0d ideal_clean=%0b nonideal_clean=%0b no_leak=%0b",
             errors, ideal_clean, nonideal_clean, no_leak);
    end
  end

  // Global watchdog.
  initial begin
    #8_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_xilinx: global timeout");
  end

endmodule
