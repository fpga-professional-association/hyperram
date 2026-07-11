// tb_dbg — self-checking Verilator TB for the issue-#13 instrumented build: REG_DBG / REG_EMAP /
// REG_PAT / REG_WRAP driven live against the W957D8NB wound model, no recompile.
//
// Fuses tb_cal (a live CSR knob retunes the datapath and a re-pulsed run proves it, no rebuild) with
// tb_commit (the write-CA "wound" stack + a read-only array-state probe). One board stack — bench
// engine (hyperram_bw_test) -> hyperram_avalon (SDR) -> golden hyperram_model — is reused across
// every check; only the CSRs change between runs. The model reproduces the silicon defects
// (WR_WOUND_WORDS=4 with WR_WOUND_SAMPLE_BUS=1 so the wound content is whatever the controller parks
// on the bus in the pre-data window; WR_END_GARBLE_ROW_WORDS=1024; WR_WOUND_WRAP_IMMUNE=1) and the
// controller's new dbg_* knobs drive/heal them.
//
// The controller chops every write at MAX_BURST_WORDS=64. With BURSTW=LEN the bench issues ONE Avalon
// burst, so the whole transfer rides a single command that the controller segments into 64-word
// pieces; each internal reopen at boundary C wounds [C-4,C) (words the prior segment wrote and the
// read phase checks) -> ERR = 4 per chop. BASE=0x120 is deliberately NOT 64-aligned, so no chop
// boundary and no burst end ever lands on a 1024-word row multiple -> the end-of-row garble
// (WR_END_GARBLE) never fires during the wound checks and the count is pure wound.
//
// Checks (spec §5.2; check 5 recast by amendment A7 for a trim-less model):
//   1  baseline default-legacy: POR REG_DBG==0x60 (A2); a normal run wounds where the model wounds.
//   2  REG_PAT sweep: pat 0/1/3 show the wound (write & read agree on a NON-zero pattern); pat 2
//      (0x0000) aliases the idle-bus wound (0x0000) -> ERR=0, demonstrating the gen(0)=0 pitfall.
//   3  REG_DBG heal (L-D): dbg_prewin_drive parks the replay shadow [C-4..C-1] in the pre-window; the
//      model samples it -> wound content = the correct words -> ERR=0. n>=4 heals fully, n=3 partial.
//   4  REG_DBG marker (L-C): dbg_prewin_marker parks 0xA5xx -> wound reads 0xA5xx (attribution).
//   5  REG_DBG latency reprogram (L-B, A7): dbg_lat_clocks=7 + CR0-reprogram strobe -> controller and
//      model latency stay coherent (ERR == wound baseline, no desync). Trim!=0 is skipped: the model
//      has no WR_LAT_TRIM analog (A2), so a trim mismatch would desync by construction.
//   6  REG_EMAP: every wounded word is captured (count == ERR for <=64; overflow sticky beyond 64).
//   7  REG_WRAP (L-F): a wrapped write REPAIRS a wound zone and does NOT itself wound (vs a linear
//      write at the same base, which does) -> the immunity/repair primitive.
//   8  REG_DBG postwin/ck_stretch (L-E smoke): both are inert on a normal (non-row-end) burst in sim
//      (dbg_ck_stretch_off is a board-only gpio knob, unrouted here; dbg_postwin_hold parks the last
//      word into a CK-less tail the device never clocks) -> ERR unchanged.
//
// NOTE (integration): the dbg_* bundle + wrap_en + CSR_ADDR_WIDTH=5 + DBG_RESET reach the controller
// only after WP-A/WP-B are applied, so this TB elaborates ONLY in the integrated tree (per A9 it is
// written blind to the frozen contract of §0/§1). Every poll loop is BOUNDED so a hang is a FAIL, not
// an infinite loop. Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_dbg;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH       = HB_DQ_WIDTH_DEFAULT;    // 8
  localparam int unsigned DATA_WIDTH     = 2 * DQ_WIDTH;           // 16
  localparam int unsigned ADDR_WIDTH     = HB_ADDR_WIDTH;          // 32
  localparam int unsigned LEN_WIDTH      = HB_LEN_WIDTH_DEFAULT;   // 16
  localparam int unsigned CSR_ADDR_WIDTH = 5;                      // 32 word-regs — REG_EMAP_DATA is word 20
  localparam int unsigned BURST_WORDS    = HB_BURST_WORDS_DEFAULT; // 16
  localparam int unsigned MAXBURST       = 64;                     // controller chop -> a wound at each reopen

  // CR0 image: latency code 0001 (=6), fixed-latency, legacy wrap, 32B group (matches the board/model).
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam logic [31:0] TB_MAGIC    = 32'h4842_5755;            // "HBWU" — instrumented build (A10)
  localparam logic [31:0] TB_DBG_POR  = 32'h0000_0060;            // sim DBG_RESET: lat=6, trim=0 (A2)

  // CSR word-register indices (byte offset >> 2) — must match hyperram_bw_test @ CSR_ADDR_WIDTH=5.
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_CTRL      = CSR_ADDR_WIDTH'(0);   // W: CTRL / R: STATUS
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_LEN       = CSR_ADDR_WIDTH'(1);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_BASE      = CSR_ADDR_WIDTH'(2);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRCNT    = CSR_ADDR_WIDTH'(5);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_MAGIC     = CSR_ADDR_WIDTH'(7);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_BURSTW    = CSR_ADDR_WIDTH'(11);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_RBURSTW   = CSR_ADDR_WIDTH'(12);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_DBG       = CSR_ADDR_WIDTH'(14);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_EMAP_STAT = CSR_ADDR_WIDTH'(15);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_PAT       = CSR_ADDR_WIDTH'(16);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_WRAP      = CSR_ADDR_WIDTH'(17);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_EMAP_IDX  = CSR_ADDR_WIDTH'(18);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_EMAP_ADDR = CSR_ADDR_WIDTH'(19);
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_EMAP_DATA = CSR_ADDR_WIDTH'(20);

  // CTRL write bits.
  localparam logic [31:0] CTRL_START   = 32'h0000_0001;   // bit0 = start
  localparam logic [31:0] CTRL_START_RO= 32'h0000_0003;   // bit0 = start, bit1 = READ-ONLY run

  // REG_DBG bit-field images (bit8 = CR0-reprogram strobe, self-clearing).
  localparam logic [31:0] DBG_HEAL_N4 = 32'h0000_0060 | (32'h1<<9) | (32'd4<<10); // prewin_drive, n=4 = 0x1260
  localparam logic [31:0] DBG_HEAL_N3 = 32'h0000_0060 | (32'h1<<9) | (32'd3<<10); // n=3                = 0x0E60
  localparam logic [31:0] DBG_HEAL_N5 = 32'h0000_0060 | (32'h1<<9) | (32'd5<<10); // n=5                = 0x1460
  localparam logic [31:0] DBG_MARKER  = 32'h0000_0060 | (32'h1<<9) | (32'd4<<10) | (32'h1<<13); // marker = 0x3260
  localparam logic [31:0] DBG_LAT7    = 32'h0000_0070;                 // dbg_lat_clocks=7 (live seed only)
  localparam logic [31:0] DBG_LAT7_RP = 32'h0000_0070 | (32'h1<<8);    // lat=7 + fire CR0-reprogram   = 0x0170
  localparam logic [31:0] DBG_LAT6_RP = 32'h0000_0060 | (32'h1<<8);    // lat=6 + fire CR0-reprogram   = 0x0160
  localparam logic [31:0] DBG_POSTWIN = 32'h0000_0060 | (32'h1<<14);   // dbg_postwin_hold             = 0x4060
  localparam logic [31:0] DBG_CKSTR   = 32'h0000_0060 | (32'h1<<15);   // dbg_ck_stretch_off           = 0x8060

  // Wound-test geometry: single Avalon burst, controller chops at 64. BASE not 64-aligned => no chop
  // boundary and no burst end lands on a 1024 row multiple (no end-of-row garble interference).
  localparam logic [ADDR_WIDTH-1:0] BASE_C  = 32'h0000_0120;   // 288
  localparam logic [31:0]           LEN_HEAL= 32'd256;         // 4 segments -> 3 chops -> 12 wounds
  localparam logic [31:0]           LEN_OVF = 32'd1152;        // 18 segments -> 17 chops -> 68 wounds (>64)

  // ------------------------------------------------------------------
  // Clocking / reset — SDR arrangement (as tb_cal / tb_commit): clk 50 MHz, clk90 100 MHz (2x byte clk).
  // ------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk     = 1'b0; forever #10.0 clk     = ~clk;     end   // 50 MHz
  initial begin clk90   = 1'b0; forever #5.0  clk90   = ~clk90;   end   // 100 MHz
  initial begin clk_ref = 1'b0; forever #5.0  clk_ref = ~clk_ref; end   // (tie-off)

  // ------------------------------------------------------------------
  // Bench CSR slave signals
  // ------------------------------------------------------------------
  logic [CSR_ADDR_WIDTH-1:0] csr_address;
  logic                      csr_read, csr_write;
  logic [31:0]               csr_writedata;
  logic [31:0]               csr_readdata;
  logic                      csr_waitrequest;
  logic                      init_done;

  // ------------------------------------------------------------------
  // HyperBus device pins: master (PHY) side + device (model) side + split-driver resolution.
  // The model samples the UNDELAYED bus (dq_line); the master reads through the RTT-delayed copy.
  // ------------------------------------------------------------------
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

  // ------------------------------------------------------------------
  // DUT: bandwidth-test top (bench engine + hyperram_avalon SDR IP), instrumented build. MAX_BURST_
  // WORDS=64 makes every write chop internally so a wound lands at each reopen. CAL_RESET seeds
  // cal_preamble_skip=1 (REG_CAL[3:1]=1) to match the model's 1-cycle read preamble. DBG_RESET=0x60
  // (A2): the sim controller ran trim=0, so a hard 0x63 would shift every write by 3 words.
  // ------------------------------------------------------------------
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
    .MAX_BURST_WORDS  (MAXBURST),
    .PROGRAM_CR       (1'b1),
    .POR_DELAY_CYCLES (0),
    .INIT_CR0         (TB_INIT_CR0),
    .PHY_VARIANT      ("SDR"),
    .DIFF_CK          (1'b1),
    .RD_PREAMBLE_SKIP (1),
    .CAL_RESET        (32'h0000_0002),  // cal_preamble_skip=1 (align to model preamble)
    .DBG_RESET        (TB_DBG_POR)      // POR REG_DBG = 0x60 (lat=6, trim=0) — A2
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

  // ------------------------------------------------------------------
  // Golden device model — the W957D8NB silicon defects, opt-in (issue #13):
  //   WR_WOUND_WORDS=4 + WR_WOUND_SAMPLE_BUS=1 : the wound content is the DQ words sampled in the
  //     pre-data window, so dbg_prewin_drive can heal it (and dbg_prewin_marker attribute it);
  //   WR_END_GARBLE_ROW_WORDS=1024 + WR_BOUNDARY_END_GARBLE=1 : the true row-multiple end garble;
  //   WR_WOUND_WRAP_IMMUNE=1 : a wrapped write over a wound zone repairs it (does not re-wound);
  //   RD_PREAMBLE_CLOCKS=1 : the read-strobe preamble the SDR PHY (RD_PREAMBLE_SKIP=1) pairs off.
  // ------------------------------------------------------------------
  hyperram_model #(
    .DQ_WIDTH                 (DQ_WIDTH),
    .MEM_WORDS                (1 << 16),
    .LATENCY_CLOCKS           (6),
    .FIXED_LATENCY            (1'b1),
    .FIXED_2X                 (1'b1),   // issue #13: W957D8NB fixed latency IS 2x — the device drives
                                        //   RWDS High during CA, the controller inserts the SECOND latency
                                        //   count, and dbg_prewin_drive parks [B-4..B-1] in exactly that
                                        //   window (§2.3 heal is gated on lat_extra_done = the 2nd count).
    .ROW_WORDS                (0),
    .REFRESH_EVERY            (0),
    .RD_PREAMBLE_CLOCKS       (1),
    .WR_WOUND_WORDS           (4),
    .WR_WOUND_SAMPLE_BUS      (1'b1),
    .WR_WOUND_WRAP_IMMUNE     (1'b1),
    .WR_BOUNDARY_END_GARBLE   (1'b1),
    .WR_END_GARBLE_ROW_WORDS  (1024),
    .WR_END_GARBLE_VALUE      (16'h5050)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i  (dq_line),  .hb_dq_ie  (phy_dq_oe),
    .hb_dq_o  (mdl_dq_o), .hb_dq_oe  (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (phy_rwds_oe),
    .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );

  // ------------------------------------------------------------------
  // Scoreboard
  // ------------------------------------------------------------------
  int unsigned errors = 0;

  task automatic check(input logic cond, input string msg);
    if (!cond) begin
      $display("[%0t] ERROR: %s", $time, msg);
      errors = errors + 1;
    end
  endtask

  // ------------------------------------------------------------------
  // CSR access. Drive on the falling edge; hold a read a FULL clock and sample at the next negedge so
  // csr_readdata is settled (tb_commit's idiom — a within-cycle #1 sample races the model's 1 ns
  // over-stream watchdog across the array-connected readdata, spec §5.2). waitrequest tied low.
  // ------------------------------------------------------------------
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
    @(negedge clk);
    data        = csr_readdata;
    csr_read    = 1'b0;
    csr_address = '0;
  endtask

  // One bench pass over LEN/BASE. ro=1 => READ-ONLY run (score the array as-is, no rewrite). BURSTW=
  // RBURSTW=LEN so the transfer is a SINGLE Avalon burst that the controller chops at MAX_BURST_WORDS.
  // Every poll is bounded (a hang -> FAIL, not an infinite loop).
  logic [31:0] g_status;
  task automatic run(input logic ro, input logic [31:0] len, input logic [ADDR_WIDTH-1:0] base,
                     output logic done, output logic [31:0] err);
    int unsigned guard;
    csr_wr(REG_LEN,     len);
    csr_wr(REG_BASE,    32'(base));
    csr_wr(REG_BURSTW,  len);
    csr_wr(REG_RBURSTW, len);
    csr_wr(REG_CTRL,    ro ? CTRL_START_RO : CTRL_START);
    guard = 0;
    do begin
      csr_rd(REG_CTRL, g_status);
      guard = guard + 1;
    end while (!g_status[1] && guard < 400000);
    done = g_status[1];
    csr_rd(REG_ERRCNT, err);
  endtask

  // Arm ONE wrapped write at word address b (REG_WRAP), then wait for the autonomous burst to finish.
  // Two-phase poll: first catch busy=1 (the wrap started, clearing the previous run's done), then wait
  // for done=1 — so a stale done from the prior run is never mistaken for completion.
  task automatic wrap_write(input logic [ADDR_WIDTH-1:0] b);
    int unsigned guard;
    csr_wr(REG_WRAP, 32'(b));
    guard = 0;                                   // phase 1: wait for the wrapped write to go busy
    do begin csr_rd(REG_CTRL, g_status); guard = guard + 1; end while (!g_status[0] && guard < 2000);
    check(g_status[0] === 1'b1, "wrap_write: bench never went busy after REG_WRAP arm");
    guard = 0;                                   // phase 2: wait for completion
    do begin csr_rd(REG_CTRL, g_status); guard = guard + 1; end while (!g_status[1] && guard < 400000);
    check(g_status[1] === 1'b1, "wrap_write: wrapped write did not complete (STATUS.done)");
  endtask

  // Read EMAP entry i (A5 registered RAM read: write IDX, then read ADDR/DATA on later transactions).
  task automatic emap_read(input logic [6:0] i, output logic [31:0] eaddr,
                           output logic [15:0] egot, output logic [15:0] eexp);
    logic [31:0] edata;
    csr_wr(REG_EMAP_IDX, {25'b0, i});
    csr_rd(REG_EMAP_ADDR, eaddr);
    csr_rd(REG_EMAP_DATA, edata);       // {got[31:16], exp[15:0]}
    egot = edata[31:16];
    eexp = edata[15:0];
  endtask

  // ------------------------------------------------------------------
  // Stimulus
  // ------------------------------------------------------------------
  logic [31:0] status, err, rb, stat;
  logic [31:0] eaddr;
  logic [15:0] egot, eexp;
  logic        done;
  int unsigned guard;
  logic [31:0] wound_base;   // ERR of the PAT=1 drive=0 run — the pure wound count, reused everywhere
  logic [31:0] emap_count;

  initial begin
    csr_address = '0; csr_read = 1'b0; csr_write = 1'b0; csr_writedata = '0;
    rst = 1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk);
    rst = 1'b0;

    // ---- wait for POR init + CR0 programming ----
    guard = 0;
    while (!init_done && guard < 100000) begin @(posedge clk); guard = guard + 1; end
    check(init_done, "init_done never asserted");
    repeat (4) @(posedge clk);

    $display("==================================================================");
    $display("tb_dbg: issue-#13 instrumented build (REG_DBG/EMAP/PAT/WRAP vs wound model)");
    $display("==================================================================");

    // Instrumented-build identity: MAGIC bumped to 0x48425755 (A10).
    csr_rd(REG_MAGIC, rb);
    check(rb === TB_MAGIC, $sformatf("MAGIC 0x%08x exp 0x%08x (instrumented build)", rb, TB_MAGIC));

    // =============================================================================================
    // CHECK 1 — baseline default-legacy. POR REG_DBG readback == 0x60 (A2: bit8 reads 0, lat=6,
    // trim=0). REG_PAT/REG_WRAP POR == 0. A normal run (drive off) then wounds where the model wounds.
    // =============================================================================================
    csr_rd(REG_DBG,  rb); check(rb === TB_DBG_POR, $sformatf("[1] POR REG_DBG=0x%08x exp 0x%08x (A2)", rb, TB_DBG_POR));
    csr_rd(REG_PAT,  rb); check(rb === 32'd0,      $sformatf("[1] POR REG_PAT=0x%08x exp 0", rb));
    csr_rd(REG_WRAP, rb); check(rb === 32'd0,      $sformatf("[1] POR REG_WRAP=0x%08x exp 0", rb));

    csr_wr(REG_PAT, 32'd1);                        // 0xFFFF background: the idle-bus wound (0x0000) is always visible
    run(1'b0, LEN_HEAL, BASE_C, done, err);
    wound_base = err;                              // the pure wound count (reused by checks 2/3/5/6)
    $display("-- [1] baseline: LEN=%0d base=0x%03x drive=off -> ERR=%0d (wound present)", LEN_HEAL, BASE_C, err);
    check(done,        "[1] baseline run did not complete");
    check(err > 32'd0, "[1] baseline ERR=0 — the model did not wound (harness broken)");

    // =============================================================================================
    // CHECK 2 — REG_PAT. Same wounding geometry, sweep the pattern. pat 0/1/3 are non-zero at the
    // wound addresses so the idle-bus wound (0x0000) shows as ERR==wound_base and write/read agree on
    // the pattern (else the non-wound words would mismatch too). pat 2 (0x0000) ALIASES the wound ->
    // ERR==0: the gen(0)=0 pitfall generalized, why a pass assertion must use PAT!=0/2 or BASE!=0.
    // =============================================================================================
    begin
      logic [31:0] exp_err [4];
      exp_err[0] = wound_base; exp_err[1] = wound_base; exp_err[2] = 32'd0; exp_err[3] = wound_base;
      for (int p = 0; p < 4; p++) begin
        csr_wr(REG_PAT, 32'(p));
        csr_rd(REG_PAT, rb); check(rb === 32'(p), $sformatf("[2] REG_PAT readback 0x%08x exp %0d", rb, p));
        run(1'b0, LEN_HEAL, BASE_C, done, err);
        $display("   [2] pat=%0d -> ERR=%0d (expect %0d)", p, err, exp_err[p]);
        check(done, $sformatf("[2] pat=%0d run did not complete", p));
        check(err === exp_err[p],
              $sformatf("[2] pat=%0d ERR=%0d exp=%0d (write/read pattern agreement; pat2 aliases the wound)",
                        p, err, exp_err[p]));
      end
    end
    csr_wr(REG_PAT, 32'd1);                        // restore 0xFFFF for the remaining checks

    // =============================================================================================
    // CHECK 3 — the HEAL (L-D). dbg_prewin_drive parks the replay shadow [C-4..C-1] in the pre-data
    // window; the model (WR_WOUND_SAMPLE_BUS) stores exactly those words as the wound -> the read
    // back is clean. n>=4 covers all four wound words; n=3 leaves B-4 (the earliest edge) idle -> a
    // partial heal (fewer errors, but not zero).
    // =============================================================================================
    csr_wr(REG_DBG, TB_DBG_POR);                   // drive off
    run(1'b0, LEN_HEAL, BASE_C, done, err);
    $display("-- [3] heal Run1 (drive off): ERR=%0d (expect %0d)", err, wound_base);
    check(done && err === wound_base, $sformatf("[3] Run1 ERR=%0d exp=%0d", err, wound_base));

    csr_wr(REG_DBG, DBG_HEAL_N4);
    csr_rd(REG_DBG, rb); check(rb === DBG_HEAL_N4, $sformatf("[3] REG_DBG readback 0x%08x exp 0x%08x", rb, DBG_HEAL_N4));
    run(1'b0, LEN_HEAL, BASE_C, done, err);
    $display("   [3] heal n=4 -> ERR=%0d (expect 0)", err);
    check(done && err === 32'd0, $sformatf("[3] n=4 heal ERR=%0d expected 0 (shadow drive heals every chop)", err));

    csr_wr(REG_DBG, DBG_HEAL_N5);
    run(1'b0, LEN_HEAL, BASE_C, done, err);
    $display("   [3] heal n=5 -> ERR=%0d (expect 0)", err);
    check(done && err === 32'd0, $sformatf("[3] n=5 heal ERR=%0d expected 0", err));

    csr_wr(REG_DBG, DBG_HEAL_N3);
    run(1'b0, LEN_HEAL, BASE_C, done, err);
    $display("   [3] heal n=3 -> ERR=%0d (expect partial: 0 < ERR < %0d)", err, wound_base);
    check(done, "[3] n=3 run did not complete");
    check(err > 32'd0 && err < wound_base,
          $sformatf("[3] n=3 ERR=%0d expected partial heal in (0,%0d) — B-4 stays wounded", err, wound_base));

    // =============================================================================================
    // CHECK 4 — marker attribution (L-C). dbg_prewin_marker parks 0xA500|k instead of the shadow, so
    // the wound content becomes 0xA5xx: the sampling window is proven and located. Every EMAP got
    // must be one of 0xA500..0xA503.
    // =============================================================================================
    csr_wr(REG_DBG, DBG_MARKER);
    run(1'b0, LEN_HEAL, BASE_C, done, err);
    $display("-- [4] marker: ERR=%0d (wound reads 0xA5xx)", err);
    check(done && err > 32'd0, $sformatf("[4] marker ERR=%0d expected >0 (marker still corrupts the wound)", err));
    csr_rd(REG_EMAP_STAT, stat);
    emap_count = 32'(stat[6:0]);
    check(emap_count > 32'd0, "[4] EMAP empty after a marker run");
    for (int i = 0; i < int'(emap_count); i++) begin
      emap_read(7'(i), eaddr, egot, eexp);
      check((egot & 16'hFFFC) === 16'hA500,
            $sformatf("[4] EMAP[%0d] addr=0x%08x got=0x%04x — expected 0xA500..0xA503 (marker attribution)",
                      i, eaddr, egot));
    end
    $display("   [4] all %0d EMAP entries carry the 0xA5xx marker (sampling window located)", emap_count);

    // =============================================================================================
    // CHECK 5 — latency reprogram coherence (L-B, recast by A7 for a trim-less model). Program
    // dbg_lat_clocks=7, fire the CR0-reprogram strobe (bit8), let the controller rewrite CR0 with the
    // new latency code; the model follows CR0 dynamically (its lat_active tracks cr0[7:4]). A run then
    // stays coherent — the ONLY errors are the same wound baseline, never a per-word latency desync
    // (which would blow ERR far past the baseline). trim!=0 is skipped: the sim model has no
    // WR_LAT_TRIM analog (A2), so any trim!=0 is a desync by construction, not a meaningful check.
    // =============================================================================================
    csr_wr(REG_DBG, DBG_LAT7);                      // set the live seed first (bit8=0)
    repeat (60) @(posedge clk);                     // host contract (§2.2/A3): the reprogram strobe is a
                                                    //   1-cycle pulse the ctrl consumes ONLY at ST_IDLE — let
                                                    //   the ctrl fully drain the prior run first (a real JTAG
                                                    //   host pokes bit8 ms after busy=0; back-to-back sim CSR
                                                    //   writes can otherwise land the pulse mid-recovery).
    csr_wr(REG_DBG, DBG_LAT7_RP);                   // then fire the CR0-reprogram strobe (bit8=1)
    csr_rd(REG_DBG, rb);                            // bit8 always reads 0 (strobe, never stored)
    check(rb === DBG_LAT7, $sformatf("[5] REG_DBG after reprogram=0x%08x exp 0x%08x (bit8 self-clears)", rb, DBG_LAT7));
    repeat (300) @(posedge clk);                    // let the autonomous CR0 rewrite complete (idle-gated)
    run(1'b0, LEN_HEAL, BASE_C, done, err);
    $display("-- [5] lat=7 (CR0 reprogrammed) -> ERR=%0d (expect coherent == %0d)", err, wound_base);
    check(done, "[5] lat=7 run did not complete (latency desync / hang)");
    check(err === wound_base,
          $sformatf("[5] lat=7 ERR=%0d exp=%0d — controller/model latency incoherent if != baseline", err, wound_base));

    csr_wr(REG_DBG, 32'h0000_0060);                 // restore lat=6 seed
    repeat (60) @(posedge clk);                     // settle to ST_IDLE before the strobe (see above)
    csr_wr(REG_DBG, DBG_LAT6_RP);                   // reprogram CR0 back to code(6)
    repeat (300) @(posedge clk);
    run(1'b0, LEN_HEAL, BASE_C, done, err);
    $display("   [5] lat=6 restored -> ERR=%0d (expect %0d)", err, wound_base);
    check(done && err === wound_base, $sformatf("[5] lat=6 restore ERR=%0d exp=%0d", err, wound_base));

    // =============================================================================================
    // CHECK 6 — REG_EMAP wound-map. After a multi-wound run EMAP holds EVERY mismatch (not just the
    // first, unlike REG_ERRADDR): count == ERR for ERR<=64, and each entry is a real wound (got=0x0000
    // the idle-bus wound, exp=0xFFFF, addr in the last 4 words of a 64-word chop block). Beyond 64
    // wounds the count saturates and the sticky overflow bit sets.
    // =============================================================================================
    csr_wr(REG_DBG, TB_DBG_POR);                    // drive off -> idle-bus wound (0x0000)
    run(1'b0, LEN_HEAL, BASE_C, done, err);
    csr_rd(REG_EMAP_STAT, stat);
    emap_count = 32'(stat[6:0]);
    $display("-- [6] EMAP: LEN=%0d ERR=%0d count=%0d valid=%0b ov=%0b", LEN_HEAL, err, emap_count, stat[7], stat[8]);
    check(done, "[6] EMAP run did not complete");
    check(emap_count === err, $sformatf("[6] EMAP count=%0d != ERR=%0d (must record every wound)", emap_count, err));
    check(stat[7] === (err != 0), "[6] EMAP valid bit disagrees with count>0");
    check(stat[8] === 1'b0, "[6] EMAP overflow set spuriously (ERR<=64)");
    for (int i = 0; i < int'(emap_count); i++) begin
      emap_read(7'(i), eaddr, egot, eexp);
      check(egot === 16'h0000 && eexp === 16'hFFFF,
            $sformatf("[6] EMAP[%0d] got=0x%04x exp=0x%04x — expected 0x0000/0xFFFF (idle-bus wound)", i, egot, eexp));
      check(((eaddr - 32'(BASE_C)) % 32'd64) >= 32'd60,
            $sformatf("[6] EMAP[%0d] addr=0x%08x not in a chop-tail [C-4,C)", i, eaddr));
    end
    // Overflow: > 64 wounds -> count saturates at 64, sticky overflow sets, ERR still counts all.
    run(1'b0, LEN_OVF, BASE_C, done, err);
    csr_rd(REG_EMAP_STAT, stat);
    emap_count = 32'(stat[6:0]);
    $display("   [6] overflow: LEN=%0d ERR=%0d count=%0d ov=%0b (expect count=64 ov=1 ERR>64)", LEN_OVF, err, emap_count, stat[8]);
    check(done,                    "[6] overflow run did not complete");
    check(err > 32'd64,            $sformatf("[6] overflow ERR=%0d expected >64", err));
    check(emap_count === 32'd64,   $sformatf("[6] overflow count=%0d expected 64 (saturated)", emap_count));
    check(stat[8] === 1'b1,        "[6] overflow bit not set beyond 64 wounds");

    // =============================================================================================
    // CHECK 7 — wrapped-write repair + immunity (L-F). Establish 0xFFFF over [0x1F0,0x200); a LINEAR
    // write at 0x200 wounds [0x1FC,0x200) (RO-probe -> ERR=4). A WRAPPED write over that group repairs
    // it (RO-probe -> ERR=0), and a WRAPPED write at 0x200 does NOT wound [0x1FC,0x200) the way the
    // linear one did (RO-probe -> ERR=0) — the immunity that makes wrapped writes a repair primitive.
    // =============================================================================================
    csr_wr(REG_DBG, TB_DBG_POR);
    csr_wr(REG_PAT, 32'd1);                          // 0xFFFF everywhere: wrap-address arithmetic is irrelevant
    // background: [0x1F0,0x200) = 0xFFFF (single 16-word burst, no chop; its own wound is below 0x1F0)
    run(1'b0, 32'd16, 32'h0000_01F0, done, err);
    // linear write at 0x200 wounds [0x1FC,0x200); RO-probe the background zone.
    run(1'b0, 32'd16, 32'h0000_0200, done, err);
    run(1'b1, 32'd16, 32'h0000_01F0, done, err);    // RO probe [0x1F0,0x200)
    $display("-- [7] linear write @0x200 wounds [0x1FC,0x200): RO ERR=%0d (expect 4)", err);
    check(done && err === 32'd4, $sformatf("[7] linear-wound RO ERR=%0d expected 4", err));
    // wrapped write over the wounded group [0x1F0,0x200) repairs it (WR_WOUND_WRAP_IMMUNE: no re-wound).
    wrap_write(32'h0000_01F0);
    run(1'b1, 32'd16, 32'h0000_01F0, done, err);
    $display("   [7] wrapped write @0x1F0 repairs the zone: RO ERR=%0d (expect 0)", err);
    check(done && err === 32'd0, $sformatf("[7] wrap-repair RO ERR=%0d expected 0 (wrapped write heals the wound)", err));
    // now the zone is clean; a WRAPPED write at 0x200 must NOT wound [0x1FC,0x200) (a linear one did).
    wrap_write(32'h0000_0200);
    run(1'b1, 32'd16, 32'h0000_01F0, done, err);
    $display("   [7] wrapped write @0x200 does NOT wound [0x1FC,0x200): RO ERR=%0d (expect 0)", err);
    check(done && err === 32'd0, $sformatf("[7] wrap-immune RO ERR=%0d expected 0 (wrapped write never wounds)", err));

    // =============================================================================================
    // CHECK 8 — postwin / ck_stretch smoke (L-E). Both are inert on a normal (non-row-end) burst in
    // sim: dbg_ck_stretch_off is a board gpio knob with no sim wire; dbg_postwin_hold parks the last
    // word into a CK-less tail the device never clocks. A single non-row-end burst reads back clean
    // (no in-range wound) with the knobs off, and stays clean with each knob on.
    // =============================================================================================
    csr_wr(REG_DBG, TB_DBG_POR);
    run(1'b0, 32'd16, 32'h0000_0130, done, err);    // baseline: single burst, non-row-end -> ERR=0
    $display("-- [8] postwin/ck_stretch baseline (knobs off): ERR=%0d (expect 0)", err);
    check(done && err === 32'd0, $sformatf("[8] baseline ERR=%0d expected 0", err));
    csr_wr(REG_DBG, DBG_POSTWIN);
    run(1'b0, 32'd16, 32'h0000_0130, done, err);
    $display("   [8] dbg_postwin_hold on: ERR=%0d (expect 0, normal data uncorrupted)", err);
    check(done && err === 32'd0, $sformatf("[8] postwin_hold ERR=%0d expected 0", err));
    csr_wr(REG_DBG, DBG_CKSTR);
    run(1'b0, 32'd16, 32'h0000_0130, done, err);
    $display("   [8] dbg_ck_stretch_off on: ERR=%0d (expect 0, board-only knob inert in sim)", err);
    check(done && err === 32'd0, $sformatf("[8] ck_stretch_off ERR=%0d expected 0", err));

    csr_wr(REG_DBG, TB_DBG_POR);
    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_dbg done: %0d errors  (wound baseline = %0d, healed to 0 via dbg_prewin_drive)",
             $time, errors, wound_base);
    if (errors == 0) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_dbg: %0d errors", errors);
    end
  end

  // Global watchdog — a true infinite hang (every poll above is bounded).
  initial begin
    #20_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_dbg: global timeout");
  end

endmodule
