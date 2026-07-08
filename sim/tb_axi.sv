// tb_axi — self-checking Verilator testbench for hyperram_axi + hyperram_model.
//
// Instantiates the AXI4 top and the golden device model, resolves the shared split HyperBus bus
// (DQ/RWDS) between master (PHY) and device (model), and exercises:
//   * POR init + CR0 programming (waits for init_done),
//   * single-beat memory write-then-read-back (INCR) at several addresses,
//   * multi-beat (burst) memory write-then-read-back (INCR) at several addresses/lengths,
//   * a WRAP burst read across a wrap-group boundary, checked against the device's legacy-wrap
//     address sequence,
//   * a CR0 register write + read-back,
//   * an ID0 register read (expects the reset device-ID from the package).
// Data is address-derived (genword); every readback beat is checked. B/R responses must be OKAY.
// Any mismatch -> error count > 0 -> $fatal (non-zero exit). Success -> $finish.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_axi;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH       = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH     = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH     = HB_ADDR_WIDTH;         // 32 (word address)
  localparam int unsigned LEN_WIDTH      = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned ID_WIDTH       = 4;
  localparam int unsigned AXI_DATA_WIDTH = DATA_WIDTH;            // 16 (1:1 beat<->word)
  localparam int unsigned AXI_ADDR_WIDTH = ADDR_WIDTH + 1;        // 33 (byte address; MSB=reg space)
  localparam int unsigned AXI_STRB_WIDTH = AXI_DATA_WIDTH / 8;    // 2

  localparam logic [1:0] AXI_FIXED  = 2'b00;
  localparam logic [1:0] AXI_INCR   = 2'b01;
  localparam logic [1:0] AXI_WRAP   = 2'b10;
  localparam logic [1:0] AXI_OKAY   = 2'b00;
  localparam logic [1:0] AXI_SLVERR = 2'b10;

  // INIT_CR0 image programmed at init: latency code 0001 (=6 clocks), fixed-latency, legacy wrap,
  // 32-byte (16-word) wrap group. Must match the controller's LATENCY_CLOCKS so the model agrees.
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam int unsigned WRAP_WORDS   = 16;                       // hb_wrap_words(2'b11)

  // --------------------------------------------------------------------
  // Clocking / reset
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;    end   // 100 MHz
  initial begin #2.5; clk90  = 1'b0; forever #5.0 clk90  = ~clk90;  end // +90 deg
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  // --------------------------------------------------------------------
  // AXI4 slave signals
  // --------------------------------------------------------------------
  logic [ID_WIDTH-1:0]         awid;
  logic [AXI_ADDR_WIDTH-1:0]   awaddr;
  logic [7:0]                  awlen;
  logic [2:0]                  awsize;
  logic [1:0]                  awburst;
  logic                        awvalid, awready;
  logic [AXI_DATA_WIDTH-1:0]   wdata;
  logic [AXI_STRB_WIDTH-1:0]   wstrb;
  logic                        wlast, wvalid, wready;
  logic [ID_WIDTH-1:0]         bid;
  logic [1:0]                  bresp;
  logic                        bvalid, bready;
  logic [ID_WIDTH-1:0]         arid;
  logic [AXI_ADDR_WIDTH-1:0]   araddr;
  logic [7:0]                  arlen;
  logic [2:0]                  arsize;
  logic [1:0]                  arburst;
  logic                        arvalid, arready;
  logic [ID_WIDTH-1:0]         rid;
  logic [AXI_DATA_WIDTH-1:0]   rdata;
  logic [1:0]                  rresp;
  logic                        rlast, rvalid, rready;
  logic                        init_done;

  // --------------------------------------------------------------------
  // HyperBus device pins + shared-bus resolution
  // --------------------------------------------------------------------
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  phy_dq_o;   logic phy_dq_oe;
  logic                 phy_rwds_o; logic phy_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;
  logic                 mdl_rwds_o; logic mdl_rwds_oe;

  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (phy_dq_oe   ? phy_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (phy_rwds_oe ? phy_rwds_o : 1'b0);

  // Round-trip DQ/RWDS flight delay from the device back to the master. The read path (slave->master)
  // is delayed so the PHY only recovers correct data if its capture is source-synchronous to RWDS
  // (finding #4). The write/CA path (master->slave) is undelayed (the master owns that timing).
  localparam realtime RTT = 3.0;    // ns (> a fixed-phase sampler's half-eye margin, so this exercises
                                    //  finding #4: only an RWDS-source-synchronous capture recovers it)
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  // --------------------------------------------------------------------
  // DUT: AXI top
  // --------------------------------------------------------------------
  hyperram_axi #(
    .ID_WIDTH         (ID_WIDTH),
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
    .awid (awid), .awaddr (awaddr), .awlen (awlen), .awsize (awsize), .awburst (awburst),
    .awvalid (awvalid), .awready (awready),
    .wdata (wdata), .wstrb (wstrb), .wlast (wlast), .wvalid (wvalid), .wready (wready),
    .bid (bid), .bresp (bresp), .bvalid (bvalid), .bready (bready),
    .arid (arid), .araddr (araddr), .arlen (arlen), .arsize (arsize), .arburst (arburst),
    .arvalid (arvalid), .arready (arready),
    .rid (rid), .rdata (rdata), .rresp (rresp), .rlast (rlast), .rvalid (rvalid), .rready (rready),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (phy_dq_o), .hb_dq_oe (phy_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (phy_rwds_o), .hb_rwds_oe (phy_rwds_oe), .hb_rwds_i (rwds_line_dly),
    .init_done (init_done)
  );

  // --------------------------------------------------------------------
  // Golden device model
  // --------------------------------------------------------------------
  hyperram_model #(
    .DQ_WIDTH       (DQ_WIDTH),
    .MEM_WORDS      (1 << 16),
    .LATENCY_CLOCKS (6),
    .FIXED_LATENCY  (1'b1),
    .ROW_WORDS      (0),
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
  int unsigned checks = 0;

  localparam int unsigned CAP_MAX = 256;
  logic [DATA_WIDTH-1:0] cap [CAP_MAX];
  int unsigned           cap_n;

  function automatic logic [DATA_WIDTH-1:0] genword(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction

  // Device legacy-wrap next-address (cr0[2]=1): stay in the wrap group forever.
  function automatic logic [ADDR_WIDTH-1:0] wrap_next(input logic [ADDR_WIDTH-1:0] a,
                                                      input logic [ADDR_WIDTH-1:0] gbase,
                                                      input logic [ADDR_WIDTH-1:0] gtop);
    return (a == gtop) ? gbase : (a + 1);
  endfunction

  // Byte address for a word address + register-space select (MSB) + byte offset 0.
  function automatic logic [AXI_ADDR_WIDTH-1:0] byte_addr(input logic [ADDR_WIDTH-1:0] word_addr,
                                                          input logic reg_space);
    logic [AXI_ADDR_WIDTH-1:0] b;
    b = '0;
    b[AXI_ADDR_WIDTH-1]     = reg_space;                     // MSB (bit 32) = register space
    b[ADDR_WIDTH-1:1]       = word_addr[ADDR_WIDTH-2:0];     // word address in bits [31:1]
    return b;
  endfunction

  // --------------------------------------------------------------------
  // AXI tasks. Stimulus driven on negedge (blocking), status sampled on posedge.
  // --------------------------------------------------------------------
  task automatic axi_idle();
    @(negedge clk);
    awid = '0; awaddr = '0; awlen = '0; awsize = 3'd1; awburst = AXI_INCR; awvalid = 1'b0;
    wdata = '0; wstrb = '1; wlast = 1'b0; wvalid = 1'b0;
    bready = 1'b0;
    arid = '0; araddr = '0; arlen = '0; arsize = 3'd1; arburst = AXI_INCR; arvalid = 1'b0;
    rready = 1'b0;
  endtask

  // AXI write burst of n beats (words) at word `addr`.
  task automatic axi_write(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                           input logic reg_space, input logic [1:0] burst,
                           input logic [DATA_WIDTH-1:0] data [$]);
    int unsigned idx, g;
    // ---- AW ----
    @(negedge clk);
    awid    = 4'h5;
    awaddr  = byte_addr(addr, reg_space);
    awlen   = 8'(n - 1);
    awsize  = 3'd1;
    awburst = burst;
    awvalid = 1'b1;
    g = 0;
    forever begin
      @(posedge clk); g = g + 1;
      if (awready) break;
      if (g > 3000) begin $display("[%0t] HANG AW @0x%08x", $time, addr); errors = errors + 1; break; end
    end
    @(negedge clk); awvalid = 1'b0;
    // ---- W ----
    idx = 0;
    @(negedge clk);
    wvalid = 1'b1; wstrb = '1; wdata = data[0]; wlast = (n == 1);
    g = 0;
    forever begin
      @(posedge clk); g = g + 1;
      if (g > 5000) begin $display("[%0t] HANG W @0x%08x idx=%0d", $time, addr, idx); errors = errors + 1; break; end
      if (wready) begin
        idx = idx + 1;
        if (idx == n) break;
        @(negedge clk);
        wdata = data[idx];
        wlast = (idx == n - 1);
      end
    end
    @(negedge clk); wvalid = 1'b0; wlast = 1'b0;
    // ---- B ----
    @(negedge clk); bready = 1'b1;
    g = 0;
    forever begin
      @(posedge clk); g = g + 1;
      if (bvalid) begin
        checks = checks + 1;
        if (bresp !== AXI_OKAY) begin
          $display("[%0t] ERROR: bresp=0x%0x (exp OKAY) @0x%08x", $time, bresp, addr);
          errors = errors + 1;
        end
        break;
      end
      if (g > 3000) begin $display("[%0t] HANG B @0x%08x", $time, addr); errors = errors + 1; break; end
    end
    @(negedge clk); bready = 1'b0;
    axi_idle();
  endtask

  // AXI read burst of n beats at word `addr`; results land in cap[0..n-1].
  task automatic axi_read(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                          input logic reg_space, input logic [1:0] burst);
    int unsigned g;
    cap_n = 0;
    // ---- AR ----
    @(negedge clk);
    arid    = 4'h5;
    araddr  = byte_addr(addr, reg_space);
    arlen   = 8'(n - 1);
    arsize  = 3'd1;
    arburst = burst;
    arvalid = 1'b1;
    g = 0;
    forever begin
      @(posedge clk); g = g + 1;
      if (arready) break;
      if (g > 3000) begin $display("[%0t] HANG AR @0x%08x", $time, addr); errors = errors + 1; break; end
    end
    @(negedge clk); arvalid = 1'b0;
    // ---- R ----
    @(negedge clk); rready = 1'b1;
    g = 0;
    forever begin
      @(posedge clk); g = g + 1;
      if (rvalid) begin
        if (cap_n < CAP_MAX) cap[cap_n] = rdata;
        checks = checks + 1;
        if (rresp !== AXI_OKAY) begin
          $display("[%0t] ERROR: rresp=0x%0x (exp OKAY) @0x%08x beat %0d", $time, rresp, addr, cap_n);
          errors = errors + 1;
        end
        if ((cap_n == n - 1) && !rlast) begin
          $display("[%0t] ERROR: rlast not set on final beat @0x%08x", $time, addr);
          errors = errors + 1;
        end
        cap_n = cap_n + 1;
        if (cap_n == n) break;
      end
      if (g > 5000) begin $display("[%0t] HANG R @0x%08x got %0d/%0d", $time, addr, cap_n, n); errors = errors + 1; break; end
    end
    @(negedge clk); rready = 1'b0;
    axi_idle();
  endtask

  // INCR write-then-read-back, checked per beat.
  task automatic wr_rd_incr(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n, input string tag);
    logic [DATA_WIDTH-1:0] wdat [$];
    logic [DATA_WIDTH-1:0] exp;
    int unsigned i;
    wdat = {};
    for (i = 0; i < n; i++) wdat.push_back(genword(addr + i));
    axi_write(addr, n, 1'b0, AXI_INCR, wdat);
    axi_read (addr, n, 1'b0, AXI_INCR);
    for (i = 0; i < n; i++) begin
      exp = genword(addr + i);
      checks = checks + 1;
      if (cap[i] !== exp) begin
        $display("[%0t] ERROR (%s): addr 0x%08x beat %0d got 0x%04x exp 0x%04x",
                 $time, tag, addr, i, cap[i], exp);
        errors = errors + 1;
      end
    end
    $display("[%0t] %s: %0d-beat INCR wr/rd @0x%08x done (errors so far %0d)", $time, tag, n, addr, errors);
  endtask

  // Single-beat read with an explicit AxSIZE, capturing RRESP (no OKAY assertion). Used to verify
  // that a narrow (unsupported) beat size is reported as SLVERR rather than silently mis-decoded.
  logic [1:0] last_rresp;
  task automatic axi_read_size(input logic [ADDR_WIDTH-1:0] addr, input logic [2:0] size);
    int unsigned g;
    last_rresp = 2'bxx;
    @(negedge clk);
    arid = 4'h7; araddr = byte_addr(addr, 1'b0); arlen = 8'd0; arsize = size; arburst = AXI_INCR;
    arvalid = 1'b1;
    g = 0;
    forever begin @(posedge clk); g = g + 1; if (arready) break;
      if (g > 3000) begin $display("[%0t] HANG AR(size) @0x%08x", $time, addr); errors = errors + 1; break; end end
    @(negedge clk); arvalid = 1'b0;
    @(negedge clk); rready = 1'b1;
    g = 0;
    forever begin @(posedge clk); g = g + 1;
      if (rvalid) begin last_rresp = rresp; if (rlast) begin end
        if (rvalid & rready & rlast) break; end
      if (g > 5000) begin $display("[%0t] HANG R(size) @0x%08x", $time, addr); errors = errors + 1; break; end
    end
    @(negedge clk); rready = 1'b0;
    axi_idle();
  endtask

  // Check a WRAP read of n beats at word `addr` against the AXI wrap sequence (region = n words,
  // aligned down from addr). Memory must already hold genword() at every region word.
  task automatic wrap_check(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n, input string tag);
    logic [ADDR_WIDTH-1:0] gbase, gtop, wa;
    logic [DATA_WIDTH-1:0] exp;
    int unsigned i;
    axi_read(addr, n, 1'b0, AXI_WRAP);
    gbase = addr & ~(ADDR_WIDTH'(n) - 1);
    gtop  = gbase | (ADDR_WIDTH'(n) - 1);
    wa    = addr;
    for (i = 0; i < n; i++) begin
      exp = genword(wa);
      checks = checks + 1;
      if (cap[i] !== exp) begin
        $display("[%0t] ERROR (%s): beat %0d addr 0x%08x got 0x%04x exp 0x%04x",
                 $time, tag, i, wa, cap[i], exp);
        errors = errors + 1;
      end
      wa = (wa == gtop) ? gbase : (wa + 1);
    end
    $display("[%0t] %s: %0d-beat WRAP read @0x%08x done (errors so far %0d)", $time, tag, n, addr, errors);
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  logic [DATA_WIDTH-1:0] one [$];
  logic [DATA_WIDTH-1:0] fill [$];
  logic [DATA_WIDTH-1:0] cr0_rb, id0_rb, exp;
  logic [ADDR_WIDTH-1:0] gbase, gtop, wa;
  int unsigned guard, i;

  initial begin
    // Idle drive
    awid = '0; awaddr = '0; awlen = '0; awsize = 3'd1; awburst = AXI_INCR; awvalid = 1'b0;
    wdata = '0; wstrb = '1; wlast = 1'b0; wvalid = 1'b0; bready = 1'b0;
    arid = '0; araddr = '0; arlen = '0; arsize = 3'd1; arburst = AXI_INCR; arvalid = 1'b0;
    rready = 1'b0;
    cap_n = 0;
    rst   = 1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk); rst = 1'b0;

    guard = 0;
    while (!init_done && guard < 100000) begin @(posedge clk); guard = guard + 1; end
    if (!init_done) begin
      $display("[%0t] FATAL: init_done never asserted", $time);
      errors = errors + 1;
    end else $display("[%0t] init_done asserted", $time);
    repeat (4) @(posedge clk);

    // ---- single-beat INCR ----
    wr_rd_incr(32'h0000_0000, 1, "single");
    wr_rd_incr(32'h0000_0055, 1, "single");
    wr_rd_incr(32'h0000_1abc, 1, "single");

    // ---- multi-beat INCR ----
    wr_rd_incr(32'h0000_0020, 4,  "burst4");
    wr_rd_incr(32'h0000_0200, 8,  "burst8");
    wr_rd_incr(32'h0000_3000, 16, "burst16");

    // ---- WRAP burst read across a wrap-group boundary ----
    // Fill the whole 16-word group at 0x40..0x4F linearly, then WRAP-read from an offset (0x48).
    fill = {};
    for (i = 0; i < WRAP_WORDS; i++) fill.push_back(genword(32'h40 + i));
    axi_write(32'h0000_0040, WRAP_WORDS, 1'b0, AXI_INCR, fill);
    // Wrapped read of WRAP_WORDS beats starting at 0x48.
    axi_read(32'h0000_0048, WRAP_WORDS, 1'b0, AXI_WRAP);
    gbase = 32'h48 & ~(ADDR_WIDTH'(WRAP_WORDS) - 1);
    gtop  = gbase | (ADDR_WIDTH'(WRAP_WORDS) - 1);
    wa    = 32'h48;
    for (i = 0; i < WRAP_WORDS; i++) begin
      exp = genword(wa);
      checks = checks + 1;
      if (cap[i] !== exp) begin
        $display("[%0t] ERROR (wrap): beat %0d addr 0x%08x got 0x%04x exp 0x%04x",
                 $time, i, wa, cap[i], exp);
        errors = errors + 1;
      end
      wa = wrap_next(wa, gbase, gtop);
    end
    $display("[%0t] wrap: %0d-beat WRAP read @0x48 done (errors so far %0d)", $time, WRAP_WORDS, errors);

    // ---- WRAP bursts that HyperBus cannot express as a native device wrap group ----
    // WRAP4 (4-word region) and WRAP2 (2-word region) are smaller than the smallest HyperBus wrap
    // group; the front-end must reproduce them via linear-segment decomposition.
    fill = {};
    for (i = 0; i < 4; i++) fill.push_back(genword(32'h80 + i));
    axi_write(32'h0000_0080, 4, 1'b0, AXI_INCR, fill);
    wrap_check(32'h0000_0082, 4, "wrap4");

    fill = {};
    for (i = 0; i < 2; i++) fill.push_back(genword(32'h90 + i));
    axi_write(32'h0000_0090, 2, 1'b0, AXI_INCR, fill);
    wrap_check(32'h0000_0091, 2, "wrap2");

    // ---- FIXED burst: every beat accesses the SAME address ----
    // 4-beat FIXED write of distinct values -> last value wins in memory; then a 1-beat and a 4-beat
    // FIXED read must all return that last value (not an incrementing sequence).
    fill = {};
    for (i = 0; i < 4; i++) fill.push_back(genword(32'h120 + i));
    axi_write(32'h0000_0120, 4, 1'b0, AXI_FIXED, fill);
    axi_read (32'h0000_0120, 1, 1'b0, AXI_INCR);
    checks = checks + 1;
    if (cap[0] !== genword(32'h123)) begin
      $display("[%0t] ERROR (fixed-wr): @0x120 got 0x%04x exp 0x%04x", $time, cap[0], genword(32'h123));
      errors = errors + 1;
    end
    axi_read(32'h0000_0120, 4, 1'b0, AXI_FIXED);
    for (i = 0; i < 4; i++) begin
      checks = checks + 1;
      if (cap[i] !== genword(32'h123)) begin
        $display("[%0t] ERROR (fixed-rd): beat %0d got 0x%04x exp 0x%04x", $time, i, cap[i], genword(32'h123));
        errors = errors + 1;
      end
    end
    $display("[%0t] fixed: 4-beat FIXED wr + FIXED rd @0x120 done (errors so far %0d)", $time, errors);

    // ---- narrow (unsupported) beat size must return SLVERR, not silent mis-decode ----
    axi_read_size(32'h0000_0000, 3'd0);   // arsize=0 (1 byte/beat) on a 16-bit bus
    checks = checks + 1;
    if (last_rresp !== AXI_SLVERR) begin
      $display("[%0t] ERROR (narrow): rresp=0x%0x exp SLVERR(0x2)", $time, last_rresp);
      errors = errors + 1;
    end else $display("[%0t] narrow: arsize=0 -> SLVERR as required", $time);

    // ---- CR0 register write + read-back ----
    one = {}; one.push_back(TB_INIT_CR0);
    axi_write(HB_REG_CR0[ADDR_WIDTH-1:0], 1, 1'b1, AXI_INCR, one);
    axi_read (HB_REG_CR0[ADDR_WIDTH-1:0], 1, 1'b1, AXI_INCR);
    cr0_rb = cap[0];
    checks = checks + 1;
    if (cr0_rb !== TB_INIT_CR0) begin
      $display("[%0t] ERROR: CR0 readback 0x%04x exp 0x%04x", $time, cr0_rb, TB_INIT_CR0);
      errors = errors + 1;
    end else $display("[%0t] CR0 write+readback ok (0x%04x)", $time, cr0_rb);

    // ---- ID0 register read ----
    axi_read(HB_REG_ID0[ADDR_WIDTH-1:0], 1, 1'b1, AXI_INCR);
    id0_rb = cap[0];
    checks = checks + 1;
    if (id0_rb !== HB_ID0_RESET) begin
      $display("[%0t] ERROR: ID0 readback 0x%04x exp 0x%04x", $time, id0_rb, HB_ID0_RESET);
      errors = errors + 1;
    end else $display("[%0t] ID0 read ok (0x%04x)", $time, id0_rb);

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_axi done: %0d checks, %0d errors", $time, checks, errors);
    if (errors == 0) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_axi: %0d errors", errors);
    end
  end

  initial begin
    #2_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_axi: global timeout");
  end

endmodule
