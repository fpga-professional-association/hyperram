// tb_masked — self-checking Verilator TB for byte-masked writes (issue #4, B3; SPEC_DIGEST §4) and
// the write-underrun path (issue #4, B6; hyperbus_ctrl:530 err_underrun + rwds=2'b11 mask-both).
//
// Every other TB writes full-word strobes (wstrb/byteenable all-ones), so the controller's per-byte
// write mask (phy_rwds_o = {~wsrc_strb[1], ~wsrc_strb[0]}) and the model's read-modify-merge are
// unverified. This TB drives the controller natively so it can present arbitrary wr_strb and can
// starve wr_valid mid-burst:
//   * B3: pre-fill a word full, then write it again with a partial strobe; read back and verify the
//     enabled byte took the new value while the masked byte kept the old one — for every strobe combo
//     (00/01/10/11) and a multi-word burst with mixed per-beat strobes.
//   * B6: start a write burst, deliver only the first word, then drop wr_valid. The controller must
//     pulse err_underrun and mask every starved beat (array unchanged) while the burst still
//     completes; read-back confirms only the delivered word changed.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_masked;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2
  localparam int unsigned PHYW       = 2 * DQ_WIDTH;          // 16

  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F

  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;    end
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90 = ~clk90;  end
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  // Native channels
  logic                    cmd_valid, cmd_ready, cmd_read, cmd_reg, cmd_wrap;
  logic [ADDR_WIDTH-1:0]   cmd_addr;
  logic [LEN_WIDTH-1:0]    cmd_len;
  logic                    wr_valid, wr_ready, wr_last;
  logic [DATA_WIDTH-1:0]   wr_data;
  logic [STRB_WIDTH-1:0]   wr_strb;
  logic                    rd_valid, rd_ready, rd_last;
  logic [DATA_WIDTH-1:0]   rd_data;
  logic                    busy, init_done, err_underrun, err_timeout;

  // ctrl <-> phy
  logic                    phy_cs_n, phy_rst_n, phy_ck_en, phy_dq_oe, phy_rwds_oe, phy_rd_arm;
  logic [PHYW-1:0]         phy_dq_o, phy_dq_i;
  logic [1:0]              phy_rwds_o;
  logic                    phy_dq_i_valid, phy_rwds_i;

  // device pins + bus resolution
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  hbp_dq_o;   logic hbp_dq_oe;
  logic                 hbp_rwds_o; logic hbp_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;
  logic                 mdl_rwds_o; logic mdl_rwds_oe;
  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (hbp_dq_oe   ? hbp_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (hbp_rwds_oe ? hbp_rwds_o : 1'b0);
  localparam realtime RTT = 3.0;
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  hyperbus_ctrl #(
    .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1), .MAX_BURST_WORDS (0),
    .PROGRAM_CR (1'b1), .POR_DELAY_CYCLES (0), .INIT_CR0 (TB_INIT_CR0)
  ) u_ctrl (
    .clk (clk), .rst (rst),
    .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_read (cmd_read), .cmd_reg (cmd_reg),
    .cmd_wrap (cmd_wrap), .cmd_addr (cmd_addr), .cmd_len (cmd_len),
    .wr_valid (wr_valid), .wr_ready (wr_ready), .wr_data (wr_data), .wr_strb (wr_strb),
    .wr_last (wr_last),
    .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_data (rd_data), .rd_last (rd_last),
    .busy (busy), .init_done (init_done), .err_underrun (err_underrun), .err_timeout (err_timeout),
    .phy_cs_n (phy_cs_n), .phy_rst_n (phy_rst_n), .phy_ck_en (phy_ck_en),
    .phy_dq_o (phy_dq_o), .phy_dq_oe (phy_dq_oe), .phy_rwds_o (phy_rwds_o),
    .phy_rwds_oe (phy_rwds_oe), .phy_rd_arm (phy_rd_arm),
    .phy_dq_i (phy_dq_i), .phy_dq_i_valid (phy_dq_i_valid), .phy_rwds_i (phy_rwds_i),
    .dbg_state (/* unused */), .dbg_rd_wptr (/* unused */), .dbg_rd_rptr (/* unused */),
    // issue #13: new hyperbus_ctrl debug bundle tied to per-instance legacy (A1; no wrap_en on ctrl).
    .dbg_wr_lat_trim (4'd0), .dbg_lat_clocks (4'd6), .dbg_cr0_reprog (1'b0),
    .dbg_prewin_drive (1'b0), .dbg_prewin_n (3'd0), .dbg_prewin_marker (1'b0), .dbg_postwin_hold (1'b0)
  );

  hyperbus_phy #(.PHY_VARIANT ("GENERIC"), .DIFF_CK (1'b1)) u_phy (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    // runtime read-eye calibration tied to POR-equivalent constants (reproduces pre-cal behaviour)
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd0), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .phy_cs_n (phy_cs_n), .phy_rst_n (phy_rst_n), .phy_ck_en (phy_ck_en),
    .phy_dq_o (phy_dq_o), .phy_dq_oe (phy_dq_oe), .phy_rwds_o (phy_rwds_o),
    .phy_rwds_oe (phy_rwds_oe), .phy_rd_arm (phy_rd_arm),
    .phy_dq_i (phy_dq_i), .phy_dq_i_valid (phy_dq_i_valid), .phy_rwds_i (phy_rwds_i),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (hbp_dq_o), .hb_dq_oe (hbp_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (hbp_rwds_o), .hb_rwds_oe (hbp_rwds_oe), .hb_rwds_i (rwds_line_dly)
  );

  hyperram_model #(
    .DQ_WIDTH (DQ_WIDTH), .MEM_WORDS (1 << 16), .LATENCY_CLOCKS (6), .FIXED_LATENCY (1'b1),
    .ROW_WORDS (0), .ROW_PENALTY (4), .REFRESH_EVERY (0)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i (dq_line), .hb_dq_ie (hbp_dq_oe), .hb_dq_o (mdl_dq_o), .hb_dq_oe (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (hbp_rwds_oe), .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );

  // --------------------------------------------------------------------
  // Scoreboard + capture + sticky underrun counter
  // --------------------------------------------------------------------
  int unsigned errors = 0, checks = 0;
  int unsigned underrun_cnt = 0;
  localparam int unsigned CAP_MAX = 64;
  logic [DATA_WIDTH-1:0] cap [CAP_MAX];
  int unsigned           cap_n;
  logic                  capturing;

  always @(posedge clk) begin
    if (capturing && rd_valid && rd_ready) begin
      if (cap_n < CAP_MAX) cap[cap_n] <= rd_data;
      cap_n <= cap_n + 1;
    end
    if (err_underrun) underrun_cnt <= underrun_cnt + 1;
  end

  function automatic logic [DATA_WIDTH-1:0] genA(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction
  function automatic logic [DATA_WIDTH-1:0] genB(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h2545) ^ 16'h1234;
  endfunction
  // Expected word after writing `nw` over old `ow` under 2-bit strobe s (s[1]=byte A/high, s[0]=byte B/low).
  function automatic logic [DATA_WIDTH-1:0] merge(input logic [DATA_WIDTH-1:0] ow,
                                                  input logic [DATA_WIDTH-1:0] nw,
                                                  input logic [STRB_WIDTH-1:0] s);
    logic [DATA_WIDTH-1:0] r;
    r[DATA_WIDTH-1:DQ_WIDTH] = s[1] ? nw[DATA_WIDTH-1:DQ_WIDTH] : ow[DATA_WIDTH-1:DQ_WIDTH];
    r[DQ_WIDTH-1:0]          = s[0] ? nw[DQ_WIDTH-1:0]          : ow[DQ_WIDTH-1:0];
    return r;
  endfunction

  // --------------------------------------------------------------------
  // Native tasks
  // --------------------------------------------------------------------
  task automatic nat_idle();
    @(negedge clk);
    cmd_valid=1'b0; cmd_read=1'b0; cmd_reg=1'b0; cmd_wrap=1'b0; cmd_addr='0; cmd_len='0;
    wr_valid=1'b0; wr_data='0; wr_strb='1; wr_last=1'b0;
  endtask

  task automatic nat_cmd(input logic rd, input logic rg, input logic [ADDR_WIDTH-1:0] addr,
                         input int unsigned n);
    int unsigned g;
    @(negedge clk);
    cmd_valid=1'b1; cmd_read=rd; cmd_reg=rg; cmd_wrap=1'b0; cmd_addr=addr; cmd_len=LEN_WIDTH'(n);
    g=0;
    forever begin @(posedge clk); g=g+1; if (cmd_ready) break;
      if (g>20000) begin $display("[%0t] HANG nat_cmd @0x%08x", $time, addr); errors=errors+1; break; end end
    @(negedge clk); cmd_valid=1'b0;
  endtask

  // Write n words with a per-beat strobe queue (full stream, no starvation).
  task automatic nat_write_m(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                             input logic [DATA_WIDTH-1:0] data [$],
                             input logic [STRB_WIDTH-1:0] strb [$]);
    int unsigned idx, g;
    nat_cmd(1'b0, 1'b0, addr, n);
    idx=0;
    @(negedge clk); wr_valid=1'b1; wr_data=data[0]; wr_strb=strb[0]; wr_last=(n==1);
    g=0;
    forever begin @(posedge clk); g=g+1;
      if (g>8000) begin $display("[%0t] HANG nat_write_m @0x%08x idx=%0d/%0d", $time, addr, idx, n);
                        errors=errors+1; break; end
      if (wr_ready) begin idx=idx+1; if (idx==n) break;
        @(negedge clk); wr_data=data[idx]; wr_strb=strb[idx]; wr_last=(idx==n-1); end
    end
    @(negedge clk); wr_valid=1'b0; wr_last=1'b0; wr_strb='1;
  endtask

  // Convenience: full-strobe write.
  task automatic nat_write_full(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                                input logic [DATA_WIDTH-1:0] data [$]);
    logic [STRB_WIDTH-1:0] strb [$];
    int unsigned i;
    strb = {}; for (i=0;i<n;i++) strb.push_back('1);
    nat_write_m(addr, n, data, strb);
  endtask

  // Underrun: deliver only word0, then drop wr_valid so beats 1..n-1 are starved (masked + underrun).
  task automatic nat_write_starve(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n,
                                  input logic [DATA_WIDTH-1:0] word0);
    int unsigned g;
    nat_cmd(1'b0, 1'b0, addr, n);
    @(negedge clk); wr_valid=1'b1; wr_strb='1; wr_data=word0; wr_last=1'b0;
    g=0;
    forever begin @(posedge clk); g=g+1; if (wr_ready) break;
      if (g>8000) begin $display("[%0t] HANG starve accept @0x%08x", $time, addr); errors=errors+1; break; end end
    @(negedge clk); wr_valid=1'b0;         // starve the remaining beats
    g=0; while (busy && g<8000) begin @(posedge clk); g=g+1; end
    nat_idle();
  endtask

  task automatic nat_read(input logic [ADDR_WIDTH-1:0] addr, input int unsigned n);
    int unsigned g;
    nat_cmd(1'b1, 1'b0, addr, n);
    cap_n=0; capturing=1'b1;
    g=0; while (cap_n<n && g<20000) begin @(posedge clk); g=g+1; end
    @(posedge clk); capturing=1'b0;
    if (cap_n<n) begin $display("[%0t] ERROR: read %0d @0x%08x got %0d", $time, n, addr, cap_n);
                       errors=errors+1; end
  endtask

  // Single-word masked-write check: pre-fill A, masked-write B with strobe s, verify merge.
  task automatic masked_single(input logic [ADDR_WIDTH-1:0] a, input logic [STRB_WIDTH-1:0] s,
                               input string tag);
    logic [DATA_WIDTH-1:0] wa [$]; logic [DATA_WIDTH-1:0] wb [$]; logic [STRB_WIDTH-1:0] sb [$];
    logic [DATA_WIDTH-1:0] exp;
    wa = {}; wa.push_back(genA(a));
    nat_write_full(a, 1, wa);                   // pre-fill full (pattern A)
    wb = {}; wb.push_back(genB(a));
    sb = {}; sb.push_back(s);
    nat_write_m(a, 1, wb, sb);                  // masked overwrite (pattern B, strobe s)
    nat_read(a, 1);
    exp = merge(genA(a), genB(a), s);
    checks = checks + 1;
    if (cap[0] !== exp) begin
      $display("[%0t] ERROR (%s strb=%02b): @0x%08x got 0x%04x exp 0x%04x (old 0x%04x new 0x%04x)",
               $time, tag, s, a, cap[0], exp, genA(a), genB(a));
      errors = errors + 1;
    end else
      $display("[%0t] %s strb=%02b @0x%08x ok (0x%04x)", $time, tag, s, a, cap[0]);
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  int unsigned guard, i;
  logic [DATA_WIDTH-1:0] pre [$]; logic [DATA_WIDTH-1:0] nb [$]; logic [STRB_WIDTH-1:0] ms [$];
  int unsigned before_uc;
  initial begin
    nat_idle();
    rd_ready=1'b1; capturing=1'b0; cap_n=0;
    rst=1'b1;
    repeat (5) @(posedge clk); @(negedge clk); rst=1'b0;
    guard=0; while (!init_done && guard<100000) begin @(posedge clk); guard=guard+1; end
    if (!init_done) begin $display("[%0t] FATAL init_done", $time); errors=errors+1; end
    else $display("[%0t] init_done asserted (masked/underrun harness)", $time);
    repeat (4) @(posedge clk);

    // ---- B3: single-word masked writes, every strobe combo ----
    masked_single(32'h0000_0010, 2'b01, "byteB-only");  // low byte written, high kept
    masked_single(32'h0000_0011, 2'b10, "byteA-only");  // high byte written, low kept
    masked_single(32'h0000_0012, 2'b00, "both-masked"); // nothing written (stays pattern A)
    masked_single(32'h0000_0013, 2'b11, "full");        // both written (pattern B)

    // ---- B3: multi-word burst with mixed per-beat strobes ----
    pre = {}; for (i=0;i<6;i++) pre.push_back(genA(32'h0000_0040 + i));
    nat_write_full(32'h0000_0040, 6, pre);
    nb = {}; ms = {};
    nb.push_back(genB(32'h0000_0040+0)); ms.push_back(2'b11);
    nb.push_back(genB(32'h0000_0040+1)); ms.push_back(2'b01);
    nb.push_back(genB(32'h0000_0040+2)); ms.push_back(2'b10);
    nb.push_back(genB(32'h0000_0040+3)); ms.push_back(2'b00);
    nb.push_back(genB(32'h0000_0040+4)); ms.push_back(2'b11);
    nb.push_back(genB(32'h0000_0040+5)); ms.push_back(2'b01);
    nat_write_m(32'h0000_0040, 6, nb, ms);
    nat_read(32'h0000_0040, 6);
    for (i=0;i<6;i++) begin
      logic [DATA_WIDTH-1:0] exp;
      exp = merge(genA(32'h0000_0040+i), genB(32'h0000_0040+i), ms[i]);
      checks = checks + 1;
      if (cap[i] !== exp) begin
        $display("[%0t] ERROR (burst-mask): word %0d strb=%02b got 0x%04x exp 0x%04x",
                 $time, i, ms[i], cap[i], exp);
        errors = errors + 1;
      end
    end
    $display("[%0t] burst mixed-strobe write ok (errors so far %0d)", $time, errors);

    // ---- B6: write underrun — deliver only word0, starve the rest ----
    pre = {}; for (i=0;i<4;i++) pre.push_back(genA(32'h0000_0080 + i));
    nat_write_full(32'h0000_0080, 4, pre);       // pre-fill full (pattern A)
    before_uc = underrun_cnt;
    nat_write_starve(32'h0000_0080, 4, genB(32'h0000_0080));  // only word0 delivered
    nat_read(32'h0000_0080, 4);
    // word0 = new (delivered full); words 1..3 = old (masked by underrun)
    begin
      logic [DATA_WIDTH-1:0] exp0;
      exp0 = genB(32'h0000_0080);
      checks = checks + 1;
      if (cap[0] !== exp0) begin
        $display("[%0t] ERROR (underrun word0): got 0x%04x exp 0x%04x", $time, cap[0], exp0);
        errors = errors + 1;
      end
      for (i=1;i<4;i++) begin
        logic [DATA_WIDTH-1:0] expo;
        expo = genA(32'h0000_0080 + i);          // unchanged (starved beat masked)
        checks = checks + 1;
        if (cap[i] !== expo) begin
          $display("[%0t] ERROR (underrun word%0d): got 0x%04x exp OLD 0x%04x (starved beat not masked!)",
                   $time, i, cap[i], expo);
          errors = errors + 1;
        end
      end
    end
    checks = checks + 1;
    if (underrun_cnt <= before_uc) begin
      $display("[%0t] ERROR: err_underrun never pulsed during starved write (%0d)", $time, underrun_cnt);
      errors = errors + 1;
    end else
      $display("[%0t] err_underrun pulsed %0d time(s) during starved write; masked beats kept old data",
               $time, underrun_cnt - before_uc);

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_masked done: %0d checks, %0d errors", $time, checks, errors);
    if (errors==0) begin $display("TB_RESULT: PASS"); $finish; end
    else begin $display("TB_RESULT: FAIL"); $fatal(1, "tb_masked: %0d errors", errors); end
  end

  initial begin
    #4_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_masked: global timeout");
  end

endmodule
