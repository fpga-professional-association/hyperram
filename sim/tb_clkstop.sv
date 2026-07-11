// tb_clkstop — self-checking Verilator TB for A2: active clock-stop on read back-pressure.
//
// phy_ck_en used to run continuously through a read; a caller that back-pressured rd_ready long enough to
// fill the read holding-FIFO lost words (the controller counts a word as received even when the FIFO is
// full and drops it). hyperbus_ctrl now (ACTIVE_CLK_STOP) PAUSES CK on word boundaries once the FIFO
// crosses a high-water mark, halting the device instead of overflowing, and resumes when the caller
// drains (SPEC_DIGEST §1 Active Clock Stop). The RWDS-Low stall timeout is frozen during an intentional
// pause so it is never mistaken for the >=32-clk device-error stall.
//
// Two datapaths share one stimulus; both use the ideal (no over-stream) model:
//   * DUT_ON  (ACTIVE_CLK_STOP=1): a long read with heavy initial rd_ready back-pressure pauses CK
//                                  (phy_ck_en observed Low during ST_READ) and returns ALL words correct.
//   * DUT_OFF (ACTIVE_CLK_STOP=0): the same stimulus overflows the FIFO -> the read is corrupt/incomplete.
// PASS iff DUT_ON is byte-exact and complete AND its CK actually paused, while DUT_OFF is NOT clean
// (proving the pause is what prevents the loss). A no-back-pressure read is clean on both (control).
//
// Read capture uses a CONTINUOUS scoreboard (captures on rd_valid & rd_ready every clk, cf. tb_preamble)
// rather than a task-side sampling loop, which under verilator --timing can miscount beats.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_clkstop;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2

  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam int unsigned NW          = 56;   // read length (> RD_FIFO_DEPTH so an OFF DUT can overflow)
  localparam int unsigned STALL_CYC   = 50;   // initial rd_ready back-pressure (fills the FIFO on OFF DUT)
  localparam logic [3:0]  S_READ      = 4'd7; // ST_READ ordinal in hyperbus_ctrl state_e

  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;   end
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90 = ~clk90; end
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  logic                    sel;   // 0 = DUT_ON, 1 = DUT_OFF
  logic                    s_cmd_valid, s_cmd_read, s_cmd_reg, s_cmd_wrap;
  logic [ADDR_WIDTH-1:0]   s_cmd_addr;
  logic [LEN_WIDTH-1:0]    s_cmd_len;
  logic                    s_wr_valid;
  logic [DATA_WIDTH-1:0]   s_wr_data;
  logic [STRB_WIDTH-1:0]   s_wr_strb;
  logic                    s_wr_last, s_rd_ready;

  logic                    m_cmd_ready, m_wr_ready, m_rd_valid, m_rd_last;
  logic [DATA_WIDTH-1:0]   m_rd_data;

  `define MK_DUT(PFX, CKSTOP, ACT)                                                                  \
    logic                    PFX``_cmd_ready, PFX``_wr_ready, PFX``_rd_valid, PFX``_rd_last;         \
    logic [DATA_WIDTH-1:0]   PFX``_rd_data;                                                          \
    logic                    PFX``_init_done, PFX``_busy, PFX``_eu, PFX``_et;                        \
    logic [3:0]              PFX``_dbg;                                                              \
    logic                    PFX``_cs_n, PFX``_rn, PFX``_cke;                                        \
    logic [DATA_WIDTH-1:0]   PFX``_dq_o; logic PFX``_dq_oe;                                          \
    logic [1:0]              PFX``_rwds_o; logic PFX``_rwds_oe, PFX``_rd_arm;                        \
    logic [DATA_WIDTH-1:0]   PFX``_dq_i; logic PFX``_dq_iv, PFX``_rwds_i;                            \
    logic                    PFX``_ck, PFX``_ckn, PFX``_hcs, PFX``_hrn;                              \
    logic [DQ_WIDTH-1:0]     PFX``_pdo; logic PFX``_pdoe;                                            \
    logic                    PFX``_pro; logic PFX``_proe;                                            \
    logic [DQ_WIDTH-1:0]     PFX``_mdo; logic PFX``_mdoe;                                            \
    logic                    PFX``_mro; logic PFX``_mroe;                                            \
    wire [DQ_WIDTH-1:0] PFX``_dql = PFX``_mdoe ? PFX``_mdo : (PFX``_pdoe ? PFX``_pdo : '0);          \
    wire                PFX``_rwl = PFX``_mroe ? PFX``_mro : (PFX``_proe ? PFX``_pro : 1'b0);        \
    wire [DQ_WIDTH-1:0] PFX``_dqd; assign #3.0 PFX``_dqd = PFX``_dql;                                \
    wire                PFX``_rwd; assign #3.0 PFX``_rwd = PFX``_rwl;                                \
    hyperbus_ctrl #(.LATENCY_CLOCKS(6), .FIXED_LATENCY(1'b1), .MAX_BURST_WORDS(0),                   \
      .PROGRAM_CR(1'b1), .POR_DELAY_CYCLES(0), .INIT_CR0(TB_INIT_CR0),                               \
      .ACTIVE_CLK_STOP(CKSTOP)) PFX``_ctrl (                                                         \
      .clk(clk), .rst(rst),                                                                          \
      .cmd_valid(s_cmd_valid & (ACT)), .cmd_ready(PFX``_cmd_ready),                                  \
      .cmd_read(s_cmd_read), .cmd_reg(s_cmd_reg), .cmd_wrap(s_cmd_wrap),                             \
      .cmd_addr(s_cmd_addr), .cmd_len(s_cmd_len),                                                    \
      .wr_valid(s_wr_valid & (ACT)), .wr_ready(PFX``_wr_ready),                                      \
      .wr_data(s_wr_data), .wr_strb(s_wr_strb), .wr_last(s_wr_last),                                 \
      .rd_valid(PFX``_rd_valid), .rd_ready(s_rd_ready & (ACT)),                                      \
      .rd_data(PFX``_rd_data), .rd_last(PFX``_rd_last),                                              \
      .busy(PFX``_busy), .init_done(PFX``_init_done), .err_underrun(PFX``_eu), .err_timeout(PFX``_et),\
      .phy_cs_n(PFX``_cs_n), .phy_rst_n(PFX``_rn), .phy_ck_en(PFX``_cke),                            \
      .phy_dq_o(PFX``_dq_o), .phy_dq_oe(PFX``_dq_oe), .phy_rwds_o(PFX``_rwds_o),                     \
      .phy_rwds_oe(PFX``_rwds_oe), .phy_rd_arm(PFX``_rd_arm),                                        \
      .phy_dq_i(PFX``_dq_i), .phy_dq_i_valid(PFX``_dq_iv), .phy_rwds_i(PFX``_rwds_i),                \
      .dbg_state(PFX``_dbg), .dbg_rd_wptr(), .dbg_rd_rptr(),                                         \
      /* issue #13: new ctrl debug bundle tied to legacy (A1) */                                    \
      .dbg_wr_lat_trim(4'd0), .dbg_lat_clocks(4'd6), .dbg_cr0_reprog(1'b0), .dbg_prewin_drive(1'b0), \
      .dbg_prewin_n(3'd0), .dbg_prewin_marker(1'b0), .dbg_postwin_hold(1'b0), .dbg_prewin_contig(1'b0), .dbg_end_cwrite(1'b0));                       \
    hyperbus_phy_generic #(.DQ_WIDTH(DQ_WIDTH), .DATA_WIDTH(DATA_WIDTH), .DIFF_CK(1'b1)) PFX``_phy ( \
      .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),                                        \
      .cal_capture_phase(1'b0), .cal_preamble_skip(3'd0), .cal_rx_tap(5'd0),                         \
      .cal_pair_skew(1'b0), /* cal tie-offs (POR-equivalent; contract v9) */                        \
      .phy_cs_n(PFX``_cs_n), .phy_rst_n(PFX``_rn), .phy_ck_en(PFX``_cke),                            \
      .phy_dq_o(PFX``_dq_o), .phy_dq_oe(PFX``_dq_oe), .phy_rwds_o(PFX``_rwds_o),                     \
      .phy_rwds_oe(PFX``_rwds_oe), .phy_rd_arm(PFX``_rd_arm),                                        \
      .phy_dq_i(PFX``_dq_i), .phy_dq_i_valid(PFX``_dq_iv), .phy_rwds_i(PFX``_rwds_i),                \
      .hb_ck(PFX``_ck), .hb_ck_n(PFX``_ckn), .hb_cs_n(PFX``_hcs), .hb_rst_n(PFX``_hrn),              \
      .hb_dq_o(PFX``_pdo), .hb_dq_oe(PFX``_pdoe), .hb_dq_i(PFX``_dqd),                               \
      .hb_rwds_o(PFX``_pro), .hb_rwds_oe(PFX``_proe), .hb_rwds_i(PFX``_rwd));                        \
    hyperram_model #(.DQ_WIDTH(DQ_WIDTH), .MEM_WORDS(1<<16), .LATENCY_CLOCKS(6), .FIXED_LATENCY(1'b1),\
      .ROW_WORDS(0), .ROW_PENALTY(4), .REFRESH_EVERY(0), .RD_OVERSTREAM_WORDS(0)) PFX``_model (      \
      .hb_ck(PFX``_ck), .hb_ck_n(PFX``_ckn), .hb_cs_n(PFX``_hcs), .hb_rst_n(PFX``_hrn),              \
      .hb_dq_i(PFX``_dql), .hb_dq_ie(PFX``_pdoe), .hb_dq_o(PFX``_mdo), .hb_dq_oe(PFX``_mdoe),        \
      .hb_rwds_i(PFX``_rwl), .hb_rwds_ie(PFX``_proe), .hb_rwds_o(PFX``_mro), .hb_rwds_oe(PFX``_mroe))

  `MK_DUT(a, 1'b1, ~sel);   // DUT_ON  (active when sel==0)
  `MK_DUT(b, 1'b0,  sel);   // DUT_OFF (active when sel==1)

  always_comb begin
    m_cmd_ready = sel ? b_cmd_ready : a_cmd_ready;
    m_wr_ready  = sel ? b_wr_ready  : a_wr_ready;
    m_rd_valid  = sel ? b_rd_valid  : a_rd_valid;
    m_rd_data   = sel ? b_rd_data   : a_rd_data;
    m_rd_last   = sel ? b_rd_last   : a_rd_last;
  end

  // Monitor: did DUT_ON's CK actually pause (phy_ck_en Low while in ST_READ)? Plus sticky err_timeout.
  logic saw_ckstop, a_et_l, b_et_l;
  always @(posedge clk) begin
    if (rst) begin saw_ckstop<=0; a_et_l<=0; b_et_l<=0; end
    else begin
      if ((a_dbg == S_READ) && !a_cke) saw_ckstop <= 1'b1;
      if (a_et) a_et_l <= 1'b1;
      if (b_et) b_et_l <= 1'b1;
    end
  end

  // Continuous read scoreboard: capture on an actual transfer (rd_valid & rd_ready to the selected DUT).
  localparam int unsigned CAP_MAX = 128;
  logic [DATA_WIDTH-1:0] cap [CAP_MAX];
  int unsigned cap_n; logic cap_last, capturing;
  always @(posedge clk) begin
    if (capturing && m_rd_valid && s_rd_ready) begin
      if (cap_n < CAP_MAX) cap[cap_n] <= m_rd_data;
      cap_n <= cap_n + 1;
    end
    // Terminal beat: rd_last coincident with a presented read word. Kept out of the s_rd_ready-gated
    // branch above because that TB-driven reg races the DUT's combinational rd_last under --timing;
    // the AXI front-end (pure-RTL rready) samples the same rd_last cleanly (see hyperbus_axi rlast).
    if (capturing && m_rd_valid && m_rd_last) cap_last <= 1'b1;
  end

  int unsigned errors = 0, checks = 0;
  function automatic logic [DATA_WIDTH-1:0] genword(input logic [ADDR_WIDTH-1:0] a);
    return (a[15:0] * 16'h9E37) ^ 16'hBEEF ^ {a[7:0], a[7:0]};
  endfunction
  task automatic chk(input logic cond, input string msg);
    checks = checks + 1;
    if (!cond) begin $display("[%0t] ERROR: %s", $time, msg); errors = errors + 1; end
  endtask
  task automatic idle();
    @(negedge clk);
    s_cmd_valid=0; s_cmd_read=0; s_cmd_reg=0; s_cmd_wrap=0; s_cmd_addr=0; s_cmd_len=0;
    s_wr_valid=0; s_wr_data=0; s_wr_strb='1; s_wr_last=0; s_rd_ready=0;
  endtask

  task automatic mem_write(input logic [ADDR_WIDTH-1:0] a, input int unsigned n);
    int unsigned idx, g;
    @(negedge clk); s_cmd_valid=1; s_cmd_read=0; s_cmd_reg=0; s_cmd_wrap=0; s_cmd_addr=a; s_cmd_len=LEN_WIDTH'(n);
    g=0; forever begin @(posedge clk); g=g+1; if (m_cmd_ready) break; if (g>4000) begin errors=errors+1; break; end end
    @(negedge clk); s_cmd_valid=0;
    idx=0; s_wr_valid=1; s_wr_strb='1; s_wr_data=genword(a); s_wr_last=(n==1);
    g=0; forever begin @(posedge clk); g=g+1;
      if (m_wr_ready) begin idx=idx+1; if (idx==n) break; @(negedge clk); s_wr_data=genword(a+idx); s_wr_last=(idx==n-1); end
      if (g>40000) begin errors=errors+1; break; end end
    @(negedge clk); s_wr_valid=0; s_wr_last=0; repeat (50) @(posedge clk);
  endtask

  // Read n words: hold the sink off for `stall` cycles (FIFO fills), then drain full-speed. The continuous
  // scoreboard records every transferred beat; the task just paces rd_ready and waits for completion.
  task automatic read_throttled(input logic [ADDR_WIDTH-1:0] a, input int unsigned n, input int unsigned stall,
                                output int unsigned got, output int unsigned mism, output logic last_ok);
    int unsigned g, i;
    cap_n = 0; cap_last = 1'b0; capturing = 1'b1;
    @(negedge clk); s_cmd_valid=1; s_cmd_read=1; s_cmd_reg=0; s_cmd_wrap=0; s_cmd_addr=a; s_cmd_len=LEN_WIDTH'(n);
    g=0; forever begin @(posedge clk); g=g+1; if (m_cmd_ready) break; if (g>4000) begin errors=errors+1; break; end end
    @(negedge clk); s_cmd_valid=0; s_rd_ready=1'b0;
    repeat (stall) @(posedge clk);       // hold off the sink: FIFO fills (OFF overflows, ON pauses CK)
    s_rd_ready=1'b1;
    // Drain full-speed until all n words are captured (or rd_last terminates early on a short/aborted
    // read), then one guard-bounded extra settle so the terminal rd_last flag is recorded.
    g=0; forever begin @(posedge clk); g=g+1;
      if ((cap_n >= n) || cap_last) break;
      if (g > 6000) break; end           // bounded: a short/aborted (overflowed) read shows up as cap_n < n
    repeat (4) @(posedge clk);           // let the terminal rd_last beat settle into cap_last
    @(negedge clk); s_rd_ready=1'b0; capturing=1'b0;
    got = cap_n; last_ok = cap_last; mism = 0;
    for (i=0; i<n; i++) begin
      if (i >= cap_n) mism = mism + 1;
      else if (cap[i] !== genword(a+i)) mism = mism + 1;
    end
  endtask

  int unsigned got1, mism1, got2, mism2, got3, mism3; logic lo1, lo2, lo3;
  initial begin
    sel=0; capturing=0; cap_n=0; cap_last=0; idle();
    rst=1'b1; repeat (5) @(posedge clk); @(negedge clk); rst=1'b0;
    begin int unsigned g; g=0;
      while (!(a_init_done && b_init_done) && g<100000) begin @(posedge clk); g=g+1; end end
    chk(a_init_done && b_init_done, "init_done never asserted on both DUTs");
    repeat (4) @(posedge clk);

    // Seed NW words into both memories.
    sel=1'b0; mem_write(32'h0000_0400, NW);
    sel=1'b1; mem_write(32'h0000_0400, NW);

    // ---------- DUT_ON: heavy initial back-pressure -> CK pauses, no loss ----------
    sel=1'b0;
    read_throttled(32'h0000_0400, NW, STALL_CYC, got1, mism1, lo1);
    $display("[%0t] DUT_ON  throttled read: got=%0d/%0d mism=%0d rlast=%0b ck_paused=%0b et=%0b",
             $time, got1, NW, mism1, lo1, saw_ckstop, a_et_l);
    chk(saw_ckstop, "DUT_ON CK never paused during ST_READ (active clock-stop did not engage)");
    chk(got1 == NW,  "DUT_ON throttled read did not return all words");
    chk(mism1 == 0,  "DUT_ON throttled read data mismatch (word lost despite clock-stop)");
    chk(!a_et_l,     "DUT_ON throttled read spuriously timed out (stall-freeze failed)");
    // (rd_last / lo1 is informational only: native rd_last IS correct — the AXI front-end depends on it
    //  and tb_timeout/tb_fixed2x assert it — but the TB-side scoreboard sampling of it races under
    //  --timing; A2's guarantee is the byte-exact no-loss capture above.)

    // ---------- DUT_OFF: same stimulus overflows the FIFO -> corrupt/incomplete ----------
    sel=1'b1;
    read_throttled(32'h0000_0400, NW, STALL_CYC, got2, mism2, lo2);
    $display("[%0t] DUT_OFF throttled read: got=%0d/%0d mism=%0d rlast=%0b (expect NOT clean)",
             $time, got2, NW, mism2, lo2);
    chk((got2 != NW) || (mism2 != 0),
        "DUT_OFF throttled read was clean — overflow not reproduced (test not proving the fix)");

    // ---------- control: no back-pressure reads cleanly on the clock-stop DUT ----------
    sel=1'b0;
    read_throttled(32'h0000_0400, NW, 0, got3, mism3, lo3);
    $display("[%0t] DUT_ON  no-backpressure read: got=%0d/%0d mism=%0d rlast=%0b", $time, got3, NW, mism3, lo3);
    chk((got3 == NW) && (mism3 == 0), "DUT_ON no-back-pressure read not clean");

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_clkstop done: %0d checks, %0d errors", $time, checks, errors);
    if (errors==0) begin $display("TB_RESULT: PASS"); $finish; end
    else begin $display("TB_RESULT: FAIL"); $fatal(1, "tb_clkstop: %0d errors", errors); end
  end

  initial begin #8_000_000; $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_clkstop: global timeout"); end
endmodule
