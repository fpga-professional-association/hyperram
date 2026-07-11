// tb_cr1init — self-checking Verilator TB for A3: optional CR1 programming at init.
//
// hyperbus_ctrl's POR init used to write ONLY CR0, leaving CR1 (distributed-refresh interval / PASR /
// hybrid-sleep on real HyperRAM) at its reset default — a tCSM/refresh risk. The controller now takes
// PROGRAM_CR1 + INIT_CR1 and, when enabled, issues a SECOND zero-latency register write of CR1 after
// the CR0 write, before init_done (SPEC_DIGEST §5/§8.2).
//
// Two controller+PHY+model datapaths share one native-command stimulus (sel-mux idiom, cf. tb_preamble):
//   * DUT_ON  : PROGRAM_CR1=1, INIT_CR1=0xA55A  -> CR1 must read back 0xA55A (the programmed image)
//   * DUT_OFF : PROGRAM_CR1=0                    -> CR1 must stay at its reset default (0x0000)
// Both also program CR0=0x8F1F; reading CR0 back on each proves the register-read path and that the CR0
// write still happens. A memory write/read on each proves the (now two-write) init didn't break traffic.
// PASS iff CR1(on)=image, CR1(off)=reset, CR0 correct on both, and memory read-back is byte-exact.
//
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps
module tb_cr1init;
  import hyperbus_pkg::*;

  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;   // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;         // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;  // 16
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;        // 2

  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam logic [15:0] TB_INIT_CR1 = 16'hA55A;                                        // distinct, non-reset

  logic clk, clk90, clk_ref, rst;
  initial begin clk    = 1'b0; forever #5.0 clk    = ~clk;   end
  initial begin #2.5; clk90 = 1'b0; forever #5.0 clk90 = ~clk90; end
  initial begin clk_ref = 1'b0; forever #2.5 clk_ref = ~clk_ref; end

  // --------------------------------------------------------------------
  // Shared native-command stimulus, fanned to both DUTs (active one picked by `sel`).
  // --------------------------------------------------------------------
  logic                    sel;   // 0 = DUT_ON, 1 = DUT_OFF
  logic                    s_cmd_valid, s_cmd_read, s_cmd_reg, s_cmd_wrap;
  logic [ADDR_WIDTH-1:0]   s_cmd_addr;
  logic [LEN_WIDTH-1:0]    s_cmd_len;
  logic                    s_wr_valid;
  logic [DATA_WIDTH-1:0]   s_wr_data;
  logic [STRB_WIDTH-1:0]   s_wr_strb;
  logic                    s_wr_last, s_rd_ready;

  // Muxed observation.
  logic                    m_cmd_ready, m_wr_ready, m_rd_valid, m_rd_last, m_init_done;
  logic [DATA_WIDTH-1:0]   m_rd_data;

  // ====================================================================
  //  Reusable one-datapath macro (ctrl + generic PHY + model), instantiated twice with PROGRAM_CR1.
  // ====================================================================
  `define MK_DUT(PFX, EN_CR1, ACT)                                                                  \
    logic                    PFX``_cmd_ready, PFX``_wr_ready, PFX``_rd_valid, PFX``_rd_last;         \
    logic [DATA_WIDTH-1:0]   PFX``_rd_data;                                                          \
    logic                    PFX``_init_done, PFX``_busy, PFX``_eu, PFX``_et;                        \
    logic                    PFX``_cs_n, PFX``_rn, PFX``_cke;                                        \
    logic [DATA_WIDTH-1:0]   PFX``_dq_o; logic PFX``_dq_oe;                                          \
    logic [1:0]              PFX``_rwds_o; logic PFX``_rwds_oe, PFX``_rd_arm;                        \
    logic [DATA_WIDTH-1:0]   PFX``_dq_i; logic PFX``_dq_iv, PFX``_rwds_i;                            \
    logic                    PFX``_ck, PFX``_ckn, PFX``_hcs, PFX``_hrn;                              \
    logic [DQ_WIDTH-1:0]     PFX``_pdo; logic PFX``_pdoe;                                            \
    logic                    PFX``_pro; logic PFX``_proe;                                            \
    logic [DQ_WIDTH-1:0]     PFX``_mdo; logic PFX``_mdoe;                                            \
    logic                    PFX``_mro; logic PFX``_mroe;                                            \
    wire [DQ_WIDTH-1:0] PFX``_dql   = PFX``_mdoe ? PFX``_mdo : (PFX``_pdoe ? PFX``_pdo : '0);        \
    wire                PFX``_rwl   = PFX``_mroe ? PFX``_mro : (PFX``_proe ? PFX``_pro : 1'b0);      \
    wire [DQ_WIDTH-1:0] PFX``_dqd;  assign #3.0 PFX``_dqd = PFX``_dql;                               \
    wire                PFX``_rwd;  assign #3.0 PFX``_rwd = PFX``_rwl;                               \
    hyperbus_ctrl #(.LATENCY_CLOCKS(6), .FIXED_LATENCY(1'b1), .MAX_BURST_WORDS(0),                   \
      .PROGRAM_CR(1'b1), .POR_DELAY_CYCLES(0), .INIT_CR0(TB_INIT_CR0),                               \
      .PROGRAM_CR1(EN_CR1), .INIT_CR1(TB_INIT_CR1)) PFX``_ctrl (                                     \
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
      .dbg_state(), .dbg_rd_wptr(), .dbg_rd_rptr(),                                                  \
      /* issue #13: new ctrl debug bundle tied to legacy (A1) */                                    \
      .dbg_wr_lat_trim(4'd0), .dbg_lat_clocks(4'd6), .dbg_cr0_reprog(1'b0), .dbg_prewin_drive(1'b0), \
      .dbg_prewin_n(3'd0), .dbg_prewin_marker(1'b0), .dbg_postwin_hold(1'b0), .dbg_prewin_contig(1'b0), .dbg_end_cread(1'b0));                       \
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
      .ROW_WORDS(0), .ROW_PENALTY(4), .REFRESH_EVERY(0)) PFX``_model (                               \
      .hb_ck(PFX``_ck), .hb_ck_n(PFX``_ckn), .hb_cs_n(PFX``_hcs), .hb_rst_n(PFX``_hrn),              \
      .hb_dq_i(PFX``_dql), .hb_dq_ie(PFX``_pdoe), .hb_dq_o(PFX``_mdo), .hb_dq_oe(PFX``_mdoe),        \
      .hb_rwds_i(PFX``_rwl), .hb_rwds_ie(PFX``_proe), .hb_rwds_o(PFX``_mro), .hb_rwds_oe(PFX``_mroe))

  `MK_DUT(a, 1'b1, ~sel);   // DUT_ON  (active when sel==0): CR1 programmed
  `MK_DUT(b, 1'b0,  sel);   // DUT_OFF (active when sel==1): CR1 left at reset

  always_comb begin
    m_cmd_ready = sel ? b_cmd_ready : a_cmd_ready;
    m_wr_ready  = sel ? b_wr_ready  : a_wr_ready;
    m_rd_valid  = sel ? b_rd_valid  : a_rd_valid;
    m_rd_data   = sel ? b_rd_data   : a_rd_data;
    m_rd_last   = sel ? b_rd_last   : a_rd_last;
    m_init_done = sel ? b_init_done : a_init_done;
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

  // Register-space read of a single word (returns the value).
  task automatic reg_read(input logic [ADDR_WIDTH-1:0] a, output logic [DATA_WIDTH-1:0] d);
    int unsigned g;
    @(negedge clk); s_cmd_valid=1; s_cmd_read=1; s_cmd_reg=1; s_cmd_wrap=0; s_cmd_addr=a; s_cmd_len=LEN_WIDTH'(1);
    g=0; forever begin @(posedge clk); g=g+1; if (m_cmd_ready) break; if (g>3000) begin errors=errors+1; break; end end
    @(negedge clk); s_cmd_valid=0; s_rd_ready=1;
    d='0; g=0; forever begin @(posedge clk); g=g+1; if (m_rd_valid) begin d=m_rd_data; break; end
      if (g>4000) begin $display("[%0t] reg_read timeout @0x%08x", $time, a); errors=errors+1; break; end end
    @(negedge clk); s_rd_ready=0;
  endtask

  task automatic mem_write1(input logic [ADDR_WIDTH-1:0] a, input logic [DATA_WIDTH-1:0] d);
    int unsigned g;
    @(negedge clk); s_cmd_valid=1; s_cmd_read=0; s_cmd_reg=0; s_cmd_wrap=0; s_cmd_addr=a; s_cmd_len=LEN_WIDTH'(1);
    g=0; forever begin @(posedge clk); g=g+1; if (m_cmd_ready) break; if (g>3000) begin errors=errors+1; break; end end
    @(negedge clk); s_cmd_valid=0; s_wr_valid=1; s_wr_data=d; s_wr_strb='1; s_wr_last=1;
    g=0; forever begin @(posedge clk); g=g+1; if (m_wr_ready) break; if (g>3000) begin errors=errors+1; break; end end
    @(negedge clk); s_wr_valid=0; s_wr_last=0; repeat (30) @(posedge clk);
  endtask

  task automatic mem_read1(input logic [ADDR_WIDTH-1:0] a, output logic [DATA_WIDTH-1:0] d);
    int unsigned g;
    @(negedge clk); s_cmd_valid=1; s_cmd_read=1; s_cmd_reg=0; s_cmd_wrap=0; s_cmd_addr=a; s_cmd_len=LEN_WIDTH'(1);
    g=0; forever begin @(posedge clk); g=g+1; if (m_cmd_ready) break; if (g>3000) begin errors=errors+1; break; end end
    @(negedge clk); s_cmd_valid=0; s_rd_ready=1;
    d='0; g=0; forever begin @(posedge clk); g=g+1; if (m_rd_valid) begin d=m_rd_data; break; end
      if (g>4000) begin errors=errors+1; break; end end
    @(negedge clk); s_rd_ready=0;
  endtask

  logic [DATA_WIDTH-1:0] cr0v, cr1v, mv;
  initial begin
    sel=0; idle();
    rst=1'b1; repeat (5) @(posedge clk); @(negedge clk); rst=1'b0;
    begin int unsigned g; g=0;
      while (!(a_init_done && b_init_done) && g<100000) begin @(posedge clk); g=g+1; end end
    chk(a_init_done && b_init_done, "init_done never asserted on both DUTs");
    repeat (4) @(posedge clk);

    // ---- DUT_ON (CR1 programmed) ----
    sel = 1'b0;
    reg_read(HB_REG_CR0, cr0v);
    reg_read(HB_REG_CR1, cr1v);
    $display("[%0t] DUT_ON : CR0=0x%04x (exp 0x%04x)  CR1=0x%04x (exp 0x%04x)",
             $time, cr0v, TB_INIT_CR0, cr1v, TB_INIT_CR1);
    chk(cr0v === TB_INIT_CR0, "DUT_ON CR0 not programmed");
    chk(cr1v === TB_INIT_CR1, "DUT_ON CR1 not programmed to INIT_CR1");
    chk(cr1v !== HB_CR1_RESET, "DUT_ON CR1 still at reset (CR1 write did not happen)");
    mem_write1(32'h0000_0055, genword(32'h55));
    mem_read1 (32'h0000_0055, mv);
    chk(mv === genword(32'h55), "DUT_ON memory read-back mismatch");

    // ---- DUT_OFF (CR1 not programmed) ----
    sel = 1'b1;
    reg_read(HB_REG_CR0, cr0v);
    reg_read(HB_REG_CR1, cr1v);
    $display("[%0t] DUT_OFF: CR0=0x%04x (exp 0x%04x)  CR1=0x%04x (exp reset 0x%04x)",
             $time, cr0v, TB_INIT_CR0, cr1v, HB_CR1_RESET);
    chk(cr0v === TB_INIT_CR0, "DUT_OFF CR0 not programmed");
    chk(cr1v === HB_CR1_RESET, "DUT_OFF CR1 changed despite PROGRAM_CR1=0");
    mem_write1(32'h0000_0077, genword(32'h77));
    mem_read1 (32'h0000_0077, mv);
    chk(mv === genword(32'h77), "DUT_OFF memory read-back mismatch");

    repeat (8) @(posedge clk);
    $display("==================================================================");
    $display("[%0t] tb_cr1init done: %0d checks, %0d errors", $time, checks, errors);
    if (errors==0) begin $display("TB_RESULT: PASS"); $finish; end
    else begin $display("TB_RESULT: FAIL"); $fatal(1, "tb_cr1init: %0d errors", errors); end
  end

  initial begin #3_000_000; $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_cr1init: global timeout"); end
endmodule
