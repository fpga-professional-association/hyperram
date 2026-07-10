// hyperram_avalon — TOP: Avalon-MM slave + HyperBus device pins.
//
// Normative boundary: docs/INTERFACES.md §hyperram_avalon (frozen). Pure structural wrapper:
// instantiates exactly three blocks and wires them strictly per INTERFACES.md —
//   hyperbus_avalon (Avalon-MM slave -> native master)
//   hyperbus_ctrl   (native slave -> PHY master, the protocol engine)
//   hyperbus_phy    (DDR SERDES + I/O; PHY_VARIANT="GENERIC" = inferrable-DDR, Verilator-clean)
// and exposes { clocking } U { full Avalon-MM slave } U { HyperBus device pins } U { init_done }.
//
// No functional logic lives here (DESIGN.md §1): the front-end owns bus translation, the ctrl
// owns all protocol semantics, the PHY owns serialization / CK / the RWDS->clk crossing.
//
// hyperbus_pkg is compiled into the design (build command line); it is imported below and its
// package-scoped constants/functions resolve at elaboration, so no textual `include is needed.
module hyperram_avalon
  import hyperbus_pkg::*;
#(
  // ---- common bus geometry ------------------------------------------------
  parameter int unsigned DQ_WIDTH          = HB_DQ_WIDTH_DEFAULT,        // HyperBus DQ pins
  parameter int unsigned DATA_WIDTH        = 2 * DQ_WIDTH,               // native word = 2*DQ
  parameter int unsigned ADDR_WIDTH        = HB_ADDR_WIDTH,              // word-address width
  parameter int unsigned LEN_WIDTH         = HB_LEN_WIDTH_DEFAULT,       // burst-length (words)
  // ---- protocol engine (hyperbus_ctrl) ------------------------------------
  parameter int unsigned LATENCY_CLOCKS    = HB_LATENCY_CLOCKS_DEFAULT,  // CA1->data, clocks
  parameter bit          FIXED_LATENCY     = HB_FIXED_LATENCY_DEFAULT,   // 1 = fixed (POR default)
  parameter int unsigned MAX_BURST_WORDS   = 0,                         // 0 = no chop; else tCSM/tCK
  parameter int unsigned BURST_BOUNDARY_WORDS = 0,                       // 0 = off; else chop at this
                                                                         //   WORD boundary (W957D8NB)
  parameter bit          WR_COMMIT_READ    = 1'b0,                       // interpose commit-read after
                                                                         //   each split memory write
  parameter int unsigned COMMIT_READ_WORDS = 4,                          // commit-read length (words)
  parameter              COMMIT_READ_MODE  = "SPAN_END",                 // SPAN_END|FULL_BURST|NEXT_ROW
  parameter bit          WR_COALESCE       = 1'b0,                       // CS#-coalescing (issue #1 #4)
  parameter int unsigned WR_COALESCE_WAIT  = 8,                          // cycles to await a splice cmd
  parameter bit          PROGRAM_CR        = 1'b1,                       // program CR0 at init
  parameter int unsigned POR_DELAY_CYCLES  = 0,                         // POR init delay (sim: 0)
  parameter logic [3:0]  INIT_LATENCY_CODE = hb_clocks_to_latency_code(LATENCY_CLOCKS),
  parameter logic [15:0] INIT_CR0          = HB_CR0_RESET,               // CR0 image written at init
  // ---- spec-feature options (hyperbus_ctrl; all default OFF = legacy behavior) -----------
  parameter bit          PROGRAM_CR1       = 1'b0,                       // A3: also program CR1 at init
  parameter logic [15:0] INIT_CR1          = HB_CR1_RESET,               // A3: CR1 image written at init
  parameter int unsigned CLK_FREQ_MHZ      = 0,                          // A4: CK freq (MHz); 0 = legacy POR
  parameter int unsigned T_RP_NS           = 200,                        // A4: tRP  RST# pulse   (>=200 ns)
  parameter int unsigned T_RPH_NS          = 400,                        // A4: tRPH RST#->CS#    (>=400 ns)
  parameter int unsigned T_RH_NS           = 200,                        // A4: tRH  RST#hi->CS#  (>=200 ns)
  parameter int unsigned T_VCS_US          = 150,                        // A4: tVCS VCC->access  (<=150 µs)
  parameter bit          SUPPORT_DPD       = 1'b0,                       // A1: Deep-Power-Down enter/exit
  parameter int unsigned TDPDOUT_CYCLES    = 0,                          // A1: tDPDOUT exit dwell (cycles)
  parameter bit          ACTIVE_CLK_STOP   = 1'b0,                       // A2: pause CK on RD back-pressure
  // ---- PHY (hyperbus_phy) -------------------------------------------------
  parameter              PHY_VARIANT       = "GENERIC",                  // GENERIC | INTEL | XILINX
  parameter bit          DIFF_CK           = 1'b1,                       // drive hb_ck_n
  parameter int unsigned RD_PREAMBLE_SKIP  = 0,                          // SDR/INTEL PHY: read-strobe
                                                                         // preamble rwds-rise edges to ignore
  parameter              CK_SCHEME         = "CLK90"                     // INTEL PHY only: "CLK90" |
                                                                         // "CLK_DLY" (issue #8)
) (
  // ---- clocking / reset ---------------------------------------------------
  input  logic                        clk,       // system + bus word clock
  input  logic                        clk90,     // 90-deg phase, to PHY (CK/write centering)
  input  logic                        clk_ref,   // PHY delay/SERDES ref (tie for GENERIC)
  input  logic                        rst,       // synchronous, active-high

  // ---- Avalon-MM slave ----------------------------------------------------
  input  logic [ADDR_WIDTH-1:0]       avs_address,      // word address; MSB = register-space select
  input  logic                        avs_read,
  input  logic                        avs_write,
  input  logic [DATA_WIDTH-1:0]       avs_writedata,
  input  logic [DATA_WIDTH/8-1:0]     avs_byteenable,
  input  logic [LEN_WIDTH-1:0]        avs_burstcount,   // words in burst
  output logic [DATA_WIDTH-1:0]       avs_readdata,
  output logic                        avs_readdatavalid,
  output logic                        avs_waitrequest,

  // ---- HyperBus device pins (split; board wrapper adds tristate) ----------
  output logic                        hb_ck,
  output logic                        hb_ck_n,
  output logic                        hb_cs_n,
  output logic                        hb_rst_n,
  output logic [DQ_WIDTH-1:0]         hb_dq_o,
  output logic                        hb_dq_oe,
  input  logic [DQ_WIDTH-1:0]         hb_dq_i,
  output logic                        hb_rwds_o,
  output logic                        hb_rwds_oe,
  input  logic                        hb_rwds_i,

  // ---- status -------------------------------------------------------------
  output logic                        init_done,
  output logic                        err_underrun,  // pulse: controller write-data underrun (Avalon
                                                     //   has no SLVERR channel, so this is surfaced as
                                                     //   a top-level status strobe; see INTERFACES v4)

  // ---- DEBUG tap (bring-up only; leave unconnected in normal instantiations) --
  //   [3:0]=ctrl state  [5:4]=front-end state  [6]=cmd_valid  [7]=cmd_ready
  //   [8]=phy_dq_i_valid  [9]=rd_valid  [10]=rd_ready  [11]=rd_last
  //   [17:12]=ctrl rd_fifo wptr  [23:18]=ctrl rd_fifo rptr  [31:24]=0
  output logic [31:0]                 dbg_bus
);

  // Derived widths (INTERFACES.md common-parameters note).
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;   // native byte-strobes (= 2)
  localparam int unsigned PHYW       = 2 * DQ_WIDTH;      // PHY parallel width (= 16)

  // ---- native command channel (avalon -> ctrl) ----------------------------
  logic                    cmd_valid;
  logic                    cmd_ready;
  logic                    cmd_read;
  logic                    cmd_reg;
  logic                    cmd_wrap;
  logic [ADDR_WIDTH-1:0]   cmd_addr;
  logic [LEN_WIDTH-1:0]    cmd_len;

  // ---- native write-data channel (avalon -> ctrl) -------------------------
  logic                    wr_valid;
  logic                    wr_ready;
  logic [DATA_WIDTH-1:0]   wr_data;
  logic [STRB_WIDTH-1:0]   wr_strb;
  logic                    wr_last;

  // ---- native read-data channel (ctrl -> avalon) --------------------------
  logic                    rd_valid;
  logic                    rd_ready;
  logic [DATA_WIDTH-1:0]   rd_data;
  logic                    rd_last;

  // ---- DEBUG taps from ctrl / front-end ----
  logic [3:0]              ctrl_dbg_state;
  logic [5:0]              ctrl_dbg_rd_wptr;
  logic [5:0]              ctrl_dbg_rd_rptr;
  logic [1:0]              fe_dbg_state;
  // dbg_bus: [3:0]=ctrl_state [5:4]=fe_state [6]=cmd_valid [7]=cmd_ready [8]=phy_dq_i_valid
  //   [9]=rd_valid [10]=rd_ready [11]=rd_last [17:12]=rem_left [23:18]=seg_left [29:24]=cmd_len [31:30]=0
  assign dbg_bus = {2'd0, cmd_len[5:0], ctrl_dbg_rd_rptr, ctrl_dbg_rd_wptr, rd_last, rd_ready, rd_valid,
                    phy_dq_i_valid, cmd_ready, cmd_valid, fe_dbg_state, ctrl_dbg_state};

  // ---- ctrl <-> phy DDR-parallel interface --------------------------------
  logic                    phy_cs_n;
  logic                    phy_rst_n;
  logic                    phy_ck_en;
  logic [PHYW-1:0]         phy_dq_o;
  logic                    phy_dq_oe;
  logic [1:0]              phy_rwds_o;
  logic                    phy_rwds_oe;
  logic                    phy_rd_arm;
  logic [PHYW-1:0]         phy_dq_i;
  logic                    phy_dq_i_valid;
  logic                    phy_rwds_i;

  // -------------------------------------------------------------------------
  // Avalon-MM slave front-end -> native master
  // -------------------------------------------------------------------------
  hyperbus_avalon #(
    .DQ_WIDTH   (DQ_WIDTH),
    .DATA_WIDTH (DATA_WIDTH),
    .ADDR_WIDTH (ADDR_WIDTH),
    .LEN_WIDTH  (LEN_WIDTH)
  ) u_avalon (
    .clk               (clk),
    .rst               (rst),
    // Avalon-MM slave
    .avs_address       (avs_address),
    .avs_read          (avs_read),
    .avs_write         (avs_write),
    .avs_writedata     (avs_writedata),
    .avs_byteenable    (avs_byteenable),
    .avs_burstcount    (avs_burstcount),
    .avs_readdata      (avs_readdata),
    .avs_readdatavalid (avs_readdatavalid),
    .avs_waitrequest   (avs_waitrequest),
    // native master -> ctrl
    .cmd_valid         (cmd_valid),
    .cmd_ready         (cmd_ready),
    .cmd_read          (cmd_read),
    .cmd_reg           (cmd_reg),
    .cmd_wrap          (cmd_wrap),
    .cmd_addr          (cmd_addr),
    .cmd_len           (cmd_len),
    .wr_valid          (wr_valid),
    .wr_ready          (wr_ready),
    .wr_data           (wr_data),
    .wr_strb           (wr_strb),
    .wr_last           (wr_last),
    .rd_valid          (rd_valid),
    .rd_ready          (rd_ready),
    .rd_data           (rd_data),
    .rd_last           (rd_last),
    .dbg_state         (fe_dbg_state)
  );

  // -------------------------------------------------------------------------
  // Protocol engine: native slave <-> PHY master
  // -------------------------------------------------------------------------
  hyperbus_ctrl #(
    .DQ_WIDTH          (DQ_WIDTH),
    .DATA_WIDTH        (DATA_WIDTH),
    .ADDR_WIDTH        (ADDR_WIDTH),
    .LEN_WIDTH         (LEN_WIDTH),
    .LATENCY_CLOCKS    (LATENCY_CLOCKS),
    .FIXED_LATENCY     (FIXED_LATENCY),
    .MAX_BURST_WORDS   (MAX_BURST_WORDS),
    .BURST_BOUNDARY_WORDS (BURST_BOUNDARY_WORDS),
    .WR_COMMIT_READ    (WR_COMMIT_READ),
    .COMMIT_READ_WORDS (COMMIT_READ_WORDS),
    .COMMIT_READ_MODE  (COMMIT_READ_MODE),
    .WR_COALESCE       (WR_COALESCE),
    .WR_COALESCE_WAIT  (WR_COALESCE_WAIT),
    .PROGRAM_CR        (PROGRAM_CR),
    .POR_DELAY_CYCLES  (POR_DELAY_CYCLES),
    .INIT_LATENCY_CODE (INIT_LATENCY_CODE),
    .INIT_CR0          (INIT_CR0),
    .PROGRAM_CR1       (PROGRAM_CR1),
    .INIT_CR1          (INIT_CR1),
    .CLK_FREQ_MHZ      (CLK_FREQ_MHZ),
    .T_RP_NS           (T_RP_NS),
    .T_RPH_NS          (T_RPH_NS),
    .T_RH_NS           (T_RH_NS),
    .T_VCS_US          (T_VCS_US),
    .SUPPORT_DPD       (SUPPORT_DPD),
    .TDPDOUT_CYCLES    (TDPDOUT_CYCLES),
    .ACTIVE_CLK_STOP   (ACTIVE_CLK_STOP)
  ) u_ctrl (
    .clk            (clk),
    .rst            (rst),
    // native command channel (slave)
    .cmd_valid      (cmd_valid),
    .cmd_ready      (cmd_ready),
    .cmd_read       (cmd_read),
    .cmd_reg        (cmd_reg),
    .cmd_wrap       (cmd_wrap),
    .cmd_addr       (cmd_addr),
    .cmd_len        (cmd_len),
    // native write-data channel (slave)
    .wr_valid       (wr_valid),
    .wr_ready       (wr_ready),
    .wr_data        (wr_data),
    .wr_strb        (wr_strb),
    .wr_last        (wr_last),
    // native read-data channel (master)
    .rd_valid       (rd_valid),
    .rd_ready       (rd_ready),
    .rd_data        (rd_data),
    .rd_last        (rd_last),
    // status
    .busy           (/* unused */),
    .init_done      (init_done),
    .err_underrun   (err_underrun),
    .err_timeout    (/* unused */),
    // PHY TX
    .phy_cs_n       (phy_cs_n),
    .phy_rst_n      (phy_rst_n),
    .phy_ck_en      (phy_ck_en),
    .phy_dq_o       (phy_dq_o),
    .phy_dq_oe      (phy_dq_oe),
    .phy_rwds_o     (phy_rwds_o),
    .phy_rwds_oe    (phy_rwds_oe),
    .phy_rd_arm     (phy_rd_arm),
    // PHY RX
    .phy_dq_i       (phy_dq_i),
    .phy_dq_i_valid (phy_dq_i_valid),
    .phy_rwds_i     (phy_rwds_i),
    // debug taps
    .dbg_state      (ctrl_dbg_state),
    .dbg_rd_wptr    (ctrl_dbg_rd_wptr),
    .dbg_rd_rptr    (ctrl_dbg_rd_rptr)
  );

  // -------------------------------------------------------------------------
  // PHY: DDR SERDES + I/O (generic inferrable-DDR variant)
  // -------------------------------------------------------------------------
  hyperbus_phy #(
    .DQ_WIDTH     (DQ_WIDTH),
    .DATA_WIDTH   (DATA_WIDTH),
    .ADDR_WIDTH   (ADDR_WIDTH),
    .LEN_WIDTH    (LEN_WIDTH),
    .PHY_VARIANT  (PHY_VARIANT),
    .DIFF_CK      (DIFF_CK),
    .RD_PREAMBLE_SKIP (RD_PREAMBLE_SKIP),
    .CK_SCHEME    (CK_SCHEME)
  ) u_phy (
    .clk            (clk),
    .clk90          (clk90),
    .clk_ref        (clk_ref),
    .rst            (rst),
    // ctrl-facing (slave, mirror of ctrl TX/RX)
    .phy_cs_n       (phy_cs_n),
    .phy_rst_n      (phy_rst_n),
    .phy_ck_en      (phy_ck_en),
    .phy_dq_o       (phy_dq_o),
    .phy_dq_oe      (phy_dq_oe),
    .phy_rwds_o     (phy_rwds_o),
    .phy_rwds_oe    (phy_rwds_oe),
    .phy_rd_arm     (phy_rd_arm),
    .phy_dq_i       (phy_dq_i),
    .phy_dq_i_valid (phy_dq_i_valid),
    .phy_rwds_i     (phy_rwds_i),
    // device pins
    .hb_ck          (hb_ck),
    .hb_ck_n        (hb_ck_n),
    .hb_cs_n        (hb_cs_n),
    .hb_rst_n       (hb_rst_n),
    .hb_dq_o        (hb_dq_o),
    .hb_dq_oe       (hb_dq_oe),
    .hb_dq_i        (hb_dq_i),
    .hb_rwds_o      (hb_rwds_o),
    .hb_rwds_oe     (hb_rwds_oe),
    .hb_rwds_i      (hb_rwds_i)
  );

endmodule
