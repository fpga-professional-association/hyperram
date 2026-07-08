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
  parameter bit          PROGRAM_CR        = 1'b1,                       // program CR0 at init
  parameter int unsigned POR_DELAY_CYCLES  = 0,                         // POR init delay (sim: 0)
  parameter logic [3:0]  INIT_LATENCY_CODE = hb_clocks_to_latency_code(LATENCY_CLOCKS),
  parameter logic [15:0] INIT_CR0          = HB_CR0_RESET,               // CR0 image written at init
  // ---- PHY (hyperbus_phy) -------------------------------------------------
  parameter              PHY_VARIANT       = "GENERIC",                  // GENERIC | INTEL | XILINX
  parameter bit          DIFF_CK           = 1'b1                        // drive hb_ck_n
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
  output logic                        init_done
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
    .rd_last           (rd_last)
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
    .PROGRAM_CR        (PROGRAM_CR),
    .POR_DELAY_CYCLES  (POR_DELAY_CYCLES),
    .INIT_LATENCY_CODE (INIT_LATENCY_CODE),
    .INIT_CR0          (INIT_CR0)
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
    .err_underrun   (/* unused */),
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
    .phy_rwds_i     (phy_rwds_i)
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
    .DIFF_CK      (DIFF_CK)
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
