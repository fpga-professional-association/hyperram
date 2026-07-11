// hyperram_axi — TOP: AXI4 slave + HyperBus device pins.
//
// Normative boundary: docs/INTERFACES.md §hyperram_axi (frozen). This is a pure structural
// wrapper: it instantiates exactly three blocks and wires them strictly per INTERFACES.md —
//   hyperbus_axi  (AXI4 slave  -> native master)
//   hyperbus_ctrl (native slave -> PHY master, the protocol engine)
//   hyperbus_phy  (DDR SERDES + I/O; PHY_VARIANT="GENERIC" = inferrable-DDR, Verilator-clean)
// and exposes { clocking } U { full AXI4 slave } U { HyperBus device pins } U { init_done }.
//
// No functional logic lives here (DESIGN.md §1): the front-end owns bus translation, the ctrl
// owns all protocol semantics, the PHY owns serialization / CK / the RWDS->clk crossing.
//
// hyperbus_pkg is compiled into the design (build command line); it is imported below and its
// package-scoped constants/functions resolve at elaboration, so no textual `include is needed.
module hyperram_axi
  import hyperbus_pkg::*;
#(
  // ---- common bus geometry ------------------------------------------------
  parameter int unsigned DQ_WIDTH          = HB_DQ_WIDTH_DEFAULT,        // HyperBus DQ pins
  parameter int unsigned DATA_WIDTH        = 2 * DQ_WIDTH,               // native word = 2*DQ
  parameter int unsigned ADDR_WIDTH        = HB_ADDR_WIDTH,              // word-address width
  parameter int unsigned LEN_WIDTH         = HB_LEN_WIDTH_DEFAULT,       // burst-length (words)
  // ---- AXI4 front-end (hyperbus_axi) --------------------------------------
  parameter int unsigned ID_WIDTH          = 4,
  parameter int unsigned AXI_DATA_WIDTH    = DATA_WIDTH,                 // 16 => 1:1 beat<->word
  parameter int unsigned AXI_ADDR_WIDTH    = ADDR_WIDTH + 1,            // byte address; MSB=reg space
  // ---- protocol engine (hyperbus_ctrl) ------------------------------------
  parameter int unsigned LATENCY_CLOCKS    = HB_LATENCY_CLOCKS_DEFAULT,  // CA1->data, clocks
  parameter bit          FIXED_LATENCY     = HB_FIXED_LATENCY_DEFAULT,   // 1 = fixed (POR default)
  parameter int unsigned MAX_BURST_WORDS   = 0,                         // 0 = no chop; else tCSM/tCK
  parameter int unsigned BURST_BOUNDARY_WORDS = 0,                       // 0 = off; else chop at this
                                                                         //   WORD boundary (W957D8NB)
  parameter bit          WR_COMMIT_READ    = 1'b0,                       // interpose commit-read after
                                                                         //   each split memory write
  parameter bit          WR_CHOP_REPLAY    = 1'b0,                       // re-send the last words at
                                                                         //   intra-command write chops
                                                                         //   (issue #1 direction 5)
  parameter int unsigned WR_REPLAY_WORDS   = 4,
  parameter int unsigned WR_REPLAY_PEND    = 4,                          // device pending depth (align floor)
  parameter int unsigned WR_REPLAY_ALIGN   = 0,                          // row-aligned rollback (0 = legacy)
  parameter int unsigned WR_REPLAY_MASK_LEAD = 0,                        // masked lead-in beats on reopen
  parameter int unsigned WR_CHOP_PAUSE_CYCLES = 0,                       // post-write CS#-High dwell
  parameter bit          WR_CHOP_PAUSE_CK  = 1'b0,                       // CK toggling through the dwell                          // replay depth (= device
                                                                         //   pending depth; W957D8NB: 4)
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
  parameter int unsigned RD_PREAMBLE_SKIP  = 0,                          // SDR/GENERIC/INTEL PHY:
                                                                         // preamble rwds-rise edges to skip
  parameter              CK_SCHEME         = "CLK90",                    // INTEL PHY only (issue #8)
  // POR seed for the runtime cal_capture_phase knob (forwarded to hyperbus_phy; default = legacy).
  parameter bit          CAPTURE_PHASE     = 1'b0                        // SDR read-capture edge seed
) (
  // ---- clocking / reset ---------------------------------------------------
  input  logic                        clk,       // aclk = system + bus word clock
  input  logic                        clk90,     // 90-deg phase, to PHY (CK/write centering)
  input  logic                        clk_ref,   // PHY delay/SERDES ref (tie for GENERIC)
  input  logic                        rst,       // synchronous, active-high (invert of aresetn)

  // ---- runtime PHY read-eye calibration (mandatory, no defaults; quasi-static). See INTERFACES.md v9. ----
  input  logic                                  cal_capture_phase,
  input  logic [HB_CAL_PREAMBLE_SKIP_WIDTH-1:0] cal_preamble_skip,
  input  logic [HB_CAL_RX_TAP_WIDTH-1:0]        cal_rx_tap,
  input  logic                                  cal_pair_skew,

  // ---- AXI4 slave ---------------------------------------------------------
  // AW
  input  logic [ID_WIDTH-1:0]         awid,
  input  logic [AXI_ADDR_WIDTH-1:0]   awaddr,
  input  logic [7:0]                  awlen,
  input  logic [2:0]                  awsize,
  input  logic [1:0]                  awburst,
  input  logic                        awvalid,
  output logic                        awready,
  // W
  input  logic [AXI_DATA_WIDTH-1:0]   wdata,
  input  logic [AXI_DATA_WIDTH/8-1:0] wstrb,
  input  logic                        wlast,
  input  logic                        wvalid,
  output logic                        wready,
  // B
  output logic [ID_WIDTH-1:0]         bid,
  output logic [1:0]                  bresp,
  output logic                        bvalid,
  input  logic                        bready,
  // AR
  input  logic [ID_WIDTH-1:0]         arid,
  input  logic [AXI_ADDR_WIDTH-1:0]   araddr,
  input  logic [7:0]                  arlen,
  input  logic [2:0]                  arsize,
  input  logic [1:0]                  arburst,
  input  logic                        arvalid,
  output logic                        arready,
  // R
  output logic [ID_WIDTH-1:0]         rid,
  output logic [AXI_DATA_WIDTH-1:0]   rdata,
  output logic [1:0]                  rresp,
  output logic                        rlast,
  output logic                        rvalid,
  input  logic                        rready,

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

  // ---- native command channel (axi -> ctrl) -------------------------------
  logic                    cmd_valid;
  logic                    cmd_ready;
  logic                    cmd_read;
  logic                    cmd_reg;
  logic                    cmd_wrap;
  logic [ADDR_WIDTH-1:0]   cmd_addr;
  logic [LEN_WIDTH-1:0]    cmd_len;

  // ---- native write-data channel (axi -> ctrl) ----------------------------
  logic                    wr_valid;
  logic                    wr_ready;
  logic [DATA_WIDTH-1:0]   wr_data;
  logic [STRB_WIDTH-1:0]   wr_strb;
  logic                    wr_last;

  // ---- native read-data channel (ctrl -> axi) -----------------------------
  logic                    rd_valid;
  logic                    rd_ready;
  logic [DATA_WIDTH-1:0]   rd_data;
  logic                    rd_last;

  // ---- controller error status (ctrl -> axi front-end, for SLVERR) --------
  logic                    err_underrun;
  logic                    err_timeout;

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
  // AXI4 slave front-end -> native master
  // -------------------------------------------------------------------------
  hyperbus_axi #(
    .DQ_WIDTH       (DQ_WIDTH),
    .DATA_WIDTH     (DATA_WIDTH),
    .ADDR_WIDTH     (ADDR_WIDTH),
    .LEN_WIDTH      (LEN_WIDTH),
    .ID_WIDTH       (ID_WIDTH),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH)
  ) u_axi (
    .clk        (clk),
    .rst        (rst),
    // AW
    .awid       (awid),
    .awaddr     (awaddr),
    .awlen      (awlen),
    .awsize     (awsize),
    .awburst    (awburst),
    .awvalid    (awvalid),
    .awready    (awready),
    // W
    .wdata      (wdata),
    .wstrb      (wstrb),
    .wlast      (wlast),
    .wvalid     (wvalid),
    .wready     (wready),
    // B
    .bid        (bid),
    .bresp      (bresp),
    .bvalid     (bvalid),
    .bready     (bready),
    // AR
    .arid       (arid),
    .araddr     (araddr),
    .arlen      (arlen),
    .arsize     (arsize),
    .arburst    (arburst),
    .arvalid    (arvalid),
    .arready    (arready),
    // R
    .rid        (rid),
    .rdata      (rdata),
    .rresp      (rresp),
    .rlast      (rlast),
    .rvalid     (rvalid),
    .rready     (rready),
    // native master -> ctrl
    .cmd_valid  (cmd_valid),
    .cmd_ready  (cmd_ready),
    .cmd_read   (cmd_read),
    .cmd_reg    (cmd_reg),
    .cmd_wrap   (cmd_wrap),
    .cmd_addr   (cmd_addr),
    .cmd_len    (cmd_len),
    .wr_valid   (wr_valid),
    .wr_ready   (wr_ready),
    .wr_data    (wr_data),
    .wr_strb    (wr_strb),
    .wr_last    (wr_last),
    .rd_valid   (rd_valid),
    .rd_ready   (rd_ready),
    .rd_data    (rd_data),
    .rd_last    (rd_last),
    // controller error status -> SLVERR mapping
    .err_underrun (err_underrun),
    .err_timeout  (err_timeout)
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
    .WR_CHOP_REPLAY    (WR_CHOP_REPLAY),
    .WR_REPLAY_WORDS   (WR_REPLAY_WORDS),
    .WR_REPLAY_PEND    (WR_REPLAY_PEND),
    .WR_REPLAY_ALIGN   (WR_REPLAY_ALIGN),
    .WR_REPLAY_MASK_LEAD (WR_REPLAY_MASK_LEAD),
    .WR_CHOP_PAUSE_CYCLES (WR_CHOP_PAUSE_CYCLES),
    .WR_CHOP_PAUSE_CK  (WR_CHOP_PAUSE_CK),
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
    .err_timeout    (err_timeout),
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
    // debug taps (bring-up only; unused in the AXI top)
    .dbg_state      (),
    .dbg_rd_wptr    (),
    .dbg_rd_rptr    (),
    // issue #13 live controller knobs — legacy tie-offs (A1: no port defaults). The AXI top drives
    // no runtime instrumentation, so every knob is pinned at its POR-legacy value: the trim/latency
    // seeds match this instance's ctrl elaboration constants (u_ctrl leaves WR_LAT_TRIM at its
    // default 0; dbg_lat_clocks = LATENCY_CLOCKS), the rest are 0 = bit-identical to today.
    .dbg_wr_lat_trim  (4'd0),
    .dbg_lat_clocks   (4'(LATENCY_CLOCKS)),
    .dbg_cr0_reprog   (1'b0),
    .dbg_prewin_drive (1'b0),
    .dbg_prewin_n     (3'd0),
    .dbg_prewin_marker(1'b0),
    .dbg_postwin_hold (1'b0),
    .dbg_prewin_contig(1'b0),   // issue #13 round 2: legacy tie-off (bit-identical to today)
    .dbg_end_cwrite    (1'b0)
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
    .CK_SCHEME    (CK_SCHEME),
    .CAPTURE_PHASE (CAPTURE_PHASE)
  ) u_phy (
    .clk            (clk),
    .clk90          (clk90),
    .clk_ref        (clk_ref),
    .rst            (rst),
    // runtime read-eye calibration (forwarded 1:1 to the PHY)
    .cal_capture_phase (cal_capture_phase),
    .cal_preamble_skip (cal_preamble_skip),
    .cal_rx_tap        (cal_rx_tap),
    .cal_pair_skew     (cal_pair_skew),
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
