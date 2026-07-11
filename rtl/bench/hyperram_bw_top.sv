// hyperram_bw_top — SIM / on-chip top: bandwidth-test engine + HyperBus master IP.
//
// Wires the synthesizable traffic generator (hyperram_bw_test) as the Avalon-MM MASTER onto the
// hyperram_avalon Avalon-MM SLAVE, and hoists to the top: the clocks/reset, the bench CSR slave
// (host/JTAG control + MB/s readback), and the split HyperBus device pins. This is exactly what a
// simulation testbench drives (resolving the shared DQ/RWDS bus against hyperram_model, the same
// way sim/tb_avalon.sv does) and what a later board wrapper re-uses (CSR fed by a JTAG-to-Avalon
// master; device pins fed through IOBUFs).
//
// Pure structural: no functional logic lives here. Defaults mirror sim/tb_avalon.sv so that, when a
// testbench attaches hyperram_model, the programmed CR0 latency (code 0001 = 6 clocks, fixed) lines
// up with the controller's LATENCY_CLOCKS and read-back data matches.
//
// Simulates cleanly under Verilator (verilator --binary --timing) with PHY_VARIANT="GENERIC".

module hyperram_bw_top
    import hyperbus_pkg::*;
#(
    // ---- bus geometry (must match hyperram_avalon's slave) ------------------
    parameter int unsigned DQ_WIDTH         = HB_DQ_WIDTH_DEFAULT,          // 8
    parameter int unsigned DATA_WIDTH       = 2 * DQ_WIDTH,                 // 16
    parameter int unsigned ADDR_WIDTH       = HB_ADDR_WIDTH,               // 32 (word address)
    parameter int unsigned LEN_WIDTH        = HB_LEN_WIDTH_DEFAULT,        // 16 (burstcount)
    // ---- bandwidth-test engine ---------------------------------------------
    parameter int unsigned BURST_WORDS      = HB_BURST_WORDS_DEFAULT,      // 16 words / Avalon burst
    parameter int unsigned CSR_ADDR_WIDTH   = 5,                          // 32 CSR word-registers (issue #13
                                                                          //   REG_EMAP_DATA at word 20 needs 5)
    parameter logic [31:0] VERSION_MAGIC    = 32'h4842_5755,              // "HBWU" (instrumented; was 0x..54)
    // ---- controller / PHY (defaults mirror sim/tb_avalon.sv) ---------------
    parameter int unsigned LATENCY_CLOCKS   = 6,
    parameter bit          FIXED_LATENCY    = 1'b1,
    parameter int unsigned MAX_BURST_WORDS  = 0,                          // 0 = no controller chop
    parameter int unsigned BURST_BOUNDARY_WORDS = 0,                      // 0 = off (W957D8NB: 0x2000)
    parameter bit          WR_COMMIT_READ   = 1'b0,                       // split-write commit-read fix
    parameter bit          PROGRAM_CR       = 1'b1,
    parameter int unsigned POR_DELAY_CYCLES = 0,
    parameter logic [15:0] INIT_CR0         = 16'h8F1F,                    // latency code 6, fixed
    parameter              PHY_VARIANT      = "GENERIC",
    parameter bit          DIFF_CK          = 1'b1,
    // ---- runtime read-eye calibration (POR seeds + REG_CAL image; defaults reproduce today) --------
    parameter int unsigned RD_PREAMBLE_SKIP = 0,                          // SDR PHY preamble-skip POR seed
    parameter bit          CAPTURE_PHASE    = 1'b0,                       // SDR read-capture edge seed
    parameter logic [31:0] CAL_RESET        = 32'h0000_0000,              // POR image of REG_CAL
    // ---- issue #13 REG_DBG POR image (per-instance-derived, A2) --------------
    // This sim/GENERIC leg's hyperram_avalon uses the ctrl default WR_LAT_TRIM=0 (the sim model has no
    // trim analog and always ran trim=0), so the POR trim field is 0 — a hard 0x63 would shift every
    // sim write by 3 words and break the suite. dbg_lat_clocks = this instance's LATENCY_CLOCKS.
    // (Board top.sv overrides this to 0x63 to match its ctrl WR_LAT_TRIM(3).)
    parameter logic [31:0] DBG_RESET        = {24'h0, 4'(LATENCY_CLOCKS), 4'h0}   // [7:4]=lat, [3:0]=trim=0
) (
    // ---- clocking / reset ---------------------------------------------------
    input  logic                        clk,        // system + bus word clock
    input  logic                        clk90,      // 90-deg phase, to PHY
    input  logic                        clk_ref,    // PHY delay/SERDES ref (tie for GENERIC)
    input  logic                        rst,        // synchronous, active-high

    // ---- bandwidth-test CSR slave (host / JTAG) -----------------------------
    input  logic [CSR_ADDR_WIDTH-1:0]   csr_address,
    input  logic                        csr_read,
    output logic [31:0]                 csr_readdata,
    input  logic                        csr_write,
    input  logic [31:0]                 csr_writedata,
    output logic                        csr_waitrequest,

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

    localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;   // native byte-strobes (= 2)

    // ---- Avalon-MM link: bench master  <->  hyperram_avalon slave ----------
    logic [ADDR_WIDTH-1:0] m_address;
    logic [LEN_WIDTH-1:0]  m_burstcount;
    logic                  m_read;
    logic                  m_write;
    logic [DATA_WIDTH-1:0] m_writedata;
    logic [DATA_WIDTH-1:0] m_readdata;
    logic                  m_readdatavalid;
    logic                  m_waitrequest;

    // Full-word writes only: byte-enables tied all-ones (the bench streams whole words).
    logic [STRB_WIDTH-1:0] m_byteenable;
    assign m_byteenable = '1;

    // ---- runtime read-eye calibration: bench REG_CAL image -> hyperram_avalon cal_* inputs ---------
    logic                                  cal_capture_phase;
    logic [HB_CAL_PREAMBLE_SKIP_WIDTH-1:0] cal_preamble_skip;
    logic [HB_CAL_RX_TAP_WIDTH-1:0]        cal_rx_tap;
    logic                                  cal_pair_skew;

    // ---- issue #13 debug bundle: bench REG_DBG image -> hyperram_avalon dbg_* + wrap_en inputs ------
    // Same single clk domain end-to-end (bench/ctrl), quasi-static, no synchronizer. dbg_ck_stretch_off
    // is BOARD-only (drives the gpio_io ck_stretch, which this sim wrapper does not instantiate) so the
    // bench output is left unconnected here. All knobs are POR-legacy at REG_DBG=DBG_RESET.
    logic [3:0] dbg_wr_lat_trim;
    logic [3:0] dbg_lat_clocks;
    logic       dbg_cr0_reprog;
    logic       dbg_prewin_drive;
    logic [2:0] dbg_prewin_n;
    logic       dbg_prewin_marker;
    logic       dbg_postwin_hold;
    logic       dbg_prewin_contig;   // issue #13 round 2 (A)
    logic       dbg_end_cwrite;       // issue #13 round 3 (B): end-of-row commit-WRITE
    logic       dbg_spray_defuse;     // issue #13 round 4: per-boundary spray defuse (REG_DBG[18])
    logic       wrap_en;

    // ------------------------------------------------------------------------
    // Bandwidth-test engine (Avalon-MM master + CSR slave)
    // ------------------------------------------------------------------------
    hyperram_bw_test #(
        .DATA_WIDTH     (DATA_WIDTH),
        .ADDR_WIDTH     (ADDR_WIDTH),
        .LEN_WIDTH      (LEN_WIDTH),
        .BURST_WORDS    (BURST_WORDS),
        .CSR_ADDR_WIDTH (CSR_ADDR_WIDTH),
        .VERSION_MAGIC  (VERSION_MAGIC),
        .CAL_RESET      (CAL_RESET),
        .DBG_RESET      (DBG_RESET)
    ) u_bw (
        .clk             (clk),
        .rst             (rst),
        // CSR slave
        .csr_address     (csr_address),
        .csr_read        (csr_read),
        .csr_readdata    (csr_readdata),
        .csr_write       (csr_write),
        .csr_writedata   (csr_writedata),
        .csr_waitrequest (csr_waitrequest),
        // Avalon-MM master
        .m_address       (m_address),
        .m_burstcount    (m_burstcount),
        .m_read          (m_read),
        .m_write         (m_write),
        .m_writedata     (m_writedata),
        .m_readdata      (m_readdata),
        .m_readdatavalid (m_readdatavalid),
        .m_waitrequest   (m_waitrequest),
        // runtime PHY read-eye calibration (REG_CAL image -> hyperram_avalon)
        .cal_capture_phase (cal_capture_phase),
        .cal_preamble_skip (cal_preamble_skip),
        .cal_rx_tap        (cal_rx_tap),
        .cal_pair_skew     (cal_pair_skew),
        // issue #13 debug bundle (REG_DBG image -> hyperram_avalon -> ctrl). dbg_ck_stretch_off is
        // board-only (gpio_io not present in sim) -> left unconnected.
        .dbg_wr_lat_trim   (dbg_wr_lat_trim),
        .dbg_lat_clocks    (dbg_lat_clocks),
        .dbg_cr0_reprog    (dbg_cr0_reprog),
        .dbg_prewin_drive  (dbg_prewin_drive),
        .dbg_prewin_n      (dbg_prewin_n),
        .dbg_prewin_marker (dbg_prewin_marker),
        .dbg_postwin_hold  (dbg_postwin_hold),
        .dbg_ck_stretch_off (/* board-only: gpio ck_stretch, no sim consumer */),
        .dbg_prewin_contig (dbg_prewin_contig),
        .dbg_end_cwrite     (dbg_end_cwrite),
        .dbg_spray_defuse   (dbg_spray_defuse),
        .wrap_en           (wrap_en)
    );

    // ------------------------------------------------------------------------
    // HyperBus master IP (Avalon-MM slave + generic PHY + device pins)
    // ------------------------------------------------------------------------
    hyperram_avalon #(
        .DQ_WIDTH         (DQ_WIDTH),
        .DATA_WIDTH       (DATA_WIDTH),
        .ADDR_WIDTH       (ADDR_WIDTH),
        .LEN_WIDTH        (LEN_WIDTH),
        .LATENCY_CLOCKS   (LATENCY_CLOCKS),
        .FIXED_LATENCY    (FIXED_LATENCY),
        .MAX_BURST_WORDS  (MAX_BURST_WORDS),
        .BURST_BOUNDARY_WORDS (BURST_BOUNDARY_WORDS),
        .WR_COMMIT_READ   (WR_COMMIT_READ),
        .PROGRAM_CR       (PROGRAM_CR),
        .POR_DELAY_CYCLES (POR_DELAY_CYCLES),
        .INIT_CR0         (INIT_CR0),
        .PHY_VARIANT      (PHY_VARIANT),
        .DIFF_CK          (DIFF_CK),
        .RD_PREAMBLE_SKIP (RD_PREAMBLE_SKIP),
        .CAPTURE_PHASE    (CAPTURE_PHASE)
    ) u_hyperram (
        .clk               (clk),
        .clk90             (clk90),
        .clk_ref           (clk_ref),
        .rst               (rst),
        // runtime read-eye calibration (from the bench REG_CAL image)
        .cal_capture_phase (cal_capture_phase),
        .cal_preamble_skip (cal_preamble_skip),
        .cal_rx_tap        (cal_rx_tap),
        .cal_pair_skew     (cal_pair_skew),
        // issue #13 debug bundle (frozen §0 contract; hyperram_avalon forwards to u_ctrl / front-end).
        // WP-A adds these inputs to hyperram_avalon — written blind to the frozen names/widths, so this
        // wrapper cannot elaborate against the BASELINE hyperram_avalon (expected, A9).
        .dbg_wr_lat_trim   (dbg_wr_lat_trim),
        .dbg_lat_clocks    (dbg_lat_clocks),
        .dbg_cr0_reprog    (dbg_cr0_reprog),
        .dbg_prewin_drive  (dbg_prewin_drive),
        .dbg_prewin_n      (dbg_prewin_n),
        .dbg_prewin_marker (dbg_prewin_marker),
        .dbg_postwin_hold  (dbg_postwin_hold),
        .dbg_prewin_contig (dbg_prewin_contig),
        .dbg_end_cwrite     (dbg_end_cwrite),
        .dbg_spray_defuse   (dbg_spray_defuse),
        .wrap_en           (wrap_en),
        // Avalon-MM slave (driven by the bench master)
        .avs_address       (m_address),
        .avs_read          (m_read),
        .avs_write         (m_write),
        .avs_writedata     (m_writedata),
        .avs_byteenable    (m_byteenable),
        .avs_burstcount    (m_burstcount),
        .avs_readdata      (m_readdata),
        .avs_readdatavalid (m_readdatavalid),
        .avs_waitrequest   (m_waitrequest),
        // HyperBus device pins
        .hb_ck             (hb_ck),
        .hb_ck_n           (hb_ck_n),
        .hb_cs_n           (hb_cs_n),
        .hb_rst_n          (hb_rst_n),
        .hb_dq_o           (hb_dq_o),
        .hb_dq_oe          (hb_dq_oe),
        .hb_dq_i           (hb_dq_i),
        .hb_rwds_o         (hb_rwds_o),
        .hb_rwds_oe        (hb_rwds_oe),
        .hb_rwds_i         (hb_rwds_i),
        // status
        .init_done         (init_done),
        .err_underrun      (/* unused: bench engine streams full-rate, never underruns */),
        // debug tap (bring-up only; unused in this top)
        .dbg_bus           ()
    );

endmodule
