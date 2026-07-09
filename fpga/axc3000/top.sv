// top — AXC3000 board top level for the HyperBus bandwidth test (Agilex-3 A3CY100BM16AE7S).
//
// Data path:
//   bw_sys (Qsys)                              fpga/axc3000/qsys/
//     * IOPLL: clk (50 MHz, 0deg = HyperBus CK word clock) + clk2x (100 MHz, 0deg = SDR byte clock),
//       both from the 25 MHz board XO. (The Qsys export is still named "clk90"; for the SDR PHY it
//       now carries the 100 MHz 2x byte clock — NOT a 90deg phase. See make_bw_sys.tcl.)
//     * reset controller -> synchronous active-high fabric reset (clk / 50 MHz domain)
//     * JTAG-to-Avalon-MM master bridge  --- exported Avalon-MM master (byte addressed)
//         |
//         v  (byte->word CSR adapter below)
//   hyperram_bw_test (traffic gen + scoreboard, CSR slave @ base 0x000)
//     -> hyperram_avalon (Avalon-MM slave -> ctrl -> SDR PHY)
//   (the two are wired here directly — the pure-structural rtl/bench/hyperram_bw_top.sv inlined —
//    so the av_* Avalon handshake is visible to the hyperbus_capture DEBUG module, CSR @ base 0x100)
//         |  split HyperBus device pins (hb_*_o / hb_*_oe / hb_*_i)
//         v
//   hyperbus_pads_altera (fpga/axc3000/)  = inferred tri-state I/O -> real inout board pads
//
// The measured WR_CYCLES/RD_CYCLES cover only the on-chip Avalon datapath; JTAG is control plane
// only (PLAN §8 method E compliant). Read back with sysconsole/bw_read.tcl.
//
// CLOCK PLAN (SDR PHY — unblocks the Fitter err 24403/24404 by using ONE clock in the I/O periphery
// instead of two IOPLL phases): the frozen controller is WORD-per-clk, so the fabric byte engine must
// run at 2x the CK rate (see rtl/phy/hyperbus_phy_sdr.sv). clk = 50 MHz drives the controller/fabric;
// clk2x = 100 MHz (single PLL, 0deg) is the ONLY clock at the Bank-3A SDR I/O registers + hb_ck
// generator. hb_ck = clk2x/2 = 50 MHz => 1 byte per clk2x cycle = ~100 MB/s/direction on the x8 bus
// (same peak as the DDR plan; the 90deg CK-centring phase is now derived from clk2x's negedge, from
// ONE clock). Deliberately low so the un-calibrated read eye is wide.
//
// Board signal names below match quartus/constraints/axc3000_board.tcl (sourced via pins.tcl).

`timescale 1ns/1ps

module top (
    // ---- board clock / reset ----
    input  wire        CLK_25M_C,   // 25 MHz fixed XO (PIN_A7, 1.2 V)
    input  wire        USER_BTN,    // S2, active-low, weak pull-up (PIN_A12, 1.2 V)

    // ---- HyperRAM (Winbond W957D8NB, single-ended x8 HyperBus, 1.2 V) ----
    inout  wire [7:0]  hb_dq,       // DQ[7:0]
    inout  wire        hb_rwds,     // RWDS
    output wire        hb_cs_n,     // chip select
    output wire        hb_ck,       // HyperBus clock (single-ended: no hb_ck_n board pin)
    output wire        hb_rst_n,    // device reset

    // ---- user LEDs (active-low, 3.3-V LVCMOS) — quick visual STATUS ----
    output wire        LED1,        // STATUS.done  (lit = done)
    output wire        RLED,        // STATUS.error (lit = error)
    output wire        GLED         // PLL locked   (lit = locked)
);

  // =========================================================================
  // Qsys backbone: clocks, reset, JTAG-Avalon master
  // =========================================================================
  wire        clk;          // 50 MHz HyperBus CK word clock (controller + fabric)
  wire        clk2x;        // 100 MHz SDR byte clock (single PLL, 0 deg) — feeds PHY clk90 port
  wire        pll_locked;
  wire        sys_rst;      // synchronous, active-high fabric reset (clk domain)

  // Exported (byte-addressed) Avalon-MM master from the JTAG bridge
  wire [31:0] m_address;
  wire [31:0] m_readdata;
  wire        m_read;
  wire        m_write;
  wire [31:0] m_writedata;
  wire        m_waitrequest;
  wire        m_readdatavalid;
  wire [3:0]  m_byteenable;

  bw_sys u_sys (
    .clk_25_clk           (CLK_25M_C),
    .reset_reset          (~USER_BTN),      // button pressed (low) => assert active-high reset
    .clk_clk              (clk),
    .clk90_clk            (clk2x),          // Qsys export "clk90" now = 100 MHz 2x byte clock (0 deg)
    .locked_export        (pll_locked),
    .sys_reset_reset      (sys_rst),
    .master_address       (m_address),
    .master_readdata      (m_readdata),
    .master_read          (m_read),
    .master_write         (m_write),
    .master_writedata     (m_writedata),
    .master_waitrequest   (m_waitrequest),
    .master_readdatavalid (m_readdatavalid),
    .master_byteenable    (m_byteenable)
  );

  // =========================================================================
  // Byte-address (JTAG master)  ->  word-address CSR slave adapter + 2-slave address decode
  //   Two CSR slaves hang off the single JTAG-Avalon master, decoded on byte-address bit [8]:
  //     m_address[8] == 0  ->  hyperram_bw_test CSR   (base 0x000, regs 0x00..0x1C)
  //     m_address[8] == 1  ->  hyperbus_capture CSR   (base 0x100, regs 0x100..0x10C)
  //   Both slaves are word-addressed (csr_address = byte_offset>>2), read combinationally, and tie
  //   waitrequest low. The JTAG-Avalon master is byte-addressed, pipelined (expects readdatavalid),
  //   and single-outstanding, so a combinational readdata/waitrequest mux on the decode bit is safe.
  // =========================================================================
  localparam int CSR_AW = 4;   // 16 bw_test regs (0x00..0x3C): STATUS/LEN/BASE/cycles/ERR + first-err diag

  wire              sel_cap        = m_address[8];    // 0x100 window = capture CSR

  // hyperram_bw_test CSR: 16 registers (0x00..0x3C) => 4 word-address bits = m_address[5:2]
  wire [CSR_AW-1:0] csr_address    = m_address[CSR_AW+1:2];
  wire              csr_read       = m_read  & ~sel_cap;
  wire              csr_write      = m_write & ~sel_cap;
  wire [31:0]       csr_writedata  = m_writedata;
  wire [31:0]       csr_readdata;
  wire              csr_waitrequest;

  // hyperbus_capture CSR: 4 registers (0x100..0x10C) => 2 word-address bits = m_address[3:2]
  wire [1:0]        cap_address    = m_address[3:2];
  wire              cap_read       = m_read  & sel_cap;
  wire              cap_write      = m_write & sel_cap;
  wire [31:0]       cap_readdata;
  wire              cap_waitrequest;

  wire [31:0] mux_readdata    = sel_cap ? cap_readdata    : csr_readdata;
  wire        mux_waitrequest = sel_cap ? cap_waitrequest : csr_waitrequest;

  assign m_waitrequest = mux_waitrequest;   // tied low inside both slaves (0 wait states)

  // Pipeline the combinational CSR read to the Avalon read-latency-1 contract the JTAG master expects:
  // capture readdata on the accept cycle, present it with readdatavalid the next cycle.
  logic [31:0] rd_hold;
  logic        rdv_q;
  always_ff @(posedge clk) begin
    if (sys_rst) begin
      rd_hold <= 32'd0;
      rdv_q   <= 1'b0;
    end else begin
      rd_hold <= mux_readdata;                       // both slaves read combinationally off the address
      rdv_q   <= m_read & ~mux_waitrequest;          // accepted this cycle (waitrequest is 0)
    end
  end
  assign m_readdata      = rd_hold;
  assign m_readdatavalid = rdv_q;

  // =========================================================================
  // LED status snoop — latch STATUS bits whenever the host polls STATUS (read of word 0).
  //   STATUS: bit0 = busy, bit1 = done, bit2 = error  (see docs/BW_TEST.md)
  //   bw_test exposes STATUS only via the CSR, so we passively observe the read bus (no 2nd master).
  // =========================================================================
  logic led_done_q, led_error_q;
  always_ff @(posedge clk) begin
    if (sys_rst) begin
      led_done_q  <= 1'b0;
      led_error_q <= 1'b0;
    end else if (csr_read && (csr_address == CSR_AW'(0))) begin
      led_done_q  <= csr_readdata[1];
      led_error_q <= csr_readdata[2];
    end
  end

  // Heartbeat blink so a freshly-flashed bitstream is visually obvious on the board (this build:
  // GLED BLINKS ~1.5 Hz while the PLL is locked, instead of the previous steady-on). Bump the toggle
  // bit / LED on each new build to tell successive flashes apart.
  logic [25:0] hb_cnt;
  always_ff @(posedge clk) begin
    if (sys_rst) hb_cnt <= '0;
    else         hb_cnt <= hb_cnt + 1'b1;
  end
  wire heartbeat = hb_cnt[23];   // ~3 Hz medium blink (compile #4: distinct from #3's slow 0.75 Hz)

  assign LED1 = ~led_done_q;                 // active-low: lit when done
  assign RLED = ~led_error_q;                // active-low: lit when error
  assign GLED = ~(pll_locked & heartbeat);   // active-low: BLINKS ~1.5 Hz while PLL locked (new build marker)

  // =========================================================================
  // Bandwidth harness: bw_test (CSR + Avalon master) -> hyperram_avalon (Agilex PHY) -> split pins
  // =========================================================================
  // Split HyperBus device pins between the IP and the board pad ring.
  wire        hb_ck_int;
  wire        hb_ck_n_int;   // single-ended board: driven but not brought to a pin (DIFF_CK=0 ties it)
  wire        hb_cs_n_int;
  wire        hb_rst_n_int;
  wire [7:0]  hb_dq_o_int;
  wire        hb_dq_oe_int;
  wire [7:0]  hb_dq_i_int;
  wire        hb_rwds_o_int;
  wire        hb_rwds_oe_int;
  wire        hb_rwds_i_int;
  wire        init_done;

  // ---- bench master <-> HyperBus IP slave Avalon-MM link, hoisted to the top --------------------
  // DEBUG TAP: this inlines the pure-structural rtl/bench/hyperram_bw_top.sv (identical parameters
  // and wiring — the core modules are NOT modified) so the hyperbus_capture debug module below can
  // observe the av_* handshake alongside the HyperBus pins.
  wire [31:0] av_address;
  wire [15:0] av_burstcount;
  wire        av_read;
  wire        av_write;
  wire [15:0] av_writedata;
  wire [15:0] av_readdata;
  wire        av_readdatavalid;
  wire        av_waitrequest;
  wire [31:0] av_dbg;         // DEBUG: ctrl/front-end/FIFO taps (see hyperram_avalon dbg_bus map)

  hyperram_bw_test #(
    .DATA_WIDTH     (16),
    .ADDR_WIDTH     (32),
    .LEN_WIDTH      (16),
    .BURST_WORDS    (64),              // EXPERIMENT: larger Avalon burst — writes LEN<=64 as ONE HyperBus
                                       // burst (no write->write boundary) to confirm the boundary drop
    .CSR_ADDR_WIDTH (CSR_AW),
    .VERSION_MAGIC  (32'h4842_5754)    // "HBWT"
  ) u_bw (
    .clk             (clk),     // 50 MHz CK word clock (controller)
    .rst             (sys_rst),
    // CSR slave (word addressed) — driven by the JTAG-Avalon master through the adapter above
    .csr_address     (csr_address),
    .csr_read        (csr_read),
    .csr_readdata    (csr_readdata),
    .csr_write       (csr_write),
    .csr_writedata   (csr_writedata),
    .csr_waitrequest (csr_waitrequest),
    // Avalon-MM master -> hyperram_avalon slave (tapped by hyperbus_capture)
    .m_address       (av_address),
    .m_burstcount    (av_burstcount),
    .m_read          (av_read),
    .m_write         (av_write),
    .m_writedata     (av_writedata),
    .m_readdata      (av_readdata),
    .m_readdatavalid (av_readdatavalid),
    .m_waitrequest   (av_waitrequest)
  );

  hyperram_avalon #(
    .DQ_WIDTH         (8),
    .DATA_WIDTH       (16),
    .ADDR_WIDTH       (32),
    .LEN_WIDTH        (16),
    .LATENCY_CLOCKS   (6),             // hyperram_bw_top defaults (mirror sim/tb_avalon.sv)
    .FIXED_LATENCY    (1'b1),
    .MAX_BURST_WORDS  (0),
    .PROGRAM_CR       (1'b1),
    .POR_DELAY_CYCLES (0),
    .INIT_CR0         (16'h8F1F),      // latency code 6, fixed
    .PHY_VARIANT      ("SDR"),         // portable single-clock-phase SDR PHY (unblocks 24403/24404)
    .DIFF_CK          (1'b0),          // single-ended CK on AXC3000 (no hb_ck_n pin)
    // Winbond W957D8NB drives a read-strobe PREAMBLE: RWDS toggles with DQ Hi-Z (=0x00) for ONE CK
    // cycle before the first real read byte (on-silicon capture cap_sample_dump.txt: preamble pulse
    // at idx85/86, then the first data word at idx87..). The SDR PHY discards that one leading RWDS
    // rising edge so byte pairing starts on the real read data — without it, the preamble edge paired
    // into a phantom {0x00,0x00} word and the bandwidth test hung (STATUS never reached done).
    .RD_PREAMBLE_SKIP (1)
  ) u_hyperram (
    .clk               (clk),
    .clk90             (clk2x),  // 100 MHz 2x byte clock (SDR PHY repurposes clk90 as the byte clock)
    .clk_ref           (clk),    // unused by the SDR PHY (tie-off)
    .rst               (sys_rst),
    // Avalon-MM slave (driven by the bench master above); full-word writes only
    .avs_address       (av_address),
    .avs_read          (av_read),
    .avs_write         (av_write),
    .avs_writedata     (av_writedata),
    .avs_byteenable    (2'b11),
    .avs_burstcount    (av_burstcount),
    .avs_readdata      (av_readdata),
    .avs_readdatavalid (av_readdatavalid),
    .avs_waitrequest   (av_waitrequest),
    // split HyperBus device pins
    .hb_ck             (hb_ck_int),
    .hb_ck_n           (hb_ck_n_int),
    .hb_cs_n           (hb_cs_n_int),
    .hb_rst_n          (hb_rst_n_int),
    .hb_dq_o           (hb_dq_o_int),
    .hb_dq_oe          (hb_dq_oe_int),
    .hb_dq_i           (hb_dq_i_int),
    .hb_rwds_o         (hb_rwds_o_int),
    .hb_rwds_oe        (hb_rwds_oe_int),
    .hb_rwds_i         (hb_rwds_i_int),
    // status
    .init_done         (init_done),
    // debug taps -> capture
    .dbg_bus           (av_dbg)
  );

  // =========================================================================
  // On-chip logic analyzer (DEBUG): records the HyperBus pin-side signals + the av_* handshake at
  // 100 MHz from the first hb_cs_n low after arming. CSR at base 0x100 (decode above). See
  // hyperbus_capture.sv for the sample bit map and sysconsole/cap_dump.tcl for the host dump flow.
  // =========================================================================
  hyperbus_capture #(
    .DEPTH (1024)
  ) u_cap (
    .clk              (clk),
    .rst              (sys_rst),
    .cap_clk          (clk2x),
    // probes
    .hb_cs_n          (hb_cs_n_int),
    .hb_ck            (hb_ck_int),
    .hb_dq_oe         (hb_dq_oe_int),
    .hb_dq_o          (hb_dq_o_int),
    .hb_dq_i          (hb_dq_i_int),
    .hb_rwds_oe       (hb_rwds_oe_int),
    .hb_rwds_o        (hb_rwds_o_int),
    .hb_rwds_i        (hb_rwds_i_int),
    .av_read          (av_read),
    .av_write         (av_write),
    .av_waitrequest   (av_waitrequest),
    .av_readdatavalid (av_readdatavalid),
    .dbg_bus          (av_dbg),
    // CSR slave (base 0x100)
    .csr_address      (cap_address),
    .csr_read         (cap_read),
    .csr_readdata     (cap_readdata),
    .csr_write        (cap_write),
    .csr_writedata    (m_writedata),
    .csr_waitrequest  (cap_waitrequest)
  );

  // =========================================================================
  // Board pad ring: split PHY signals -> real Agilex inout pads (tennm_ph2 io_obuf/io_ibuf)
  // =========================================================================
  wire hb_ck_n_nc;   // single-ended: ck_n obuf output unused (constant, swept by the fitter)

  hyperbus_pads_altera #(
    .DQ_WIDTH (8)
  ) u_pads (
    // split PHY side
    .phy_hb_ck      (hb_ck_int),
    .phy_hb_ck_n    (hb_ck_n_int),
    .phy_hb_cs_n    (hb_cs_n_int),
    .phy_hb_rst_n   (hb_rst_n_int),
    .phy_hb_dq_o    (hb_dq_o_int),
    .phy_hb_dq_oe   (hb_dq_oe_int),
    .phy_hb_dq_i    (hb_dq_i_int),
    .phy_hb_rwds_o  (hb_rwds_o_int),
    .phy_hb_rwds_oe (hb_rwds_oe_int),
    .phy_hb_rwds_i  (hb_rwds_i_int),
    // device pads
    .hb_ck          (hb_ck),
    .hb_ck_n        (hb_ck_n_nc),
    .hb_cs_n        (hb_cs_n),
    .hb_rst_n       (hb_rst_n),
    .hb_dq          (hb_dq),
    .hb_rwds        (hb_rwds)
  );

endmodule
