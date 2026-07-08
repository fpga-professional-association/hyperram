// top — AXC3000 board top level for the HyperBus bandwidth test (Agilex-3 A3CY100BM16AE7S).
//
// Data path (all on the IOPLL 50 MHz word clock):
//   bw_sys (Qsys)                              fpga/axc3000/qsys/
//     * IOPLL: clk (50 MHz, 0deg) + clk90 (50 MHz, +90deg) from the 25 MHz board XO
//     * reset controller -> synchronous active-high fabric reset
//     * JTAG-to-Avalon-MM master bridge  --- exported Avalon-MM master (byte addressed)
//         |
//         v  (byte->word CSR adapter below)
//   hyperram_bw_top  (rtl/bench/)  = hyperram_bw_test (traffic gen + scoreboard, CSR slave)
//                                    -> hyperram_avalon (Avalon-MM slave -> ctrl -> Agilex PHY)
//         |  split HyperBus device pins (hb_*_o / hb_*_oe / hb_*_i)
//         v
//   hyperbus_pads_altera (fpga/axc3000/)  = tennm_ph2 I/O buffers -> real inout board pads
//
// The measured WR_CYCLES/RD_CYCLES cover only the on-chip Avalon datapath; JTAG is control plane
// only (PLAN §8 method E compliant). Read back with sysconsole/bw_read.tcl.
//
// CONSERVATIVE CLOCK PLAN: 50 MHz HyperBus word clock => DDR hb_ck ~50 MHz, ~100 MB/s/direction
// theoretical on the x8 bus. Deliberately low so the un-calibrated Agilex read eye is wide.
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
  wire        clk;          // 50 MHz word clock
  wire        clk90;        // 50 MHz, +90 deg
  wire        pll_locked;
  wire        sys_rst;      // synchronous, active-high fabric reset

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
    .clk90_clk            (clk90),
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
  // Byte-address (JTAG master)  ->  word-address CSR slave adapter
  //   The bw_test CSR slave is word-addressed (csr_address = byte_offset>>2), reads combinational,
  //   0 wait states. The JTAG-Avalon master is byte-addressed and pipelined (expects readdatavalid).
  //   8 registers (0x00..0x1C) => 3 word-address bits = m_address[4:2].
  // =========================================================================
  localparam int CSR_AW = 3;

  wire [CSR_AW-1:0] csr_address    = m_address[CSR_AW+1:2];
  wire              csr_read       = m_read;
  wire              csr_write      = m_write;
  wire [31:0]       csr_writedata  = m_writedata;
  wire [31:0]       csr_readdata;
  wire              csr_waitrequest;

  assign m_waitrequest = csr_waitrequest;   // tied low inside the slave (0 wait states)

  // Pipeline the combinational CSR read to the Avalon read-latency-1 contract the JTAG master expects:
  // capture readdata on the accept cycle, present it with readdatavalid the next cycle.
  logic [31:0] rd_hold;
  logic        rdv_q;
  always_ff @(posedge clk) begin
    if (sys_rst) begin
      rd_hold <= 32'd0;
      rdv_q   <= 1'b0;
    end else begin
      rd_hold <= csr_readdata;                       // csr_readdata is combinational off csr_address
      rdv_q   <= csr_read & ~csr_waitrequest;        // accepted this cycle (waitrequest is 0)
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

  assign LED1 = ~led_done_q;    // active-low: lit when done
  assign RLED = ~led_error_q;   // active-low: lit when error
  assign GLED = ~pll_locked;    // active-low: lit when PLL locked

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

  hyperram_bw_top #(
    .PHY_VARIANT ("ALTERA"),   // Agilex-3 tennm_ph2 DDR-IO PHY
    .DIFF_CK     (1'b0)        // single-ended CK on AXC3000 (no hb_ck_n pin)
  ) u_bw (
    .clk             (clk),
    .clk90           (clk90),
    .clk_ref         (clk),     // calibration ref unused for functional correctness in the PHY
    .rst             (sys_rst),
    // CSR slave (word addressed) — driven by the JTAG-Avalon master through the adapter above
    .csr_address     (csr_address),
    .csr_read        (csr_read),
    .csr_readdata    (csr_readdata),
    .csr_write       (csr_write),
    .csr_writedata   (csr_writedata),
    .csr_waitrequest (csr_waitrequest),
    // split HyperBus device pins
    .hb_ck           (hb_ck_int),
    .hb_ck_n         (hb_ck_n_int),
    .hb_cs_n         (hb_cs_n_int),
    .hb_rst_n        (hb_rst_n_int),
    .hb_dq_o         (hb_dq_o_int),
    .hb_dq_oe        (hb_dq_oe_int),
    .hb_dq_i         (hb_dq_i_int),
    .hb_rwds_o       (hb_rwds_o_int),
    .hb_rwds_oe      (hb_rwds_oe_int),
    .hb_rwds_i       (hb_rwds_i_int),
    .init_done       (init_done)
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
