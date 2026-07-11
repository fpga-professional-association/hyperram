// hyperbus_gpio_io — AXC3000 board I/O layer for the 200 MHz DDIO build, built from VENDOR-WRAPPED
// altera_gpio cells instead of raw tennm_ph2 atoms (qsys/hbgpio.qsys: hbgpio_dq_cell, hbgpio_ck_cell).
//
// Why this exists (2026-07-09 bring-up findings, see the ddio-200 branch history):
//   * Raw tennm_ph2_ddio_out for CK produced NO wire activity on this board in any configuration; the
//     FABRIC2X workaround (SDR-PHY CK generator) works but needs a 2x core clock whose minimum pulse
//     width caps CK at ~176 MHz. The vendor ck_cell (altera_gpio, DDR out + CKE) runs the I/O at 1x —
//     no 2x clock anywhere — which is the only honest route to the device's 200 MHz rating.
//   * dq_cell is a bidir DDR GPIO with DDIO_WITH_DELAY (MODE_DDR_W_DLY): the cell delays the DATA
//     input against its input-register clock (ck_in = raw RWDS), giving the hold margin that the
//     undelayed strobe-clocked fabric capture lacked at 175 MHz.
//   * RWDS itself stays on the bring-up-proven path: tennm TX atom (write mask) + inferred tristate
//     pad + raw input (it must supply the raw strobe for dq_cell.ck_in and the controller level sync).
//
// This module presents the hyperbus_ctrl-facing phy_* interface (same shape as the frozen PHY
// contract's ctrl side) and OWNS the hb_dq / hb_rwds / hb_ck pins (pad ring inside the GPIO cells /
// inferred tristate). hb_cs_n / hb_rst_n are plain registered outputs. Board-only file — not part of
// the portable IP; compiled by Quartus only.
//
// Alignment knobs (the wire tells the truth — sweep with the capture module + first-error CSRs):
//   TX_B_DLY   : 1 = delay the byte-B (lo) TX stream one clk (the alignment the FABRIC2X CK needed;
//                the vendor CK cell's phase may differ — flip if the {A(k),B(k+1)} signature appears).
//   CK_DIN_HI  : 1 = ck_cell.din = {1,0} (high sub-phase first); 0 = {0,1} (half-cycle CK shift).
//   RD_PREAMBLE_SKIP : device read-strobe preamble CK cycles to discard (W957D8NB: 1).

`timescale 1ns/1ps

/* verilator lint_off UNDRIVEN */
/* verilator lint_off UNUSEDSIGNAL */
module hyperbus_gpio_io #(
    parameter int unsigned DQ_WIDTH         = 8,
    parameter int unsigned RD_PREAMBLE_SKIP = 1,
    parameter bit          TX_B_DLY         = 1'b1,
    parameter bit          CK_DIN_HI        = 1'b1,
    // Effective-arm delay (clk cycles): the controller arms the receiver at latency start, but
    // between the device's latency-indicator release and the read preamble both pins FLOAT — at
    // 200 MHz the floating RWDS couples CK runts that the pairing would eat as words. Delaying the
    // effective arm keeps the receiver blind through the float window; data arrives >= 2x-latency
    // (24 CK at latency 6x2) after the controller arms, so 16 leaves >= 6 CK of margin ahead of
    // the real preamble. Sweepable; 0 = arm immediately (the original exposure).
    parameter int unsigned ARM_DELAY_CYCLES = 16,
    // CK generator select: "VENDOR" = hbgpio_ck_cell at 1x (no 2x clock anywhere; needed for a
    // true 200 MHz build). "FABRIC2X" = the silicon-proven SDR-style fabric generator — REQUIRES
    // clk_smp to carry a 2x-CK 0-deg core clock (caps CK at ~176 MHz via min-pulse, but proven).
    parameter              CK_GEN           = "VENDOR",
    // Vendor-cell gating style: "CKE" = the cell's clock-enable pin (BROKEN for writes on this
    // silicon: reads fine, writes never commit — likely a truncated/degenerate final pulse).
    // "DIN" = cke tied 1, gating through the registered din data path (the mechanism the working
    // FABRIC2X generator uses).
    parameter              CK_GATE          = "DIN"
) (
    input  logic                  clk,          // CK-rate word clock (controller + all launches)
    input  logic                  clk_smp,      // CK-rate RX sampling clock, PLL-phase-shifted (90 deg
                                                // default) — CORE-ONLY; the LOCAL1X eye-position knob
    input  logic                  rst,          // synchronous, active-high

    // ---- ctrl-facing (mirror of hyperbus_ctrl's PHY master interface) ----
    input  logic                  phy_cs_n,
    input  logic                  phy_rst_n,
    input  logic                  phy_ck_en,
    input  logic [2*DQ_WIDTH-1:0] phy_dq_o,     // [hi]=byte A (1st edge), [lo]=byte B (2nd edge)
    input  logic                  phy_dq_oe,
    input  logic [1:0]            phy_rwds_o,   // [1]=1st-phase mask, [0]=2nd-phase mask
    input  logic                  phy_rwds_oe,
    input  logic                  phy_rd_arm,
    input  logic                  dbg_ck_stretch_off,  // issue #13 L-E: 1 = disable the ck_stretch trailing
                                                       // masked cycle (A/B whether law-3 is stretch-inflicted)
    output logic [2*DQ_WIDTH-1:0] phy_dq_i,     // recovered read word (byte A high half)
    output logic                  phy_dq_i_valid,
    output logic                  phy_rwds_i,

    // ---- board pins (this module owns the pad ring for dq/rwds/ck) ----
    output logic                  hb_ck,
    output logic                  hb_cs_n,
    output logic                  hb_rst_n,
    inout  wire  [DQ_WIDTH-1:0]   hb_dq,
    inout  wire                   hb_rwds
);

  localparam int unsigned PHYW = 2 * DQ_WIDTH;

  // ================================================================================================
  //  Control pipeline — one uniform clk of latency so cs#/rst#/enables track the GPIO-cell DDR
  //  launch latency (the cells register din on ck_out).
  // ================================================================================================
  logic csn_q, rstn_q, dqoe_q, rwoe_q, cken_q;
  always_ff @(posedge clk) begin
    if (rst) begin
      csn_q  <= 1'b1;
      rstn_q <= 1'b0;
      dqoe_q <= 1'b0;
      rwoe_q <= 1'b0;
      cken_q <= 1'b0;
    end else begin
      csn_q  <= phy_cs_n;
      rstn_q <= phy_rst_n;
      dqoe_q <= phy_dq_oe;
      rwoe_q <= phy_rwds_oe | ck_stretch;   // keep driving the mask through the stretch cycle
      cken_q <= phy_ck_en;
    end
  end
  assign hb_cs_n  = csn_q;
  assign hb_rst_n = rstn_q;

  // ================================================================================================
  //  TX byte-B one-clk delay (silicon alignment, see header). Reset-less datapath regs.
  // ================================================================================================
  logic [DQ_WIDTH-1:0] dq_b_dly;
  logic                rwds_b_dly;
  always_ff @(posedge clk) begin
    dq_b_dly   <= phy_dq_o[DQ_WIDTH-1:0];
    rwds_b_dly <= phy_rwds_o[0];
  end
  wire [DQ_WIDTH-1:0] tx_b = TX_B_DLY ? dq_b_dly : phy_dq_o[DQ_WIDTH-1:0];
  wire                tx_rwds_b = TX_B_DLY ? rwds_b_dly : phy_rwds_o[0];

  // ================================================================================================
  //  DQ — raw tennm_ph2_ddio_out TX atoms (proven at 175 in the atom PHY) + inferred tristate pads.
  //  The vendor dq_cell was retired: its strobe-clocked input DDIO registers read constant zeros
  //  (hard input DDR regs cannot be clocked from a fabric-routed data pin on this silicon — confirmed
  //  for raw atoms AND vendor cells), and leaving them configured-but-unused recreates the pad's
  //  P2X input-term conflict. RX is LOCAL1X fabric sampling below; the raw ibuf nets feed it.
  // ================================================================================================
  logic [DQ_WIDTH-1:0] dq_ddr_out;
  genvar gi;
  generate
    for (gi = 0; gi < DQ_WIDTH; gi = gi + 1) begin : g_dq_tx
      tennm_ph2_ddio_out #(
        .mode      ("MODE_DDR"),
        .asclr_ena ("ASCLR_ENA_NONE"),
        .sclr_ena  ("SCLR_ENA_NONE")
      ) u_dq_tx (
        .ena      (1'b1),
        .areset   (1'b1),   // ACTIVE-LOW reset_n on ph2 atoms
        .sreset   (1'b0),
        // PROVEN-STREAMING config (200 MHz, RD_CYCLES=798 with real device strobes): byte A on
        // datainhi + one-clk-delayed byte B on datainlo — the same alignment the FABRIC2X 175 MHz
        // build validated. (The A-on-datainlo "physics" swap regressed the CA to a dead device —
        // the ck_cell's rise evidently lands mid-second-half, exactly where A sits here.)
        .datainhi (phy_dq_o[DQ_WIDTH + gi]),  // byte A bit i
        .datainlo (tx_b[gi]),                 // byte B bit i (TX_B_DLY one-clk-delayed)
        .dataout  (dq_ddr_out[gi]),
        .clk      (clk)
      );
      assign hb_dq[gi] = dqoe_q ? dq_ddr_out[gi] : 1'bz;
    end
  endgenerate

  wire [DQ_WIDTH-1:0] dq_raw   = hb_dq;     // raw input-buffer view (fabric)
  wire                rwds_raw = hb_rwds;   // raw RWDS view

  // ================================================================================================
  //  CK — vendor DDR-out GPIO cell with clock-enable: emits clk on the pin while cke is high.
  //  CK_DIN_HI selects which sub-phase carries the high level (a half-cycle phase knob).
  // ================================================================================================
  // CK-train stretch: one extra masked cycle after phy_ck_en falls. Writes are open-loop — if the
  // vendor cell's CKE gating clips the final pulse, the device never latches the burst end and the
  // whole write is discarded (reads are RWDS-gated and tolerate it, hence the read/write asymmetry
  // on silicon). The extra cycle is driven with RWDS mask HIGH, which the device provably skips
  // (issue #1 trailing-masked-edge experiment).
  logic cken_d1;
  always_ff @(posedge clk) cken_d1 <= rst ? 1'b0 : cken_q;
  // issue #13 L-E: dbg_ck_stretch_off=1 kills the stretch cycle at its single source, so all three
  // fan-outs (rwoe_q hold, RWDS datainhi/datainlo mask, vendor-CK cke) drop together. Default 0 =
  // legacy (the stretch cycle is present exactly as shipped).
  wire ck_stretch = (cken_d1 & ~cken_q) & ~dbg_ck_stretch_off;   // first cycle after enable falls

  generate
    if (CK_GEN == "FABRIC2X") begin : g_ck_fab2x
      // Verbatim port of the silicon-proven SDR/altera-PHY CK generator. clk_smp = 2x CK, 0 deg,
      // CORE-ONLY. Edges land at T/4 and 3T/4 of the word cycle by construction.
      logic rst2x_meta, rst2x;
      always_ff @(posedge clk_smp) begin
        rst2x_meta <= rst;
        rst2x      <= rst2x_meta;
      end
      logic tgl;
      always_ff @(posedge clk) tgl <= rst ? 1'b0 : ~tgl;
      logic tgl_s1, tgl_s2, tgl_s3;
      always_ff @(posedge clk_smp) begin
        if (rst2x) begin tgl_s1 <= 1'b0; tgl_s2 <= 1'b0; tgl_s3 <= 1'b0; end
        else       begin tgl_s1 <= tgl;  tgl_s2 <= tgl_s1; tgl_s3 <= tgl_s2; end
      end
      wire beat_a = tgl_s2 ^ tgl_s3;
      logic beat_a_d1;
      always_ff @(posedge clk_smp) beat_a_d1 <= rst2x ? 1'b0 : beat_a;
      logic cken_w;
      always_ff @(posedge clk_smp) begin
        if (rst2x)       cken_w <= 1'b0;
        else if (beat_a) cken_w <= cken_q;
      end
      logic ck_r;
      /* verilator lint_off SYNCASYNCNET */
      always_ff @(negedge clk_smp) ck_r <= rst2x ? 1'b0 : (cken_w & beat_a_d1);
      /* verilator lint_on SYNCASYNCNET */
      assign hb_ck = ck_r;
    end else begin : g_ck_vendor
      hbgpio_ck_cell u_ck_cell (
        .ck      (clk),
        // DIN gating uses the PRE-pipeline enable (phy_ck_en): the cell's registered din path
        // adds one cycle vs the cke path, and gating from cken_q shifted the whole train +1 CK
        // (CA garbage, dead bus — silicon-observed). phy_ck_en restores the cke-equivalent timing.
        .din     ((CK_GATE == "DIN") ? (phy_ck_en ? (CK_DIN_HI ? 2'b10 : 2'b01) : 2'b00)
                                     : (CK_DIN_HI ? 2'b10 : 2'b01)),
        .cke     ((CK_GATE == "DIN") ? 1'b1 : (cken_q | ck_stretch)),
        .pad_out (hb_ck)
      );
    end
  endgenerate

  // ================================================================================================
  //  RWDS — bring-up-proven path: tennm TX atom for the DDR write mask + inferred tristate pad;
  //  the raw input feeds dq_cell.ck_in (above), the strobe-domain RX below, and the level sync.
  // ================================================================================================
  logic rwds_ddr_out;
  tennm_ph2_ddio_out #(
    .mode      ("MODE_DDR"),
    .asclr_ena ("ASCLR_ENA_NONE"),
    .sclr_ena  ("SCLR_ENA_NONE")
  ) u_rwds_tx (
    .ena      (1'b1),
    .areset   (1'b1),   // ACTIVE-LOW reset_n on ph2 atoms
    .sreset   (1'b0),
    .datainhi (phy_rwds_o[1] | ck_stretch),   // stretch cycle: mask HIGH (device skips it)
    .datainlo (tx_rwds_b     | ck_stretch),
    .dataout  (rwds_ddr_out),
    .clk      (clk)
  );
  assign hb_rwds = rwoe_q ? rwds_ddr_out : 1'bz;

  // ================================================================================================
  //  RX : LOCAL1X — both-edge fabric sampling at the 1x clock + the silicon-proven SDR edge-detect
  //  pairing. posedge and negedge registers each sample DQ/RWDS once per clk, yielding the same
  //  2-samples-per-CK stream the SDR PHY's algorithm consumed from its 2x clock — with NO 2x clock.
  //  The pairing FSM is fully synchronous in the clk domain and processes both samples per cycle in
  //  arrival order (pos_q from the previous posedge, then neg_q from the mid-cycle negedge). RWDS
  //  RISING edges across consecutive samples tag byte A, falling edges complete {A,B} words —
  //  self-aligning against the device's tCKDS flight-delay phase, exactly like the SDR PHY.
  // ================================================================================================
  localparam int unsigned RXF_DEPTH = 32;
  localparam int unsigned RXF_AW    = $clog2(RXF_DEPTH);
  localparam int unsigned SKIPW     = (RD_PREAMBLE_SKIP == 0) ? 1 : $clog2(RD_PREAMBLE_SKIP + 1);

  // Both-edge input sampling (reset-less datapath), read DIRECTLY by the posedge-clk FSM below —
  // NO extra retiming stage on any of the four taps. This matters: a register that is itself
  // clocked by posedge clk (dq_p0_q) is necessarily read STALE by another posedge-clk process (the
  // FSM), because both fire on the identical edge and the FSM's Active-region read always sees the
  // pre-this-edge value — i.e. one full clk cycle after capture (s0 = k-1, not k-0). dq_p180_q
  // avoids that penalty because its OWN capture edge (negedge clk) falls chronologically BEFORE the
  // FSM's posedge within the same cycle, so the FSM always sees the freshly captured value (s2 =
  // k-0.5, a plain 2.5 ns related-clock path). The clk_smp taps (dq_p90_q/dq_p270_q) are exactly
  // analogous to dq_p180_q, not to dq_p0_q: clk_smp's posedge/negedge (T/4, 3T/4 into the clk cycle)
  // also land chronologically before the FSM's NEXT posedge clk, so a DIRECT read lands at s1 =
  // k-0.75 / s3 = k-0.25 (related-clock paths, STA-timed, same treatment as dq_p180_q) with no extra
  // latency. An earlier revision added a `dq_p90_r/dq_p270_r <= dq_p90_q/dq_p270_q` retiming stage
  // clocked by posedge clk "to bring them into the clk domain" — but that retiming register is
  // ITSELF a posedge-clk register read by the posedge-clk FSM, so it suffers the exact dq_p0_q-style
  // one-cycle read penalty ON TOP of its intended 0.75/0.25-cycle age, landing at k-1.75 / k-1.25
  // instead — scrambling the true chronological sample order (90 deg data ends up OLDER than 0 deg,
  // 270 deg OLDER than 0 deg) and producing a phantom word at the very first processed edge
  // (sim-confirmed: a free-running time-fingerprint probe showed the retimed taps trailing the
  // direct taps by exactly one clk period, every cycle). Removed; do not reintroduce it.
  // QUAD1X sampling: four phases per word cycle from TWO 1x clocks — clk (0/180 deg via pos/neg
  // edge) and clk_smp (90/270 deg) — the oversampling density of a 2x clock with NO 2x clock (the
  // min-pulse ceiling does not apply). The device's output timing wobbles briefly at its internal
  // 32-byte page crossings (silicon: one bit0 flip every 16 words on a fixed grid — knob-immune);
  // edge-detect pairing over the 4-phase stream follows RWDS at quarter-cycle resolution the way
  // the source-synchronous SDR PHY did, absorbing the wobble.
  // Under CK_GEN="FABRIC2X" clk_smp carries the 2x CK clock instead — fall back to 2-phase clk-only
  // sampling (the 175 MHz configuration, already silicon-proven clean).
  logic [DQ_WIDTH-1:0] dq_p0_q, dq_p90_q, dq_p180_q, dq_p270_q;
  logic                rw_p0_q, rw_p90_q, rw_p180_q, rw_p270_q;
  always_ff @(posedge clk) begin
    dq_p0_q <= dq_raw; rw_p0_q <= rwds_raw;
  end
  /* verilator lint_off SYNCASYNCNET */
  always_ff @(negedge clk) begin
    dq_p180_q <= dq_raw; rw_p180_q <= rwds_raw;
  end
  always_ff @(posedge clk_smp) begin
    dq_p90_q <= dq_raw; rw_p90_q <= rwds_raw;
  end
  always_ff @(negedge clk_smp) begin
    dq_p270_q <= dq_raw; rw_p270_q <= rwds_raw;
  end
  /* verilator lint_on SYNCASYNCNET */

  logic [PHYW-1:0]  rxf_mem [RXF_DEPTH];
  logic [RXF_AW:0]  wptr_bin, rptr_bin;
  logic [SKIPW-1:0] pre_skip;
  logic [DQ_WIDTH-1:0] rx_byte_a;
  logic have_a, rwds_prev;

  // Four samples per processing cycle, arrival order (all values from the PREVIOUS word cycle,
  // stable in the clk domain at this posedge):
  //   s[0] = 0 deg (dq_p0_q),  s[1] = 90 deg (dq_p90_q), s[2] = 180 deg (dq_p180_q),
  //   s[3] = 270 deg (dq_p270_q)
  // Under FABRIC2X only s[0]/s[2] carry unique data (clk_smp is the 2x clock there) — the rise/fall
  // detect still works because duplicate samples produce no extra edges.
  wire [3:0]              smp_rw = (CK_GEN == "FABRIC2X")
                                   ? {rw_p180_q, rw_p180_q, rw_p0_q, rw_p0_q}
                                   : {rw_p270_q, rw_p180_q, rw_p90_q, rw_p0_q};
  logic [DQ_WIDTH-1:0]    smp_dq [4];
  always_comb begin
    if (CK_GEN == "FABRIC2X") begin
      smp_dq[0] = dq_p0_q;   smp_dq[1] = dq_p0_q;
      smp_dq[2] = dq_p180_q; smp_dq[3] = dq_p180_q;
    end else begin
      smp_dq[0] = dq_p0_q;   smp_dq[1] = dq_p90_q;
      smp_dq[2] = dq_p180_q; smp_dq[3] = dq_p270_q;
    end
  end

  // Effective arm = controller arm, rise-delayed by ARM_DELAY_CYCLES (fall passes through, so the
  // disarm flush is never delayed). Keeps the receiver blind through the latency-window float
  // (coupling runts on RWDS at 200 MHz read as phantom words otherwise).
  localparam int unsigned ARMW = (ARM_DELAY_CYCLES == 0) ? 1 : $clog2(ARM_DELAY_CYCLES + 1);
  logic [ARMW-1:0] arm_cnt;
  logic            rd_arm_eff;
  always_ff @(posedge clk) begin
    if (rst || !phy_rd_arm) begin
      arm_cnt    <= '0;
      rd_arm_eff <= 1'b0;
    end else if (arm_cnt != ARMW'(ARM_DELAY_CYCLES)) begin
      arm_cnt    <= arm_cnt + 1'b1;
    end else begin
      rd_arm_eff <= 1'b1;
    end
  end

  // Sequential 4-sample scan per clk cycle (loop unrolled by synthesis). At a CK-rate strobe at
  // most TWO words can complete per cycle (a full RWDS period spans 4 samples; wobble can squeeze
  // two falls into one processing window) — the scan handles any mix, pushing per fall.
  always_ff @(posedge clk) begin
    if (rst || !rd_arm_eff) begin
      wptr_bin  <= '0;
      have_a    <= 1'b0;
      rwds_prev <= smp_rw[3];   // track the LIVE level while blind: no phantom edge at arm release
      pre_skip  <= SKIPW'(RD_PREAMBLE_SKIP);
      rx_byte_a <= '0;
    end else begin
      logic                prev;
      logic                hav;
      logic [DQ_WIDTH-1:0] abyte;
      logic [SKIPW-1:0]    skp;
      logic [RXF_AW:0]     wp;
      prev  = rwds_prev;
      hav   = have_a;
      abyte = rx_byte_a;
      skp   = pre_skip;
      wp    = wptr_bin;
      for (int s = 0; s < 4; s++) begin
        if (smp_rw[s] & ~prev) begin            // rise: byte A candidate
          if (skp != '0) begin
            skp = skp - 1'b1;
            hav = 1'b0;
          end else begin
            abyte = smp_dq[s];
            hav   = 1'b1;
          end
        end else if (~smp_rw[s] & prev & hav) begin   // fall: complete {A,B}
          rxf_mem[wp[RXF_AW-1:0]] <= {abyte, smp_dq[s]};
          wp  = wp + 1'b1;
          hav = 1'b0;
        end
        prev = smp_rw[s];
      end
      rwds_prev <= prev;
      have_a    <= hav;
      rx_byte_a <= abyte;
      pre_skip  <= skp;
      wptr_bin  <= wp;
    end
  end

  wire  [RXF_AW:0] wptr_gray = wptr_bin ^ (wptr_bin >> 1);
  logic [RXF_AW:0] wgray_s1, wgray_s2;
  always_ff @(posedge clk) begin
    if (rst || !rd_arm_eff) begin
      wgray_s1 <= '0;
      wgray_s2 <= '0;
    end else begin
      wgray_s1 <= wptr_gray;
      wgray_s2 <= wgray_s1;
    end
  end

  function automatic logic [RXF_AW:0] gray2bin(input logic [RXF_AW:0] g);
    logic [RXF_AW:0] b;
    for (int i = RXF_AW; i >= 0; i--)
      b[i] = (i == RXF_AW) ? g[RXF_AW] : (b[i+1] ^ g[i]);
    return b;
  endfunction
  wire [RXF_AW:0] wptr_bin_s = gray2bin(wgray_s2);
  wire            rxf_empty  = (rptr_bin == wptr_bin_s);

  always_ff @(posedge clk) begin
    if (rst) begin
      rptr_bin       <= '0;
      phy_dq_i       <= '0;
      phy_dq_i_valid <= 1'b0;
    end else begin
      phy_dq_i_valid <= 1'b0;
      if (!rd_arm_eff) begin
        rptr_bin <= '0;
      end else if (!rxf_empty) begin
        phy_dq_i       <= rxf_mem[rptr_bin[RXF_AW-1:0]];
        phy_dq_i_valid <= 1'b1;
        rptr_bin       <= rptr_bin + 1'b1;
      end
    end
  end

  // RWDS level sync for the controller's latency select / stall watch.
  logic rwds_s1;
  always_ff @(posedge clk) begin
    if (rst) begin
      rwds_s1    <= 1'b0;
      phy_rwds_i <= 1'b0;
    end else begin
      rwds_s1    <= rwds_raw;
      phy_rwds_i <= rwds_s1;
    end
  end

endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNDRIVEN */
