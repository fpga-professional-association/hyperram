// hyperbus_phy_sdr — PORTABLE single-clock-phase SDR HyperBus PHY (LOW-SPEED bring-up variant).
//
// Implements the frozen `hyperbus_phy` port contract (docs/INTERFACES.md) with NO vendor primitives
// and NO DDR I/O (no tennm_ph2_ddio, no ODDR/IDDR): every pin is driven / sampled by ordinary
// posedge-triggered flops and the board tri-states with `assign dq = oe ? dq_o : 'z` in the pad ring.
// Fully Verilator-simulable (5.020) AND synthesizable on any FPGA.
//
//   *** Header note: this is the LOW-SPEED path (fabric byte clock = 2x the HyperBus CK). The DDIO
//   *** variant (hyperbus_phy_altera) is the >~100 MHz path. This SDR PHY exists to UNBLOCK the
//   *** AXC3000 fit: the DDIO PHY routed TWO IOPLL phases (clk 0deg + clk90 90deg) into the Bank-3A
//   *** I/O periphery (Quartus err 24403/24404). An SDR PHY needs only ONE clock in the periphery.
//
// ---------------------------------------------------------------------------------------------------
// WHY TWO CLOCKS, and WHAT THEY ARE (this is the crux — read before touching the timing):
//
//   The frozen controller (hyperbus_ctrl, DESIGN.md §2/§4) is WORD-per-clk: it presents one 16-bit
//   HyperBus word — TWO DQ bytes, [PHYW-1:DQ_WIDTH]=byte A, [DQ_WIDTH-1:0]=byte B — on `phy_dq_o`
//   every `clk` cycle, and expects one HyperBus CK period per `clk` cycle. A pure-SDR wire can only
//   launch ONE byte per fabric edge, so to place two bytes per CK period the fabric byte engine must
//   run at 2x the controller/CK rate. Hence:
//
//     * `clk`   = the CK-rate WORD clock  (e.g. 50 MHz). Clocks the controller + this PHY's
//                 controller-facing logic. hb_ck comes out at THIS rate.  == the HyperBus CK.
//     * `clk90` = REPURPOSED as the 2x BYTE clock (e.g. 100 MHz, 0deg). Clocks the SDR output
//                 registers, the hb_ck generator, and the read capture. This is the SINGLE clock
//                 that reaches the I/O periphery on the board — there is NO 90deg phase anymore.
//                 (The frozen port is named `clk90`; only its SDR-variant MEANING differs. The
//                 port name/direction/width are unchanged, so the interface contract still holds.)
//     * `clk_ref` = unused (tie-off).
//
//   Both clocks come from the SAME PLL (clk = clk90 / 2, phase-related). "hb_ck = a divide-by-2 of
//   the fabric byte clock" per the task: hb_ck = clk90/2 = clk. In task terms: fabric = clk90 = 2xCK,
//   CK = hb_ck = clk. One byte leaves the pins per clk90 cycle => peak = f(clk90) * 1 byte/s
//   = 2 * f(CK) * 1 byte/s (x8 SDR, both bytes of a word ride two consecutive clk90 cycles).
//
//   The controller and the wire live in a 2:1 mesochronous relationship. The single true CDC of the
//   read path (RWDS->clk) still lives here (DESIGN.md §2); the TX word->byte gearbox and the RX
//   byte->word assembly + elastic hand-off are the other clk<->clk90 crossings, all local to the PHY.
//
// ---------------------------------------------------------------------------------------------------
// TX  (clk word domain -> clk90 byte domain):
//   * Latch the controller's word + controls on `clk`.
//   * A toggle bit in the `clk` domain is 2-flop synchronised into `clk90` and edge-detected to make
//     `beat_a` — a one-clk90-cycle marker, exactly once per `clk` period, of the byte-A sub-cycle.
//   * On the byte-A sub-cycle drive byte A and CAPTURE byte B into a hold reg; on the byte-B sub-cycle
//     drive the held byte B. Capturing BOTH halves on the same clk90 edge guarantees a byte pair is
//     always from ONE controller word regardless of the exact clk/clk90 phase.
//   * hb_ck is generated on the clk90 NEGEDGE from (ck_en & beat_a): its rising edge lands in the
//     CENTRE of the byte-A eye and its falling edge in the centre of the byte-B eye — the 90deg
//     write-centring relationship, derived from ONE clock. CK idles Low, gated by phy_ck_en, and can
//     never emit a runt because ck_en is registered per word at beat_a (SPEC_DIGEST §1).
//
// RX  (clk90 byte domain -> clk word domain):
//   * Sample hb_dq_i / hb_rwds_i into the clk90 domain (registered input; CAPTURE_PHASE selects the
//     sampling edge for read-eye tuning on hardware — default centre).
//   * The device returns read data source-synchronous to RWDS (= CK echoed). Detect RWDS edges on the
//     local clk90 sample: a rising edge tags byte A, a falling edge tags byte B and completes a word.
//     Push each assembled word into a small gray-pointer elastic FIFO, drained in the `clk` domain to
//     emit one clk-synchronous `phy_dq_i` + `phy_dq_i_valid` per word (the controller word-counts).
//   * A stalled RWDS (row-crossing gap, or the >=32-clk error stall) simply produces no edges, so no
//     spurious word is captured — the controller's stall/timeout logic handles it.
//   * The raw RWDS level is separately 2-flop synchronised into `clk` for the controller's CA
//     latency-select (High during CA => 2x) and stall watch (SPEC_DIGEST §3/§4).
//
// ON-HARDWARE RISK: read capture samples on the LOCAL clk90 clock (not the returning strobe), so it is
// only tolerant of the DQ/RWDS round-trip flight delay up to a fraction of the byte window. At the
// conservative low CK rate the eye is wide; CAPTURE_PHASE is the tuning handle. This is the remaining
// bring-up risk (a production high-speed build should use the DDIO/strobe-clocked variant).

`timescale 1ns/1ps

/* verilator lint_off UNUSEDPARAM */
module hyperbus_phy_sdr
  import hyperbus_pkg::*;
#(
    parameter int unsigned DQ_WIDTH    = HB_DQ_WIDTH_DEFAULT,   // HyperBus DQ pins
    parameter int unsigned DATA_WIDTH  = 2 * DQ_WIDTH,          // native word width (= PHYW)
    parameter int unsigned ADDR_WIDTH  = HB_ADDR_WIDTH,         // (unused here; contract parameter)
    parameter int unsigned LEN_WIDTH   = HB_LEN_WIDTH_DEFAULT,  // (unused here; contract parameter)
    parameter              PHY_VARIANT = "SDR",                 // (this file is the SDR variant)
    parameter bit          DIFF_CK     = 1'b1,                  // 1: drive hb_ck_n = ~hb_ck; 0: tie High
    // Read-capture eye phase select (non-frozen, defaulted). 0 = sample DQ/RWDS on the clk90 posedge
    // (centres the eye at nominal flight delay); 1 = pre-sample on the clk90 negedge (half-clk90 earlier
    // sampling point) for on-hardware read-eye tuning. Realised in silicon by an input-delay tap.
    parameter bit          CAPTURE_PHASE = 1'b0,
    // Read-strobe PREAMBLE skip (non-frozen, defaulted 0). A real HyperRAM (Winbond W957D8NB on the
    // AXC3000) toggles RWDS for RD_PREAMBLE_SKIP CK cycles with DQ Hi-Z (=0x00) BEFORE the first read
    // data byte — a turn-around preamble the ideal model does not emit. Those leading RWDS rising
    // edges would otherwise pair into PHANTOM {0x00,0x00} words and mis-align the whole read (the
    // AXC3000 bring-up hang: STATUS never `done`). Ignore the first RD_PREAMBLE_SKIP rwds rising edges
    // after the receiver arms, so byte pairing begins on the REAL read-data window. 0 = no preamble
    // (spec-ideal model / all existing TBs) — the pairing then starts on the very first edge as before.
    parameter int unsigned RD_PREAMBLE_SKIP = 0
) (
    input  logic                clk,       // CK-rate WORD clock (controller domain); hb_ck runs at this rate
    input  logic                clk90,     // REPURPOSED: 2x BYTE clock (single PLL, 0deg) — the periphery clock
    input  logic                clk_ref,   // unused (tie-off; kept so all PHY variants share one port list)
    input  logic                rst,       // synchronous, active high (clk domain)

    // ---- ctrl-facing (slave; mirror of hyperbus_ctrl TX/RX) ----
    input  logic                phy_cs_n,
    input  logic                phy_rst_n,
    input  logic                phy_ck_en,
    input  logic [2*DQ_WIDTH-1:0] phy_dq_o,   // word: [hi]=byte A (1st sub-cycle), [lo]=byte B (2nd)
    input  logic                phy_dq_oe,
    input  logic [1:0]          phy_rwds_o,   // write byte-mask: [1]=1st phase, [0]=2nd phase
    input  logic                phy_rwds_oe,
    input  logic                phy_rd_arm,   // arm receiver for a read-data phase
    // ---- runtime read-eye calibration (mandatory, no defaults; quasi-static — change only while the
    //      controller is idle / STATUS.busy=0). Reset-seeded from the legacy parameters; see REG_CAL. ----
    input  logic                              cal_capture_phase, // live CAPTURE_PHASE (read-capture edge)
    input  logic [HB_CAL_PREAMBLE_SKIP_WIDTH-1:0] cal_preamble_skip, // live RD_PREAMBLE_SKIP (rwds-rise edges)
    input  logic [HB_CAL_RX_TAP_WIDTH-1:0]        cal_rx_tap,        // (unused in SDR — DDIO tap select)
    input  logic                              cal_pair_skew,     // (unused in SDR — DDIO byte-pairing)
    output logic [2*DQ_WIDTH-1:0] phy_dq_i,   // recovered, clk-synchronised read word (byte A hi half)
    output logic                phy_dq_i_valid,
    output logic                phy_rwds_i,   // synchronised RWDS level to ctrl

    // ---- device pins (split; board wrapper adds tristate) ----
    output logic                hb_ck,
    output logic                hb_ck_n,
    output logic                hb_cs_n,
    output logic                hb_rst_n,
    output logic [DQ_WIDTH-1:0] hb_dq_o,
    output logic                hb_dq_oe,     // 1 = master drives DQ
    input  logic [DQ_WIDTH-1:0] hb_dq_i,
    output logic                hb_rwds_o,
    output logic                hb_rwds_oe,
    input  logic                hb_rwds_i
);
  /* verilator lint_on UNUSEDPARAM */

  localparam int unsigned PHYW = 2 * DQ_WIDTH;  // PHY parallel width (one HyperBus word)

  // ==================================================================================================
  //  Reset into the clk90 (byte) domain. `rst` is a clk-domain synchronous, active-high level; it is
  //  stable for many clk90 cycles, but we 2-flop it into clk90 so the byte-domain logic has a clean
  //  release. Everything here is synchronous — no async reset anywhere (Hyperflex discipline).
  // ==================================================================================================
  logic rst2x_meta, rst2x;
  always_ff @(posedge clk90) begin
    rst2x_meta <= rst;
    rst2x      <= rst2x_meta;
  end

  // ==================================================================================================
  //  TX word latch (clk domain). One uniform clk of latency captures the controller's word + controls
  //  so the byte gearbox sees a stable pair for the whole clk period. Architectural enables/cs get the
  //  synchronous reset to a safe idle; the data halves are reset-less datapath (Hyperflex).
  // ==================================================================================================
  logic [DQ_WIDTH-1:0] dqa_l, dqb_l;   // byte A / byte B of the latched word
  logic                rwa_l, rwb_l;   // RWDS write-mask phases
  logic                csn_l, rstn_l, dqoe_l, rwoe_l, cken_l;
  logic                tgl;            // toggles once per clk period => gearbox phase reference

  always_ff @(posedge clk) begin
    if (rst) begin
      csn_l   <= 1'b1;   // idle: chip deselected
      rstn_l  <= 1'b0;   // hold device in reset while core is reset
      dqoe_l  <= 1'b0;
      rwoe_l  <= 1'b0;
      cken_l  <= 1'b0;
      tgl     <= 1'b0;
    end else begin
      csn_l   <= phy_cs_n;
      rstn_l  <= phy_rst_n;
      dqoe_l  <= phy_dq_oe;
      rwoe_l  <= phy_rwds_oe;
      cken_l  <= phy_ck_en;
      tgl     <= ~tgl;
    end
    // reset-less datapath halves
    dqa_l <= phy_dq_o[PHYW-1:DQ_WIDTH];
    dqb_l <= phy_dq_o[DQ_WIDTH-1:0];
    rwa_l <= phy_rwds_o[1];
    rwb_l <= phy_rwds_o[0];
  end

  // ==================================================================================================
  //  Gearbox phase recovery (clk90 domain). Synchronise the clk-domain toggle and edge-detect it: one
  //  transition per clk period => `beat_a` is High for exactly the byte-A sub-cycle of each clk period,
  //  Low for the byte-B sub-cycle. Robust to the clk/clk90 phase (proper 2-flop synchroniser).
  // ==================================================================================================
  logic tgl_s1, tgl_s2, tgl_s3;
  always_ff @(posedge clk90) begin
    if (rst2x) begin
      tgl_s1 <= 1'b0; tgl_s2 <= 1'b0; tgl_s3 <= 1'b0;
    end else begin
      tgl_s1 <= tgl;
      tgl_s2 <= tgl_s1;
      tgl_s3 <= tgl_s2;
    end
  end
  wire beat_a = tgl_s2 ^ tgl_s3;   // 1-clk90 pulse: the byte-A sub-cycle of each clk period

  // The TX serialiser (a posedge process below) branches on `beat_a` and therefore acts on its
  // pre-edge (NBA) value — i.e. byte A is launched in the clk90 cycle whose START saw beat_a High.
  // hb_ck is generated on the clk90 NEGEDGE and would otherwise see the SETTLED beat_a, one cycle
  // ahead of the data. Register beat_a by one clk90 so hb_ck uses the SAME view the serialiser did:
  // beat_a_d1 is High exactly during the cycle in which byte A is on the pins.
  logic beat_a_d1;
  always_ff @(posedge clk90) beat_a_d1 <= rst2x ? 1'b0 : beat_a;

  // ==================================================================================================
  //  TX SDR serialiser (clk90 domain). On the byte-A sub-cycle drive byte A and latch byte B (+ the
  //  per-word controls) into holds; on the byte-B sub-cycle drive the held byte B. Capturing both
  //  halves + controls together at beat_a guarantees each launched pair is from ONE controller word.
  //  Ordinary posedge flops feed the pins (no DDR / no primitives): the board pad ring tri-states.
  // ==================================================================================================
  logic [DQ_WIDTH-1:0] dqb_hold;
  logic                rwb_hold;
  logic                cken_w;      // ck-enable for the current word (held across the pair)

  always_ff @(posedge clk90) begin
    if (rst2x) begin
      hb_dq_o    <= '0;
      hb_rwds_o  <= 1'b0;
      hb_dq_oe   <= 1'b0;
      hb_rwds_oe <= 1'b0;
      hb_cs_n    <= 1'b1;
      hb_rst_n   <= 1'b0;
      dqb_hold   <= '0;
      rwb_hold   <= 1'b0;
      cken_w     <= 1'b0;
    end else if (beat_a) begin
      // byte-A sub-cycle: launch byte A, latch byte B + per-word controls for the pair
      hb_dq_o    <= dqa_l;
      hb_rwds_o  <= rwa_l;
      dqb_hold   <= dqb_l;
      rwb_hold   <= rwb_l;
      hb_dq_oe   <= dqoe_l;
      hb_rwds_oe <= rwoe_l;
      hb_cs_n    <= csn_l;
      hb_rst_n   <= rstn_l;
      cken_w     <= cken_l;
    end else begin
      // byte-B sub-cycle: launch the held byte B (controls/cs/oe hold across the whole word)
      hb_dq_o    <= dqb_hold;
      hb_rwds_o  <= rwb_hold;
    end
  end

  // ==================================================================================================
  //  hb_ck generation (clk90 NEGEDGE). CK = (ck_en & beat_a) sampled half a clk90 period after the
  //  byte launch, so hb_ck's RISING edge sits in the centre of the byte-A eye and its FALLING edge in
  //  the centre of the byte-B eye — one centred CK period per word, idle Low, glitch-free (ck_en is a
  //  per-word registered level). This is the "90deg from one clock" write-centring (PHY_PORTING §2.1).
  // ==================================================================================================
  always_ff @(negedge clk90) begin
    if (rst2x) hb_ck <= 1'b0;
    else       hb_ck <= cken_w & beat_a_d1;   // beat_a_d1: same cycle byte A is on the pins
  end
  assign hb_ck_n = DIFF_CK ? ~hb_ck : 1'b1;   // idle CK Low => idle CK# High (single-ended ties High)

  // ==================================================================================================
  //  RX read capture (clk90 domain). Registered input of DQ/RWDS with a selectable sampling edge
  //  (CAPTURE_PHASE), then RWDS-edge-detected byte pairing. hb_rwds_i toggles like CK during read data;
  //  its rising edge tags byte A, its falling edge tags byte B and completes the 16-bit word.
  // ==================================================================================================
  // Read-capture eye phase is now RUNTIME-selectable (was a generate-if on CAPTURE_PHASE). BOTH sampling
  // pipelines are always instantiated and a registered select (cap_phase_q) chooses between them live, so
  // the host can retune the read eye by a CSR write with no recompile. cap_phase_q resets to the legacy
  // CAPTURE_PHASE parameter (POR seed), then tracks cal_capture_phase. It is the clk->clk90 synchroniser
  // for that 1-bit quasi-static knob (host changes cal_capture_phase only while STATUS.busy=0), so a
  // single flop is sufficient. dq_cap/rwds_cap are muxes of registered samples => stable per clk90 cycle.
  logic [DQ_WIDTH-1:0] dq_pos;                 // posedge sample (centre eye at nominal flight delay)
  logic                rwds_pos;
  logic [DQ_WIDTH-1:0] dq_neg, dq_neg_al;      // negedge pre-sample (half-clk90 earlier), then aligned
  logic                rwds_neg, rwds_neg_al;  //   into the posedge domain
  always_ff @(posedge clk90) begin
    dq_pos   <= hb_dq_i;
    rwds_pos <= hb_rwds_i;
  end
  always_ff @(negedge clk90) begin
    dq_neg   <= hb_dq_i;
    rwds_neg <= hb_rwds_i;
  end
  always_ff @(posedge clk90) begin
    dq_neg_al   <= dq_neg;
    rwds_neg_al <= rwds_neg;
  end
  logic cap_phase_q;
  always_ff @(posedge clk90) cap_phase_q <= rst2x ? CAPTURE_PHASE : cal_capture_phase;

  logic [DQ_WIDTH-1:0] dq_cap;
  logic                rwds_cap;
  assign dq_cap   = cap_phase_q ? dq_neg_al   : dq_pos;
  assign rwds_cap = cap_phase_q ? rwds_neg_al : rwds_pos;

  // Arm the receiver: bring phy_rd_arm (clk domain) into clk90.
  logic rdarm_s1, rdarm_s2;
  always_ff @(posedge clk90) begin
    if (rst2x) begin rdarm_s1 <= 1'b0; rdarm_s2 <= 1'b0; end
    else       begin rdarm_s1 <= phy_rd_arm; rdarm_s2 <= rdarm_s1; end
  end

  // Elastic FIFO (write side = clk90 byte domain, read side = clk word domain). Gray-pointer async
  // FIFO — the designated CDC of the read path (DESIGN.md §2).
  localparam int unsigned RXF_DEPTH = 32;   // elastic read FIFO (byte->word). Deepened from 8 so a full
                                            // board read burst (16 words) plus the clk90->clk gray-pointer
                                            // hand-off latency never laps the pointer / stalls the drain.
  localparam int unsigned RXF_AW    = $clog2(RXF_DEPTH);

  logic [PHYW-1:0]     rxf_mem [RXF_DEPTH];
  logic [DQ_WIDTH-1:0] rx_byte_a;                 // byte A held between the RWDS rising and falling edge
  logic                have_a;                    // a byte A has been captured, awaiting its byte B
  logic                rwds_cap_q;                // previous RWDS sample (edge detect)
  logic [RXF_AW:0]     wptr_bin;                  // clk90-domain binary write pointer (extra MSB)
  logic [RXF_AW:0]     rptr_bin;                  // clk-domain binary read pointer

  // Leading rwds-rise edges still to be discarded as read-strobe preamble for this burst. Loaded from
  // the RUNTIME cal_preamble_skip each time the receiver is disarmed, so every read transaction (each
  // CS# / each ST_LAT->ST_READ arm) re-skips whatever preamble length the host has programmed.
  // SKIPW is a FIXED width (HB_CAL_PREAMBLE_SKIP_WIDTH), NOT derived from RD_PREAMBLE_SKIP's value — a
  // value-derived width (1 bit at the board's RD_PREAMBLE_SKIP=1) would truncate a wider runtime skip.
  localparam int unsigned SKIPW = HB_CAL_PREAMBLE_SKIP_WIDTH;
  logic [SKIPW-1:0]    pre_skip;

  // 2-flop synchronise the runtime cal_preamble_skip (clk / r_cal domain) into clk90, reset-seeded from
  // the RD_PREAMBLE_SKIP POR value so out-of-reset behaviour matches the legacy parameter. Quasi-static
  // (host changes it only while STATUS.busy=0), so the multi-bit 2-flop sync is safe. Mirrors rdarm_s1/2.
  logic [SKIPW-1:0]    cal_skip_s1, cal_skip_s2;
  always_ff @(posedge clk90) begin
    if (rst2x) begin
      cal_skip_s1 <= SKIPW'(RD_PREAMBLE_SKIP);
      cal_skip_s2 <= SKIPW'(RD_PREAMBLE_SKIP);
    end else begin
      cal_skip_s1 <= cal_preamble_skip;
      cal_skip_s2 <= cal_skip_s1;
    end
  end

  wire rwds_rise = rwds_cap & ~rwds_cap_q;        // start of byte A
  wire rwds_fall = ~rwds_cap & rwds_cap_q;        // start of byte B (word complete)

  always_ff @(posedge clk90) begin
    if (rst2x) begin
      rx_byte_a  <= '0;
      have_a     <= 1'b0;
      rwds_cap_q <= 1'b0;
      wptr_bin   <= '0;
      pre_skip   <= SKIPW'(RD_PREAMBLE_SKIP);
    end else begin
      rwds_cap_q <= rwds_cap;
      if (!rdarm_s2) begin
        // Not in a read-data phase: clear the pairing pipeline, re-arm the preamble skip, and flush
        // the elastic-FIFO write side so a fresh burst starts clean and discards its own turn-around
        // preamble edges AND any trailing words the device streamed past the master's burst count
        // (which would otherwise leak into the next read). Paired with the read-side rptr reset below.
        have_a   <= 1'b0;
        pre_skip <= cal_skip_s2;   // re-arm to the LIVE programmed skip (was SKIPW'(RD_PREAMBLE_SKIP))
        wptr_bin <= '0;
      end else begin
        if (rwds_rise) begin
          if (pre_skip != '0) begin
            // Preamble rising edge: discard it (DQ is Hi-Z here). Do NOT start a word so the
            // following falling edge cannot complete a phantom {0x00,0x00} pair.
            pre_skip <= pre_skip - 1'b1;
            have_a   <= 1'b0;
          end else begin
            rx_byte_a <= dq_cap;      // byte A (mid byte-A eye)
            have_a    <= 1'b1;
          end
        end else if (rwds_fall && have_a) begin
          rxf_mem[wptr_bin[RXF_AW-1:0]] <= {rx_byte_a, dq_cap};  // {byte A (hi), byte B (lo)}
          wptr_bin                       <= wptr_bin + 1'b1;
          have_a    <= 1'b0;
        end
      end
    end
  end

  // Gray-code the write pointer and 2-flop synchronise it into the clk domain. While the receiver is
  // disarmed (phy_rd_arm Low, i.e. between/around read bursts) the write side resets wptr_bin to 0 in
  // the clk90 domain; that is a MULTI-BIT gray transition (e.g. gray(20)->gray(0)), which a plain
  // 2-flop synchroniser can mis-sample mid-flight and momentarily present a bogus wptr in the clk
  // domain — on hardware that stray non-empty read leaks an over-streamed word into the next burst.
  // So force the synchronised copy directly to 0 while disarmed (the source is 0 anyway): the flush is
  // then deterministic (rxf_empty is cleanly asserted) with no reliance on the gray-pointer reset
  // surviving the CDC. Normal +1 gray sync resumes from 0 when the next read arms.
  wire  [RXF_AW:0] wptr_gray = wptr_bin ^ (wptr_bin >> 1);
  logic [RXF_AW:0] wgray_s1, wgray_s2;
  always_ff @(posedge clk) begin
    if (rst || !phy_rd_arm) begin
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

  // Read side (clk word domain): one recovered word + valid per FIFO entry. While the receiver is
  // disarmed (phy_rd_arm Low, i.e. between read bursts) hold the read pointer at 0 so the elastic
  // FIFO is flushed to empty — the write side (wptr_bin) is likewise reset to 0 in the clk90 domain
  // when rdarm_s2 falls. Both sides therefore start every read burst at 0, discarding any trailing
  // over-streamed words from the previous burst (the multi-burst hang on the AXC3000).
  always_ff @(posedge clk) begin
    if (rst) begin
      rptr_bin       <= '0;
      phy_dq_i       <= '0;
      phy_dq_i_valid <= 1'b0;
    end else begin
      phy_dq_i_valid <= 1'b0;
      if (!phy_rd_arm) begin
        rptr_bin <= '0;
      end else if (!rxf_empty) begin
        phy_dq_i       <= rxf_mem[rptr_bin[RXF_AW-1:0]];
        phy_dq_i_valid <= 1'b1;
        rptr_bin       <= rptr_bin + 1'b1;
      end
    end
  end

  // ==================================================================================================
  //  RWDS level synchroniser (clk word domain). 2-flop sync of the raw RWDS pin for the controller's
  //  CA latency-select (High during CA => 2x latency) and read-stall detection (SPEC_DIGEST §3/§4).
  // ==================================================================================================
  logic rwds_s1;
  always_ff @(posedge clk) begin
    if (rst) begin
      rwds_s1    <= 1'b0;
      phy_rwds_i <= 1'b0;
    end else begin
      rwds_s1    <= hb_rwds_i;
      phy_rwds_i <= rwds_s1;
    end
  end

  // clk_ref / ADDR_WIDTH / LEN_WIDTH / PHY_VARIANT are contract-only for this variant; cal_rx_tap and
  // cal_pair_skew are DDIO-PHY read-eye knobs with no meaning in the SDR datapath (tie-off).
  logic _unused_ok;
  /* verilator lint_off UNUSEDSIGNAL */
  assign _unused_ok = &{1'b0, clk_ref, cal_rx_tap, cal_pair_skew};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
