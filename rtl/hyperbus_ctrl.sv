// hyperbus_ctrl — HyperBus master controller (protocol layer, PHY-agnostic).
//
// Normative spec: Infineon/Cypress HyperBus 001-99253 Rev *H, distilled in docs/SPEC_DIGEST.md.
// Architecture / clocking: docs/DESIGN.md. Frozen ports: docs/INTERFACES.md. Shared defs:
// rtl/hyperbus_pkg.sv. Clean-room implementation; no vendor primitives; fully Verilator-simulable.
//
// Responsibilities (DESIGN.md §4/§5):
//   * Accept one native command per transaction and build the 48-bit Command-Address (hb_pack_ca).
//   * Drive CS#, run CK (phy_ck_en), and serialize CA/write data as DDR word pairs to the PHY.
//   * Handle FIXED and VARIABLE initial latency. In variable mode the slave-driven RWDS level during
//     the CA phase (phy_rwds_i High => 2x latency) doubles the configured latency count.
//   * DDR reads: arm the PHY receiver (phy_rd_arm) and count RWDS-gated recovered words
//     (phy_dq_i_valid). Row/page latency gaps are absorbed because completion is word-counted, not
//     clock-counted. RWDS stalled Low >= 32 clocks => err_timeout + abort (SPEC_DIGEST §4/§7).
//   * DDR writes (with latency): RWDS is master-driven byte-mask (High = masked; = ~wr_strb).
//   * Register-space / zero-latency writes: data follows CA immediately, master does NOT drive RWDS,
//     full-word only (SPEC_DIGEST §5/§6).
//   * Linear + wrapped bursts (CA[45] = ~wrap). Linear bursts are chopped to MAX_BURST_WORDS so CS#
//     never exceeds tCSM, transparently to the caller (DESIGN.md §5.6).
//   * POR init: pulse device reset, wait POR_DELAY_CYCLES, optionally program CR0 (PROGRAM_CR), then
//     raise init_done. User commands are gated until init_done.
//
// Controller <-> PHY timing contract (a companion GENERIC PHY / model is written to match):
//   CS# is asserted one setup cycle (CK idle) before the CA phase. The 3 CA words are then clocked
//   over 3 consecutive cycles (byte A in [PHYW-1:DQ_WIDTH], byte B in [DQ_WIDTH-1:0]). Non-zero
//   latency inserts LATENCY_CLOCKS (x2 if variable+doubled) CK cycles before the data phase.
//
// Sync reset (active-high) for all architectural state; no async reset, no datapath clock gating.
`ifndef HYPERBUS_CTRL_SV
`define HYPERBUS_CTRL_SV
module hyperbus_ctrl
  import hyperbus_pkg::*;
#(
    parameter int unsigned DQ_WIDTH         = HB_DQ_WIDTH_DEFAULT,
    parameter int unsigned DATA_WIDTH       = HB_DATA_WIDTH_DEFAULT,
    parameter int unsigned ADDR_WIDTH       = HB_ADDR_WIDTH,
    parameter int unsigned LEN_WIDTH        = HB_LEN_WIDTH_DEFAULT,
    parameter int unsigned LATENCY_CLOCKS   = HB_LATENCY_CLOCKS_DEFAULT,
    parameter bit          FIXED_LATENCY    = HB_FIXED_LATENCY_DEFAULT,
    parameter int unsigned MAX_BURST_WORDS  = 0,                 // 0 = no chopping; else tCSM/tCK
    parameter bit          PROGRAM_CR       = 1'b1,              // program CR0 at init
    parameter int unsigned POR_DELAY_CYCLES = 0,                 // ~150us in cycles (0 in sim)
    parameter logic [3:0]  INIT_LATENCY_CODE= hb_clocks_to_latency_code(LATENCY_CLOCKS),
    // CR0 image programmed at init: [15]=1 normal, [14:12]=000 drive, [11:8]=1111 reserved-as-1,
    // [7:4]=latency code, [3]=fixed-latency, [2]=1 legacy wrap, [1:0]=11 (32B). (SPEC_DIGEST §8.1)
    parameter logic [15:0] INIT_CR0         = {1'b1, 3'b000, 4'b1111, INIT_LATENCY_CODE,
                                               FIXED_LATENCY, 3'b111},
    // -- Device-specific multi-burst work-arounds (Winbond W957D8NB, AXC3000). Both DEFAULT OFF so
    //    existing instantiations are bit-identical; the board top / bench enable them explicitly. --
    // A linear segment is chopped so it NEVER crosses a BURST_BOUNDARY_WORDS-aligned WORD boundary
    // (0 = disabled). The W957D8NB releases the bus ~1.5 CK into a burst that crosses a 0x2000-word
    // boundary, so everything past it reads as floating junk; chopping at the boundary avoids it
    // (DESIGN.md §5.6 / issue: 0x2000-word boundary chop).
    parameter int unsigned BURST_BOUNDARY_WORDS = 0,
    // After every SPLIT memory-write segment, self-issue an internal COMMIT-READ that spans the last
    // written word (the device only commits a write burst's final word when the next command is a
    // read that covers it; a following write drops it). 1 = interpose the commit-read (issue #1).
    // 2026-07-10 SILICON STATUS: DOCUMENTED-INEFFECTIVE for write->write streams. On the real
    // W957D8NB NO read-shaped interpose (any COMMIT_READ_MODE) commits the pending words when
    // another write follows — the pending tail is lost the moment the next memory-space WRITE CS#
    // opens, interposed reads notwithstanding. Kept for configurations whose traffic genuinely ends
    // in a read; for chopped write streams use WR_CHOP_REPLAY (and WR_COALESCE for command-level
    // boundaries) instead.
    parameter bit          WR_COMMIT_READ       = 1'b0,
    // Write-latency extension (board-calibration): ADD this many CK to the SECOND (2x) latency
    // count for WRITES only. Reads are RWDS-gated and self-align, so they never need it; on the
    // AXC3000 GPIO-I/O bring-up the device's write window opens exactly 3 CK after the
    // spec-anchored wait ends (silicon-measured: mem[k]=pat(k+3) at trim 0, pat(k+6) at trim -3 —
    // the offset tracks the knob 1:1). Default 0 = spec behavior.
    parameter int unsigned WR_LAT_TRIM          = 0,
    // Internal commit-read length (WR_COMMIT_READ), in words. Must be >= 2 (a 1-word dummy read does
    // NOT trigger the device write-commit; issue #1 attempt #3) and, for COMMIT_READ_MODE=SPAN_END,
    // small enough to never itself cross a BURST_BOUNDARY_WORDS boundary. 4 matched the on-silicon
    // "working read phase" trigger against a 1-word pending write buffer — see COMMIT_READ_MODE.
    parameter int unsigned COMMIT_READ_WORDS    = 4,
    // Shape of the internal commit-read (issue #1 direction iteration, 2026-07-09 silicon evidence).
    // SPAN_END (default; the original fix): a COMMIT_READ_WORDS-word linear read ENDING on the
    // just-written last word. Silicon-proven against a 1-word pending write buffer, but new evidence
    // (175 MHz GPIO-I/O image) shows the W957D8NB actually holds the LAST FOUR words of every
    // non-final write burst pending (ERR_COUNT = 4*(n_bursts-1), always words [len-4..len-1]) — a
    // 4-word SPAN_END read exactly spans those four words yet no longer triggers the commit (same
    // failure class as issue #1 attempt #3's short dummy read). FULL_BURST: re-read the ENTIRE
    // just-closed write segment from its true base (wr_seg_base/wr_seg_len). NEXT_ROW (issue #1
    // direction 3, untried on silicon): read COMMIT_READ_WORDS words at last_wr_addr with bit 12
    // flipped (a different 4K row), on the row-close/precharge hypothesis.
    // 2026-07-10 CORRECTION: FULL_BURST was believed silicon-proven because bw's own full-range
    // read-back phase always observes committed data — but that observation only holds when NO
    // WRITE follows the read. On silicon a FULL_BURST commit-read interposed between two writes
    // does NOT preserve the pending words (~4 lost per chop, same as no fix); the old sim model's
    // covering-read-commits trigger was the artifact that made it look like a fix. See
    // WR_CHOP_REPLAY for the working chop-boundary remedy.
    parameter              COMMIT_READ_MODE     = "SPAN_END",  // SPAN_END | FULL_BURST | NEXT_ROW
    // CS#-COALESCING (issue #1 direction 4 — the deterministic fix for transfers under tCSM). When a
    // memory-write command finishes and a NEW, contiguous (cmd_addr == just-written-end), linear,
    // memory-space write command is already valid — or arrives within WR_COALESCE_WAIT cycles — do
    // NOT close CS#: hold the SAME HyperBus burst open (CS# stays Low) and wait for the new command's
    // first data word, then continue streaming without a new CA/latency phase. This makes split writes
    // single-CS# by construction, sidestepping the write-commit quirk entirely for transfers that fit
    // under the MAX_BURST_WORDS / BURST_BOUNDARY_WORDS caps (still respected across the coalesced
    // stream — the combined burst chops exactly like one long native command would; the chop boundary
    // itself may still need WR_COMMIT_READ). The wait holds CK STOPPED rather than streaming a masked
    // filler word: the master's own burst address auto-increments on every CK edge regardless of RWDS
    // masking (SPEC_DIGEST §4 — RWDS only masks which BYTES of a word are written, not whether the
    // word's address slot is consumed), so a *toggling* masked filler would silently shift every word
    // of the spliced-in command's data by one address — verified against the golden model, which
    // reproduces this address-continuity rule. Holding CK idle (mirroring the ST_RD_DRAIN precedent)
    // keeps the burst address exactly at last_wr_addr+1 for as long as the wait lasts, so the spliced
    // command's first real word lands exactly where it should. Default 0 = off (bit-identical to prior
    // instantiations).
    parameter bit          WR_COALESCE          = 1'b0,
    parameter int unsigned WR_COALESCE_WAIT     = 8,           // cycles to wait for a coalescing cmd
    // WRITE-CHOP REPLAY (issue #1 direction 5 — 2026-07-10 silicon). The commit-read hypothesis is
    // FALSIFIED on silicon: any CS#-closing write boundary permanently loses the final
    // WR_REPLAY_WORDS (= device pending depth, 4 on the W957D8NB) words of the closed segment the
    // moment the next memory-space WRITE CS# opens — NO read-shaped interpose (SPAN_END, FULL_BURST,
    // anything) commits them when another write follows. The only robust fix for a boundary that
    // MUST exist (a tCSM / MAX_BURST_WORDS / BURST_BOUNDARY_WORDS chop of one long write stream) is
    // to RE-SEND the words the device is about to drop. When 1, the controller keeps a
    // WR_REPLAY_WORDS-deep shadow of the last data words + byte-strobes sent in the current write
    // CS# burst and, at every INTRA-COMMAND chop (ST_RECOVER with rem_left != 0 on a memory write),
    // reopens the next segment rb = min(WR_REPLAY_WORDS, words sent in the closed CS#) words EARLY:
    // cur_addr rolled back by rb, seg/rem accounting widened by rb so the total front-end word count
    // is unchanged, and the first rb beats of the new segment sourced from the replay shadow
    // (wr_ready is HELD LOW for them — no new front-end data is consumed for replayed beats). The
    // re-write lands after the device has discarded/zeroed its pending tail, so the chopped stream
    // reads back clean.
    // COMPOSITION: replay handles exactly the boundaries WR_COALESCE cannot avoid — the
    // intra-command chops. It does NOT apply at command-level (write-command -> write-command) CS#
    // boundaries: avoiding those is coalescing's job (WR_COALESCE), and a command boundary that
    // coalescing declines (non-contiguous, wait timeout, cap exhausted) still loses the closed
    // burst's pending tail. When WR_COMMIT_READ is also enabled, replay takes precedence at chop
    // boundaries and the chop commit-read is gated off there (silicon-proven useless for
    // write->write); WR_COMMIT_READ's command-level deferred interpose is left as configured.
    // BURST_BOUNDARY note: at a BURST_BOUNDARY_WORDS chop the rolled-back segment deliberately
    // STARTS rb words BEFORE the boundary it just chopped at (the reopen's boundary budget is taken
    // from the pre-rollback base, so the replayed prefix is not itself re-chopped) — the only way
    // the pre-boundary tail can ever be re-written.
    // RESIDUAL (inherent device behavior; this IP cannot fix it without inventing traffic): the
    // FINAL segment's last WR_REPLAY_WORDS words still pend inside the device at stream end. They
    // are observably committed by any subsequent covering READ the host performs
    // (silicon-observed), and are lost for write-then-idle-forever workloads.
    // Sizing: WR_REPLAY_WORDS must equal the device pending depth (W957D8NB: 4) and be smaller than
    // MAX_BURST_WORDS when that cap is set (the replayed prefix + at least one real word share a
    // segment; if MAX_BURST_WORDS <= WR_REPLAY_WORDS the reopened segment may exceed the cap by up
    // to rb words). Default 0 = off (bit-identical to prior instantiations).
    parameter bit          WR_CHOP_REPLAY       = 1'b0,
    parameter int unsigned WR_REPLAY_WORDS      = 4,
    // Device pending-tail depth (W957D8NB: 4) — the rollback floor when WR_REPLAY_ALIGN is used.
    parameter int unsigned WR_REPLAY_PEND       = 4,
    // 0 = legacy rollback (= words captured, saturating at WR_REPLAY_WORDS). N = roll back past the
    // pending tail to the previous N-word-aligned address (2026-07-09 silicon: reopening INSIDE the
    // device's internal buffer row garbles the row's lower half; see rp_rollback below). The shadow
    // must be deep enough: WR_REPLAY_WORDS >= WR_REPLAY_PEND + N - 1.
    parameter int unsigned WR_REPLAY_ALIGN      = 0,
    // Extra CS#-High dwell (clk cycles) after every memory-WRITE burst closes, BEFORE the next
    // CS# opens (chop reopen, coalesce-declined next command, anything). 2026-07-09 silicon
    // experiment: the device appears to need internal-merge time after a write burst; a next write
    // CS# arriving too soon discards the pending tail AND zeroes/garbles the 4 words below the new
    // CA base. 0 = off. Sizing: ~1-3 us of clk cycles; costs once per chop, negligible for
    // >=512-word segments.
    parameter int unsigned WR_CHOP_PAUSE_CYCLES = 0,
    // Mask-led replay reopen (2026-07-09 silicon, the wound-suppression probe): open the replay
    // reopen this many words EARLIER still, sending that many fully-RWDS-MASKED dummy beats before
    // the replayed real words. Tests whether the device's below-base 4-word wound (see the ladder
    // findings: ANY write CA zeroes [base-4, base) — reads do not) is suppressed when the burst
    // leads with masked beats. 0 = off.
    parameter int unsigned WR_REPLAY_MASK_LEAD  = 0,
    // Keep CK toggling (CS# High, spec-legal) through the post-write recovery/pause dwell — tests
    // whether the device's pending-tail merge needs CK edges rather than wall time.
    parameter bit          WR_CHOP_PAUSE_CK     = 1'b0,
    // ---- A3: optional CR1 programming at init (SPEC_DIGEST §5/§8.2). Default OFF = today's behavior
    //          (CR0 only). When set, a SECOND zero-latency register write of CR1 follows the CR0 write
    //          during init, before init_done. The device's distributed-refresh/PASR/hybrid-sleep fields
    //          live in CR1; leaving it at reset is a tCSM/refresh risk on real silicon.
    parameter bit          PROGRAM_CR1      = 1'b0,              // 1 = also program CR1 at init (after CR0)
    parameter logic [15:0] INIT_CR1         = HB_CR1_RESET,      // CR1 image written at init
    // ---- A4: POR / reset AC-timing (SPEC_DIGEST §9, Table 8.3). The reset-pulse and post-reset gaps
    //          are DERIVED from ns/µs spec timings once CLK_FREQ_MHZ != 0 (cycles = ceil(t / tCK)).
    //          CLK_FREQ_MHZ == 0 keeps the LEGACY fixed counts (RESET_CYCLES=8, POR dwell =
    //          POR_DELAY_CYCLES) so every existing instantiation is bit-for-bit unchanged.
    parameter int unsigned CLK_FREQ_MHZ     = 0,                 // CK word-clock frequency (MHz); 0 = derive-off
    parameter int unsigned T_RP_NS          = 200,              // tRP  : RST# low pulse width      (spec >= 200 ns)
    parameter int unsigned T_RPH_NS         = 400,              // tRPH : RST# low  -> first CS# low (spec >= 400 ns)
    parameter int unsigned T_RH_NS          = 200,              // tRH  : RST# high -> first CS# low (spec >= 200 ns)
    parameter int unsigned T_VCS_US         = 150,              // tVCS : VCC valid -> first access  (spec <= 150 µs)
    // ---- A1: Deep Power-Down enter/exit (SPEC_DIGEST §5.2.1 / §8.7, CR0[15]). Default OFF. When set,
    //          the controller snoops a host CR0 register-write: CR0[15]=0 latches an internal DPD flag;
    //          the NEXT command is preceded by a guarded wake (CS# wake pulse + tDPDOUT dwell) before it
    //          is allowed to reach the bus. CR0[15]=1 (via write) clears the flag (normal).
    parameter bit          SUPPORT_DPD      = 1'b0,             // 1 = enable DPD enter-detect + guarded wake
    parameter int unsigned TDPDOUT_CYCLES   = 0,               // tDPDOUT exit dwell (cycles = ceil(tDPDOUT/tCK))
    // ---- A2: active clock-stop (SPEC_DIGEST §1). Default OFF. When set, CK (phy_ck_en) is paused on
    //          word boundaries while the read holding-FIFO is above its high-water mark (caller back-
    //          pressure via rd_ready), halting the device instead of dropping words; CK resumes when the
    //          FIFO drains. Off = CK runs continuously through the read (today's behavior).
    parameter bit          ACTIVE_CLK_STOP  = 1'b0
) (
    input  logic                    clk,
    input  logic                    rst,            // synchronous, active high

    // -- native command channel (slave) --
    input  logic                    cmd_valid,
    output logic                    cmd_ready,
    input  logic                    cmd_read,       // 1 = read  (CA[47])
    input  logic                    cmd_reg,        // 1 = register space (CA[46])
    input  logic                    cmd_wrap,       // 1 = wrapped burst (CA[45] = ~wrap)
    input  logic [ADDR_WIDTH-1:0]   cmd_addr,       // WORD address
    input  logic [LEN_WIDTH-1:0]    cmd_len,        // burst length in words, >= 1

    // -- native write-data channel (slave) --
    input  logic                    wr_valid,
    output logic                    wr_ready,
    input  logic [DATA_WIDTH-1:0]   wr_data,        // byte A = [DATA_WIDTH-1:DQ_WIDTH]
    input  logic [DATA_WIDTH/8-1:0] wr_strb,        // per-byte write-enable (1 = write)
    input  logic                    wr_last,        // final word of burst (informational)

    // -- native read-data channel (master) --
    output logic                    rd_valid,
    input  logic                    rd_ready,
    output logic [DATA_WIDTH-1:0]   rd_data,
    output logic                    rd_last,

    // -- status --
    output logic                    busy,
    output logic                    init_done,
    output logic                    err_underrun,   // pulse: write data not delivered in time
    output logic                    err_timeout,    // pulse: read RWDS stalled >= 32 clocks

    // -- PHY interface (master) : TX --
    output logic                    phy_cs_n,
    output logic                    phy_rst_n,
    output logic                    phy_ck_en,
    output logic [2*DQ_WIDTH-1:0]   phy_dq_o,       // [PHYW-1:DQ_WIDTH]=byte A, [DQ_WIDTH-1:0]=byte B
    output logic                    phy_dq_oe,
    output logic [1:0]              phy_rwds_o,      // [1]=1st phase, [0]=2nd phase (write mask)
    output logic                    phy_rwds_oe,
    output logic                    phy_rd_arm,

    // -- PHY interface (master) : RX --
    input  logic [2*DQ_WIDTH-1:0]   phy_dq_i,       // recovered read word (byte A in high half)
    input  logic                    phy_dq_i_valid,
    input  logic                    phy_rwds_i,      // synchronized RWDS level

    // -- DEBUG taps (bring-up only; leave unconnected in normal instantiations) --
    output logic [3:0]              dbg_state,      // FSM state (state_e ordinal)
    output logic [5:0]              dbg_rd_wptr,    // (repurposed) rem_left[5:0]  — words remaining in burst
    output logic [5:0]              dbg_rd_rptr,    // (repurposed) seg_left[5:0]  — words remaining in segment

    // -- issue #13 live controller knobs (bench REG_DBG). Quasi-static, driven from the SAME `clk`
    //    domain as bench+PHY (host changes them only while STATUS.busy=0) so they are used DIRECTLY —
    //    NO synchronizer (contrast cal_*'s clk->clk90 crossing). Out of reset the bench holds
    //    dbg_wr_lat_trim/dbg_lat_clocks at the WR_LAT_TRIM/LATENCY_CLOCKS parameter seeds, so behavior
    //    is bit-identical to the elaboration constants until the host pokes. NO port default values
    //    (Verilator rejects them) — every instantiation ties these to per-instance legacy values. --
    input  logic [3:0]              dbg_wr_lat_trim,  // overrides WR_LAT_TRIM (2x write-latency add; POR 3)
    input  logic [3:0]              dbg_lat_clocks,   // overrides LATENCY_CLOCKS (both latency seeds; 6/7)
    input  logic                    dbg_cr0_reprog,   // 1-clk pulse: relaunch init CR0 write w/ new code
    input  logic                    dbg_prewin_drive, // heal probe: drive [B-4..B-1] in the pre-data window
    input  logic [2:0]              dbg_prewin_n,     // # trailing latency CK to drive (0..7)
    input  logic                    dbg_prewin_marker,// 1 = drive 0xA5xx marker instead of shadow data
    input  logic                    dbg_postwin_hold  // hold last data word 4 CK into the tail (Law-3 analog)
);

  // ------------------------------------------------------------------------
  // POR / reset AC-timing derivation (A4). Pure elaboration-time integer math (Verilator-clean, no
  // reals): cycles = ceil(time / tCK) = ceil(t_ns * f_MHz / 1000). f_MHz == 0 disables derivation.
  // ------------------------------------------------------------------------
  function automatic int unsigned ns_to_cyc(input int unsigned t_ns, input int unsigned f_mhz);
    return (f_mhz == 0) ? 0 : ((t_ns * f_mhz + 32'd999) / 32'd1000);   // ceil
  endfunction
  function automatic int unsigned max2(input int unsigned a, input int unsigned b);
    return (a > b) ? a : b;
  endfunction

  // ------------------------------------------------------------------------
  // Derived widths / internal constants
  // ------------------------------------------------------------------------
  localparam int unsigned STRB_WIDTH       = DATA_WIDTH / 8;
  // RST# low pulse. Legacy fixed 8 cycles, or tRP-derived when CLK_FREQ_MHZ is given (>= 1 cycle).
  localparam int unsigned RESET_CYCLES     = (CLK_FREQ_MHZ == 0) ? 8
                                           : max2(1, ns_to_cyc(T_RP_NS, CLK_FREQ_MHZ)); // >= tRP (SPEC §9)
  // tVCS (VCC-valid -> first access): 1 µs = f_MHz cycles exactly.
  localparam int unsigned VCS_CYCLES       = (CLK_FREQ_MHZ == 0) ? 0 : (T_VCS_US * CLK_FREQ_MHZ);
  // Post-reset gap before first CS#: must satisfy tRH AND (tRPH - tRP) from reset RELEASE.
  localparam int unsigned RH_CYCLES        = (CLK_FREQ_MHZ == 0) ? 0
             : max2(ns_to_cyc(T_RH_NS, CLK_FREQ_MHZ),
                    (T_RPH_NS > T_RP_NS) ? ns_to_cyc(T_RPH_NS - T_RP_NS, CLK_FREQ_MHZ) : 0);
  // ST_POR dwell = the largest post-reset requirement (tVCS dominates once derived), never below the
  // caller's explicit POR_DELAY_CYCLES. Legacy mode uses POR_DELAY_CYCLES verbatim.
  localparam int unsigned POR_CYCLES       = (CLK_FREQ_MHZ == 0) ? POR_DELAY_CYCLES
                                           : max2(POR_DELAY_CYCLES, max2(RH_CYCLES, VCS_CYCLES));
  localparam int unsigned DPD_WAKE_CYCLES  = 4;                    // A1: CS# wake-pulse width (DPD exit trigger)
  localparam int unsigned RECOVERY_CYCLES  = 4;                    // tCSHI + tRWR gap (SPEC §6)
  localparam int unsigned TAIL_CYCLES      = 2;                    // extra CS# Low after last word (tCSH note)
  localparam int unsigned READ_STALL_LIMIT = 32;                   // RWDS Low >= 32 clk => timeout (SPEC §4/§7)
  // Read over-stream drain/settle (AXC3000 multi-burst fix). After a read burst the real HyperRAM
  // keeps driving a few EXTRA source-synchronous words: its read output pipeline drains past the
  // master's CK-stop (which has flight/pipeline latency), so stray words keep arriving AFTER the master
  // has counted its burst. If the NEXT read arms its receiver while those stragglers are still arriving,
  // they are captured as phantom leading words and corrupt / hang the next burst (see tb_multiburst).
  // ST_RD_DRAIN therefore follows every read burst: CK is stopped (no new device words) but the PHY
  // receiver is kept ARMED so the over-stream words are drained out (phy_dq_i_valid) and DISCARDED,
  // until the PHY has been quiet for DRAIN_QUIET_CYCLES consecutive cycles (over-stream ended). Only
  // then does the transaction recover and re-arm/accept the next read, so no straggler leaks across the
  // burst boundary. Word-rate (phy_dq_i_valid, one pulse per straggler — no RWDS-level aliasing);
  // adaptive (robust to a VARIABLE extra-word count, not a hardcoded 7); bounded by DRAIN_MAX_CYCLES so
  // a stuck receiver can never hang the drain.
  localparam int unsigned DRAIN_QUIET_CYCLES = 4;                  // consecutive quiet cycles => drained
  localparam int unsigned DRAIN_MAX_CYCLES   = 128;               // safety bound on the drain wait (cycles)
  localparam int unsigned RD_FIFO_DEPTH    = 32;                  // read holding buffer (rd_ready slack).
                                                                    // Must exceed the largest single read
                                                                    // segment (Avalon burstcount) + the SDR
                                                                    // PHY read-pipeline depth so the final,
                                                                    // rd_last-tagged word is never dropped on
                                                                    // rd_fifo_full: on the AXC3000 SDR path a
                                                                    // depth of 8 capped completing reads at ~5
                                                                    // words (bursts >=6 hung). 32 covers the
                                                                    // board's 16-word bursts with slack.
  localparam int unsigned RD_AW            = $clog2(RD_FIFO_DEPTH);
  // A2 active clock-stop high-water mark: pause CK once the read FIFO reaches this occupancy (SPEC_DIGEST
  // §1). Sized to two constraints: (a) leave slack BELOW full for words already in the PHY RX pipeline as
  // CK winds down, so nothing is dropped mid-burst while stopping (DEPTH - level = plenty); (b) keep the
  // residual small enough to drain within the post-burst window (ST_RD_DRAIN + tail + recovery) BEFORE
  // ST_IDLE's unconditional FIFO flush, so a caller that keeps draining loses nothing. A permanently
  // stalled sink can still lose the residual to that flush — the controller does not extend a completed
  // transaction to wait for a dead sink (see docs). A quarter-depth balances both.
  localparam logic [RD_AW:0] RD_STOP_LEVEL = (RD_AW+1)'(RD_FIFO_DEPTH / 4);

  typedef enum logic [3:0] {
    ST_RESET,   // device reset asserted
    ST_POR,     // reset released, POR delay
    ST_INIT,    // launch optional CR0 write
    ST_IDLE,    // ready for a user command
    ST_CS,      // CS# setup (CK idle), first CA word presented
    ST_CA,      // 3 CA words clocked out
    ST_LAT,     // initial latency
    ST_READ,    // read-data phase (RWDS-gated word count)
    ST_RD_ABORT,// read aborted (RWDS timeout): drain remaining beats as flagged filler
    ST_RD_DRAIN,// post-read: CK stopped, receiver kept armed to absorb+discard over-stream stragglers
    ST_WRITE,   // write-data phase
    ST_WR_COALESCE, // WR_COALESCE: burst held open past command end, CS# Low / CK STOPPED, waiting
                    // (up to WR_COALESCE_WAIT cycles) for a contiguous write command to splice in
    ST_TAIL,    // CS# held Low after last word (tCSH note)
    ST_RECOVER, // CS# High, tCSHI/tRWR recovery
    ST_DPD_WAKE,// A1: Deep-Power-Down exit — CS# wake pulse (CK idle), device leaves DPD
    ST_DPD_OUT  // A1: CS# High, waiting tDPDOUT before the pending command is issued
  } state_e;

  state_e                  state;
  logic [31:0]             cnt;          // general dwell counter (reset/POR/latency/tail/recovery)
  logic [5:0]              stall_cnt;    // read RWDS-Low stall counter

  // Read over-stream drain/settle (see DRAIN_* localparams above).
  logic [7:0]              drain_cnt;    // consecutive cycles with no straggler word (quiet detector)
  logic [7:0]              drain_dwell;  // total cycles spent in ST_RD_DRAIN (safety bound)

  // transaction context
  logic                    cur_read;
  logic                    cur_reg;
  logic                    cur_wrap;     // 1 = wrapped
  logic [ADDR_WIDTH-1:0]   cur_addr;     // current segment start (WORD address)
  logic [LEN_WIDTH-1:0]    rem_left;     // words remaining in whole burst (incl. current)
  logic [LEN_WIDTH-1:0]    seg_left;     // words remaining in current CS# segment (incl. current)
  logic [LEN_WIDTH-1:0]    seg_count;    // words in current segment (for linear address advance)
  hb_ca_t                  ca_reg;       // packed 48-bit CA for the current segment
  logic [1:0]              ca_idx;       // CA word index 0..2
  logic                    rwds_hi;      // RWDS sampled High during CA (latency doubling indicator)
  logic                    lat_extra_done; // the additional (2x) latency count has been inserted
  logic                    doing_init;   // current transaction is an internal CR0/CR1 init write
  logic                    init_cr1;     // A3: the internal init write in flight is CR1 (else CR0)
  logic                    doing_cr0_reprog; // issue #13: the internal init write in flight is a RUNTIME
                                             //   CR0 reprogram (dbg_lat_clocks code), not the POR CR0/CR1
  logic                    in_dpd;       // A1: device believed to be in Deep-Power-Down (needs wake)

  // -- Write-commit interpose (WR_COMMIT_READ). A memory-write burst's final word is only committed
  //    by the device when a subsequent read covers it; between two writes the master interposes an
  //    internal COMMIT-READ that spans that word. `doing_commit` mirrors `doing_init` for this
  //    internal read (its data is discarded); `wr_pending_commit` marks that a write segment closed
  //    and still needs committing; `commit_resume` distinguishes an intra-command split (restore the
  //    shadow write context afterwards) from a deferred write->write interpose (return to idle). --
  logic                    doing_commit;
  logic                    wr_pending_commit;
  logic                    commit_resume;
  logic [ADDR_WIDTH-1:0]   last_wr_addr; // last WORD address written in the most recent write segment
  logic [ADDR_WIDTH-1:0]   wr_seg_base;  // base WORD address of the most recently CLOSED write segment
  logic [LEN_WIDTH-1:0]    wr_seg_len;   // word length of the most recently CLOSED write segment
                                        //   (COMMIT_READ_MODE=FULL_BURST re-reads [wr_seg_base +:
                                        //   wr_seg_len]; both mirror last_wr_addr's capture point)
  logic [ADDR_WIDTH-1:0]   sv_addr;      // shadow: write-segment start to resume after a chop commit-read
  logic [LEN_WIDTH-1:0]    sv_rem;       // shadow: words remaining to resume after a chop commit-read

  // -- CS#-coalescing (WR_COALESCE). `seg_cap_left` is the hardware-cap (MAX_BURST_WORDS /
  //    BURST_BOUNDARY_WORDS) word budget LEFT in the CURRENTLY OPEN CS# burst: (re-)initialized to
  //    hw_cap(base) at every genuine new CS# open (ST_IDLE accept, ST_RECOVER reopen) and decremented
  //    by exactly ONE for every REAL word actually sent in ST_WRITE thereafter, for as long as this
  //    SAME CS# stays open, even across multiple spliced-in native commands (ST_WR_COALESCE's wait
  //    holds CK stopped — see its state comment — so it never itself consumes budget). This continuous,
  //    single-counter tracking (rather than re-deriving a fresh per-address budget at each splice) is
  //    what keeps a multi-command coalescing chain correctly bounded by the ORIGINAL burst's
  //    tCSM/boundary budget. `coalesce_wait_cnt` bounds how long the bus is held open with no
  //    qualifying command (WR_COALESCE_WAIT cycles). --
  logic [LEN_WIDTH-1:0]    seg_cap_left;
  logic [15:0]             coalesce_wait_cnt;

  // -- Write-chop replay (WR_CHOP_REPLAY). rp_word/rp_strb is a WR_REPLAY_WORDS-deep shift shadow
  //    of the last words sent in the CURRENT write CS# burst (index 0 = newest). It is captured at
  //    every memory-space ST_WRITE beat — replayed beats included: a replayed word is the most
  //    recently SENT word and would pend again if this CS# closed right behind it. An underrun
  //    filler slot is captured with an all-zero strobe (a replay of it re-sends a fully-masked,
  //    address-consuming, data-neutral word). rp_cnt counts words sent since this CS# opened,
  //    saturating at WR_REPLAY_WORDS; it is reset at every genuine CS# open but NOT at a
  //    ST_WR_COALESCE splice (the splice keeps the same CS#/HyperBus burst open, and coalescing is
  //    address-contiguous, so the shadow always holds exactly addresses
  //    [cur_addr-rp_cnt .. cur_addr-1] regardless of command splices). replay_left counts the
  //    rolled-back segment's replayed-prefix beats still to send; the emit index replay_idx is
  //    FIXED at rb-1 for the whole prefix — each emitted word re-enters the shift shadow at [0]
  //    pushing the pipe down one, so the next-oldest word lands at the same index (and after rb
  //    emits the shadow is back in newest-first order, holding the same rb words). --
  localparam int unsigned RP_AW = (WR_REPLAY_WORDS > 1) ? $clog2(WR_REPLAY_WORDS) : 1;
  logic [DATA_WIDTH-1:0]   rp_word [WR_REPLAY_WORDS];
  logic [STRB_WIDTH-1:0]   rp_strb [WR_REPLAY_WORDS];
  logic [LEN_WIDTH-1:0]    rp_cnt;
  logic [LEN_WIDTH-1:0]    replay_left;
  logic [LEN_WIDTH-1:0]    replay_real;   // real (shadow-sourced) beats within replay_left (mask lead)
  logic [RP_AW-1:0]        replay_idx;

  // -- issue #13 heal/hold state ---------------------------------------------------------------------
  //  shadow_full  : sticky "rp_word holds a full WR_REPLAY_WORDS-deep tail [B-4..B-1]". Set when rp_cnt
  //                 saturates in ST_WRITE; cleared ONLY at a genuine ST_IDLE write CS# open and at reset
  //                 (NOT in ST_RECOVER) — so an internal row-chop reopen (ST_RECOVER->ST_CS, bypassing
  //                 ST_IDLE) still sees the prior segment's tail even though rp_cnt is reset there. That
  //                 is what makes dbg_prewin_drive a HEAL for the contiguous row chops (the L-D target).
  //  last_wr_word : the final data word of the FINAL segment, replayed onto DQ through the extended tail
  //                 when dbg_postwin_hold (Law-3 analog).  postwin_active : that hold is armed.
  logic                    shadow_full;
  logic [DATA_WIDTH-1:0]   last_wr_word;
  logic                    postwin_active;

  // read holding FIFO ({last, data})
  logic [DATA_WIDTH:0]     rd_fifo [RD_FIFO_DEPTH];
  logic [RD_AW:0]          rd_wptr, rd_rptr;

  // ------------------------------------------------------------------------
  // Combinational helpers
  // ------------------------------------------------------------------------
  wire                     zlw = ~cur_read & cur_reg;             // zero-latency (register) write

  // Deferred write->write commit interpose (WR_COMMIT_READ): when a NEW memory-write command is
  // presented while a previous write still needs committing, DON'T accept it — gate cmd_ready and
  // self-issue the internal commit-read first (launched in ST_IDLE below). A read/register command
  // is accepted normally (a covering read commits the pending write on silicon).
  wire                     commit_gate = WR_COMMIT_READ & wr_pending_commit &
                                         cmd_valid & ~cmd_read & ~cmd_reg;

  // issue #13 runtime CR0 image: the POR INIT_CR0 with its latency code (CR0[7:4]) replaced by the
  // code synthesized from the LIVE dbg_lat_clocks, written by the dbg_cr0_reprog strobe (§2.2). Pure
  // combinational — hb_clocks_to_latency_code is a `function automatic` LUT, evaluable on live inputs
  // (no pkg edit needed). dbg_lat_clocks=6 => code 0001 => cr0_rt == INIT_CR0 (POR-legacy).
  wire [DATA_WIDTH-1:0]    cr0_rt     = {INIT_CR0[15:8],
                                         hb_clocks_to_latency_code(32'(dbg_lat_clocks)),
                                         INIT_CR0[3:0]};

  // write-data source: internal CR0/CR1 image during init (A3: CR1 when init_cr1; issue #13: the live
  // cr0_rt when doing_cr0_reprog), else native write.
  wire [DATA_WIDTH-1:0]    wsrc_data  = doing_init
                                        ? (doing_cr0_reprog ? cr0_rt
                                           : (init_cr1 ? DATA_WIDTH'(INIT_CR1)
                                                       : DATA_WIDTH'(INIT_CR0)))
                                        : wr_data;
  wire [STRB_WIDTH-1:0]    wsrc_strb  = doing_init ? {STRB_WIDTH{1'b1}} : wr_strb;
  wire                     wsrc_valid = doing_init ? 1'b1 : wr_valid;

  // Initial-latency doubling select (SPEC_DIGEST §3; spec 663-664/779-780; §5.2.4). The master ALWAYS
  // decodes the slave-driven RWDS level presented during CA to choose 1x vs 2x latency counts — this
  // is UNCONDITIONAL and must NOT branch on FIXED_LATENCY. A fixed-latency device simply drives RWDS to
  // a constant level; HyperRAM commonly fixes it HIGH to always require two counts (§5.2.4). One
  // latency count = LATENCY_CLOCKS cycles; when doubled a SECOND count of LATENCY_CLOCKS is inserted
  // back-to-back in ST_LAT. `rwds_hi` latches the indicator across CA *and* the latency window so the
  // decision is robust to the PHY's RWDS synchroniser pipeline delay (cf. OpenHBMC hbmc_ctrl.v:557,
  // which likewise samples RWDS-during-CA regardless of the fixed/variable parameter). FIXED_LATENCY
  // now only seeds the CR0[3] image at init (INIT_CR0 default).
  wire                     lat_double = rwds_hi | phy_rwds_i;

  // Read over-stream considered drained once the PHY has stopped delivering straggler words for
  // DRAIN_QUIET_CYCLES consecutive cycles (the over-stream has ended), or the safety bound is hit.
  wire                     drain_done = (drain_cnt   >= 8'(DRAIN_QUIET_CYCLES)) |
                                        (drain_dwell >= 8'(DRAIN_MAX_CYCLES));

  wire                     rd_fifo_empty = (rd_wptr == rd_rptr);
  wire                     rd_fifo_full  = (rd_wptr[RD_AW-1:0] == rd_rptr[RD_AW-1:0]) &
                                           (rd_wptr[RD_AW]     != rd_rptr[RD_AW]);
  wire [RD_AW-1:0]         rd_wa = rd_wptr[RD_AW-1:0];
  wire [RD_AW-1:0]         rd_ra = rd_rptr[RD_AW-1:0];

  // A2 active clock-stop: read-FIFO occupancy and the "buffer nearly full -> pause CK" condition.
  // rd_backpressure gates phy_ck_en during ST_READ (on word boundaries) and freezes the RWDS-Low
  // stall counter so an INTENTIONAL pause is never mistaken for the >=32-clk error stall.
  wire [RD_AW:0]           rd_occ = rd_wptr - rd_rptr;                 // occupancy 0..RD_FIFO_DEPTH
  wire                     rd_backpressure = ACTIVE_CLK_STOP & (rd_occ >= RD_STOP_LEVEL);

  // 16-bit CA word selected by index (MSB word first, SPEC_DIGEST §2)
  function automatic logic [DATA_WIDTH-1:0] ca_slice(input logic [1:0] idx);
    unique case (idx)
      2'd0:    return ca_reg[47:32];
      2'd1:    return ca_reg[31:16];
      default: return ca_reg[15:0];
    endcase
  endfunction

  // segment size for a (remaining) burst starting at `addr`. Linear bursts are chopped so a single
  // CS# segment never (a) exceeds MAX_BURST_WORDS (tCSM), nor (b) crosses a BURST_BOUNDARY_WORDS-word
  // aligned boundary (the W957D8NB bus-release quirk). Wrapped bursts stay in their group -> never
  // chopped. `addr` is the WORD start of the segment (only used for the boundary limit).
  function automatic logic [LEN_WIDTH-1:0] seg_size(input logic [LEN_WIDTH-1:0]  total,
                                                    input logic                  wrapped,
                                                    input logic [ADDR_WIDTH-1:0] addr);
    logic [LEN_WIDTH-1:0] lim;
    logic [LEN_WIDTH-1:0] to_bound;
    lim = total;
    if ((MAX_BURST_WORDS != 0) && !wrapped && (lim > LEN_WIDTH'(MAX_BURST_WORDS)))
      lim = LEN_WIDTH'(MAX_BURST_WORDS);
    if ((BURST_BOUNDARY_WORDS != 0) && !wrapped) begin
      // words from `addr` to the next boundary = BOUND - (addr mod BOUND); chop `lim` down to it.
      to_bound = LEN_WIDTH'(BURST_BOUNDARY_WORDS - (32'(addr) % BURST_BOUNDARY_WORDS));
      if (lim > to_bound) lim = to_bound;
    end
    return lim;
  endfunction

  // Hardware-cap (MAX_BURST_WORDS / BURST_BOUNDARY_WORDS) word budget for a CS# burst based at word
  // address `addr` — seg_size() with an "unbounded" request length, so only the two caps apply. Used
  // to (re-)initialize seg_cap_left at every genuine new CS# open (WR_COALESCE then tracks it
  // CONTINUOUSLY from there, one word at a time — see seg_cap_left's declaration — so a multi-command
  // coalescing chain is bounded by the budget of the burst that ACTUALLY stayed open, not a fresh
  // budget re-derived from whichever command's address happens to be current at each splice).
  function automatic logic [LEN_WIDTH-1:0] hw_cap(input logic [ADDR_WIDTH-1:0] addr);
    return seg_size({LEN_WIDTH{1'b1}}, 1'b0, addr);
  endfunction

  // Base WORD address for the internal commit-read so that a COMMIT_READ_WORDS-word linear read
  // [base .. base+COMMIT_READ_WORDS-1] SPANS (ends on) the just-written word `la` (issue #1: the read
  // must cover the pending address). Clamped at 0 for very low addresses. Used by SPAN_END; also the
  // NEXT_ROW clamp guard (both index off the last written word, not the segment base).
  function automatic logic [ADDR_WIDTH-1:0] commit_base(input logic [ADDR_WIDTH-1:0] la);
    if (la >= ADDR_WIDTH'(COMMIT_READ_WORDS - 1))
      return la - ADDR_WIDTH'(COMMIT_READ_WORDS - 1);
    else
      return '0;
  endfunction

  // Commit-read base/length per COMMIT_READ_MODE (see the parameter comment for the rationale of
  // each shape). `la` = last_wr_addr (the just-written segment's last word); `base`/`len` = the
  // just-closed segment's own [wr_seg_base, wr_seg_len] (FULL_BURST only).
  function automatic logic [ADDR_WIDTH-1:0] commit_read_base(input logic [ADDR_WIDTH-1:0] la,
                                                              input logic [ADDR_WIDTH-1:0] base);
    if (COMMIT_READ_MODE == "FULL_BURST")
      return base;
    else if (COMMIT_READ_MODE == "NEXT_ROW")
      return la ^ ADDR_WIDTH'(32'h0000_1000);
    else // "SPAN_END" (default)
      return commit_base(la);
  endfunction

  function automatic logic [LEN_WIDTH-1:0] commit_read_len(input logic [LEN_WIDTH-1:0] len);
    if (COMMIT_READ_MODE == "FULL_BURST")
      return len;
    else // SPAN_END / NEXT_ROW: fixed-length read
      return LEN_WIDTH'(COMMIT_READ_WORDS);
  endfunction

  // -- CS#-coalescing (WR_COALESCE) combinational helpers -------------------------------------------
  // Coalescing eligibility, sampled at the ST_WRITE seg_left==1 & rem_left==1 transition (a plain
  // linear memory write is completing): only worth trying if seg_cap_left — the LIVE, continuously
  // tracked hw-cap budget for the CURRENTLY OPEN CS# burst (see its declaration) — shows room for at
  // least one more word after the one this cycle is sending. seg_cap_left is read here BEFORE this
  // cycle's own decrement takes effect, so ">1" (not "!=0") is the correct "room after this word" test.
  wire coalesce_try = WR_COALESCE & ~cur_read & ~cur_reg & ~cur_wrap & ~doing_init & ~doing_commit &
                      (seg_cap_left > LEN_WIDTH'(1));

  // A new, contiguous (picks up exactly where the closed segment left off), linear, memory-space
  // WRITE command, with hw-cap room left to accept at least one of its words. NOTE: unlike the write-
  // data underrun filler (ST_WRITE, wsrc_valid low -> RWDS=11 for ONE word with CK still toggling),
  // ST_WR_COALESCE's wait holds CK STOPPED (see its combinational drive) rather than toggling a masked
  // filler word, so seg_cap_left is not consumed while waiting — it still holds exactly the value left
  // after the last REAL word (ST_WRITE's own per-word countdown).
  wire coalesce_match = cmd_valid & ~cmd_read & ~cmd_reg & ~cmd_wrap &
                        (cmd_addr == last_wr_addr + ADDR_WIDTH'(1)) & (seg_cap_left != '0);

  // Accepted length for a coalescing splice: the new command's own boundary/MAX_BURST_WORDS cap (from
  // its own address), further clipped to the room actually left in the still-open CS# burst. If this
  // is less than cmd_len the remainder becomes a further chop, handled by the existing seg_left/
  // rem_left machinery once this spliced-in segment itself closes (no special-casing needed).
  wire [LEN_WIDTH-1:0] coalesce_new_seg    = seg_size(cmd_len, 1'b0, cmd_addr);
  wire [LEN_WIDTH-1:0] coalesce_accept_len = (coalesce_new_seg < seg_cap_left) ? coalesce_new_seg
                                                                                : seg_cap_left;

  // -- Write-chop replay (WR_CHOP_REPLAY) combinational helpers -------------------------------------
  // This ST_WRITE beat is a REPLAYED one: sourced from the replay shadow, not the front-end
  // (wr_ready held Low, underrun detection suppressed — the data is by construction available).
  wire                  replay_beat = WR_CHOP_REPLAY & (replay_left != '0);
  // Mask-led beats: the first WR_REPLAY_MASK_LEAD beats of a replay reopen are fully-masked
  // dummies (address-consuming, array-neutral); replay_real tracks how many REAL shadow beats
  // remain so lead_beat = "still inside the masked prefix".
  wire                  lead_beat   = replay_beat & (replay_left > replay_real);
  wire [DATA_WIDTH-1:0] rp_data_sel = rp_word[replay_idx];
  wire [STRB_WIDTH-1:0] rp_strb_sel = lead_beat ? '0 : rp_strb[replay_idx];

  // Replay-reopen geometry (consumed at the ST_RECOVER chop branch, where cur_addr has already been
  // advanced to the un-rolled next-segment base and rem_left/rp_cnt are stable):
  //   rp_rb   — rollback. WR_REPLAY_ALIGN=0 (legacy): min(WR_REPLAY_WORDS, words sent in the
  //             just-closed CS#) (rp_cnt saturates at WR_REPLAY_WORDS in ST_WRITE, so it IS that
  //             min). WR_REPLAY_ALIGN=N (2026-07-09 silicon): the W957D8NB garbles the LOWER half
  //             of its internal write-buffer row when a new write CA opens INSIDE the row holding
  //             its pending tail (reopen@508 for pending [508..511] read back [504..507] as foreign
  //             data while the replayed words were fine) — so roll back PAST the pending tail
  //             (WR_REPLAY_PEND words) to the previous N-word-aligned address, rewriting the whole
  //             row from its base. Clamped to rp_cnt (the shadow can only replay what it captured;
  //             a segment shorter than the row is rewritten from its own start).
  //   rp_real — the segment's REAL (front-end-sourced) word budget. Boundary-capped from the
  //             PRE-rollback base (cur_addr): the replayed prefix sits BEFORE that base, so capping
  //             from rp_base instead would re-chop the reopen at the very boundary it just chopped
  //             at (a 100%-replay segment => no forward progress => livelock). The prefix still
  //             counts toward the tCSM budget, so rp_real is clipped to MAX_BURST_WORDS - rp_rb
  //             (skipped if the cap is too small to honor — see the WR_CHOP_REPLAY sizing note);
  //   rp_base — rolled-back segment base;  rp_seg — total reopened segment length.
  function automatic logic [LEN_WIDTH-1:0] rp_rollback(input logic [ADDR_WIDTH-1:0] chop_base);
    logic [LEN_WIDTH-1:0] want;
    if (WR_REPLAY_ALIGN == 0) begin
      want = rp_cnt;
    end else begin
      want = LEN_WIDTH'(WR_REPLAY_PEND)
             + LEN_WIDTH'((chop_base - ADDR_WIDTH'(WR_REPLAY_PEND))
                          & ADDR_WIDTH'(WR_REPLAY_ALIGN - 1));
      if (want > rp_cnt) want = rp_cnt;
    end
    return want;
  endfunction
  wire [LEN_WIDTH-1:0]  rp_lead     = LEN_WIDTH'(WR_REPLAY_MASK_LEAD);
  wire [LEN_WIDTH-1:0]  rp_rb       = rp_rollback(cur_addr);
  wire [LEN_WIDTH-1:0]  rp_pfx      = rp_rb + rp_lead;            // total reopen prefix (lead + real)
  wire [LEN_WIDTH-1:0]  rp_real_raw = seg_size(rem_left, 1'b0, cur_addr);
  wire [LEN_WIDTH-1:0]  rp_real     = ((MAX_BURST_WORDS != 0) &&
                                       (LEN_WIDTH'(MAX_BURST_WORDS) > rp_pfx) &&
                                       (rp_real_raw > LEN_WIDTH'(MAX_BURST_WORDS) - rp_pfx))
                                      ? (LEN_WIDTH'(MAX_BURST_WORDS) - rp_pfx) : rp_real_raw;
  wire [ADDR_WIDTH-1:0] rp_base     = cur_addr - ADDR_WIDTH'(rp_pfx);
  wire [LEN_WIDTH-1:0]  rp_seg      = rp_pfx + rp_real;

  // COMMAND-level contiguous write->write replay (the coalesce-leg gap, 2026-07-09 silicon: when
  // the coalesce hw-cap budget exhausts exactly AT a command edge — e.g. MAX_BURST_WORDS=512 with
  // 64-word commands — the stream closes as a command completion, the next contiguous write is
  // accepted from ST_IDLE as a fresh CS#, and the device discards the closed burst's pending tail:
  // 4 zeroed words per chop, the pre-replay fingerprint). Same remedy through the ST_IDLE accept:
  // if the arriving command is a linear memory WRITE that starts exactly where the previous write
  // burst ended and the replay shadow still covers that tail, open it rolled back with the shadow
  // prefix. rp_cnt is preserved across READ accepts (reads do not disturb the device's pending
  // buffer — silicon-corrected model), so write->read->write-contiguous is covered too.
  wire                  acc_elig    = WR_CHOP_REPLAY & (rp_cnt != '0) & cmd_valid
                                      & ~cmd_read & ~cmd_reg & ~cmd_wrap & ~doing_init
                                      & (cmd_addr == last_wr_addr + ADDR_WIDTH'(1));
  wire [LEN_WIDTH-1:0]  acc_rb      = rp_rollback(cmd_addr);
  wire [LEN_WIDTH-1:0]  acc_pfx     = acc_rb + rp_lead;
  wire [LEN_WIDTH-1:0]  acc_real_raw= seg_size(cmd_len, 1'b0, cmd_addr);
  wire [LEN_WIDTH-1:0]  acc_real    = ((MAX_BURST_WORDS != 0) &&
                                       (LEN_WIDTH'(MAX_BURST_WORDS) > acc_pfx) &&
                                       (acc_real_raw > LEN_WIDTH'(MAX_BURST_WORDS) - acc_pfx))
                                      ? (LEN_WIDTH'(MAX_BURST_WORDS) - acc_pfx) : acc_real_raw;
  wire [ADDR_WIDTH-1:0] acc_base    = cmd_addr - ADDR_WIDTH'(acc_pfx);
  wire [LEN_WIDTH-1:0]  acc_seg     = acc_pfx + acc_real;

  // -- Pre-window heal probe (issue #13, dbg_prewin_drive) combinational helpers ---------------------
  // prewin_widx = min(cnt,3), the word index into the replay shadow for the pre-data drive window: the
  // LAST latency CK (cnt=0) emits rp_word[0] = the segment's NEWEST tail word (B-1); cnt=1 -> B-2;
  // cnt=2 -> B-3; every earlier CK (cnt>=3) holds rp_word[3] = B-4. So the 4 CK before the first data
  // beat carry B-4,B-3,B-2,B-1 in order (oldest-first) — exactly the [B-4..B-1] the device wounds,
  // turning the wound into a heal. prewin_ok gates shadow-drive to a FULL shadow (suppressed on a run's
  // first segment => legacy idle bus); marker mode is content-agnostic and always drives (attribution).
  wire [RP_AW-1:0]      prewin_widx = (cnt >= 32'd3) ? RP_AW'(3) : cnt[RP_AW-1:0];
  wire                  prewin_ok   = dbg_prewin_marker | shadow_full;

  // Post-window hold (issue #13, dbg_postwin_hold) tail-dwell seed: the ST_TAIL dwell stretches to 4 CK
  // (32'd3 => 4 cycles) when the FINAL segment of a memory write closes, so last_wr_word can be held on
  // DQ into the tail (CK not extended). The select matches the postwin_active arm in ST_WRITE below.
  wire [31:0]           postwin_seed = (dbg_postwin_hold & ~cur_read & ~cur_reg & ~doing_init &
                                        (rem_left == LEN_WIDTH'(1))) ? 32'd3 : 32'(TAIL_CYCLES - 1);

  // ------------------------------------------------------------------------
  // Read data-out (FIFO front)
  // ------------------------------------------------------------------------
  assign rd_valid = ~rd_fifo_empty;
  assign rd_data  = rd_fifo[rd_ra][DATA_WIDTH-1:0];
  assign rd_last  = rd_fifo[rd_ra][DATA_WIDTH];

  // ------------------------------------------------------------------------
  // PHY drive + native handshakes (combinational, Moore on `state`)
  // ------------------------------------------------------------------------
  always_comb begin
    cmd_ready   = 1'b0;
    wr_ready    = 1'b0;
    phy_cs_n    = 1'b1;
    phy_rst_n   = 1'b1;
    phy_ck_en   = 1'b0;
    phy_dq_o    = '0;
    phy_dq_oe   = 1'b0;
    phy_rwds_o  = 2'b00;
    phy_rwds_oe = 1'b0;
    phy_rd_arm  = 1'b0;
    busy        = 1'b0;

    unique case (state)
      ST_RESET: phy_rst_n = 1'b0;

      // A3 (issue #13): when a CR0-reprogram pulse is being consumed this cycle, do NOT complete a
      // command handshake — gate cmd_ready so the launch (ST_IDLE sequential) and a cmd accept are
      // mutually exclusive. ~dbg_cr0_reprog is 1 in legacy, so cmd_ready is unchanged when idle.
      ST_IDLE:  cmd_ready = init_done & rd_fifo_empty & ~commit_gate & ~dbg_cr0_reprog;

      ST_CS: begin
        phy_cs_n  = 1'b0;
        phy_dq_oe = 1'b1;
        phy_dq_o  = ca_slice(2'd0);
        busy      = ~doing_init;
      end

      ST_CA: begin
        phy_cs_n  = 1'b0;
        phy_ck_en = 1'b1;
        phy_dq_oe = 1'b1;
        phy_dq_o  = ca_slice(ca_idx);
        busy      = ~doing_init;
      end

      ST_LAT: begin
        phy_cs_n  = 1'b0;
        phy_ck_en = 1'b1;
        if (cur_read) begin
          phy_rd_arm = 1'b1;              // bus turn-around; slave drives DQ/RWDS
        end else begin
          phy_rwds_oe = 1'b1;             // latency write: master drives RWDS Low preamble (mask)
          phy_rwds_o  = 2'b00;
          // issue #13 pre-window HEAL probe: in the LAST dbg_prewin_n CK of the SECOND (2x) latency
          // count, drive DQ with the just-closed segment's tail [B-4..B-1] (replay shadow, oldest
          // first — prewin_widx) instead of leaving the bus idle; or an 0xA5xx marker for content
          // attribution. The RWDS masked-Low preamble above is UNTOUCHED. lat_extra_done restricts
          // this to the second count only, long after CA (ST_CS/ST_CA), so OE is never re-asserted
          // during the CA phase and there is no bus gap at cnt=0 -> ST_WRITE (OE stays 1). With
          // dbg_prewin_drive=0 the branch is inert (phy_dq_oe keeps its ST_LAT default 0) = today.
          if (dbg_prewin_drive && lat_extra_done && (cnt < 32'(dbg_prewin_n)) && prewin_ok) begin
            phy_dq_oe = 1'b1;
            phy_dq_o  = dbg_prewin_marker ? (16'hA500 | 16'(prewin_widx))
                                          : rp_word[prewin_widx];
          end
        end
        busy = ~doing_init;
      end

      ST_READ: begin
        phy_cs_n   = 1'b0;
        phy_ck_en  = ~rd_backpressure;   // A2: pause CK on word boundary while the RD FIFO is full-ish
        phy_rd_arm = 1'b1;
        busy       = ~doing_init;
      end

      ST_WRITE: begin
        phy_cs_n  = 1'b0;
        phy_ck_en = 1'b1;
        phy_dq_oe = 1'b1;
        // WR_CHOP_REPLAY: a replayed beat is sourced from the replay shadow — the front-end's
        // wr channel is NOT consumed (wr_ready stays Low) and its current word is left untouched
        // for the first post-replay beat.
        phy_dq_o  = replay_beat ? rp_data_sel : wsrc_data;
        if (!zlw) begin
          phy_rwds_oe = 1'b1;            // RWDS = byte mask (High = masked); underrun => mask both
          phy_rwds_o  = replay_beat ? {~rp_strb_sel[1], ~rp_strb_sel[0]}
                                    : (wsrc_valid ? {~wsrc_strb[1], ~wsrc_strb[0]} : 2'b11);
        end
        if (wsrc_valid & ~doing_init & ~replay_beat) wr_ready = 1'b1;
        busy = ~doing_init;
      end

      ST_WR_COALESCE: begin
        // WR_COALESCE: hold the burst open — CS# stays Low — while waiting for the splice command.
        // phy_ck_en is deliberately left at its default (0, CK STOPPED): the burst address advances on
        // every CK edge regardless of RWDS masking (SPEC_DIGEST §4), so toggling CK with a masked
        // filler word here would silently shift the spliced-in command's address by one per wait cycle
        // (caught in simulation against the golden model). Holding CK idle — the same technique
        // ST_RD_DRAIN already uses to stop the device without deselecting it — keeps the burst address
        // pinned at last_wr_addr+1 for the whole wait. DQ/RWDS are still driven to known (masked)
        // values rather than left floating, though with CK idle nothing latches them. cmd_ready is
        // asserted ONLY when a qualifying contiguous write command is present this cycle, so a
        // non-matching command is simply left waiting (standard ready/valid semantics) rather than
        // being incorrectly consumed.
        phy_cs_n    = 1'b0;
        phy_dq_oe   = 1'b1;
        phy_dq_o    = '0;
        phy_rwds_oe = 1'b1;
        phy_rwds_o  = 2'b11;
        cmd_ready   = coalesce_match;
        busy        = ~doing_init;
      end

      ST_TAIL: begin
        busy = ~doing_init;
        // issue #13 post-window HOLD (Law-3 analog): keep the FINAL data word on DQ through the
        // extended tail dwell (4 CK; see postwin_seed) before CS# raises — the write end-of-burst
        // counterpart of the pre-window heal. phy_ck_en stays 0 (default) so CK is NOT extended; CS#
        // is forced Low for ST_TAIL just below (:phy_cs_n). Inert when postwin_active=0 (dbg off).
        if (postwin_active) begin
          phy_dq_oe = 1'b1;
          phy_dq_o  = last_wr_word;
        end
      end
      ST_RECOVER: begin
        busy = ~doing_init;
        // WR_CHOP_PAUSE_CK (2026-07-09 silicon experiment): keep CK toggling through the post-WRITE
        // recovery/pause dwell (CS# High — spec-legal free-running CK). Tests whether the device's
        // internal pending-tail merge is CK-clocked rather than time-based.
        if (WR_CHOP_PAUSE_CK & ~cur_read & ~cur_reg & ~doing_init) phy_ck_en = 1'b1;
      end
      ST_RD_ABORT: busy = ~doing_init;   // CS# High (idle bus); draining flagged filler to the FIFO

      ST_RD_DRAIN: begin
        // Post-read over-stream drain: CS# held Low (still selected, tail window), CK STOPPED so the
        // device produces no new words, but the PHY receiver stays ARMED so any straggler/over-stream
        // words the device already launched are recovered (phy_dq_i_valid) and DISCARDED here — not
        // written to rd_fifo, not counted. Held until the PHY goes quiet, so nothing leaks to the next
        // read burst.
        phy_cs_n   = 1'b0;
        phy_rd_arm = 1'b1;
        busy       = ~doing_init;
      end

      ST_DPD_WAKE: begin
        // A1: Deep-Power-Down exit trigger — assert CS# Low with CK idle (no CA, no data). The device
        // wakes on this CS# activity; the master then holds CS# High for tDPDOUT (ST_DPD_OUT).
        phy_cs_n = 1'b0;
        busy     = 1'b1;
      end

      ST_DPD_OUT: busy = 1'b1;   // A1: CS# High (default), counting out tDPDOUT before the command

      default: ; // ST_POR, ST_INIT: idle bus
    endcase

    if (state == ST_TAIL) phy_cs_n = 1'b0;   // hold CS# Low through the tail
  end

  // Launch an internal COMMIT-READ (WR_COMMIT_READ): a linear memory read of `len` words based at
  // `base` (shaped by COMMIT_READ_MODE — see commit_read_base/commit_read_len), marked doing_commit
  // so its recovered data is discarded (never enters rd_fifo). Drives the transaction context exactly
  // like a user read accepted in ST_IDLE, then enters ST_CS.
  task automatic launch_commit_read(input logic [ADDR_WIDTH-1:0] base, input logic [LEN_WIDTH-1:0] len);
    doing_commit <= 1'b1;
    cur_read     <= 1'b1;
    cur_reg      <= 1'b0;
    cur_wrap     <= 1'b0;
    cur_addr     <= base;
    rem_left     <= len;
    seg_count    <= len;
    seg_left     <= len;
    ca_reg       <= hb_pack_ca(1'b1, 1'b0, 1'b1, base);   // read, memory, linear
    rwds_hi      <= 1'b0;
    ca_idx       <= 2'd0;
    state        <= ST_CS;
  endtask

  // ------------------------------------------------------------------------
  // Sequential FSM + datapath state
  // ------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    // registered error pulses default Low (one-cycle strobes)
    err_underrun <= 1'b0;
    err_timeout  <= 1'b0;

    if (rst) begin
      state        <= ST_RESET;
      cnt          <= 32'(RESET_CYCLES - 1);
      stall_cnt    <= '0;
      drain_cnt    <= '0;
      drain_dwell  <= '0;
      init_done    <= 1'b0;
      doing_init   <= 1'b0;
      init_cr1     <= 1'b0;
      doing_cr0_reprog <= 1'b0;
      in_dpd       <= 1'b0;
      doing_commit <= 1'b0;
      wr_pending_commit <= 1'b0;
      commit_resume     <= 1'b0;
      last_wr_addr <= '0;
      wr_seg_base  <= '0;
      wr_seg_len   <= '0;
      sv_addr      <= '0;
      sv_rem       <= '0;
      seg_cap_left      <= '0;
      coalesce_wait_cnt <= '0;
      rp_cnt       <= '0;
      replay_left  <= '0;
      replay_real  <= '0;
      replay_idx   <= '0;
      shadow_full     <= 1'b0;   // issue #13 heal/hold state
      last_wr_word    <= '0;
      postwin_active  <= 1'b0;
      cur_read     <= 1'b0;
      cur_reg      <= 1'b0;
      cur_wrap     <= 1'b0;
      cur_addr     <= '0;
      rem_left     <= '0;
      seg_left     <= '0;
      seg_count    <= '0;
      ca_reg       <= '0;
      ca_idx       <= 2'd0;
      rwds_hi      <= 1'b0;
      lat_extra_done <= 1'b0;
      rd_wptr      <= '0;
      rd_rptr      <= '0;
    end else begin
      // read FIFO drain (independent of the FSM)
      if (~rd_fifo_empty & rd_ready) rd_rptr <= rd_rptr + 1'b1;

      unique case (state)
        // ---------------- POR / init ----------------
        ST_RESET: begin
          // RST# held Low for RESET_CYCLES (>= tRP). Then the post-reset gap (>= tRH / tRPH / tVCS).
          if (cnt == 32'd0) begin
            state <= ST_POR;
            cnt   <= 32'(POR_CYCLES);
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        ST_POR: begin
          if (cnt == 32'd0) state <= ST_INIT;
          else              cnt   <= cnt - 1'b1;
        end

        ST_INIT: begin
          // Launch the internal register-space (zero-latency) init write(s): CR0 first, then CR1
          // (A3) if requested. CR1 follows CR0 in ST_RECOVER; if only CR1 is requested, start there.
          if (PROGRAM_CR || PROGRAM_CR1) begin
            doing_init <= 1'b1;
            init_cr1   <= ~PROGRAM_CR;   // CR0 unless CR0 is disabled and only CR1 is programmed
            cur_read   <= 1'b0;
            cur_reg    <= 1'b1;
            cur_wrap   <= 1'b0;
            cur_addr   <= PROGRAM_CR ? HB_REG_CR0[ADDR_WIDTH-1:0] : HB_REG_CR1[ADDR_WIDTH-1:0];
            rem_left   <= LEN_WIDTH'(1);
            seg_left   <= LEN_WIDTH'(1);
            seg_count  <= LEN_WIDTH'(1);
            ca_reg     <= hb_pack_ca(1'b0, 1'b1, 1'b1,
                                     PROGRAM_CR ? HB_REG_CR0[ADDR_WIDTH-1:0]
                                                : HB_REG_CR1[ADDR_WIDTH-1:0]);
            rwds_hi    <= 1'b0;
            ca_idx     <= 2'd0;
            state      <= ST_CS;
          end else begin
            init_done <= 1'b1;
            state     <= ST_IDLE;
          end
        end

        // ---------------- idle / accept ----------------
        ST_IDLE: begin
          // Flush the read holding FIFO between transactions. A real HyperRAM keeps streaming read
          // data for a few extra words after the master has counted its burst (the master's CK stop
          // has pipeline latency), so a completed read can leave stray words buffered. Left in place
          // they keep rd_fifo non-empty, block the next command (cmd_ready gates on rd_fifo_empty),
          // and hang multi-burst reads on hardware. In ST_IDLE the previous transaction is fully
          // drained (its rd_last was delivered), so clearing the pointers only discards those extras.
          rd_wptr <= '0;
          rd_rptr <= '0;
          if (init_done & rd_fifo_empty) begin
            if (dbg_cr0_reprog & ~doing_init & ~doing_commit) begin
              // issue #13 CR0-REPROGRAM launch (§2.2, A3): re-run the init CR0 write with the LIVE
              // dbg_lat_clocks latency code (wsrc_data selects cr0_rt while doing_cr0_reprog). Reuses
              // the init machinery — a zero-latency register write that flows
              // ST_CS->ST_CA->ST_WRITE(1 word)->ST_TAIL->ST_RECOVER. Checked FIRST and cmd_ready is
              // gated by ~dbg_cr0_reprog (above), so no command handshake completes this cycle: the
              // reprogram and a cmd accept are mutually exclusive. dbg_cr0_reprog is a 1-cycle bench
              // pulse (edge-detected there), so no ctrl-side edge detect is needed. Host contract:
              // pulse only while STATUS.busy=0 (the pulse is lost if the ctrl is busy).
              doing_init       <= 1'b1;
              doing_cr0_reprog <= 1'b1;
              init_cr1         <= 1'b0;
              cur_read         <= 1'b0;
              cur_reg          <= 1'b1;
              cur_wrap         <= 1'b0;
              cur_addr         <= HB_REG_CR0[ADDR_WIDTH-1:0];
              rem_left         <= LEN_WIDTH'(1);
              seg_left         <= LEN_WIDTH'(1);
              seg_count        <= LEN_WIDTH'(1);
              ca_reg           <= hb_pack_ca(1'b0, 1'b1, 1'b1, HB_REG_CR0[ADDR_WIDTH-1:0]);
              rwds_hi          <= 1'b0;
              ca_idx           <= 2'd0;
              state            <= ST_CS;
            end else if (commit_gate) begin
              // DEFERRED write->write interpose: a new memory write is pending but the previous
              // write still needs committing. Self-issue the commit-read (shaped by COMMIT_READ_MODE);
              // the pending write command is NOT accepted (cmd_ready gated) and is taken after the
              // commit-read completes and returns to ST_IDLE.
              launch_commit_read(commit_read_base(last_wr_addr, wr_seg_base),
                                  commit_read_len(wr_seg_len));
              commit_resume <= 1'b0;               // deferred: return to ST_IDLE afterwards
            end else if (cmd_valid) begin
              cur_read  <= cmd_read;
              cur_reg   <= cmd_reg;
              cur_wrap  <= cmd_wrap;
              // WR_CHOP_REPLAY at a COMMAND-level contiguous write->write boundary (coalesce budget
              // exhausted at a command edge, coalesce timeout, or plain back-to-back contiguous
              // writes): the new CS# would wound [cmd_addr-4, cmd_addr) — the previous burst's tail
              // (2026-07-09 ladder: ANY write CA wounds the 4 words below its base). Open rolled
              // back with the mask-lead + shadow prefix, exactly like the ST_RECOVER chop reopen.
              if (acc_elig) begin
                cur_addr    <= acc_base;
                rem_left    <= cmd_len + acc_pfx;
                seg_count   <= acc_seg;
                seg_left    <= acc_seg;
                seg_cap_left<= acc_pfx + hw_cap(cmd_addr);
                ca_reg      <= hb_pack_ca(1'b0, 1'b0, 1'b1, acc_base);   // write, memory, linear
                replay_left <= acc_pfx;
                replay_real <= acc_rb;
                replay_idx  <= RP_AW'(acc_rb - LEN_WIDTH'(1));
              end else begin
                cur_addr    <= cmd_addr;
                rem_left    <= cmd_len;
                seg_count   <= seg_size(cmd_len, cmd_wrap, cmd_addr);
                seg_left    <= seg_size(cmd_len, cmd_wrap, cmd_addr);
                seg_cap_left<= hw_cap(cmd_addr);   // fresh CS# open: full hw-cap budget (WR_COALESCE)
                ca_reg      <= hb_pack_ca(cmd_read, cmd_reg, ~cmd_wrap, cmd_addr);
                replay_left <= '0;                 // a non-replay command never starts with a replay
              end
              rwds_hi   <= 1'b0;
              ca_idx    <= 2'd0;
              // rp_cnt tracks the LAST WRITE burst's tail; preserve it across READ accepts (reads
              // do not disturb the array or the shadow validity — silicon ladder L5/X2), reset it
              // for anything that writes (a new write rebuilds it; reg/wrap writes invalidate it).
              // issue #13: clear shadow_full here too — a genuine new write CS# open starts rebuilding
              // the shadow, so its first segment's pre-window drive is suppressed (= legacy idle bus).
              if (~cmd_read) begin
                rp_cnt      <= '0;
                shadow_full <= 1'b0;
              end
              wr_pending_commit <= 1'b0;           // a normally-accepted command clears the pending
                                                   //   flag (a covering read commits the prior write)
              // A1: if the device is in Deep-Power-Down, wake it (CS# pulse + tDPDOUT) BEFORE running
              // the command; the context latched above is issued once the wake completes.
              if (SUPPORT_DPD & in_dpd) begin
                cnt   <= 32'(DPD_WAKE_CYCLES - 1);
                state <= ST_DPD_WAKE;
              end else begin
                state <= ST_CS;
              end
            end
          end
        end

        // ---------------- CA phase ----------------
        ST_CS: begin
          state  <= ST_CA;
          ca_idx <= 2'd0;
        end

        ST_CA: begin
          if (phy_rwds_i) rwds_hi <= 1'b1;         // sample slave latency indicator
          if (ca_idx == 2'd2) begin
            if (zlw) begin
              state <= ST_WRITE;                   // zero-latency write: data follows CA immediately
            end else begin
              cnt            <= 32'(dbg_lat_clocks - 1);  // one latency count; doubled in ST_LAT if RWDS-high
                                                          //   (issue #13: runtime override; POR 6 = legacy)
              lat_extra_done <= 1'b0;
              state          <= ST_LAT;
            end
          end else begin
            ca_idx <= ca_idx + 1'b1;
          end
        end

        // ---------------- initial latency ----------------
        // Keep decoding the slave RWDS level throughout the latency window (not just CA): with the PHY
        // RWDS synchroniser the CA-High indicator may only become visible a cycle or two into ST_LAT.
        // When doubling is required, a SECOND latency count of LATENCY_CLOCKS is inserted before data
        // (SPEC_DIGEST §3: "an additional latency count is inserted"), so read/write data alignment
        // tracks the device's 2x initial latency exactly.
        ST_LAT: begin
          if (phy_rwds_i) rwds_hi <= 1'b1;
          if (cnt == 32'd0) begin
            if (lat_double & ~lat_extra_done) begin
              lat_extra_done <= 1'b1;
              // WR_LAT_TRIM: writes only — see the parameter note (board write-window calibration).
              // issue #13: dbg_lat_clocks/dbg_wr_lat_trim are runtime overrides (POR 6/3 = legacy).
              cnt            <= cur_read ? 32'(dbg_lat_clocks - 1)
                                         : 32'(dbg_lat_clocks - 1 + dbg_wr_lat_trim);
            end else if (cur_read) begin
              stall_cnt <= '0;
              state     <= ST_READ;
            end else begin
              state <= ST_WRITE;
            end
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        // ---------------- read data ----------------
        ST_READ: begin
          if (phy_dq_i_valid) begin
            // Commit-read data is DISCARDED (doing_commit): count the word for burst completion but
            // never push it to rd_fifo, so it is invisible to the front-end.
            if (!rd_fifo_full & ~doing_commit) begin
              rd_fifo[rd_wa] <= {(rem_left == LEN_WIDTH'(1)), phy_dq_i[DATA_WIDTH-1:0]};
              rd_wptr        <= rd_wptr + 1'b1;
            end
            stall_cnt <= '0;
            seg_left  <= seg_left - 1'b1;
            rem_left  <= rem_left - 1'b1;
            if (seg_left == LEN_WIDTH'(1)) begin
              if (rem_left != LEN_WIDTH'(1))       // chopped: advance to next linear segment
                cur_addr <= cur_addr + ADDR_WIDTH'(seg_count);
              // Enter the over-stream drain (CK stops, receiver stays armed) before the CS# tail, so
              // the device's post-CK-stop stragglers are absorbed and cannot leak into the next read.
              drain_cnt   <= '0;
              drain_dwell <= '0;
              state       <= ST_RD_DRAIN;
            end
          end else if (!phy_rwds_i & ~rd_backpressure) begin
            // (A2: while we are deliberately clock-stopped, RWDS-Low is expected — do NOT count it as
            //  the >=32-clk error stall; hold stall_cnt until CK resumes.)
            if (stall_cnt == 6'(READ_STALL_LIMIT - 1)) begin
              // RWDS stalled >= 32 clk (SPEC_DIGEST §4/§7): abort the read. Raise CS# (ST_RD_ABORT is
              // a bus-idle state) and, so the AXI/Avalon front-end's R channel still terminates with
              // exactly cmd_len beats + a final rd_last (AXI A3.4.1 / Avalon burst completion), drain
              // the remaining rem_left words as flagged filler rather than silently dropping rd_last.
              // A commit-read is internal (data discarded); a stall there must NOT surface as a
              // user-visible error, but it still drains through ST_RD_ABORT to recover cleanly.
              err_timeout <= ~doing_commit;
              state       <= ST_RD_ABORT;
            end else begin
              stall_cnt <= stall_cnt + 1'b1;
            end
          end
        end

        // ---------------- read over-stream drain (post burst) ----------------
        // CK is stopped (see combinational drive) so the device sources no new words, but the receiver
        // is held armed: any straggler/over-stream words the device already launched are recovered as
        // phy_dq_i_valid and DISCARDED here (never written to rd_fifo, never counted). Stay until the
        // PHY has been quiet for DRAIN_QUIET_CYCLES cycles (over-stream ended), bounded by
        // DRAIN_MAX_CYCLES. This absorbs a VARIABLE number of extra words so none leak into the next
        // read burst (the AXC3000 multi-burst hang). Then take the normal CS# tail + recovery.
        ST_RD_DRAIN: begin
          drain_dwell <= drain_dwell + 8'd1;
          if (phy_dq_i_valid) drain_cnt <= '0;             // a straggler arrived: not quiet yet
          else                drain_cnt <= drain_cnt + 8'd1;
          if (drain_done) begin
            cnt   <= postwin_seed;   // issue #13: read path -> postwin select is 0 => TAIL_CYCLES (legacy)
            state <= ST_TAIL;
          end
        end

        // ---------------- read abort drain (post-timeout) ----------------
        ST_RD_ABORT: begin
          if (rem_left == LEN_WIDTH'(0)) begin
            cnt   <= 32'(RECOVERY_CYCLES - 1);
            state <= ST_RECOVER;
          end else if (doing_commit | !rd_fifo_full) begin
            // Commit-read abort: discard (no fifo write, no back-pressure); else drain flagged filler.
            if (~doing_commit) begin
              rd_fifo[rd_wa] <= {(rem_left == LEN_WIDTH'(1)), {DATA_WIDTH{1'b0}}};
              rd_wptr        <= rd_wptr + 1'b1;
            end
            rem_left       <= rem_left - 1'b1;
            seg_left       <= (seg_left == LEN_WIDTH'(0)) ? seg_left : seg_left - 1'b1;
            if (rem_left == LEN_WIDTH'(1)) begin
              cnt   <= 32'(RECOVERY_CYCLES - 1);
              state <= ST_RECOVER;
            end
          end
        end

        // ---------------- write data ----------------
        ST_WRITE: begin
          // Host underrun: word gets masked, burst continues. A replayed beat (WR_CHOP_REPLAY) is
          // sourced from the replay shadow and never consumes the front-end, so wsrc_valid Low is
          // expected there — not an underrun.
          if (!wsrc_valid && !replay_beat) err_underrun <= 1'b1;
          // A1: snoop a host CR0 register-write for the Deep-Power-Down bit (CR0[15]); 0 => the device
          // will enter DPD, 1 => normal. Register writes are single-word / zero-latency, so the value
          // is on the bus this cycle. The next command is then guarded by a wake (ST_DPD_WAKE).
          if (SUPPORT_DPD & ~doing_init & cur_reg & ~cur_read &
              (cur_addr == ADDR_WIDTH'(HB_REG_CR0)) & wsrc_valid)
            in_dpd <= ~wsrc_data[15];
          seg_left     <= seg_left - 1'b1;
          rem_left     <= rem_left - 1'b1;
          // WR_COALESCE hw-cap tracking: one word of this still-open CS# burst's budget is spent every
          // ST_WRITE cycle, regardless of which native command it belongs to (see seg_cap_left above).
          seg_cap_left <= seg_cap_left - 1'b1;
          // WR_CHOP_REPLAY shadow capture: push every memory-space write beat sent this cycle —
          // replayed beats included — into the replay shift shadow (see its declaration for why the
          // fixed emit index stays correct under this shift), and count it toward rp_cnt (saturating
          // at WR_REPLAY_WORDS). An underrun filler is captured with an all-zero strobe.
          // Mask-led beats are NOT captured: they are array-neutral dummies, and shifting them in
          // would evict the real shadow words before their own replay emission.
          // issue #13: also capture the shadow when dbg_prewin_drive is on (the heal reuses rp_word's
          // STORAGE only — no replay reopen paths are enabled). With replay off, lead_beat/replay_beat
          // are 0, so this reduces to rp_word[0]<=wsrc_data and rp_cnt saturating at WR_REPLAY_WORDS.
          if ((WR_CHOP_REPLAY | dbg_prewin_drive) && !zlw && !lead_beat) begin
            for (int p = int'(WR_REPLAY_WORDS) - 1; p > 0; p--) begin
              rp_word[p] <= rp_word[p-1];
              rp_strb[p] <= rp_strb[p-1];
            end
            rp_word[0] <= replay_beat ? rp_data_sel : wsrc_data;
            rp_strb[0] <= replay_beat ? rp_strb_sel : (wsrc_valid ? wsrc_strb : '0);
            if (rp_cnt != LEN_WIDTH'(WR_REPLAY_WORDS)) rp_cnt <= rp_cnt + 1'b1;
            // Sticky "shadow holds a full WR_REPLAY_WORDS-deep tail" — set as rp_cnt saturates. Gates
            // the pre-window heal (prewin_ok) so a run's first segment (shadow not yet full) stays idle.
            if (rp_cnt >= LEN_WIDTH'(WR_REPLAY_WORDS - 1)) shadow_full <= 1'b1;
          end
          if (WR_CHOP_REPLAY && !zlw && replay_beat) replay_left <= replay_left - 1'b1;
          if (seg_left == LEN_WIDTH'(1)) begin
            // Last word of this write segment: record its (pre-advance) WORD address, and the
            // segment's own base/length, so a subsequent commit-read (WR_COMMIT_READ) can be shaped
            // by COMMIT_READ_MODE (SPAN_END spans last_wr_addr; FULL_BURST re-reads
            // [wr_seg_base, wr_seg_base+wr_seg_len-1] — this whole just-closed segment).
            last_wr_addr <= cur_addr + ADDR_WIDTH'(seg_count) - ADDR_WIDTH'(1);
            wr_seg_base  <= cur_addr;
            wr_seg_len   <= seg_count;
            // issue #13 post-window hold: latch this beat's data word and arm the hold, but ONLY for
            // the FINAL segment of a memory write (rem_left==1); a chop (rem_left!=1) leaves it off so
            // the hold applies once, at the burst's true end (the Law-3 end-at-row target).
            last_wr_word   <= replay_beat ? rp_data_sel : wsrc_data;
            postwin_active <= dbg_postwin_hold & ~cur_read & ~cur_reg & ~doing_init &
                              (rem_left == LEN_WIDTH'(1));
            if (rem_left != LEN_WIDTH'(1)) begin
              // more of THIS native command remains beyond this segment's tCSM/boundary cap: normal
              // internal chop, unaffected by WR_COALESCE (which only applies at command completion).
              cur_addr <= cur_addr + ADDR_WIDTH'(seg_count);
              cnt      <= postwin_seed;   // issue #13: chop (rem_left!=1) => TAIL_CYCLES (legacy)
              state    <= ST_TAIL;
            end else if (coalesce_try) begin
              // WR_COALESCE: this command's data is exhausted but the CS# burst still has hw-cap room
              // left (seg_cap_left, decremented above, > 0) — hold the bus open (CS# Low, CK stopped)
              // instead of closing CS#, and wait up to WR_COALESCE_WAIT cycles for a contiguous write
              // command to splice onto the SAME burst (issue #1 direction 4).
              coalesce_wait_cnt <= '0;
              state             <= ST_WR_COALESCE;
            end else begin
              cnt   <= postwin_seed;   // issue #13: final segment => 4-CK hold dwell when dbg_postwin_hold
              state <= ST_TAIL;
            end
          end
        end

        // ---------------- CS#-coalescing wait/splice (WR_COALESCE) ----------------
        ST_WR_COALESCE: begin
          // CK is stopped throughout this state (see the combinational drive), so no word is ever
          // actually transferred while waiting — seg_cap_left is therefore left untouched here; it
          // still holds exactly the budget remaining after the last REAL word (ST_WRITE's own
          // per-word countdown resumes seamlessly once state <= ST_WRITE below).
          if (coalesce_match) begin
            // Splice the new command onto the still-open CS# burst — no CS# close/reopen, no new
            // CA/latency phase. If its own cap-limited length (coalesce_accept_len) is shorter than
            // cmd_len, the remainder becomes a further chop once this spliced segment itself closes
            // (no special-casing needed — ST_WRITE's countdown just continues from seg_cap_left).
            cur_addr  <= cmd_addr;
            cur_read  <= 1'b0;
            cur_reg   <= 1'b0;
            cur_wrap  <= 1'b0;
            rem_left  <= cmd_len;
            seg_count <= coalesce_accept_len;
            seg_left  <= coalesce_accept_len;
            state     <= ST_WRITE;
          end else if (coalesce_wait_cnt + 16'd1 >= 16'(WR_COALESCE_WAIT)) begin
            // Waited long enough without a qualifying command: close normally (exactly the
            // pre-coalescing behavior — the pending command, if any, is taken via ST_IDLE once CS#
            // recovers).
            cnt   <= 32'(TAIL_CYCLES - 1);
            state <= ST_TAIL;
          end else begin
            // Keep waiting for a qualifying command (coalesce_wait_cnt bounds the total wait).
            coalesce_wait_cnt <= coalesce_wait_cnt + 16'd1;
          end
        end

        // ---------------- tail / recovery ----------------
        ST_TAIL: begin
          if (cnt == 32'd0) begin
            // WR_CHOP_PAUSE_CYCLES (2026-07-09 silicon experiment): after a memory-WRITE burst
            // closes, dwell extra CS#-High cycles before anything else. Hypothesis under test: the
            // device self-commits its pending write tail given time (internal buffer/ECC-block
            // merge); the observed tail loss + the 4-word kill zone below a reopened write CA are
            // both "next CS# arrived before the merge finished" artifacts. 0 = today's behavior.
            cnt   <= (~cur_read & ~cur_reg & ~doing_init & (WR_CHOP_PAUSE_CYCLES != 0))
                     ? 32'(RECOVERY_CYCLES - 1 + WR_CHOP_PAUSE_CYCLES)
                     : 32'(RECOVERY_CYCLES - 1);
            postwin_active <= 1'b0;   // issue #13: the post-window hold is over; CS# raises in ST_RECOVER
            state <= ST_RECOVER;
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        ST_RECOVER: begin
          if (cnt == 32'd0) begin
            if (rem_left != LEN_WIDTH'(0)) begin
              // more linear segments remain in THIS transaction (chopped by tCSM / boundary)
              if (WR_CHOP_REPLAY & ~cur_read & ~cur_reg & ~doing_commit & (rp_cnt != '0)) begin
                // WRITE-CHOP REPLAY reopen (takes precedence over the WR_COMMIT_READ chop interpose
                // below — read-shaped interposes are silicon-proven useless for write->write): the
                // device is about to discard the just-closed segment's last rp_rb words the moment
                // this next write CS# opens, so reopen rp_rb words EARLY and re-send them from the
                // replay shadow. rem_left is widened by rp_rb so the front-end word count stays
                // exact (the rp_rb replayed beats consume no wr channel data); seg_cap_left gets
                // the same prefix allowance as rp_seg so coalescing's continuous budget tracking
                // stays consistent with the segment actually opened.
                cur_addr     <= rp_base;
                rem_left     <= rem_left + rp_pfx;
                seg_count    <= rp_seg;
                seg_left     <= rp_seg;
                seg_cap_left <= rp_pfx + hw_cap(cur_addr);
                ca_reg       <= hb_pack_ca(1'b0, 1'b0, 1'b1, rp_base);   // write, memory, linear
                replay_left  <= rp_pfx;
                replay_real  <= rp_rb;
                replay_idx   <= RP_AW'(rp_rb - LEN_WIDTH'(1));
                rp_cnt       <= '0;                   // fresh CS#: nothing sent in it yet
                rwds_hi      <= 1'b0;
                ca_idx       <= 2'd0;
                state        <= ST_CS;
              end else if (WR_COMMIT_READ & ~cur_read & ~cur_reg & ~doing_commit) begin
                // tCSM / boundary CHOP interpose: commit the just-closed write segment's last word
                // with an internal commit-read, remembering where to resume the write afterwards.
                // (2026-07-10: documented-ineffective on silicon — see the parameter notes; reached
                // only when WR_CHOP_REPLAY is off.)
                sv_addr           <= cur_addr;        // next write-segment start (advanced in ST_WRITE)
                sv_rem            <= rem_left;
                commit_resume     <= 1'b1;
                wr_pending_commit <= 1'b1;            // segment closed -> needs committing
                launch_commit_read(commit_read_base(last_wr_addr, wr_seg_base),
                                    commit_read_len(wr_seg_len));
              end else begin
                // normal reopen (read / reg / commit-read, or WR_COMMIT_READ disabled)
                seg_count <= seg_size(rem_left, cur_wrap, cur_addr);
                seg_left  <= seg_size(rem_left, cur_wrap, cur_addr);
                seg_cap_left <= hw_cap(cur_addr);  // fresh CS# open: full hw-cap budget (WR_COALESCE)
                ca_reg    <= hb_pack_ca(cur_read, cur_reg, ~cur_wrap, cur_addr);
                rwds_hi   <= 1'b0;
                ca_idx    <= 2'd0;
                rp_cnt    <= '0;                   // fresh CS# (WR_CHOP_REPLAY word count)
                state     <= ST_CS;
              end
            end else if (doing_commit) begin
              // Internal commit-read finished (its recovered data was discarded).
              doing_commit      <= 1'b0;
              wr_pending_commit <= 1'b0;
              if (commit_resume) begin
                // Intra-command split write: restore the shadow context, reopen the write segment.
                commit_resume <= 1'b0;
                cur_read  <= 1'b0;
                cur_reg   <= 1'b0;
                cur_wrap  <= 1'b0;
                cur_addr  <= sv_addr;
                rem_left  <= sv_rem;
                seg_count <= seg_size(sv_rem, 1'b0, sv_addr);
                seg_left  <= seg_size(sv_rem, 1'b0, sv_addr);
                seg_cap_left <= hw_cap(sv_addr);   // fresh CS# open: full hw-cap budget (WR_COALESCE)
                ca_reg    <= hb_pack_ca(1'b0, 1'b0, 1'b1, sv_addr);   // write, memory, linear
                rwds_hi   <= 1'b0;
                ca_idx    <= 2'd0;
                rp_cnt    <= '0;                   // fresh CS# (WR_CHOP_REPLAY word count)
                state     <= ST_CS;
              end else begin
                state <= ST_IDLE;                     // deferred interpose done: take the pending cmd
              end
            end else if (doing_init & ~init_cr1 & PROGRAM_CR1 & ~doing_cr0_reprog) begin
              // A3: CR0 init write done -> chain the CR1 zero-latency register write before init_done.
              // issue #13: a runtime CR0 REPROGRAM (doing_cr0_reprog) never chains CR1 — it completes
              // straight to ST_IDLE below (with PROGRAM_CR1 off everywhere relevant this is moot, but
              // keeps the reprogram a pure single-register write regardless of the CR1 option).
              // (Placed AFTER the doing_commit branch: doing_init/doing_commit are mutually exclusive —
              // an internal commit-read is never in flight during init — so ordering vs. it is moot, but
              // the commit/coalesce machinery above always gets first refusal of this cnt==0 event.)
              init_cr1  <= 1'b1;
              cur_read  <= 1'b0;
              cur_reg   <= 1'b1;
              cur_wrap  <= 1'b0;
              cur_addr  <= HB_REG_CR1[ADDR_WIDTH-1:0];
              rem_left  <= LEN_WIDTH'(1);
              seg_left  <= LEN_WIDTH'(1);
              seg_count <= LEN_WIDTH'(1);
              ca_reg    <= hb_pack_ca(1'b0, 1'b1, 1'b1, HB_REG_CR1[ADDR_WIDTH-1:0]);
              rwds_hi   <= 1'b0;
              ca_idx    <= 2'd0;
              state     <= ST_CS;
            end else begin
              // Normal user / init transaction complete.
              if (~cur_read & ~cur_reg & ~doing_init)
                wr_pending_commit <= 1'b1;            // a memory write closed -> last word pending
              if (doing_init) begin
                init_done  <= 1'b1;
                doing_init <= 1'b0;
                init_cr1   <= 1'b0;
                doing_cr0_reprog <= 1'b0;   // issue #13: reprogram write complete (init_done stays 1)
              end
              state <= ST_IDLE;
            end
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        // ---------------- Deep-Power-Down exit (A1) ----------------
        ST_DPD_WAKE: begin
          // Hold the CS# wake pulse for DPD_WAKE_CYCLES, then raise CS# and count out tDPDOUT.
          if (cnt == 32'd0) begin
            cnt   <= 32'(TDPDOUT_CYCLES);
            state <= ST_DPD_OUT;
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        ST_DPD_OUT: begin
          // CS# High for tDPDOUT; the device returns to standby. Then issue the latched command.
          if (cnt == 32'd0) begin
            in_dpd <= 1'b0;
            state  <= ST_CS;
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        default: state <= ST_IDLE;
      endcase
    end
  end

  // wr_last is informational only (burst length is authoritative); waive unused-signal lint.
  /* verilator lint_off UNUSEDSIGNAL */
  logic _unused_wr_last;
  /* verilator lint_on UNUSEDSIGNAL */
  always_comb _unused_wr_last = wr_last;

  // DEBUG taps.
  assign dbg_state   = 4'(state);
  assign dbg_rd_wptr = 6'(rem_left);   // repurposed: rem_left (words remaining in whole burst)
  assign dbg_rd_rptr = 6'(seg_left);   // repurposed: seg_left (words remaining in current segment)

endmodule
`endif
