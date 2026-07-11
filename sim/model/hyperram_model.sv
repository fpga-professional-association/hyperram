// hyperram_model — behavioral, spec-accurate HyperRAM DEVICE (slave) model.
//
// GOLDEN reference model for the HyperBus master IP testbenches. Clean-room from
// docs/SPEC_DIGEST.md (Infineon/Cypress HyperBus 001-99253 Rev *H); no reference code copied.
//
// Scope (SPEC_DIGEST §2–§8):
//   * Decodes the 48-bit Command-Address (CA), MSB byte first, over the first 6 CK edges.
//   * Honors fixed / variable initial latency (SPEC_DIGEST §3): drives RWDS during CA as the
//     latency indicator (High = 2x, Low = 1x); variable-latency doubling is driven by a
//     deterministic, parameterized internal refresh-collision model (there is no external
//     collision port in the frozen interface — see REFRESH_EVERY).
//   * Drives RWDS as the source-synchronous read strobe, edge-aligned to read data.
//   * Accepts the master-driven RWDS write byte-mask during latency-bearing memory writes
//     (RWDS High => byte masked / array unchanged).
//   * Register-space writes are zero-latency, full-word, unmasked (SPEC_DIGEST §5/§6).
//   * Linear, legacy-wrapped and hybrid-wrapped bursts (SPEC_DIGEST §7).
//   * CR0/CR1 (writable) and ID0/ID1 (read-only) register space with reset values from the
//     package, always big-endian (byte A = word[15:8]).
//   * A backing memory array; optional mid-burst row-crossing latency gap (RWDS held Low).
//   * W957D8NB silicon-fidelity quirks (all opt-in; 0/off = ideal device, keeps every existing TB
//     aligned): a write-CA "wound" that zeroes N words below a memory write's CA base, optionally
//     mask-suppressible (WR_WOUND_WORDS / WR_WOUND_MASK_SUPPRESS); an end-of-burst 0x2000-word
//     garble (WR_BOUNDARY_END_GARBLE); a bus-release boundary quirk (BURST_BOUNDARY_WORDS); read
//     preamble/over-stream timing artifacts; and Deep-Power-Down.
//
// This is a PROTOCOL-accurate model, not an AC-timing model. hb_ck is a real DDR clock: one DQ
// byte is transferred on every CK edge (two bytes / cycle). Byte A (word[15:8]) is on the CK-High
// (rising) phase, byte B (word[7:0]) on the CK-Low (falling) phase (SPEC_DIGEST §4).
//
// Alignment contract for read data (race-free under verilator --binary):
//   During the read-data phase the model drives RWDS = CK and presents the current word
//   combinationally (byte A while CK High, byte B while CK Low). Data and RWDS transition together
//   at CK edges and each byte is valid for a full half-period, so a PHY that samples the eye centre
//   (e.g. off a 90-degree-shifted clock) captures cleanly. During a row-crossing gap RWDS is held
//   Low and no strobe is produced (SPEC_DIGEST §4/§7).
//
// Split-driver pins (Verilator-safe; the TB resolves the shared bus — no inout in the model). The
// model consumes what the master drives (hb_dq_i / hb_rwds_i, with the master's *_ie enables) and
// exposes what it drives (hb_dq_o / hb_rwds_o with its own *_oe). See docs/INTERFACES.md.
`ifndef HYPERRAM_MODEL_SV
`define HYPERRAM_MODEL_SV
`timescale 1ns/1ps
module hyperram_model
  import hyperbus_pkg::*;
#(
    parameter int unsigned DQ_WIDTH       = HB_DQ_WIDTH_DEFAULT,      // 8 (HyperBus is 8-bit DQ)
    parameter int unsigned MEM_WORDS      = 1 << 16,                  // backing array depth (words)
    parameter int unsigned LATENCY_CLOCKS = HB_LATENCY_CLOCKS_DEFAULT,// initial latency (CK cycles)
    parameter bit          FIXED_LATENCY  = HB_FIXED_LATENCY_DEFAULT, // 1 = fixed, 0 = variable
    parameter int unsigned ROW_WORDS      = 64,                       // row size (words); 0 disables gaps
    parameter int unsigned ROW_PENALTY    = 4,                        // extra latency (CK cycles) per row crossing
    parameter int unsigned REFRESH_EVERY  = 0,                        // variable-latency collision cadence:
                                                                      //   0 = never; N>0 = every N-th transaction
                                                                      //   inserts the additional latency count.
    parameter bit          FIXED_2X       = 1'b0,                     // SPEC_DIGEST §5.2.4 fixed-latency
                                                                      //   variant: when in FIXED mode, drive
                                                                      //   RWDS HIGH during CA and always use
                                                                      //   two latency counts (2x). This is the
                                                                      //   common Cypress/Winbond behaviour.
    parameter int unsigned STALL_AT       = 0,                        // read-stall injection: word index into a
    parameter int unsigned STALL_CLOCKS   = 0,                        //   read burst at which to hold RWDS Low
                                                                      //   for STALL_CLOCKS CK cycles (0=never).
                                                                      //   Models the >=32-clk error stall.
    parameter int unsigned RD_PREAMBLE_CLOCKS = 0,                    // read-strobe PREAMBLE (CK cycles): before
                                                                      //   the first real read byte the device
                                                                      //   toggles RWDS (=CK) with DQ Hi-Z (reads
                                                                      //   as 0x00). Reproduces the real Winbond
                                                                      //   W957D8NB read turnaround captured on the
                                                                      //   AXC3000 board (cap idx85-88). 0 = none
                                                                      //   (spec-ideal; keeps existing TBs aligned).
    parameter int unsigned RD_OVERSTREAM_WORDS = 0,                   // READ over-stream (CK-stop pipeline latency):
                                                                      //   after the master STOPS CK ending a read burst
                                                                      //   the real W957D8NB keeps pushing this many EXTRA
                                                                      //   source-synchronous words (RWDS + DQ) before its
                                                                      //   read output pipeline drains — the master's CK-stop
                                                                      //   has flight/pipeline latency, so the device over-runs
                                                                      //   the master's word count (AXC3000: ~23 words seen for
                                                                      //   a 16-word request). The ideal edge-driven data phase
                                                                      //   below stops the instant CK stops and never exercised
                                                                      //   this; the SELF-TIMED tail below reproduces it.
                                                                      //   0 = ideal device (keeps all existing TBs aligned).
    // ---- W957D8NB WRITE-CA "wound" (2026-07-09 silicon ladder). Supersedes the WR_COMMIT_QUIRK
    //      pending/discard story further below, which read-only probing PROVED FALSE: a write burst's
    //      tail commits to the array fine on its own — [508..511] of a 512-word burst read back
    //      intact after 3 later writes elsewhere. The real defect is CA-time, not commit-time. ----
    parameter int unsigned WR_WOUND_WORDS = 0,                         // W957D8NB WRITE-CA wound (silicon
                                                                      //   ladder finding 2): ANY memory-
                                                                      //   space WRITE CS# that opens at
                                                                      //   word address B zeroes mem[B-N ..
                                                                      //   B-1] — the N words immediately
                                                                      //   BELOW its CA base — applied at
                                                                      //   CA decode, BEFORE this burst's
                                                                      //   own data beats land (so a burst
                                                                      //   whose own range later covers the
                                                                      //   wound zone heals it). READ CAs
                                                                      //   never wound (finding 3). N=4
                                                                      //   matches the real W957D8NB. 0 =
                                                                      //   ideal device (keeps all existing
                                                                      //   TBs aligned).
    parameter bit          WR_WOUND_MASK_SUPPRESS = 1'b0,              // E-D hypothesis (2026-07-09):
                                                                      //   does leading the reopened write
                                                                      //   burst with fully byte-masked
                                                                      //   beats suppress its OWN wound?
                                                                      //   When 1, the armed wound (see
                                                                      //   WR_WOUND_WORDS) is DISCARDED
                                                                      //   instead of applied if this
                                                                      //   burst's first data word (beat 0)
                                                                      //   arrives with RWDS High on BOTH
                                                                      //   phases (fully masked = no real
                                                                      //   byte written). Resolved
                                                                      //   retroactively: the wound zone is
                                                                      //   recorded at CA time and only
                                                                      //   actually applied to mem[] once
                                                                      //   beat 0's mask is known. 0 =
                                                                      //   unsuppressible (apply the wound
                                                                      //   regardless of masking; keeps all
                                                                      //   existing TBs aligned).
    parameter bit          WR_WOUND_SAMPLE_BUS = 1'b0,                 // issue #13 (L-C/L-D, the HEAL): the
                                                                      //   load-bearing sampling hypothesis.
                                                                      //   When 1 the wound content is NOT a
                                                                      //   hard zero but the DQ words the
                                                                      //   device SAMPLES off the bus in the
                                                                      //   pre-data window — the last
                                                                      //   2*WR_WOUND_WORDS latency edges
                                                                      //   before first_data_beat, mapped
                                                                      //   oldest-first (see wound_samp). So
                                                                      //   whatever the controller drives in
                                                                      //   that window lands in mem[B-N..B-1]:
                                                                      //   idle bus -> 0x0000 (the 8-aligned-
                                                                      //   zero observation); residual CA
                                                                      //   bytes -> foreign (0x0404 @508);
                                                                      //   dbg_prewin_drive shadow -> the exact
                                                                      //   [B-4..B-1] (readback ERR=0 = the
                                                                      //   heal); dbg_prewin_marker -> 0xA5xx
                                                                      //   (content attribution). 0 = hard-
                                                                      //   zero the wound (identical to the
                                                                      //   pre-#13 model; keeps every existing
                                                                      //   TB aligned).
    parameter bit          WR_WOUND_WRAP_IMMUNE = 1'b0,                // issue #13 (L-F): when 1 a WRAPPED
                                                                      //   memory write (CA[45]=0) does NOT
                                                                      //   arm a wound. Makes a wrapped write
                                                                      //   over a wound zone a REPAIR primitive
                                                                      //   (it writes real data and re-arms
                                                                      //   nothing). 0 = wrapped writes wound
                                                                      //   like linear (keeps TBs aligned).
    parameter bit          WR_BOUNDARY_END_GARBLE = 1'b0,              // silicon ladder finding 5 (a
                                                                      //   separate, rarer defect from the
                                                                      //   wound above): a memory WRITE
                                                                      //   burst whose FINAL word address +
                                                                      //   1 lands exactly on a 16'h2000-
                                                                      //   WORD (16KB) boundary gets its own
                                                                      //   last 4 words persistently
                                                                      //   garbled to 16'h5050 at CS# close.
                                                                      //   0 = disabled (keeps all existing
                                                                      //   TBs aligned).
                                                                      //   NOTE (v4/v5 silicon): the true
                                                                      //   granularity is EVERY 1024-word ROW
                                                                      //   multiple (0x2000 is one); the
                                                                      //   model keeps 0x2000 to preserve the
                                                                      //   TB's documented case — generalize
                                                                      //   alongside a row-WRAP model if a
                                                                      //   row-accurate device sim is needed
                                                                      //   (the controller now never crosses
                                                                      //   rows, so sim never exercises it).
    parameter int unsigned WR_END_GARBLE_ROW_WORDS = 32'h2000,        // issue #13: the WR_BOUNDARY_END_
                                                                      //   GARBLE granularity, parameterized.
                                                                      //   Default 0x2000 preserves tb_commit
                                                                      //   idx7 (a burst ending at 0x2000);
                                                                      //   instantiate 1024 for the true
                                                                      //   W957D8NB law — a write ending on
                                                                      //   ANY 1024-word ROW multiple garbles
                                                                      //   its own last 4 words.
    parameter logic [15:0] WR_END_GARBLE_VALUE     = 16'h5050,        // issue #13: the persistent garble fill
                                                                      //   (0x5050 observed). A param so a
                                                                      //   content-attribution run can retarget
                                                                      //   it. Default keeps tb_commit idx7.
    parameter bit          WR_END_GARBLE_SAMPLE_BUS = 1'b0,           // issue #13 (L-E, optional, default off):
                                                                      //   the Law-3 analog of WR_WOUND_SAMPLE_
                                                                      //   BUS. When 1 the end-garble stores the
                                                                      //   last written word (what dbg_postwin_
                                                                      //   hold parks on DQ into the tail)
                                                                      //   instead of WR_END_GARBLE_VALUE — so a
                                                                      //   postwin heal CHANGES the garble
                                                                      //   content. Only ONE word is held, so
                                                                      //   B-4..B-2 still cannot fully heal —
                                                                      //   matching the L-E prediction that
                                                                      //   postwin-hold is a separate, partial
                                                                      //   mechanism from the pre-window heal.
                                                                      //   0 = constant fill (keeps TBs aligned).
    parameter bit          WR_ORPHAN_MODEL = 1'b0,                    // issue #13 ROUND 4: the FULL, RO/EMAP-
                                                                      //   verified W957D8NB write-path mechanism
                                                                      //   (silicon facts 1-5). Default OFF. When
                                                                      //   ON it SUBSUMES the WR_WOUND_SAMPLE_BUS
                                                                      //   write-open behaviour (forces sampled-
                                                                      //   bus commit at [B-4,B), so requires
                                                                      //   WR_WOUND_WORDS=4) AND replaces the
                                                                      //   WR_END_GARBLE end behaviour with the
                                                                      //   ORPHAN/SPRAY model:
                                                                      //   (fact 1) write CS# open at B commits
                                                                      //     the 4 pre-data-window sampled words
                                                                      //     at [B-4,B) (reuses wound_samp);
                                                                      //   (fact 2) a memory write CLOSING exactly
                                                                      //     on a WR_END_GARBLE_ROW_WORDS multiple
                                                                      //     PARKS its last 4 real data words as an
                                                                      //     ORPHAN {home=end, 4 words} instead of
                                                                      //     the constant end-garble;
                                                                      //   (fact 3) orphans are NOT consumed by
                                                                      //     later writes and MULTIPLE COEXIST
                                                                      //     (append-only list, oldest dropped on
                                                                      //     overflow);
                                                                      //   (fact 4) the FIRST read CS# open fires
                                                                      //     ALL parked orphans — each sprays its 4
                                                                      //     words to [home-ROW-4, home-ROW) (one
                                                                      //     ROW below home; target < 0 dropped) —
                                                                      //     then clears the list;
                                                                      //   (fact 5) spray(home=R_k) lands on
                                                                      //     [R_k-ROW-4, R_k-ROW) = R_{k-1}'s home.
                                                                      //   MODEL SIMPLIFICATION (documented): the
                                                                      //   data phase still commits the last 4
                                                                      //   words to mem[home-4..home-1] (fact 2's
                                                                      //   "never commits at home" is NOT undone).
                                                                      //   It is a no-op under every configuration
                                                                      //   this exercises — on silicon home is
                                                                      //   healed anyway (round-3 end-commit-WRITE
                                                                      //   for the final home, the contiguous
                                                                      //   prewin reopen for interior homes) — so
                                                                      //   the OBSERVABLE array state (the spray
                                                                      //   onto the previous row's home) is exact,
                                                                      //   which is what the defuse must fix. A
                                                                      //   FULLY-MASKED closing burst parks NO
                                                                      //   orphan (mask suppresses parking): this
                                                                      //   is the simplest choice consistent with
                                                                      //   every silicon observation (the round-3
                                                                      //   fully-masked end-write left the real
                                                                      //   orphan intact = "did not form a second
                                                                      //   orphan"). 0 = disabled (WR_WOUND_SAMPLE_
                                                                      //   BUS / WR_END_GARBLE behave as before;
                                                                      //   keeps every existing TB aligned).
    parameter int unsigned WR_ORPHAN_DEPTH = 8,                       // issue #13 R4: orphan list depth (facts
                                                                      //   3/5: 4096/256 has 3 coexisting interior
                                                                      //   sprays; 8 is plenty). Overflow drops the
                                                                      //   OLDEST entry (documented).
    // ---- DEPRECATED (2026-07-09): the split-write "commit quirk" (pending/discard on the next
    //      write) this pair used to model is FALSIFIED on silicon — see WR_WOUND_WORDS above for the
    //      real defect. Both names are kept, ACCEPTED BUT IGNORED (no-ops), only so any stale
    //      instantiation still elaborates; no logic below reads them. Use WR_WOUND_WORDS /
    //      WR_WOUND_MASK_SUPPRESS instead. ----
    parameter bit          WR_COMMIT_QUIRK  = 1'b0,                    // DEPRECATED no-op (see above)
    parameter int unsigned WR_PENDING_WORDS = 1,                       // DEPRECATED no-op (see above)
    parameter int unsigned BURST_BOUNDARY_WORDS = 0,                  // W957D8NB 0x2000-WORD boundary quirk: when a
                                                                      //   single burst crosses this WORD-aligned boundary
                                                                      //   the device RELEASES the bus (stops driving on
                                                                      //   reads / stops capturing on writes) for the rest
                                                                      //   of that CS#; reads past it return floating junk.
                                                                      //   0 = disabled (keeps existing TBs aligned).
    parameter logic [15:0] ID0_RESET      = HB_ID0_RESET,             // read-only device ID (mfr nibble)
    parameter logic [15:0] ID1_RESET      = HB_ID1_RESET,             // read-only device ID (type nibble)
    parameter logic [15:0] CR0_RESET      = HB_CR0_RESET,             // config register 0 reset image
    parameter logic [15:0] CR1_RESET      = HB_CR1_RESET,             // config register 1 reset image
    parameter bit          SUPPORT_DPD    = 1'b0                      // Deep-Power-Down modeling (SPEC_DIGEST
                                                                      //   §5.2.1 / §8.7, CR0[15]). 0 = ignore
                                                                      //   DPD (keeps all existing TBs aligned).
                                                                      //   1 = a CR0[15]=0 write puts the device
                                                                      //   asleep (drives NOTHING: no read strobe/
                                                                      //   data, no CA latency indicator) until a
                                                                      //   CS# wake pulse (or a CR0[15]=1 write)
                                                                      //   returns it to standby.
) (
    input  logic                 hb_ck,      // clock from master
    input  logic                 hb_ck_n,    // complementary clock (tied if single-ended)
    input  logic                 hb_cs_n,    // chip select, active low
    input  logic                 hb_rst_n,   // device reset, active low
    input  logic [DQ_WIDTH-1:0]  hb_dq_i,    // DQ driven BY MASTER (CA / write data)
    input  logic                 hb_dq_ie,   // master DQ output-enable (turnaround awareness)
    output logic [DQ_WIDTH-1:0]  hb_dq_o,    // DQ driven BY MODEL (read data)
    output logic                 hb_dq_oe,   // model DQ output-enable
    input  logic                 hb_rwds_i,  // RWDS driven BY MASTER (write byte-mask)
    input  logic                 hb_rwds_ie, // master RWDS output-enable
    output logic                 hb_rwds_o,  // RWDS driven BY MODEL (CA latency indicator + read strobe)
    output logic                 hb_rwds_oe  // model RWDS output-enable
);

  // ------------------------------------------------------------------------
  // Local geometry. HyperBus words are 16-bit; DQ is DQ_WIDTH (8) so one word = 2 bytes.
  // ------------------------------------------------------------------------
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;          // 16-bit HyperBus word
  localparam int unsigned AW         = $clog2(MEM_WORDS);     // word-address width into the array
  localparam int unsigned CA_SR_BITS = HB_CA_BITS - DQ_WIDTH; // CA bytes 0..4 retained; byte 5 is live

  // Backing array (word-addressed). Not cleared by hardware reset (spec: reset restores config
  // registers, not the array).
  logic [DATA_WIDTH-1:0] mem [MEM_WORDS];

  // Config / ID registers (16-bit, always big-endian on the wire).
  logic [DATA_WIDTH-1:0] id0, id1, cr0, cr1;
  logic                  cr0_written; // once CR0 is programmed, latency follows the register
  logic                  dpd_active;  // SUPPORT_DPD: device is in Deep-Power-Down (asleep, drives nothing)

  // ------------------------------------------------------------------------
  // Transaction state (all updated in the single CK-edge process below).
  // ------------------------------------------------------------------------
  logic                    ck_q;            // previous CK level, for edge detection
  logic                    cs_q;            // previous CS# level, for start-of-transaction detection
  logic [15:0]             beat;            // number of CK edges processed since CS# fell
  logic [CA_SR_BITS-1:0]   ca_sr;           // CA shift register (MSB byte first; holds bytes 0..4)
  logic                    cur_rw;          // CA[47] : 1 = read
  logic                    cur_as;          // CA[46] : 1 = register space
  logic                    cur_linear;      // CA[45] : 1 = linear burst
  logic [HB_ADDR_WIDTH-1:0]cur_reg_addr;    // decoded word address (register-space compare)
  logic [15:0]             first_data_beat; // beat index at which the first data byte (A) appears
  logic                    collide;         // this transaction inserts the additional latency count
  logic [31:0]             txn_cnt;         // transactions since reset (drives the refresh model)

  logic [AW-1:0]           addr;            // running memory word address (data phase)
  logic [31:0]             wcnt;            // words delivered so far (wrap / hybrid bookkeeping)
  logic [AW-1:0]           group_base;      // wrap-group base address
  logic [AW-1:0]           group_top;       // wrap-group top address
  logic [AW-1:0]           wrap_words_r;    // wrap-group size in words (latched at CA decode)
  logic [15:0]             pen;             // row-crossing gap countdown (CK edges), reads only
  logic [15:0]             pen_pending;     // gap armed at byte B, activated at the next word slot
  logic [DATA_WIDTH-1:0]   out_word;        // read word currently being driven onto DQ
  logic [DQ_WIDTH-1:0]     wr_hi;           // captured byte A during a register write
  logic                    stall_done;      // read-stall (STALL_CLOCKS) already injected this txn

  // ---- W957D8NB write-CA wound (WR_WOUND_WORDS / WR_WOUND_MASK_SUPPRESS) ----
  // A memory WRITE's CA decode ARMS a candidate wound (base = this burst's own CA address); it is
  // RESOLVED once beat 0 (word index 0 of this burst's data phase) is fully sampled: applied
  // (mem[base-N .. base-1] zeroed) unless WR_WOUND_MASK_SUPPRESS is set AND beat 0 arrived fully
  // byte-masked (RWDS High on both phases), in which case it is discarded. Resolving at beat 0 rather
  // than at CA time itself is what lets WR_WOUND_MASK_SUPPRESS see the mask before deciding.
  logic                    wound_pending;   // an armed, not-yet-resolved wound from this CS#'s CA decode
  logic [AW-1:0]           wound_base;      // CA base (B) for the armed wound
  logic                    wound_mask_hi;   // beat-0 byte-A (rising-edge) RWDS sample

  // ---- WR_WOUND_SAMPLE_BUS pre-data sampling buffer (issue #13) ----
  // The DQ words captured off the bus in the last 2*WR_WOUND_WORDS latency edges before first_data_beat.
  // wound_samp[0] is the OLDEST slot (sampled first, at cnt=N-1 = the earliest window edge) and
  // wound_samp[N-1] the NEWEST (sampled last, at cnt=0 = the edge just before data). Applied newest->
  // B-1: mem[B-k] <= wound_samp[N-k]. Depth is guarded >=1 so the default WR_WOUND_WORDS=0 build
  // elaborates (the buffer is dead when WR_WOUND_SAMPLE_BUS=0).
  localparam int unsigned  WOUND_SAMP_DEPTH = (WR_WOUND_WORDS == 0) ? 1 : WR_WOUND_WORDS;
  logic [15:0]             wound_samp [WOUND_SAMP_DEPTH];

  // ---- issue #13 ROUND 4: sampled-bus commit is FORCED ON by WR_ORPHAN_MODEL (fact 1). So the
  //      write-open wound content follows the DQ pre-data window whenever EITHER the standalone
  //      WR_WOUND_SAMPLE_BUS knob or the full orphan model is enabled; OFF for both reduces to the
  //      pre-#13 hard-zero wound (identical to legacy).
  wire                     sample_bus = WR_WOUND_SAMPLE_BUS | WR_ORPHAN_MODEL;

  // ---- issue #13 ROUND 4 orphan/spray machinery (WR_ORPHAN_MODEL, silicon facts 2-5) ----
  // Rolling shadow of the last 4 REAL data words RECEIVED on the bus this write burst (mask-agnostic:
  // the value seen on DQ, since fact 2 parks what the device received). wr_tail[3] = NEWEST (the just-
  // completed word = word(addr-1) at close), wr_tail[0] = OLDEST of the last 4. wr_ba latches byte A
  // (rising edge) so the falling edge can assemble the full big-endian word {A,B}. wr_any_unmasked is
  // set if ANY byte of this burst arrived unmasked — a FULLY-masked burst parks NO orphan (fact 3
  // simplest-consistent choice, documented on WR_ORPHAN_MODEL).
  logic [DATA_WIDTH-1:0]   wr_tail [4];
  logic [DQ_WIDTH-1:0]     wr_ba;
  logic                    wr_any_unmasked;

  // Orphan list (append-only until a read fires it; oldest dropped on overflow). Each entry holds the
  // parking home (= the row-multiple close address) and its 4 tail words (wr_tail order preserved, so
  // the spray lays them one row below in the SAME relative order — fact 5 EMAP: word(home-4)->word(
  // home-ROW-4)). Depth guarded >=1 for elaboration when WR_ORPHAN_MODEL=0 (the list is then dead).
  localparam int unsigned  ORPH_DEPTH = (WR_ORPHAN_DEPTH == 0) ? 1 : WR_ORPHAN_DEPTH;
  logic                    orph_valid [ORPH_DEPTH];
  logic [AW-1:0]           orph_home  [ORPH_DEPTH];
  logic [DATA_WIDTH-1:0]   orph_word  [ORPH_DEPTH][4];
  logic [$clog2(ORPH_DEPTH+1)-1:0] orph_cnt;   // number of valid entries (0..ORPH_DEPTH)

  // ---- 0x2000-word boundary release quirk (BURST_BOUNDARY_WORDS) ----
  logic                    bnd_rel;         // this transaction crossed a boundary -> bus released

  // ---- read over-stream tail (self-timed; models the CK-stop pipeline latency) ----
  realtime                 os_half;         // measured read-data CK half-period (tail cadence)
  realtime                 os_last_edge;    // $realtime of the most recent read-data CK edge (0 = none)
  logic                    os_run;          // 1 while the device drives the over-stream tail
  logic                    os_rwds_o;       // tail RWDS drive
  logic [DQ_WIDTH-1:0]     os_dq_o;         // tail DQ byte drive

  // ------------------------------------------------------------------------
  // Combinational helpers.
  // ------------------------------------------------------------------------
  // Selected + not in reset.
  wire busy = hb_rst_n & ~hb_cs_n;

  // The 48-bit CA becomes complete on the 6th CA byte (this cycle's hb_dq_i is the last byte).
  wire [HB_CA_BITS-1:0]     full_ca   = {ca_sr, hb_dq_i};
  /* verilator lint_off UNUSEDSIGNAL */                            // upper address bits > array depth
  wire [HB_ADDR_WIDTH-1:0]  full_addr = hb_ca_addr(full_ca);       // decoded WORD address
  /* verilator lint_on UNUSEDSIGNAL */
  wire [AW-1:0]             start_addr= full_addr[AW-1:0];         // into the backing array
  wire [AW-1:0]             wrap_w    = AW'(hb_wrap_words(cr0[1:0]));

  // Effective fixed/variable and latency count: parameter defaults until CR0 is programmed, then
  // the CR0 register governs (SPEC_DIGEST §3, CR0[3] / CR0[7:4]).
  wire        fixed_active = cr0_written ? cr0[3] : (FIXED_LATENCY != 1'b0);
  wire [15:0] lat_active   = cr0_written ? 16'(hb_latency_code_to_clocks(cr0[7:4]))
                                         : 16'(LATENCY_CLOCKS);

  // Register-space readback value (aliased decode on the low word address).
  wire [DATA_WIDTH-1:0] reg_val =
      (cur_reg_addr == HB_REG_ID0) ? id0 :
      (cur_reg_addr == HB_REG_ID1) ? id1 :
      (cur_reg_addr == HB_REG_CR0) ? cr0 :
      (cur_reg_addr == HB_REG_CR1) ? cr1 : '0;

  // Next-address generator: linear increment, legacy wrap-in-group, or hybrid (wrap once then
  // linear from the next group boundary). SPEC_DIGEST §7 / Table 5.4.
  function automatic logic [AW-1:0] next_addr(input logic [AW-1:0] a, input logic [31:0] cnt);
    logic [AW-1:0] n;
    if (cur_linear) begin
      n = a + 1'b1;                                   // linear
    end else if (cr0[2]) begin
      n = (a == group_top) ? group_base : a + 1'b1;   // legacy wrap: stay in group forever
    end else begin                                    // hybrid wrap: one traversal, then linear
      if (cnt == {{(32-AW){1'b0}}, wrap_words_r} - 1)
        n = group_base + wrap_words_r;                // leave the group at the next boundary
      else if (cnt >= {{(32-AW){1'b0}}, wrap_words_r})
        n = a + 1'b1;                                 // linear tail
      else
        n = (a == group_top) ? group_base : a + 1'b1; // still wrapping
    end
    return n;
  endfunction

  // ------------------------------------------------------------------------
  // Backing-store initialization. Deterministic byte pattern so read-before-write is predictable
  // and matches the byte-addressed convention (byte at byte-address i = i*13+7): word a holds
  // {byte(2a), byte(2a+1)} = {A=high, B=low}.
  // ------------------------------------------------------------------------
  initial begin
    for (int unsigned a = 0; a < MEM_WORDS; a++)
      mem[a] = { DQ_WIDTH'((2*a  )*13 + 7),
                 DQ_WIDTH'((2*a+1)*13 + 7) };
  end

  // ------------------------------------------------------------------------
  // Single edge-detecting process. hb_ck is not free-running (idle while CS# High), so CS#/RST#
  // are in the sensitivity list; a stored ck_q recovers both CK edges (DDR). Verified to elaborate
  // and simulate clean under verilator --binary 5.020.
  // ------------------------------------------------------------------------
  always @(hb_ck or hb_cs_n or hb_rst_n) begin
    if (!hb_rst_n) begin
      // Hardware reset: restore config/ID registers, drop to idle (SPEC_DIGEST §9).
      beat        <= '0;
      ca_sr       <= '0;
      pen         <= '0;
      pen_pending <= '0;
      wcnt        <= '0;
      stall_done  <= 1'b0;
      wound_pending <= 1'b0;
      wound_base    <= '0;
      wound_mask_hi <= 1'b0;
      wr_any_unmasked <= 1'b0;
      wr_ba         <= '0;
      for (int t = 0; t < 4; t++) wr_tail[t] <= '0;
      for (int e = 0; e < int'(ORPH_DEPTH); e++) orph_valid[e] <= 1'b0;
      orph_cnt      <= '0;
      bnd_rel     <= 1'b0;
      cs_q        <= 1'b1;
      ck_q        <= hb_ck;
      txn_cnt     <= '0;
      cr0_written <= 1'b0;
      dpd_active  <= 1'b0;   // hardware reset forces exit from Deep-Power-Down (SPEC_DIGEST §8.7)
      id0         <= DATA_WIDTH'(ID0_RESET);
      id1         <= DATA_WIDTH'(ID1_RESET);
      cr0         <= DATA_WIDTH'(CR0_RESET);
      cr1         <= DATA_WIDTH'(CR1_RESET);
    end else if (hb_cs_n) begin
      // Idle between transactions: clear per-transaction state, keep registers.
      // DPD exit: a CS# pulse that carried NO clock (beat still 0 at CS# rise) is the master's wake
      // trigger — return from Deep-Power-Down to standby (SPEC_DIGEST §5.2.1). A real (clocked)
      // transaction has beat > 0 here, so it never counts as a wake.
      if (SUPPORT_DPD && dpd_active && (beat == 16'd0)) dpd_active <= 1'b0;
      beat  <= '0;
      ca_sr <= '0;
      pen   <= '0;
      pen_pending <= '0;
      wcnt  <= '0;
      cs_q  <= 1'b1;
      ck_q  <= hb_ck;
      bnd_rel <= 1'b0;                         // boundary-release latch is per-transaction
      // WR_BOUNDARY_END_GARBLE (finding 5): a memory WRITE burst that just closed with its final word
      // address + 1 landing exactly on a 0x2000-word boundary gets its own last 4 words persistently
      // garbled. `addr`/`wcnt`/`cur_rw`/`cur_as` still hold the just-ended burst's values here (not yet
      // reset — the resets below only take effect after this invocation); guard on wcnt!=0 so this
      // only fires once per CS# (a re-invocation of this branch — e.g. from CK still toggling while
      // CS# is High, WR_CHOP_PAUSE_CK — sees wcnt already cleared to 0 by the first pass).
      // issue #13 ROUND 4: WR_ORPHAN_MODEL SUPERSEDES the constant end-garble. A memory-write burst
      // that closes with `addr` (one past the last written word = its home/end) on a WR_END_GARBLE_ROW_
      // WORDS multiple PARKS an ORPHAN {home=addr, last-4 tail words} (fact 2) instead of garbling —
      // fired later by the first read (fact 4). A FULLY-masked burst parks nothing (wr_any_unmasked).
      // Append-only (facts 3): drop the OLDEST on overflow. When WR_ORPHAN_MODEL=0 the legacy constant/
      // sampled end-garble path below runs unchanged.
      if (WR_ORPHAN_MODEL) begin
        if (!cur_rw && !cur_as && (wcnt != 32'd0) && wr_any_unmasked &&
            ((32'(addr) % WR_END_GARBLE_ROW_WORDS) == 32'd0)) begin
          if (orph_cnt < ($clog2(ORPH_DEPTH+1))'(ORPH_DEPTH)) begin
            orph_valid[orph_cnt] <= 1'b1;
            orph_home [orph_cnt] <= addr;
            for (int i = 0; i < 4; i++) orph_word[orph_cnt][i] <= wr_tail[i];
            orph_cnt <= orph_cnt + 1'b1;
          end else begin
            // full: shift down (drop the oldest, entry 0), append at the top slot
            for (int e = 0; e < int'(ORPH_DEPTH) - 1; e++) begin
              orph_valid[e] <= orph_valid[e+1];
              orph_home [e] <= orph_home [e+1];
              for (int i = 0; i < 4; i++) orph_word[e][i] <= orph_word[e+1][i];
            end
            orph_valid[ORPH_DEPTH-1] <= 1'b1;
            orph_home [ORPH_DEPTH-1] <= addr;
            for (int i = 0; i < 4; i++) orph_word[ORPH_DEPTH-1][i] <= wr_tail[i];
          end
        end
      end else if (WR_BOUNDARY_END_GARBLE && !cur_rw && !cur_as && (wcnt != 32'd0) &&
          ((32'(addr) % WR_END_GARBLE_ROW_WORDS) == 32'd0)) begin
        // Fill: WR_END_GARBLE_VALUE (constant, the pre-#13 behaviour) unless WR_END_GARBLE_SAMPLE_BUS —
        // then the postwin-held last word (mem[addr-1] still holds it here, garble not yet applied) is
        // stored instead, the Law-3 analog of the pre-window sampling heal. addr>=1 guard: never index
        // below the array.
        logic [DATA_WIDTH-1:0] eg_val;
        eg_val = (WR_END_GARBLE_SAMPLE_BUS && (addr >= AW'(1))) ? mem[addr - AW'(1)]
                                                                : DATA_WIDTH'(WR_END_GARBLE_VALUE);
        for (int k = 1; k <= 4; k++)
          if (addr >= AW'(k)) mem[addr - AW'(k)] <= eg_val;
      end
      // Safety net: a wound armed at CA decode is normally resolved at beat 0 of the data phase (see
      // below) long before CS# can deassert. If CS# somehow closes with no data beats at all, apply
      // the wound unconditionally here rather than lose it silently.
      if (wound_pending) begin
        for (int k = 1; k <= int'(WR_WOUND_WORDS); k++)
          if (wound_base >= AW'(k))
            mem[wound_base - AW'(k)] <= sample_bus ? wound_samp[int'(WR_WOUND_WORDS) - k] : '0;
        wound_pending <= 1'b0;
      end
    end else if (cs_q) begin
      // CS# just fell: start of transaction. Decide the refresh-collision for this transaction
      // now, so the RWDS latency indicator is stable throughout the whole CA period.
      cs_q    <= 1'b0;
      ck_q    <= hb_ck;
      beat    <= '0;
      ca_sr   <= '0;
      pen     <= '0;
      pen_pending <= '0;
      wcnt    <= '0;
      stall_done <= 1'b0;
      // issue #13 R4: a fresh burst starts with no unmasked bytes seen and a cleared tail shadow (so a
      // short burst never parks a prior burst's stale tail). The orphan LIST persists across CS#.
      wr_any_unmasked <= 1'b0;
      for (int t = 0; t < 4; t++) wr_tail[t] <= '0;
      collide <= (!fixed_active) && (REFRESH_EVERY != 0) &&
                 (((txn_cnt + 1) % REFRESH_EVERY) == 0);
      txn_cnt <= txn_cnt + 1;
    end else if (hb_ck !== ck_q) begin
      // A CK edge while selected. hb_ck High => rising edge (byte A); Low => falling (byte B).
      logic [15:0] bnew;
      ck_q <= hb_ck;
      bnew = beat + 16'd1;   // 1-based index of the edge being processed

      if (bnew <= 16'd6) begin
        // -------- Command-Address phase (6 bytes, MSB first) --------
        ca_sr <= {ca_sr[CA_SR_BITS-DQ_WIDTH-1:0], hb_dq_i};
        if (bnew == 16'd6) begin
          // Full CA captured this edge: decode and set up the data phase.
          logic [15:0] eff;
          cur_rw       <= hb_ca_read(full_ca);
          cur_as       <= hb_ca_reg(full_ca);
          cur_linear   <= hb_ca_linear(full_ca);
          cur_reg_addr <= full_addr;

          // Effective initial latency (CK cycles). Register-space writes are zero-latency
          // (SPEC_DIGEST §5/§6); variable mode doubles the count on a refresh collision.
          if (hb_ca_reg(full_ca) && !hb_ca_read(full_ca))
            eff = 16'd0;
          else if (fixed_active)
            eff = FIXED_2X ? (lat_active << 1) : lat_active;   // §5.2.4 fixed-2x variant
          else
            eff = collide ? (lat_active << 1) : lat_active;
          // First data byte (A) lands on the rising edge after CA (6 edges) + eff latency cycles:
          //   edge index = 6 + 2*eff + 1 = 7 + 2*eff.
          first_data_beat <= 16'd7 + (eff << 1);

          // Write-CA wound (WR_WOUND_WORDS, finding 2): ARM a candidate wound at this burst's own CA
          // base. Resolution (apply, vs. WR_WOUND_MASK_SUPPRESS-discard) happens once beat 0 of the
          // data phase is fully sampled (see the write data-phase handling below) — NOT here — so the
          // mask-suppress decision can see beat 0's RWDS before committing to zeroing anything. Any
          // non-memory-write CA (read or register) leaves no wound armed. WR_WOUND_WRAP_IMMUNE (issue
          // #13, L-F): a WRAPPED memory write (CA[45]=0 => !hb_ca_linear) also leaves no wound armed, so
          // a wrapped write over a wound zone repairs it instead of re-wounding.
          if ((WR_WOUND_WORDS != 0) && !hb_ca_read(full_ca) && !hb_ca_reg(full_ca) &&
              !(WR_WOUND_WRAP_IMMUNE && !hb_ca_linear(full_ca))) begin
            wound_pending <= 1'b1;
            wound_base    <= start_addr;
          end else begin
            wound_pending <= 1'b0;
          end

          // issue #13 ROUND 4 (fact 4): the FIRST read CS# open (any memory/register read) FIRES every
          // parked orphan — each sprays its 4 words to [home-ROW-4, home-ROW) (exactly ONE ROW below
          // home; fact 5: that is R_{k-1}'s home) if the target is in-array (home >= ROW+4, else the
          // spray is DROPPED — silicon: a spray below word 0 is lost) — then the list is cleared. The
          // spray lands BEFORE this read's own data phase, so an RO probe of the sprayed region reads
          // the corruption. Orphans persist across intervening writes (fact 3): only a read fires them.
          if (WR_ORPHAN_MODEL && hb_ca_read(full_ca)) begin
            for (int e = 0; e < int'(ORPH_DEPTH); e++)
              if (orph_valid[e] &&
                  (32'(orph_home[e]) >= (WR_END_GARBLE_ROW_WORDS + 32'd4)))
                for (int i = 0; i < 4; i++)
                  mem[orph_home[e] - AW'(WR_END_GARBLE_ROW_WORDS) - AW'(4) + AW'(i)]
                      <= orph_word[e][i];
            for (int e = 0; e < int'(ORPH_DEPTH); e++) orph_valid[e] <= 1'b0;
            orph_cnt <= '0;
          end

          // Data-phase address / burst setup.
          addr         <= start_addr;
          wcnt         <= '0;
          wrap_words_r <= wrap_w;
          group_base   <= start_addr & ~(wrap_w - 1'b1);
          group_top    <= (start_addr & ~(wrap_w - 1'b1)) | (wrap_w - 1'b1);
        end
      end else if (bnew >= first_data_beat) begin
        // -------- Data phase --------
        if (cur_rw) begin
          // READ: the model sources data. Advance/latch on the rising edge (byte A) that starts a
          // word; deliver byte B on the falling edge; a row-crossing gap is armed at byte B and
          // activated at the *next* word slot so it never shadows the completing word's byte B.
          if (pen != 0) begin
            pen <= pen - 16'd1;                    // inside a row-crossing gap: no transfer
          end else if (pen_pending != 0) begin     // gap begins now (a would-be byte-A rising edge)
            pen         <= pen_pending - 16'd1;     // hold RWDS Low for the row-crossing penalty
            pen_pending <= 16'd0;
          end else if (hb_ck && (STALL_CLOCKS != 0) && !stall_done &&
                       (wcnt == 32'(STALL_AT))) begin
            // Inject a long RWDS-Low stall at word STALL_AT (models the >=32-clk error stall,
            // SPEC_DIGEST §4/§7). Hold RWDS Low for STALL_CLOCKS CK cycles (2 edges each) without
            // delivering a word; the master must abort. Only injected once per transaction.
            pen        <= 16'(2 * STALL_CLOCKS);
            stall_done <= 1'b1;
          end else if (hb_ck) begin                // rising edge: start of a word
            // Boundary-release quirk: stepping ONTO a BURST_BOUNDARY_WORDS-aligned address past the
            // first word crosses the boundary -> the device releases the bus for the rest of this
            // CS# (see the drive block). Sticky until CS# deassert.
            if ((BURST_BOUNDARY_WORDS != 0) && (wcnt != 32'd0) &&
                ((32'(addr) % BURST_BOUNDARY_WORDS) == 32'd0))
              bnd_rel <= 1'b1;
            // READ CAs never wound and always see the array truthfully (finding 3: a read CA at 0x100
            // mid-seeded-region left the array intact). No pending/buffer-readback machinery: the
            // 2026-07-09 ladder proved a write burst's tail commits to the array fine on its own (a
            // 512-word burst's [508..511] read back intact after 3 later writes elsewhere) — the write
            // side's wound (WR_WOUND_WORDS above) is the only defect, and reads are simply truthful.
            out_word <= cur_as ? reg_val : mem[addr];
            addr     <= next_addr(addr, wcnt);
            wcnt     <= wcnt + 32'd1;
          end else begin                           // falling edge: byte B just delivered
            if ((ROW_WORDS != 0) && !cur_as && ((addr % AW'(ROW_WORDS)) == 0))
              pen_pending <= 16'(2 * ROW_PENALTY);  // upcoming word starts a new row -> arm a gap
          end
        end else begin
          // WRITE: the master sources data; the model captures on both edges. Memory writes honor
          // the RWDS byte-mask (High = masked); register writes are unmasked full words.
          if (hb_ck) begin                         // rising edge: byte A (word[15:8])
            if (cur_as)
              wr_hi <= hb_dq_i;
            else begin
              if (!hb_rwds_i && !bnd_rel)
                mem[addr][DATA_WIDTH-1:DQ_WIDTH] <= hb_dq_i;
              // Beat-0 byte-A mask sample for WR_WOUND_MASK_SUPPRESS (paired with byte B's sample,
              // captured this same word's falling edge below, to decide the armed wound's fate).
              if (wcnt == 32'd0) wound_mask_hi <= hb_rwds_i;
              // issue #13 R4: latch the RECEIVED byte A (mask-agnostic) so the falling edge can assemble
              // the full {A,B} word into the orphan tail shadow; note any unmasked byte (a fully-masked
              // burst parks no orphan).
              wr_ba <= hb_dq_i;
              if (!hb_rwds_i) wr_any_unmasked <= 1'b1;
            end
          end else begin                           // falling edge: byte B (word[7:0])
            // Boundary-release quirk (writes): once the burst steps onto a boundary the device stops
            // capturing for the rest of this CS# -> post-boundary words are never stored.
            if ((BURST_BOUNDARY_WORDS != 0) && (wcnt != 32'd0) &&
                ((32'(addr) % BURST_BOUNDARY_WORDS) == 32'd0))
              bnd_rel <= 1'b1;
            if (cur_as) begin
              if (cur_reg_addr == HB_REG_CR0) begin
                cr0         <= {wr_hi, hb_dq_i};    // big-endian: A=high, B=low
                cr0_written <= 1'b1;
                // DPD enable/disable follows CR0[15] (= byte-A bit 7): 0 => enter DPD, 1 => normal.
                if (SUPPORT_DPD) dpd_active <= ~wr_hi[DQ_WIDTH-1];
              end else if (cur_reg_addr == HB_REG_CR1) begin
                cr1 <= {wr_hi, hb_dq_i};
              end
              // ID0/ID1 are read-only: writes ignored.
            end else begin
              if (!hb_rwds_i && !bnd_rel) begin
                mem[addr][DQ_WIDTH-1:0] <= hb_dq_i;
              end
              // issue #13 R4: push this word (received {A,B}, mask-agnostic) into the rolling tail
              // shadow — wr_tail[3]=newest — so a row-multiple close parks the last 4 as an orphan.
              if (WR_ORPHAN_MODEL) begin
                wr_tail[0] <= wr_tail[1];
                wr_tail[1] <= wr_tail[2];
                wr_tail[2] <= wr_tail[3];
                wr_tail[3] <= {wr_ba, hb_dq_i};
              end
              if (!hb_rwds_i) wr_any_unmasked <= 1'b1;
              // Wound resolve (WR_WOUND_WORDS / WR_WOUND_MASK_SUPPRESS): beat 0 (word index 0 of this
              // burst) is now fully sampled — byte A from this same word's rising edge (wound_mask_hi),
              // byte B this edge (hb_rwds_i). Apply the wound armed at CA decode unless mask-suppress
              // is enabled and beat 0 arrived fully byte-masked on both phases (the E-D hypothesis).
              if (wound_pending && (wcnt == 32'd0)) begin
                if (!(WR_WOUND_MASK_SUPPRESS && wound_mask_hi && hb_rwds_i)) begin
                  // Content: hard zero (pre-#13) unless WR_WOUND_SAMPLE_BUS, in which case the words
                  // sampled off the bus in the pre-data window are stored newest-first (mem[B-k] <=
                  // wound_samp[N-k]) — idle bus -> 0x0000 (== hard zero), controller shadow -> the heal.
                  for (int k = 1; k <= int'(WR_WOUND_WORDS); k++)
                    if (wound_base >= AW'(k))
                      mem[wound_base - AW'(k)] <= sample_bus ? wound_samp[int'(WR_WOUND_WORDS) - k]
                                                             : '0;
                end
                wound_pending <= 1'b0;
              end
            end
            addr <= next_addr(addr, wcnt);
            wcnt <= wcnt + 32'd1;
          end
        end
      end else if (sample_bus && wound_pending &&
                   (bnew >= first_data_beat - 16'(2*WR_WOUND_WORDS)) && (bnew < first_data_beat)) begin
        // -------- Initial-latency window: sample the pre-data bus (issue #13, WR_WOUND_SAMPLE_BUS) --------
        // These are the edges the ideal model ignores. On silicon the write-CA wound content is whatever
        // the device latches off DQ here; the controller's dbg_prewin_drive parks [B-4..B-1] on the bus
        // in exactly this window (oldest-first: B-4 earliest, B-1 just before data), so sampling it and
        // storing it as the wound turns the wound into a heal. Byte A on the rising (hb_ck High) phase,
        // byte B on the falling — the SAME big-endian mapping the write data phase uses, so a faithful
        // shadow drive reconstructs the exact [B-N..B-1] words. rel/j map the 2N window edges to N word
        // slots, oldest (rel 0..1 -> slot 0) to newest (rel 2N-2..2N-1 -> slot N-1).
        int rel; int j;
        rel = int'(bnew) - (int'(first_data_beat) - 2*int'(WR_WOUND_WORDS));  // 0 .. 2N-1
        j   = rel >> 1;                                                       // word slot 0 .. N-1
        if (j >= 0 && j < int'(WR_WOUND_WORDS)) begin
          if (hb_ck) wound_samp[j][15:8] <= hb_dq_i;   // byte A (rising edge)
          else       wound_samp[j][7:0]  <= hb_dq_i;   // byte B (falling edge)
        end
      end
      // else: initial-latency edges outside the sampling window — nothing captured/driven here.

      beat <= bnew;
    end
  end

  // ------------------------------------------------------------------------
  // Read over-stream tail (self-timed) — models the on-silicon CK-stop pipeline latency.
  //
  // Physical effect (AXC3000 W957D8NB): after the master stops CK to end a read burst the device
  // keeps driving a few more source-synchronous words before its read output pipeline empties, so
  // the master's CK-stop over-runs its word count. The edge-driven data phase above stops the instant
  // hb_ck stops (ideal device) and never reproduced this. Here a self-timed generator, armed once the
  // master stops clocking mid-read, drives RD_OVERSTREAM_WORDS extra {RWDS-rise, RWDS-fall} word
  // strobes at the measured CK cadence — exactly the stray words the master's read FIFOs must discard.
  //
  // os_last_edge tracks the most recent read-data CK edge (cleared to 0 when CS# is High / between
  // read bursts and after a tail is emitted, so the tail arms exactly once per read burst).
  // ------------------------------------------------------------------------
  always @(hb_ck or hb_cs_n) begin
    if (hb_cs_n) begin
      os_last_edge = 0.0;                                  // idle: disarm the tail watchdog
    end else if (cur_rw && (beat >= first_data_beat)) begin
      if (os_last_edge != 0.0) os_half = $realtime - os_last_edge;  // half-period between CK edges
      os_last_edge = $realtime;
    end
  end

  initial begin
    os_run = 1'b0; os_rwds_o = 1'b0; os_dq_o = '0;
    os_half = 10ns; os_last_edge = 0.0;
  end

  // Watchdog: when the gap since the last read-data CK edge exceeds ~1.8 half-periods the master has
  // stopped clocking mid-read; drive the over-stream tail, then release and re-arm on the next burst.
  always begin
    #1;
    if ((RD_OVERSTREAM_WORDS != 0) && !os_run && hb_rst_n && !hb_cs_n &&
        cur_rw && (beat >= first_data_beat) && (os_last_edge != 0.0) &&
        (($realtime - os_last_edge) > (os_half * 1.8))) begin
      logic [AW-1:0]         os_addr;
      logic [DATA_WIDTH-1:0] os_word;
      os_run  = 1'b1;
      os_addr = addr;                                       // continue from where the data phase left off
      for (int k = 0; (k < int'(RD_OVERSTREAM_WORDS)) && hb_rst_n; k++) begin
        os_word = mem[os_addr];
        os_addr = os_addr + 1'b1;
        os_rwds_o = 1'b1; os_dq_o = os_word[DATA_WIDTH-1:DQ_WIDTH]; #(os_half);  // byte A (RWDS rise)
        os_rwds_o = 1'b0; os_dq_o = os_word[DQ_WIDTH-1:0];          #(os_half);  // byte B (RWDS fall)
      end
      os_rwds_o    = 1'b0;
      os_last_edge = 0.0;                                   // consumed: re-arm on the next read burst
      os_run       = 1'b0;
    end
  end

  // ------------------------------------------------------------------------
  // Combinational bus drive. Ownership per interface state, SPEC_DIGEST §4 Table 7.1:
  //   CA period          : model drives RWDS = latency indicator (High => additional count).
  //   Read init latency  : model drives RWDS Low; DQ High-Z (bus turn-around).
  //   Read data          : model drives DQ + RWDS (RWDS = CK strobe, edge-aligned; Low in gaps).
  //   Write latency/data : model releases DQ and RWDS (master owns both).
  // ------------------------------------------------------------------------
  always_comb begin
    hb_dq_o   = '0;
    hb_dq_oe  = 1'b0;
    hb_rwds_o = 1'b0;
    hb_rwds_oe= 1'b0;
    // While in Deep-Power-Down the device is asleep and drives NOTHING (DQ/RWDS High-Z) — a read
    // therefore returns no source-synchronous strobe and the master's read stalls out (SPEC_DIGEST
    // §5.2.1). The master must wake the device (CS# pulse + tDPDOUT) before any access succeeds.
    if (busy && !(SUPPORT_DPD && dpd_active)) begin
      if (beat < 16'd6) begin
        // Command-Address: slave drives the RWDS latency indicator (High = 2x latency count).
        // Variable mode: High only on a refresh collision. Fixed-2x variant (§5.2.4): always High.
        hb_rwds_oe = 1'b1;
        hb_rwds_o  = collide | (fixed_active & FIXED_2X);
      end else if (cur_rw) begin
        if (beat < first_data_beat) begin
          // Read initial access latency: DQ Hi-Z (turn-around), RWDS driven by the slave.
          hb_rwds_oe = 1'b1;
          // Read PREAMBLE (SPEC-real device): for the final RD_PREAMBLE_CLOCKS CK cycles of the
          // turn-around window the device already toggles RWDS (= CK) while DQ is still Hi-Z (reads
          // 0x00). This is the AXC3000-captured W957D8NB behaviour (cap idx85-88): RWDS edges appear
          // BEFORE the first real data byte. DQ stays Hi-Z (hb_dq_oe=0 => resolves to 0 on the bus).
          if ((RD_PREAMBLE_CLOCKS != 0) &&
              (beat >= (first_data_beat - 16'(2*RD_PREAMBLE_CLOCKS))))
            hb_rwds_o = hb_ck;      // preamble strobe: RWDS toggles like CK, DQ = 0
          else
            hb_rwds_o = 1'b0;       // plain turn-around: RWDS held Low
        end else if (bnd_rel) begin
          // Boundary-release quirk: the device has let go of the bus (DQ + RWDS Hi-Z). The master
          // sees no strobe -> it reads floating junk and eventually stalls/aborts (SPEC_DIGEST §4/§7).
          hb_dq_oe   = 1'b0;
          hb_rwds_oe = 1'b0;
        end else begin
          // Read data: source-synchronous strobe + data, edge-aligned.
          hb_dq_oe   = 1'b1;
          hb_rwds_oe = 1'b1;
          if (pen != 0) begin
            // Row-crossing gap: hold RWDS Low (no strobe); data is don't-care to the master.
            hb_rwds_o = 1'b0;
            hb_dq_o   = out_word[DATA_WIDTH-1:DQ_WIDTH];
          end else begin
            hb_rwds_o = hb_ck;   // strobe transitions with CK (edge-aligned to data)
            hb_dq_o   = hb_ck ? out_word[DATA_WIDTH-1:DQ_WIDTH]   // byte A on CK High
                              : out_word[DQ_WIDTH-1:0];           // byte B on CK Low
          end
        end
      end
      // WRITE (cur_rw==0), beat>=6: model releases DQ and RWDS — the master drives them.
    end
    // Over-stream tail overrides the bus: the device keeps driving RWDS+DQ after CK stops (the
    // read output pipeline drains past the master's CK-stop). Wins over the CK-gated read drive
    // above (hb_ck is idle here) and over the idle release when CS# has already gone High.
    if (os_run) begin
      hb_dq_o    = os_dq_o;
      hb_dq_oe   = 1'b1;
      hb_rwds_o  = os_rwds_o;
      hb_rwds_oe = 1'b1;
    end
  end

  // ------------------------------------------------------------------------
  // Intentionally-unused inputs (differential clock leg + master output-enable hints). The model
  // derives all phase information from hb_ck and the transaction FSM.
  // ------------------------------------------------------------------------
  /* verilator lint_off UNUSEDSIGNAL */
  wire _unused = &{1'b0, hb_ck_n, hb_dq_ie, hb_rwds_ie};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
`endif
