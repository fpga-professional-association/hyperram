// tb_commit — regression for the W957D8NB WRITE-CA "wound" (2026-07-09 silicon ladder), the
// end-of-burst 0x2000-word garble (finding 5), the independent 0x2000-word bus-release boundary
// quirk (issue #2 legacy), and the independent read-burst-size CSR (issue #2). It drives the EXACT
// board stack — bench engine (hyperram_bw_test) -> hyperbus_avalon (front-end) -> hyperbus_ctrl
// (protocol engine) -> hyperbus_phy (SDR PHY, RD_PREAMBLE_SKIP=1) -> golden model — the model
// reproduces the device quirks and the controller mitigates them.
//
// 2026-07-09 SILICON LADDER OVERTURNED THE OLD MODEL. Proven with read-only probes (write region A;
// write elsewhere; read A back WITHOUT rewriting):
//   1. The old WR_COMMIT_QUIRK story (a write burst's tail is held pending, discarded by the next
//      write CS#, servable to reads from a buffer) is FALSE. Write-burst tails COMMIT to the array
//      fine: [508..511] of a 512-word burst read back intact after 3 later writes elsewhere.
//   2. The REAL defect: ANY memory-space WRITE CS# that opens at word address B WOUNDS the array at
//      [B-4, B) — the 4 words immediately below its CA base are zeroed. Proven standalone: a plain
//      16-word write at 0x100 zeroed [0xFC..0xFF]. B at the very start of never-written space is
//      harmless (nothing there).
//   3. READ CAs do NOT wound (a read CA at 0x100 mid-seeded-region left [0xFC..0xFF] intact).
//   4. Wall-time pauses and CK-toggling dwells after write close change nothing (not exercised here;
//      see hyperbus_ctrl's WR_CHOP_PAUSE_CYCLES/_CK).
//   5. A separate, rarer defect: a write burst whose END lands exactly ON a 0x2000-word (16KB)
//      boundary garbles its own last 4 words persistently (got=0x5050 style).
//   6. The E-D hypothesis (does a fully-RWDS-masked lead beat SUPPRESS the wound?) is modeled as a
//      knob (WR_WOUND_MASK_SUPPRESS) rather than hardwired either way, since silicon status varied
//      during the investigation; this TB exercises the controller's mask-led replay
//      (WR_CHOP_REPLAY/WR_REPLAY_WORDS/WR_REPLAY_MASK_LEAD, including the ST_IDLE contiguous-write
//      replay-accept path for command-edge boundaries) against BOTH model settings.
//
// See hyperram_model's WR_WOUND_WORDS / WR_WOUND_MASK_SUPPRESS / WR_BOUNDARY_END_GARBLE parameter
// comments for the model side, and hyperbus_ctrl's WR_CHOP_REPLAY / WR_COALESCE parameter comments
// for the controller side. WR_COMMIT_QUIRK / WR_PENDING_WORDS / WR_COMMIT_READ are retired from this
// TB (the story they modeled is falsified); the model keeps the first two as accepted-but-ignored
// no-ops so any stale instantiation elsewhere still elaborates.
//
// Eleven independent stacks (each = bench engine + front-end + ctrl + PHY + model, differing only in
// the DUT/model parameters below) let one testbench prove every direction at once:
//
//   idx  name              DUT MAXBURST  DUT COALESCE  DUT REPLAY  DUT MASK_LEAD  model WOUND  model SUPPRESS  proves
//   ---  ----------------  ------------  ------------  ----------  -------------  -----------  ---------------  -------------------------------
//    0   nofix                  64            0             0            0             4              0        no-fix chop: 4 err/chop at [C-4,C)
//    1   coalesce                0            1             0            0             4              0        single CS#, no reopens: ERR=0
//    2   replay_plain            64            0             1            0             4              0        plain rollback replay: errors MOVE to [C-8,C-4)
//    3   masklead_pass           64            0             1            4             4              1        mask-led + model-suppress: ERR=0 (heals fully)
//    4   masklead_cmdedge      512            1             1            4             4              1        SAME, but forces the coalesce-budget-at-
//                                                                                                                command-edge boundary -> exercises the NEW
//                                                                                                                ST_IDLE acc_elig replay-accept path: ERR=0
//    5   masklead_nosuppress     64            0             1            4             4              0        mask-led, model does NOT suppress: 4 err/chop
//                                                                                                                at [C-12,C-8) (the wound walks below the lead)
//    6   cmdedge_noreplay      512            1             0            0             4              0        idx4's DUT shape with replay OFF: 4 err per
//                                                                                                                command-edge boundary at [C-4,C) (proves the
//                                                                                                                acc-leg is what fixes it)
//    7   boundary_garble          0            0             0            0             0 (bnd=0)     -        WR_BOUNDARY_END_GARBLE=1, a burst ending
//                                                                                                                exactly at 0x2000: its own last 4 garbled
//    8   bndrel_off               -            -             -            -             0 (bnd=8192)  -        0x2000-word bus-release quirk, DUT chop OFF:
//                                                                                                                ERR>0 (model boundary-release engages)
//    9   bndrel_on                -            -             -            -             0 (bnd=8192)  -        SAME, DUT chop ON: ERR=0 (never crosses)
//   10   readsplit                -            -             -            -             0             -        write one correct burst, split the READ via
//                                                                                                                REG_RBURSTW: ERR=0 (isolates multi-burst READ)
//
// idx0/2/3/5 reuse a single-command LEN chosen so MAX_BURST_WORDS=64 chops land on a clean word count
// under each mechanism (verified against an independent Python re-simulation of the controller's
// seg_size/coalesce/replay arithmetic before being hard-coded here — see the per-test comments below
// for the exact chop-count derivation). idx4/6 use LEN=4096/wburst=256 with MAX_BURST_WORDS=512 (an
// exact multiple of wburst=256) so the coalescing budget exhausts EXACTLY at a command edge every
// other command — the only way to exercise the ST_IDLE acc_elig path (as opposed to the ST_RECOVER
// intra-command chop path idx0/2/3/5 exercise) without an intra-command MAX_BURST chop ever
// triggering.
//
// Every poll loop is BOUNDED, so a hang shows up as a FAIL (bounded timeout) not an infinite loop.
// Runs under: verilator --binary --timing (5.020).
`timescale 1ns/1ps

// ---------------------------------------------------------------------------------------------------
// One complete board stack: bench engine (Avalon master + CSR slave) -> hyperbus_avalon (front-end)
// -> hyperbus_ctrl (protocol engine) -> hyperbus_phy (SDR PHY) -> golden device model, with the
// shared DQ/RWDS bus resolved and a round-trip flight delay, exactly as sim/tb_multiburst.sv wires
// hyperram_avalon. This TB direct-instantiates the three hyperram_avalon sub-blocks (rather than
// going through hyperram_avalon itself, which is a pure structural wrapper around exactly these
// three, per its own header) so it can reach hyperbus_ctrl's WR_REPLAY_MASK_LEAD parameter, which
// hyperram_avalon does not yet forward. The CSR slave is hoisted to the top so the testbench can
// drive it.
// ---------------------------------------------------------------------------------------------------
module commit_stack
  import hyperbus_pkg::*;
#(
    parameter int unsigned CSR_ADDR_WIDTH = 5,   // issue #13: 32 word-regs — avoids REG_PAT/WRAP/EMAP aliasing
    // -- DUT (hyperbus_ctrl) chop / remedy knobs (all default OFF = a single unbounded CS#) --
    parameter int unsigned DUT_BOUNDARY        = 0,      // BURST_BOUNDARY_WORDS (0x2000-word chop)
    parameter int unsigned DUT_MAXBURST        = 0,      // MAX_BURST_WORDS (tCSM chop)
    parameter bit          WR_COALESCE         = 1'b0,
    parameter int unsigned WR_COALESCE_WAIT    = 8,
    parameter bit          WR_CHOP_REPLAY      = 1'b0,
    parameter int unsigned WR_REPLAY_WORDS     = 4,
    parameter int unsigned WR_REPLAY_MASK_LEAD = 0,
    // -- golden model (hyperram_model) W957D8NB quirk knobs (all default OFF = ideal device) --
    parameter int unsigned MDL_WOUND_WORDS     = 0,
    parameter bit          MDL_WOUND_SUPPRESS  = 1'b0,
    parameter bit          MDL_BOUNDARY_GARBLE = 1'b0,
    parameter int unsigned MDL_BOUNDARY        = 0,      // model BURST_BOUNDARY_WORDS (bus-release quirk)
    parameter int unsigned MDL_OS              = 0       // model RD_OVERSTREAM_WORDS
) (
    input  logic                        clk,
    input  logic                        clk90,
    input  logic                        clk_ref,
    input  logic                        rst,
    input  logic [CSR_ADDR_WIDTH-1:0]   csr_address,
    input  logic                        csr_read,
    input  logic                        csr_write,
    input  logic [31:0]                 csr_writedata,
    output logic [31:0]                 csr_readdata,
    output logic                        csr_waitrequest,
    output logic                        init_done
);
  localparam int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT;    // 8
  localparam int unsigned DATA_WIDTH = 2 * DQ_WIDTH;           // 16
  localparam int unsigned ADDR_WIDTH = HB_ADDR_WIDTH;          // 32
  localparam int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT;   // 16
  localparam int unsigned BURST_WORDS = HB_BURST_WORDS_DEFAULT;// 16
  // CR0 image: latency code 0001 (=6), fixed-latency, legacy wrap, 32B group (matches the board).
  localparam logic [15:0] TB_INIT_CR0 = {1'b1, 3'b000, 4'b1111, 4'b0001, 1'b1, 3'b111}; // 0x8F1F
  localparam logic [31:0] TB_MAGIC    = 32'h4842_5754;         // "HBWT"

  // HyperBus device pins + split-driver resolution (single active driver at a time).
  logic                 hb_ck, hb_ck_n, hb_cs_n, hb_rst_n;
  logic [DQ_WIDTH-1:0]  pin_dq_o;   logic pin_dq_oe;    // PHY's drive onto the shared device bus
  logic                 pin_rwds_o; logic pin_rwds_oe;
  logic [DQ_WIDTH-1:0]  mdl_dq_o;   logic mdl_dq_oe;    // model's drive onto the shared device bus
  logic                 mdl_rwds_o; logic mdl_rwds_oe;
  wire [DQ_WIDTH-1:0] dq_line   = mdl_dq_oe   ? mdl_dq_o   : (pin_dq_oe   ? pin_dq_o   : '0);
  wire                rwds_line = mdl_rwds_oe ? mdl_rwds_o : (pin_rwds_oe ? pin_rwds_o : 1'b0);
  localparam realtime RTT = 3.0;    // ns device->master flight delay
  wire [DQ_WIDTH-1:0] dq_line_dly;   assign #RTT dq_line_dly   = dq_line;
  wire                rwds_line_dly; assign #RTT rwds_line_dly = rwds_line;

  // Avalon-MM link: bench master <-> the front-end (hyperbus_avalon).
  logic [ADDR_WIDTH-1:0] av_address;
  logic [LEN_WIDTH-1:0]  av_burstcount;
  logic                  av_read, av_write;
  logic [DATA_WIDTH-1:0] av_writedata, av_readdata;
  logic                  av_readdatavalid, av_waitrequest;

  hyperram_bw_test #(
    .DATA_WIDTH     (DATA_WIDTH),
    .ADDR_WIDTH     (ADDR_WIDTH),
    .LEN_WIDTH      (LEN_WIDTH),
    .BURST_WORDS    (BURST_WORDS),
    .CSR_ADDR_WIDTH (CSR_ADDR_WIDTH),
    .VERSION_MAGIC  (TB_MAGIC)
  ) u_bw (
    .clk (clk), .rst (rst),
    .csr_address (csr_address), .csr_read (csr_read), .csr_readdata (csr_readdata),
    .csr_write (csr_write), .csr_writedata (csr_writedata), .csr_waitrequest (csr_waitrequest),
    .m_address (av_address), .m_burstcount (av_burstcount), .m_read (av_read), .m_write (av_write),
    .m_writedata (av_writedata), .m_readdata (av_readdata),
    .m_readdatavalid (av_readdatavalid), .m_waitrequest (av_waitrequest),
    // REG_CAL outputs unused here — the PHY's cal is tied to constants below (empty = PINCONNECTEMPTY)
    .cal_capture_phase (), .cal_preamble_skip (), .cal_rx_tap (), .cal_pair_skew (),
    // issue #13: new bench debug-bundle outputs left dangling here (this TB ties off the ctrl-side
    // dbg inputs to legacy constants independently) — empty = PINCONNECTEMPTY, not PINMISSING.
    .dbg_wr_lat_trim (), .dbg_lat_clocks (), .dbg_cr0_reprog (), .dbg_prewin_drive (),
    .dbg_prewin_n (), .dbg_prewin_marker (), .dbg_postwin_hold (), .dbg_ck_stretch_off (), .dbg_prewin_contig (), .dbg_end_cwrite (), .wrap_en ()
  );

  // -------------------------------------------------------------------------
  // Front-end: Avalon-MM slave -> native master.
  // -------------------------------------------------------------------------
  logic                    cmd_valid, cmd_ready, cmd_read, cmd_reg, cmd_wrap;
  logic [ADDR_WIDTH-1:0]   cmd_addr;
  logic [LEN_WIDTH-1:0]    cmd_len;
  logic                    wr_valid, wr_ready;
  logic [DATA_WIDTH-1:0]   wr_data;
  logic [DATA_WIDTH/8-1:0] wr_strb;
  logic                    wr_last;
  logic                    rd_valid, rd_ready;
  logic [DATA_WIDTH-1:0]   rd_data;
  logic                    rd_last;

  hyperbus_avalon #(
    .DQ_WIDTH (DQ_WIDTH), .DATA_WIDTH (DATA_WIDTH), .ADDR_WIDTH (ADDR_WIDTH), .LEN_WIDTH (LEN_WIDTH)
  ) u_avalon (
    .clk (clk), .rst (rst),
    .avs_address (av_address), .avs_read (av_read), .avs_write (av_write),
    .avs_writedata (av_writedata), .avs_byteenable (2'b11), .avs_burstcount (av_burstcount),
    .avs_readdata (av_readdata), .avs_readdatavalid (av_readdatavalid), .avs_waitrequest (av_waitrequest),
    .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_read (cmd_read), .cmd_reg (cmd_reg),
    .cmd_wrap (cmd_wrap), .cmd_addr (cmd_addr), .cmd_len (cmd_len),
    .wr_valid (wr_valid), .wr_ready (wr_ready), .wr_data (wr_data), .wr_strb (wr_strb), .wr_last (wr_last),
    .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_data (rd_data), .rd_last (rd_last),
    // issue #13: new front-end wrap_en input tied off (legacy linear bursts) (A1).
    .wrap_en (1'b0),
    .dbg_state ()
  );

  // -------------------------------------------------------------------------
  // Protocol engine: native slave <-> PHY master. Direct-instantiated (see module header) so
  // WR_REPLAY_MASK_LEAD is reachable; every other parameter mirrors the board / hyperram_avalon
  // defaults used by the rest of the suite.
  // -------------------------------------------------------------------------
  logic                    phy_cs_n, phy_rst_n, phy_ck_en;
  logic [DATA_WIDTH-1:0]   phy_dq_o;
  logic                    phy_dq_oe;
  logic [1:0]              phy_rwds_o;
  logic                    phy_rwds_oe;
  logic                    phy_rd_arm;
  logic [DATA_WIDTH-1:0]   phy_dq_i;
  logic                    phy_dq_i_valid;
  logic                    phy_rwds_i;

  hyperbus_ctrl #(
    .DQ_WIDTH             (DQ_WIDTH),
    .DATA_WIDTH           (DATA_WIDTH),
    .ADDR_WIDTH           (ADDR_WIDTH),
    .LEN_WIDTH            (LEN_WIDTH),
    .LATENCY_CLOCKS       (6),
    .FIXED_LATENCY        (1'b1),
    .MAX_BURST_WORDS      (DUT_MAXBURST),
    .BURST_BOUNDARY_WORDS (DUT_BOUNDARY),
    .WR_COALESCE          (WR_COALESCE),
    .WR_COALESCE_WAIT     (WR_COALESCE_WAIT),
    .WR_CHOP_REPLAY       (WR_CHOP_REPLAY),
    .WR_REPLAY_WORDS      (WR_REPLAY_WORDS),
    .WR_REPLAY_MASK_LEAD  (WR_REPLAY_MASK_LEAD),
    .PROGRAM_CR           (1'b1),
    .POR_DELAY_CYCLES     (0),
    .INIT_CR0             (TB_INIT_CR0)
  ) u_ctrl (
    .clk (clk), .rst (rst),
    .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_read (cmd_read), .cmd_reg (cmd_reg),
    .cmd_wrap (cmd_wrap), .cmd_addr (cmd_addr), .cmd_len (cmd_len),
    .wr_valid (wr_valid), .wr_ready (wr_ready), .wr_data (wr_data), .wr_strb (wr_strb), .wr_last (wr_last),
    .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_data (rd_data), .rd_last (rd_last),
    .busy (), .init_done (init_done), .err_underrun (), .err_timeout (),
    .phy_cs_n (phy_cs_n), .phy_rst_n (phy_rst_n), .phy_ck_en (phy_ck_en),
    .phy_dq_o (phy_dq_o), .phy_dq_oe (phy_dq_oe), .phy_rwds_o (phy_rwds_o), .phy_rwds_oe (phy_rwds_oe),
    .phy_rd_arm (phy_rd_arm),
    .phy_dq_i (phy_dq_i), .phy_dq_i_valid (phy_dq_i_valid), .phy_rwds_i (phy_rwds_i),
    .dbg_state (), .dbg_rd_wptr (), .dbg_rd_rptr (),
    // issue #13: new hyperbus_ctrl debug bundle tied to per-instance legacy (A1; no wrap_en on ctrl).
    .dbg_wr_lat_trim (4'd0), .dbg_lat_clocks (4'd6), .dbg_cr0_reprog (1'b0),
    .dbg_prewin_drive (1'b0), .dbg_prewin_n (3'd0), .dbg_prewin_marker (1'b0), .dbg_postwin_hold (1'b0), .dbg_prewin_contig (1'b0), .dbg_end_cwrite (1'b0)
  );

  // -------------------------------------------------------------------------
  // PHY: DDR SERDES + I/O (SDR variant, matches the board / RD_PREAMBLE_SKIP=1 setting).
  // -------------------------------------------------------------------------
  hyperbus_phy #(
    .DQ_WIDTH (DQ_WIDTH), .DATA_WIDTH (DATA_WIDTH), .ADDR_WIDTH (ADDR_WIDTH), .LEN_WIDTH (LEN_WIDTH),
    .PHY_VARIANT ("SDR"), .DIFF_CK (1'b0), .RD_PREAMBLE_SKIP (1)
  ) u_phy (
    .clk (clk), .clk90 (clk90), .clk_ref (clk_ref), .rst (rst),
    .cal_capture_phase (1'b0), .cal_preamble_skip (3'd1), .cal_rx_tap (5'd0), .cal_pair_skew (1'b0),
    .phy_cs_n (phy_cs_n), .phy_rst_n (phy_rst_n), .phy_ck_en (phy_ck_en),
    .phy_dq_o (phy_dq_o), .phy_dq_oe (phy_dq_oe), .phy_rwds_o (phy_rwds_o), .phy_rwds_oe (phy_rwds_oe),
    .phy_rd_arm (phy_rd_arm),
    .phy_dq_i (phy_dq_i), .phy_dq_i_valid (phy_dq_i_valid), .phy_rwds_i (phy_rwds_i),
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_o (pin_dq_o), .hb_dq_oe (pin_dq_oe), .hb_dq_i (dq_line_dly),
    .hb_rwds_o (pin_rwds_o), .hb_rwds_oe (pin_rwds_oe), .hb_rwds_i (rwds_line_dly)
  );

  hyperram_model #(
    .DQ_WIDTH               (DQ_WIDTH),
    .MEM_WORDS              (1 << 16),
    .LATENCY_CLOCKS         (6),
    .FIXED_LATENCY          (1'b1),
    .ROW_WORDS              (0),
    .REFRESH_EVERY          (0),
    .RD_PREAMBLE_CLOCKS     (1),               // matches RD_PREAMBLE_SKIP=1 on the SDR PHY
    .RD_OVERSTREAM_WORDS    (MDL_OS),
    .WR_WOUND_WORDS         (MDL_WOUND_WORDS),
    .WR_WOUND_MASK_SUPPRESS (MDL_WOUND_SUPPRESS),
    .WR_BOUNDARY_END_GARBLE (MDL_BOUNDARY_GARBLE),
    .BURST_BOUNDARY_WORDS   (MDL_BOUNDARY)
  ) model (
    .hb_ck (hb_ck), .hb_ck_n (hb_ck_n), .hb_cs_n (hb_cs_n), .hb_rst_n (hb_rst_n),
    .hb_dq_i (dq_line), .hb_dq_ie (pin_dq_oe), .hb_dq_o (mdl_dq_o), .hb_dq_oe (mdl_dq_oe),
    .hb_rwds_i (rwds_line), .hb_rwds_ie (pin_rwds_oe), .hb_rwds_o (mdl_rwds_o), .hb_rwds_oe (mdl_rwds_oe)
  );
endmodule


module tb_commit;
  import hyperbus_pkg::*;

  localparam int unsigned CSR_ADDR_WIDTH = 5;                  // issue #13: 32 word-regs (REG_RBURSTW = word 12) — avoids REG_PAT/WRAP/EMAP aliasing

  // CSR word-register indices (byte offset >> 2) — must match hyperram_bw_test.
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_CTRL    = 4'd0;    // W: CTRL / R: STATUS
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_LEN     = 4'd1;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_BASE    = 4'd2;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRCNT  = 4'd5;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRADDR = 4'd8;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERRGOT  = 4'd9;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_ERREXP  = 4'd10;
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_BURSTW  = 4'd11;   // WRITE-phase burst length
  localparam logic [CSR_ADDR_WIDTH-1:0] REG_RBURSTW = 4'd12;   // READ-phase  burst length

  // --------------------------------------------------------------------
  // Clocking / reset — SDR arrangement (as tb_sdr / tb_multiburst / the board).
  // --------------------------------------------------------------------
  logic clk, clk90, clk_ref, rst;
  initial begin clk     = 1'b0; forever #10.0 clk     = ~clk;     end   // 50 MHz
  initial begin clk90   = 1'b0; forever #5.0  clk90   = ~clk90;   end   // 100 MHz
  initial begin clk_ref = 1'b0; forever #5.0  clk_ref = ~clk_ref; end

  // --------------------------------------------------------------------
  // The stacks (see the header table for the full idx -> parameter -> expectation map).
  // --------------------------------------------------------------------
  localparam int NSTACK = 11;

  logic [CSR_ADDR_WIDTH-1:0] s_addr  [NSTACK];
  logic                      s_read  [NSTACK];
  logic                      s_write [NSTACK];
  logic [31:0]               s_wdata [NSTACK];
  logic [31:0]               s_rdata [NSTACK];
  logic                      s_wait  [NSTACK];
  logic                      s_initd [NSTACK];

  // idx0 — nofix: MAX_BURST_WORDS=64 chop, no coalesce, no replay, model wound=4. Every intra-command
  // chop reopens at the natural boundary C with no rollback -> wounds [C-4,C).
  commit_stack #(.CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .DUT_MAXBURST(64), .MDL_WOUND_WORDS(4)) u_nofix (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[0]), .csr_read(s_read[0]), .csr_write(s_write[0]),
    .csr_writedata(s_wdata[0]), .csr_readdata(s_rdata[0]),
    .csr_waitrequest(s_wait[0]), .init_done(s_initd[0])
  );

  // idx1 — coalesce: WR_COALESCE=1, no hardware cap at all (DUT_MAXBURST=0) -> the whole transfer
  // stays on ONE CS# regardless of how many native commands bw splits it into -> only the very first
  // (and only) CA-decode wounds anything, and that wound zone sits BELOW base, outside the verified
  // range -> ERR=0 for every LEN/wburst pair in the sweep.
  commit_stack #(.CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .WR_COALESCE(1'b1), .MDL_WOUND_WORDS(4)) u_coalesce (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[1]), .csr_read(s_read[1]), .csr_write(s_write[1]),
    .csr_writedata(s_wdata[1]), .csr_readdata(s_rdata[1]),
    .csr_waitrequest(s_wait[1]), .init_done(s_initd[1])
  );

  // idx2 — replay_plain: WR_CHOP_REPLAY=1, WR_REPLAY_MASK_LEAD=0 (plain rollback, no masked lead
  // beats). Every chop reopens rb=4 words EARLY (at C-4) and re-sends them (healing [C-4,C)), but the
  // reopened CS#'s OWN CA base (C-4) now takes the wound one step further back: [C-8,C-4).
  commit_stack #(
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .DUT_MAXBURST(64),
    .WR_CHOP_REPLAY(1'b1), .WR_REPLAY_WORDS(4), .WR_REPLAY_MASK_LEAD(0),
    .MDL_WOUND_WORDS(4)
  ) u_replay_plain (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[2]), .csr_read(s_read[2]), .csr_write(s_write[2]),
    .csr_writedata(s_wdata[2]), .csr_readdata(s_rdata[2]),
    .csr_waitrequest(s_wait[2]), .init_done(s_initd[2])
  );

  // idx3 — masklead_pass: WR_REPLAY_MASK_LEAD=4 (E-D mask-led replay: 4 masked dummy beats ahead of
  // the 4 real replayed words) against a model with WR_WOUND_MASK_SUPPRESS=1 (the E-D hypothesis
  // holds): every reopen's beat 0 is a masked dummy -> its wound is discarded -> ERR=0.
  commit_stack #(
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .DUT_MAXBURST(64),
    .WR_CHOP_REPLAY(1'b1), .WR_REPLAY_WORDS(4), .WR_REPLAY_MASK_LEAD(4),
    .MDL_WOUND_WORDS(4), .MDL_WOUND_SUPPRESS(1'b1)
  ) u_masklead_pass (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[3]), .csr_read(s_read[3]), .csr_write(s_write[3]),
    .csr_writedata(s_wdata[3]), .csr_readdata(s_rdata[3]),
    .csr_waitrequest(s_wait[3]), .init_done(s_initd[3])
  );

  // idx4 — masklead_cmdedge: SAME mask-led-replay + model-suppress composition as idx3, but shaped
  // (WR_COALESCE=1, MAX_BURST_WORDS=512 = exactly 2x wburst=256) so the coalescing hw-cap budget
  // exhausts EXACTLY at a command edge every other command -- no single command is ever long enough
  // to trigger an INTRA-command MAX_BURST chop (each is only 256 words), so every wound here is
  // healed exclusively through the NEW ST_IDLE acc_elig command-edge replay-accept path. ERR=0 proves
  // that path (not just the already-covered ST_RECOVER intra-command path) heals correctly.
  commit_stack #(
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .DUT_MAXBURST(512),
    .WR_COALESCE(1'b1), .WR_COALESCE_WAIT(8),
    .WR_CHOP_REPLAY(1'b1), .WR_REPLAY_WORDS(4), .WR_REPLAY_MASK_LEAD(4),
    .MDL_WOUND_WORDS(4), .MDL_WOUND_SUPPRESS(1'b1)
  ) u_masklead_cmdedge (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[4]), .csr_read(s_read[4]), .csr_write(s_write[4]),
    .csr_writedata(s_wdata[4]), .csr_readdata(s_rdata[4]),
    .csr_waitrequest(s_wait[4]), .init_done(s_initd[4])
  );

  // idx5 — masklead_nosuppress: same DUT shape as idx3 (mask-led replay) but the model does NOT
  // honor mask-suppress (WR_WOUND_MASK_SUPPRESS=0, the "E-D hypothesis is false" case): every reopen
  // still wounds [B-4,B) where B is now C-8 (rb=4 + lead=4 rolled back) -> wound zone [C-12,C-8),
  // BELOW the masked-lead prefix entirely (untouched by either the lead beats or the replayed reals).
  commit_stack #(
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .DUT_MAXBURST(64),
    .WR_CHOP_REPLAY(1'b1), .WR_REPLAY_WORDS(4), .WR_REPLAY_MASK_LEAD(4),
    .MDL_WOUND_WORDS(4), .MDL_WOUND_SUPPRESS(1'b0)
  ) u_masklead_nosuppress (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[5]), .csr_read(s_read[5]), .csr_write(s_write[5]),
    .csr_writedata(s_wdata[5]), .csr_readdata(s_rdata[5]),
    .csr_waitrequest(s_wait[5]), .init_done(s_initd[5])
  );

  // idx6 — cmdedge_noreplay: EXACT same DUT shape as idx4 (WR_COALESCE=1, MAX_BURST_WORDS=512,
  // wburst=256) but WR_CHOP_REPLAY=0 -> acc_elig can never trigger (it requires WR_CHOP_REPLAY=1) ->
  // every command-edge boundary opens fresh with no rollback -> wounds [C-4,C) there too. Direct
  // before/after pair with idx4: same setup, replay toggled off, proving the acc-leg (not something
  // else about this DUT shape) is what fixes it.
  commit_stack #(
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .DUT_MAXBURST(512),
    .WR_COALESCE(1'b1), .WR_COALESCE_WAIT(8),
    .MDL_WOUND_WORDS(4)
  ) u_cmdedge_noreplay (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[6]), .csr_read(s_read[6]), .csr_write(s_write[6]),
    .csr_writedata(s_wdata[6]), .csr_readdata(s_rdata[6]),
    .csr_waitrequest(s_wait[6]), .init_done(s_initd[6])
  );

  // idx7 — boundary_garble: finding 5. A single 16-word write at 0x1FF0 ends EXACTLY at 0x2000 (no
  // DUT chop needed to produce this — base+len already lands there); model WR_BOUNDARY_END_GARBLE=1
  // garbles the burst's own last 4 words ([0x1FFC..0x1FFF]) to 0x5050 at CS# close. Model wound is
  // OFF here (isolated single-quirk test, matching the isolation convention used throughout).
  commit_stack #(
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .MDL_BOUNDARY_GARBLE(1'b1)
  ) u_boundary_garble (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[7]), .csr_read(s_read[7]), .csr_write(s_write[7]),
    .csr_writedata(s_wdata[7]), .csr_readdata(s_rdata[7]),
    .csr_waitrequest(s_wait[7]), .init_done(s_initd[7])
  );

  // idx8/9 — bndrel_off/on: independent, orthogonal 0x2000-word bus-RELEASE boundary quirk (not the
  // wound): a single burst crossing 0x2000 with the DUT's own BURST_BOUNDARY_WORDS chop OFF lets the
  // model's bnd_rel engage (bus floats past the boundary) -> ERR>0; chop ON never crosses -> ERR=0.
  commit_stack #(
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .DUT_BOUNDARY(0), .MDL_BOUNDARY(32'h2000)
  ) u_bndrel_off (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[8]), .csr_read(s_read[8]), .csr_write(s_write[8]),
    .csr_writedata(s_wdata[8]), .csr_readdata(s_rdata[8]),
    .csr_waitrequest(s_wait[8]), .init_done(s_initd[8])
  );

  commit_stack #(
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .DUT_BOUNDARY(32'h2000), .MDL_BOUNDARY(32'h2000)
  ) u_bndrel_on (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[9]), .csr_read(s_read[9]), .csr_write(s_write[9]),
    .csr_writedata(s_wdata[9]), .csr_readdata(s_rdata[9]),
    .csr_waitrequest(s_wait[9]), .init_done(s_initd[9])
  );

  // idx10 — readsplit: write ONE correct burst, split the READ into many via REG_RBURSTW, against a
  // clean model that over-streams like silicon -> ERR=0 (isolates the multi-burst READ path from any
  // write-side quirk; both DUT and model quirks are OFF here).
  commit_stack #(
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH), .MDL_OS(9)
  ) u_readsplit (
    .clk(clk), .clk90(clk90), .clk_ref(clk_ref), .rst(rst),
    .csr_address(s_addr[10]), .csr_read(s_read[10]), .csr_write(s_write[10]),
    .csr_writedata(s_wdata[10]), .csr_readdata(s_rdata[10]),
    .csr_waitrequest(s_wait[10]), .init_done(s_initd[10])
  );

  // --------------------------------------------------------------------
  // Scoreboard
  // --------------------------------------------------------------------
  int unsigned errors = 0;
  task automatic check(input logic cond, input string msg);
    if (!cond) begin
      $display("[%0t] ERROR: %s", $time, msg);
      errors = errors + 1;
    end
  endtask

  // --------------------------------------------------------------------
  // Per-stack CSR access (drive on the falling edge; waitrequest tied low).
  // --------------------------------------------------------------------
  task automatic csr_wr(input int idx, input logic [CSR_ADDR_WIDTH-1:0] addr, input logic [31:0] data);
    @(negedge clk);
    s_addr[idx]  = addr;
    s_wdata[idx] = data;
    s_write[idx] = 1'b1;
    s_read[idx]  = 1'b0;
    @(negedge clk);
    s_write[idx] = 1'b0;
    s_wdata[idx] = '0;
    s_addr[idx]  = '0;
  endtask

  // Hold the (combinational) read address stable for a FULL clock and sample at the next negedge, so
  // s_rdata is guaranteed settled. (A within-cycle `#1` sample races the model's 1 ns over-stream
  // watchdog across the array-connected readdata and returns stale data.)
  task automatic csr_rd(input int idx, input logic [CSR_ADDR_WIDTH-1:0] addr, output logic [31:0] data);
    @(negedge clk);
    s_addr[idx] = addr;
    s_read[idx] = 1'b1;
    s_write[idx]= 1'b0;
    @(negedge clk);
    data        = s_rdata[idx];
    s_read[idx] = 1'b0;
    s_addr[idx] = '0;
  endtask

  // Program one stack and run a single write+read pass; return completion + counters (BOUNDED poll).
  task automatic run_one(input int idx,
                         input logic [31:0] len, input logic [31:0] base,
                         input logic [31:0] wburst, input logic [31:0] rburst,
                         output logic done, output logic [31:0] status, output logic [31:0] err);
    int unsigned guard;
    csr_wr(idx, REG_LEN,     len);
    csr_wr(idx, REG_BASE,    base);
    csr_wr(idx, REG_BURSTW,  wburst);
    csr_wr(idx, REG_RBURSTW, rburst);
    csr_wr(idx, REG_CTRL,    32'h0000_0001);   // pulse start
    guard = 0;
    do begin
      csr_rd(idx, REG_CTRL, status);
      guard = guard + 1;
    end while (!status[1] && guard < 400000);
    done = status[1];
    csr_rd(idx, REG_ERRCNT, err);
    if (err != 0) begin
      logic [31:0] ea, eg, ee;
      csr_rd(idx, REG_ERRADDR, ea);
      csr_rd(idx, REG_ERRGOT,  eg);
      csr_rd(idx, REG_ERREXP,  ee);
      $display("        first-err: addr=0x%08x got=0x%04x exp=0x%04x", ea, eg[15:0], ee[15:0]);
    end
  endtask

  // --------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------
  logic [31:0] status, err;
  logic        done;
  int unsigned guard;

  initial begin
    for (int k = 0; k < NSTACK; k++) begin
      s_addr[k] = '0; s_read[k] = 1'b0; s_write[k] = 1'b0; s_wdata[k] = '0;
    end
    rst = 1'b1;
    repeat (5) @(posedge clk);
    @(negedge clk);
    rst = 1'b0;

    // Wait for every stack's POR + CR0 programming.
    guard = 0;
    while (guard < 100000) begin
      logic all_done;
      all_done = 1'b1;
      for (int k = 0; k < NSTACK; k++) all_done &= s_initd[k];
      if (all_done) break;
      @(posedge clk); guard = guard + 1;
    end
    for (int k = 0; k < NSTACK; k++) check(s_initd[k], $sformatf("stack %0d init_done never asserted", k));
    repeat (4) @(posedge clk);

    $display("==================================================================");
    $display("tb_commit: W957D8NB write-CA wound + 0x2000 boundary quirks");
    $display("==================================================================");

    // ---- (A) idx0: no-fix chop, MAX_BURST_WORDS=64 ----
    // LEN=724 = 64 + 11*60 chops into EXACTLY 11 segments (11 chop boundaries) under the no-replay
    // arithmetic (64-word segments, ceil(724/64)-1=11) -> 11*4=44 wounded words at [C-4,C) per chop.
    $display("-- (A) idx0 nofix: LEN=724 base=0x100 MAXBURST=64 (expect ERR=44, 11 chops x 4) --");
    run_one(0, 32'd724, 32'h0000_0100, 32'd724, 32'd724, done, status, err);
    $display("   [nofix] done=%0b ERR=%0d (expect 44)", done, err);
    check(done, $sformatf("nofix did not complete (STATUS=0x%08x)", status));
    check(err == 32'd44, $sformatf("nofix ERR=%0d expected 44 (silicon wound signature, 11 chops x 4)", err));

    // ---- (B) idx1: WR_COALESCE alone, no hardware cap -> single CS#, no reopens ----
    $display("-- (B) idx1 coalesce: no hw cap, WR_COALESCE=1 (expect ERR=0, no CS# reopens) --");
    begin
      localparam int NSW = 3;
      int sw_len   [NSW] = '{32, 64, 256};
      int sw_burst [NSW] = '{16, 16, 64};
      for (int i = 0; i < NSW; i++) begin
        run_one(1, sw_len[i][31:0], 32'h0000_0100, sw_burst[i][31:0], sw_burst[i][31:0], done, status, err);
        $display("   [coalesce] LEN=%0d wburst=%0d done=%0b ERR=%0d (expect 0)", sw_len[i], sw_burst[i], done, err);
        check(done, $sformatf("coalesce LEN=%0d did not complete (STATUS=0x%08x)", sw_len[i], status));
        check(err == 32'd0, $sformatf("coalesce LEN=%0d ERR=%0d expected 0 (single-CS# by construction)", sw_len[i], err));
      end
    end

    // ---- (C) idx2: plain rollback replay (WR_CHOP_REPLAY=1, MASK_LEAD=0) ----
    // SAME LEN=724/MAXBURST=64 shape as idx0 (independently re-verified: the replay overhead (rb=4
    // words/chop) shrinks per-chop forward progress from 64 to 60 real words, and 724 = 64 + 11*60
    // divides EXACTLY under that arithmetic too -> still 11 chops) -> the errors MOVE from [C-4,C) to
    // [C-8,C-4) but the total count (44) matches idx0 exactly.
    $display("-- (C) idx2 replay_plain: LEN=724 MAXBURST=64, WR_CHOP_REPLAY=1 MASK_LEAD=0 (expect ERR=44 at [C-8,C-4)) --");
    run_one(2, 32'd724, 32'h0000_0100, 32'd724, 32'd724, done, status, err);
    $display("   [replay_plain] done=%0b ERR=%0d (expect 44)", done, err);
    check(done, $sformatf("replay_plain did not complete (STATUS=0x%08x)", status));
    check(err == 32'd44, $sformatf("replay_plain ERR=%0d expected 44 (wound moved to [C-8,C-4), same count)", err));

    // ---- (D) idx3: mask-led replay + model mask-suppress=1 ----
    $display("-- (D) idx3 masklead_pass: MASK_LEAD=4, model WR_WOUND_MASK_SUPPRESS=1 (expect ERR=0) --");
    run_one(3, 32'd768, 32'h0000_0100, 32'd768, 32'd768, done, status, err);
    $display("   [masklead_pass] LEN=768/768 done=%0b ERR=%0d (expect 0)", done, err);
    check(done, "masklead_pass LEN=768 did not complete");
    check(err == 32'd0, $sformatf("masklead_pass LEN=768 ERR=%0d expected 0 (mask-led reopen heals its own wound)", err));

    run_one(3, 32'd520, 32'h0000_0100, 32'd520, 32'd520, done, status, err);
    $display("   [masklead_pass] LEN=520/520 done=%0b ERR=%0d (expect 0)", done, err);
    check(done, "masklead_pass LEN=520 did not complete");
    check(err == 32'd0, $sformatf("masklead_pass LEN=520 ERR=%0d expected 0", err));

    // ---- (E) idx4: mask-led replay + model mask-suppress=1, forced to the ST_IDLE acc_elig
    // command-edge path (WR_COALESCE=1, MAX_BURST_WORDS=512=2*wburst -> the budget always exhausts
    // exactly at a command boundary, never mid-command) ----
    $display("-- (E) idx4 masklead_cmdedge: LEN=4096/wburst=256 MAXBURST=512 COALESCE=1 (expect ERR=0, exercises ST_IDLE acc_elig) --");
    run_one(4, 32'd4096, 32'h0000_0100, 32'd256, 32'd256, done, status, err);
    $display("   [masklead_cmdedge] done=%0b ERR=%0d (expect 0)", done, err);
    check(done, "masklead_cmdedge did not complete");
    check(err == 32'd0, $sformatf("masklead_cmdedge ERR=%0d expected 0 (acc_elig heals every command-edge boundary)", err));

    // ---- (F) idx5: mask-led replay, model mask-suppress=0 (E-D hypothesis false) ----
    // LEN=680 = 64 + 11*56 divides EXACTLY under the mask-led (rb=4,lead=4 -> pfx=8 -> 56 real/chop)
    // arithmetic -> 11 chops x 4 = 44, now at [C-12,C-8) (below the masked-lead prefix entirely).
    $display("-- (F) idx5 masklead_nosuppress: LEN=680 MAXBURST=64, model WR_WOUND_MASK_SUPPRESS=0 (expect ERR=44 at [C-12,C-8)) --");
    run_one(5, 32'd680, 32'h0000_0100, 32'd680, 32'd680, done, status, err);
    $display("   [masklead_nosuppress] done=%0b ERR=%0d (expect 44)", done, err);
    check(done, $sformatf("masklead_nosuppress did not complete (STATUS=0x%08x)", status));
    check(err == 32'd44, $sformatf("masklead_nosuppress ERR=%0d expected 44 (wound walks below the mask lead)", err));

    // ---- (G) idx6: command-edge case, replay OFF (proves the acc-leg is what fixes it) ----
    // Identical DUT shape to idx4 (WR_COALESCE=1, MAX_BURST_WORDS=512, wburst=256) with WR_CHOP_REPLAY
    // off -> acc_elig never triggers -> every one of the 7 command-edge boundaries (4096/512=8 CS#
    // opens -> 7 boundaries) opens fresh with no rollback -> 7*4=28 wounded words at [C-4,C).
    $display("-- (G) idx6 cmdedge_noreplay: SAME shape as idx4, WR_CHOP_REPLAY=0 (expect ERR=28, 7 boundaries x 4) --");
    run_one(6, 32'd4096, 32'h0000_0100, 32'd256, 32'd256, done, status, err);
    $display("   [cmdedge_noreplay] done=%0b ERR=%0d (expect 28)", done, err);
    check(done, "cmdedge_noreplay did not complete");
    check(err == 32'd28, $sformatf("cmdedge_noreplay ERR=%0d expected 28 (proves the acc-leg, not something else, fixes idx4)", err));

    // ---- (H) idx7: WR_BOUNDARY_END_GARBLE (silicon ladder finding 5) ----
    // A single 16-word write at 0x1FF0 ends EXACTLY at 0x2000 -> the model garbles its own last 4
    // words ([0x1FFC..0x1FFF]) to 0x5050 at CS# close.
    $display("-- (H) idx7 boundary_garble: LEN=16 base=0x1FF0, model WR_BOUNDARY_END_GARBLE=1 (expect ERR=4) --");
    run_one(7, 32'd16, 32'h0000_1FF0, 32'd16, 32'd16, done, status, err);
    $display("   [boundary_garble] done=%0b ERR=%0d (expect 4)", done, err);
    check(done, "boundary_garble did not complete");
    check(err == 32'd4, $sformatf("boundary_garble ERR=%0d expected 4 (burst's own last 4 words garbled to 0x5050)", err));

    // ---- (I) idx8/9: 0x2000-word bus-release boundary quirk (orthogonal to the wound) ----
    $display("-- (I) idx8/9 0x2000-word boundary (base=0x1FF8, LEN=16, burst=16 -> crosses) --");
    run_one(8, 32'd16, 32'h0000_1FF8, 32'd16, 32'd16, done, status, err);   // DUT chop OFF
    $display("   [bndrel_off] done=%0b ERR=%0d STATUS=0x%08x (expect ERR>0)", done, err, status);
    check(done, "bndrel_off did not complete (hang)");
    check(err != 32'd0, "bndrel_off ERR=0 — model boundary-release not modelled");

    run_one(9, 32'd16, 32'h0000_1FF8, 32'd16, 32'd16, done, status, err);   // DUT chop ON
    $display("   [bndrel_on ] done=%0b ERR=%0d STATUS=0x%08x (expect ERR=0)", done, err, status);
    check(done, "bndrel_on did not complete");
    check(err == 32'd0, $sformatf("bndrel_on ERR=%0d expected 0", err));

    // ---- (J) idx10: independent read-burst-size CSR (issue #2) ----
    $display("-- (J) idx10 readsplit: write single burst, split read (base=0x100, LEN=64, wburst=64, rburst=16) --");
    run_one(10, 32'd64, 32'h0000_0100, 32'd64, 32'd16, done, status, err);
    $display("   [readsplit] done=%0b ERR=%0d STATUS=0x%08x (expect ERR=0)", done, err, status);
    check(done, "readsplit did not complete (multi-burst read HANG)");
    check(err == 32'd0, $sformatf("readsplit ERR=%0d expected 0 (multi-burst READ path)", err));

    $display("==================================================================");
    $display("[%0t] tb_commit done: %0d errors", $time, errors);
    if (errors == 0) begin
      $display("TB_RESULT: PASS");
      $finish;
    end else begin
      $display("TB_RESULT: FAIL");
      $fatal(1, "tb_commit: %0d errors", errors);
    end
  end

  // Global watchdog — a true infinite hang (should never happen; every poll is bounded).
  initial begin
    #60_000_000;
    $display("[%0t] TB_RESULT: FAIL (global timeout)", $time);
    $fatal(1, "tb_commit: global timeout");
  end

endmodule
