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
                                               FIXED_LATENCY, 3'b111}
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
    input  logic                    phy_rwds_i       // synchronized RWDS level
);

  // ------------------------------------------------------------------------
  // Derived widths / internal constants
  // ------------------------------------------------------------------------
  localparam int unsigned STRB_WIDTH       = DATA_WIDTH / 8;
  localparam int unsigned RESET_CYCLES     = 8;                    // RST# low pulse (>= tRP, SPEC §9)
  localparam int unsigned RECOVERY_CYCLES  = 4;                    // tCSHI + tRWR gap (SPEC §6)
  localparam int unsigned TAIL_CYCLES      = 2;                    // extra CS# Low after last word (tCSH note)
  localparam int unsigned READ_STALL_LIMIT = 32;                   // RWDS Low >= 32 clk => timeout (SPEC §4/§7)
  localparam int unsigned RD_FIFO_DEPTH    = 32;                   // read holding buffer (rd_ready slack).
                                                                    // Must exceed the largest single read
                                                                    // segment (Avalon burstcount) + the SDR
                                                                    // PHY read-pipeline depth so the final,
                                                                    // rd_last-tagged word is never dropped on
                                                                    // rd_fifo_full: on the AXC3000 SDR path a
                                                                    // depth of 8 capped completing reads at ~5
                                                                    // words (bursts >=6 hung). 32 covers the
                                                                    // board's 16-word bursts with slack.
  localparam int unsigned RD_AW            = $clog2(RD_FIFO_DEPTH);

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
    ST_WRITE,   // write-data phase
    ST_TAIL,    // CS# held Low after last word (tCSH note)
    ST_RECOVER  // CS# High, tCSHI/tRWR recovery
  } state_e;

  state_e                  state;
  logic [31:0]             cnt;          // general dwell counter (reset/POR/latency/tail/recovery)
  logic [5:0]              stall_cnt;    // read RWDS-Low stall counter

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
  logic                    doing_init;   // current transaction is the internal CR0 write

  // read holding FIFO ({last, data})
  logic [DATA_WIDTH:0]     rd_fifo [RD_FIFO_DEPTH];
  logic [RD_AW:0]          rd_wptr, rd_rptr;

  // ------------------------------------------------------------------------
  // Combinational helpers
  // ------------------------------------------------------------------------
  wire                     zlw = ~cur_read & cur_reg;             // zero-latency (register) write

  // write-data source: internal CR0 image during init, else the native write channel
  wire [DATA_WIDTH-1:0]    wsrc_data  = doing_init ? INIT_CR0 : wr_data;
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

  wire                     rd_fifo_empty = (rd_wptr == rd_rptr);
  wire                     rd_fifo_full  = (rd_wptr[RD_AW-1:0] == rd_rptr[RD_AW-1:0]) &
                                           (rd_wptr[RD_AW]     != rd_rptr[RD_AW]);
  wire [RD_AW-1:0]         rd_wa = rd_wptr[RD_AW-1:0];
  wire [RD_AW-1:0]         rd_ra = rd_rptr[RD_AW-1:0];

  // 16-bit CA word selected by index (MSB word first, SPEC_DIGEST §2)
  function automatic logic [DATA_WIDTH-1:0] ca_slice(input logic [1:0] idx);
    unique case (idx)
      2'd0:    return ca_reg[47:32];
      2'd1:    return ca_reg[31:16];
      default: return ca_reg[15:0];
    endcase
  endfunction

  // segment size for a (remaining) burst: chop linear bursts to MAX_BURST_WORDS (tCSM), never wrapped
  function automatic logic [LEN_WIDTH-1:0] seg_size(input logic [LEN_WIDTH-1:0] total,
                                                    input logic                 wrapped);
    if ((MAX_BURST_WORDS != 0) && !wrapped && (total > LEN_WIDTH'(MAX_BURST_WORDS)))
      return LEN_WIDTH'(MAX_BURST_WORDS);
    else
      return total;
  endfunction

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

      ST_IDLE:  cmd_ready = init_done & rd_fifo_empty;

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
        end
        busy = ~doing_init;
      end

      ST_READ: begin
        phy_cs_n   = 1'b0;
        phy_ck_en  = 1'b1;
        phy_rd_arm = 1'b1;
        busy       = ~doing_init;
      end

      ST_WRITE: begin
        phy_cs_n  = 1'b0;
        phy_ck_en = 1'b1;
        phy_dq_oe = 1'b1;
        phy_dq_o  = wsrc_data;
        if (!zlw) begin
          phy_rwds_oe = 1'b1;            // RWDS = byte mask (High = masked); underrun => mask both
          phy_rwds_o  = wsrc_valid ? {~wsrc_strb[1], ~wsrc_strb[0]} : 2'b11;
        end
        if (wsrc_valid & ~doing_init) wr_ready = 1'b1;
        busy = ~doing_init;
      end

      ST_TAIL:     busy = ~doing_init;
      ST_RECOVER:  busy = ~doing_init;
      ST_RD_ABORT: busy = ~doing_init;   // CS# High (idle bus); draining flagged filler to the FIFO

      default: ; // ST_POR, ST_INIT: idle bus
    endcase

    if (state == ST_TAIL) phy_cs_n = 1'b0;   // hold CS# Low through the tail
  end

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
      init_done    <= 1'b0;
      doing_init   <= 1'b0;
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
          if (cnt == 32'd0) begin
            state <= ST_POR;
            cnt   <= 32'(POR_DELAY_CYCLES);
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        ST_POR: begin
          if (cnt == 32'd0) state <= ST_INIT;
          else              cnt   <= cnt - 1'b1;
        end

        ST_INIT: begin
          if (PROGRAM_CR) begin
            // internal register-space (zero-latency) write of CR0
            doing_init <= 1'b1;
            cur_read   <= 1'b0;
            cur_reg    <= 1'b1;
            cur_wrap   <= 1'b0;
            cur_addr   <= HB_REG_CR0[ADDR_WIDTH-1:0];
            rem_left   <= LEN_WIDTH'(1);
            seg_left   <= LEN_WIDTH'(1);
            seg_count  <= LEN_WIDTH'(1);
            ca_reg     <= hb_pack_ca(1'b0, 1'b1, 1'b1, HB_REG_CR0[ADDR_WIDTH-1:0]);
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
          if (cmd_valid & init_done & rd_fifo_empty) begin
            cur_read  <= cmd_read;
            cur_reg   <= cmd_reg;
            cur_wrap  <= cmd_wrap;
            cur_addr  <= cmd_addr;
            rem_left  <= cmd_len;
            seg_count <= seg_size(cmd_len, cmd_wrap);
            seg_left  <= seg_size(cmd_len, cmd_wrap);
            ca_reg    <= hb_pack_ca(cmd_read, cmd_reg, ~cmd_wrap, cmd_addr);
            rwds_hi   <= 1'b0;
            ca_idx    <= 2'd0;
            state     <= ST_CS;
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
              cnt            <= 32'(LATENCY_CLOCKS - 1);  // one latency count; doubled in ST_LAT if RWDS-high
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
              cnt            <= 32'(LATENCY_CLOCKS - 1);   // insert the additional (2x) latency count
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
            if (!rd_fifo_full) begin
              rd_fifo[rd_wa] <= {(rem_left == LEN_WIDTH'(1)), phy_dq_i[DATA_WIDTH-1:0]};
              rd_wptr        <= rd_wptr + 1'b1;
            end
            stall_cnt <= '0;
            seg_left  <= seg_left - 1'b1;
            rem_left  <= rem_left - 1'b1;
            if (seg_left == LEN_WIDTH'(1)) begin
              if (rem_left != LEN_WIDTH'(1))       // chopped: advance to next linear segment
                cur_addr <= cur_addr + ADDR_WIDTH'(seg_count);
              cnt   <= 32'(TAIL_CYCLES - 1);
              state <= ST_TAIL;
            end
          end else if (!phy_rwds_i) begin
            if (stall_cnt == 6'(READ_STALL_LIMIT - 1)) begin
              // RWDS stalled >= 32 clk (SPEC_DIGEST §4/§7): abort the read. Raise CS# (ST_RD_ABORT is
              // a bus-idle state) and, so the AXI/Avalon front-end's R channel still terminates with
              // exactly cmd_len beats + a final rd_last (AXI A3.4.1 / Avalon burst completion), drain
              // the remaining rem_left words as flagged filler rather than silently dropping rd_last.
              err_timeout <= 1'b1;
              state       <= ST_RD_ABORT;
            end else begin
              stall_cnt <= stall_cnt + 1'b1;
            end
          end
        end

        // ---------------- read abort drain (post-timeout) ----------------
        ST_RD_ABORT: begin
          if (rem_left == LEN_WIDTH'(0)) begin
            cnt   <= 32'(RECOVERY_CYCLES - 1);
            state <= ST_RECOVER;
          end else if (!rd_fifo_full) begin
            rd_fifo[rd_wa] <= {(rem_left == LEN_WIDTH'(1)), {DATA_WIDTH{1'b0}}};
            rd_wptr        <= rd_wptr + 1'b1;
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
          if (!wsrc_valid) err_underrun <= 1'b1;   // host underrun: word gets masked, burst continues
          seg_left <= seg_left - 1'b1;
          rem_left <= rem_left - 1'b1;
          if (seg_left == LEN_WIDTH'(1)) begin
            if (rem_left != LEN_WIDTH'(1))          // chopped: advance to next linear segment
              cur_addr <= cur_addr + ADDR_WIDTH'(seg_count);
            cnt   <= 32'(TAIL_CYCLES - 1);
            state <= ST_TAIL;
          end
        end

        // ---------------- tail / recovery ----------------
        ST_TAIL: begin
          if (cnt == 32'd0) begin
            cnt   <= 32'(RECOVERY_CYCLES - 1);
            state <= ST_RECOVER;
          end else begin
            cnt <= cnt - 1'b1;
          end
        end

        ST_RECOVER: begin
          if (cnt == 32'd0) begin
            if (rem_left != LEN_WIDTH'(0)) begin
              // open the next (chopped) linear segment at the advanced address
              seg_count <= seg_size(rem_left, cur_wrap);
              seg_left  <= seg_size(rem_left, cur_wrap);
              ca_reg    <= hb_pack_ca(cur_read, cur_reg, ~cur_wrap, cur_addr);
              rwds_hi   <= 1'b0;
              ca_idx    <= 2'd0;
              state     <= ST_CS;
            end else begin
              if (doing_init) begin
                init_done  <= 1'b1;
                doing_init <= 1'b0;
              end
              state <= ST_IDLE;
            end
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

endmodule
`endif
