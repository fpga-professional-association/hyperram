// hyperbus_axi — AXI4 slave front-end for the HyperBus master IP.
//
// Protocol adapter: translates AXI4 read/write bursts into the native controller
// command/write-data/read-data interface of `hyperbus_ctrl` and generates the
// B/R responses, echoing the AXI IDs.
//
// Frozen ports: docs/INTERFACES.md §hyperbus_axi (v2 — adds err_underrun/err_timeout
// inputs, see the interface-revision log). Semantics: docs/SPEC_DIGEST.md. Shared
// params/typedefs: rtl/hyperbus_pkg.sv. No magic numbers.
//
// AXI burst-type handling (AXI4 A3.4.1). The HyperBus device wraps only at its
// CR0[1:0]-configured group, which is independent of any per-transaction AXI wrap
// boundary; and the native controller has no "repeat-address" mode. So this adapter
// does NOT forward AXI burst geometry to the device wrap logic. Instead it decomposes
// every AXI burst into one or more *linear* native segments that exactly reproduce the
// AXI address/order semantics for ANY boundary and length:
//   * INCR  -> one linear segment (addr, len=N).
//   * WRAP  -> two linear segments that walk the AXI wrap region in critical-word-first
//              order: seg0 = (start .. region-top), seg1 = (region-base .. start-1).
//              Region base/len come from AXI (awlen+1)*2^awsize, NOT from CR0 — so the
//              returned/written word order is AXI-correct regardless of the device wrap
//              group, and WRAP2/WRAP4 (which HyperBus cannot express natively) work too.
//   * FIXED -> N single-word linear segments all at the same address (AXI FIXED accesses
//              the same location every beat).
// The AXI R/W data channel streams exactly N beats across the segments transparently;
// rlast is asserted only on the final beat of the final segment.
//
// AxSIZE: this adapter maps one AXI beat to one 16-bit native word, so it requires the
// full-width beat size (AxSIZE == log2(DATA_WIDTH/8)). A narrow burst is accepted but
// flagged with SLVERR on the response (AXI4 A3.4.4) rather than silently mis-decoded.
//
// Error responses: err_underrun / err_timeout from the controller latch a sticky
// per-transaction error that drives BRESP/RRESP = SLVERR (AXI4 A3.4.4).
//
// One transaction is outstanding at a time (the native controller has a single command
// channel); AR/AW are round-robin arbitrated. All state is architectural -> synchronous
// active-high reset. AXI-facing ready/valid outputs are additionally gated Low during
// reset (AXI4 A3.1.2).

`include "hyperbus_pkg.sv"

module hyperbus_axi
  import hyperbus_pkg::*;
#(
  parameter int unsigned DQ_WIDTH       = HB_DQ_WIDTH_DEFAULT,          // 8
  parameter int unsigned DATA_WIDTH     = 2 * DQ_WIDTH,                 // 16 (native word)
  parameter int unsigned ADDR_WIDTH     = HB_ADDR_WIDTH,               // 32 (word address)
  parameter int unsigned LEN_WIDTH      = HB_LEN_WIDTH_DEFAULT,        // 16 (words)
  parameter int unsigned ID_WIDTH       = 4,
  parameter int unsigned AXI_DATA_WIDTH = DATA_WIDTH,                  // == DATA_WIDTH (1:1 beats)
  parameter int unsigned AXI_ADDR_WIDTH = ADDR_WIDTH + 1              // byte address; MSB = reg space
) (
  input  logic                        clk,   // aclk
  input  logic                        rst,   // synchronous, active high (invert of aresetn)

  // -- AXI4 slave: write address (AW) --
  input  logic [ID_WIDTH-1:0]         awid,
  input  logic [AXI_ADDR_WIDTH-1:0]   awaddr,
  input  logic [7:0]                  awlen,
  input  logic [2:0]                  awsize,
  input  logic [1:0]                  awburst,
  input  logic                        awvalid,
  output logic                        awready,

  // -- AXI4 slave: write data (W) --
  input  logic [AXI_DATA_WIDTH-1:0]   wdata,
  input  logic [AXI_DATA_WIDTH/8-1:0] wstrb,
  input  logic                        wlast,
  input  logic                        wvalid,
  output logic                        wready,

  // -- AXI4 slave: write response (B) --
  output logic [ID_WIDTH-1:0]         bid,
  output logic [1:0]                  bresp,
  output logic                        bvalid,
  input  logic                        bready,

  // -- AXI4 slave: read address (AR) --
  input  logic [ID_WIDTH-1:0]         arid,
  input  logic [AXI_ADDR_WIDTH-1:0]   araddr,
  input  logic [7:0]                  arlen,
  input  logic [2:0]                  arsize,
  input  logic [1:0]                  arburst,
  input  logic                        arvalid,
  output logic                        arready,

  // -- AXI4 slave: read data (R) --
  output logic [ID_WIDTH-1:0]         rid,
  output logic [AXI_DATA_WIDTH-1:0]   rdata,
  output logic [1:0]                  rresp,
  output logic                        rlast,
  output logic                        rvalid,
  input  logic                        rready,

  // -- native master command channel (to hyperbus_ctrl) --
  output logic                        cmd_valid,
  input  logic                        cmd_ready,
  output logic                        cmd_read,   // 1 = read
  output logic                        cmd_reg,    // 1 = register space
  output logic                        cmd_wrap,   // 1 = wrapped burst (unused: always linear here)
  output logic [ADDR_WIDTH-1:0]       cmd_addr,   // WORD address
  output logic [LEN_WIDTH-1:0]        cmd_len,    // words, >=1

  // -- native master write-data channel --
  output logic                        wr_valid,
  input  logic                        wr_ready,
  output logic [DATA_WIDTH-1:0]       wr_data,
  output logic [DATA_WIDTH/8-1:0]     wr_strb,
  output logic                        wr_last,

  // -- native master read-data channel --
  input  logic                        rd_valid,
  output logic                        rd_ready,
  input  logic [DATA_WIDTH-1:0]       rd_data,
  input  logic                        rd_last,

  // -- controller error status (v2 interface) --
  input  logic                        err_underrun, // pulse: write-data underrun in controller
  input  logic                        err_timeout   // pulse: read RWDS timeout in controller
);

  // ------------------------------------------------------------------------
  // Local constants
  // ------------------------------------------------------------------------
  localparam int unsigned STRB_WIDTH = DATA_WIDTH / 8;             // 2
  localparam int unsigned BYTE_BITS  = $clog2(STRB_WIDTH);         // byte-offset bits within a word (1)
  localparam logic [2:0]  FULL_SIZE  = 3'(BYTE_BITS);             // AxSIZE for a full 16-bit beat

  localparam logic [1:0] AXI_BURST_FIXED = 2'b00;
  localparam logic [1:0] AXI_BURST_WRAP  = 2'b10;
  localparam logic [1:0] AXI_RESP_OKAY   = 2'b00;
  localparam logic [1:0] AXI_RESP_SLVERR = 2'b10;

  // Beats map 1:1 to native words; the sub-16-bit gearbox is out of scope here.
  initial begin
    if (AXI_DATA_WIDTH != DATA_WIDTH) begin
      $error("hyperbus_axi: AXI_DATA_WIDTH (%0d) must equal DATA_WIDTH (%0d); gearbox not implemented",
             AXI_DATA_WIDTH, DATA_WIDTH);
    end
  end

  // ------------------------------------------------------------------------
  // Transaction FSM (one outstanding transaction, AR/AW round-robin)
  // ------------------------------------------------------------------------
  typedef enum logic [2:0] {
    S_IDLE,   // arbitrate AR/AW, accept the address beat, plan the segments
    S_CMD,    // drive the native command handshake for the current segment
    S_WDATA,  // stream W beats -> native write-data (current segment)
    S_BRESP,  // drive the B response
    S_RDATA   // stream native read-data -> R beats (current segment)
  } state_e;

  state_e                  state;
  logic                    last_was_write;   // round-robin fairness token

  // Latched transaction context.
  logic [ID_WIDTH-1:0]     id_q;
  logic                    read_q;
  logic                    reg_q;
  logic                    fixed_q;      // AXI FIXED burst: repeat single-word segments at base_addr_q
  logic [ADDR_WIDTH-1:0]   base_addr_q;  // original word address (FIXED repeat target)
  logic [ADDR_WIDTH-1:0]   wrap_base_q;  // AXI wrap-region base word address (WRAP seg1 start)
  logic [ADDR_WIDTH-1:0]   addr_q;       // current native segment start word address
  logic [LEN_WIDTH-1:0]    seg_len_q;    // current native segment length (= cmd_len)
  logic [LEN_WIDTH-1:0]    rem_q;        // beats remaining AFTER the current segment completes
  logic [LEN_WIDTH-1:0]    seg_left;     // words remaining in the current write segment
  logic                    err_q;        // sticky SLVERR for the whole transaction

  // ------------------------------------------------------------------------
  // Combinational address/attribute decode for the incoming AW/AR beat.
  // ------------------------------------------------------------------------
  // Per-channel decode: reg-space select, word address, beat count N, full-size check,
  // and the WRAP region geometry (base + first-segment length), all in native words.
  logic                    aw_reg, ar_reg;
  logic [ADDR_WIDTH-1:0]   aw_word, ar_word;
  logic [LEN_WIDTH-1:0]    aw_n, ar_n;
  logic                    aw_size_ok, ar_size_ok;
  logic [ADDR_WIDTH-1:0]   aw_mask, ar_mask;      // (N-1) mask (N is a power of two for WRAP)
  logic [ADDR_WIDTH-1:0]   aw_off, ar_off;        // start offset within the wrap region
  logic [ADDR_WIDTH-1:0]   aw_wbase, ar_wbase;    // wrap-region base word address
  logic [LEN_WIDTH-1:0]    aw_len1, ar_len1;      // words from start to region top (WRAP seg0 length)

  assign aw_reg     = awaddr[AXI_ADDR_WIDTH-1];
  assign ar_reg     = araddr[AXI_ADDR_WIDTH-1];
  assign aw_word    = ADDR_WIDTH'(awaddr[AXI_ADDR_WIDTH-2:BYTE_BITS]);
  assign ar_word    = ADDR_WIDTH'(araddr[AXI_ADDR_WIDTH-2:BYTE_BITS]);
  assign aw_n       = LEN_WIDTH'(awlen) + LEN_WIDTH'(1);
  assign ar_n       = LEN_WIDTH'(arlen) + LEN_WIDTH'(1);
  assign aw_size_ok = (awsize == FULL_SIZE);
  assign ar_size_ok = (arsize == FULL_SIZE);
  assign aw_mask    = ADDR_WIDTH'(aw_n) - ADDR_WIDTH'(1);
  assign ar_mask    = ADDR_WIDTH'(ar_n) - ADDR_WIDTH'(1);
  assign aw_off     = aw_word & aw_mask;
  assign ar_off     = ar_word & ar_mask;
  assign aw_wbase   = aw_word & ~aw_mask;
  assign ar_wbase   = ar_word & ~ar_mask;
  assign aw_len1    = aw_n - LEN_WIDTH'(aw_off);
  assign ar_len1    = ar_n - LEN_WIDTH'(ar_off);

  // Round-robin grant among simultaneously valid AR/AW (IDLE only).
  logic grant_write, grant_read;
  assign grant_write = awvalid & (~arvalid | ~last_was_write);
  assign grant_read  = arvalid & (~awvalid |  last_was_write);

  // ------------------------------------------------------------------------
  // Next-segment computation (used when a segment completes with rem_q>0).
  // FIXED: another single word at the base address. WRAP: the second linear run
  // from the region base for the remaining beats.
  // ------------------------------------------------------------------------
  logic [ADDR_WIDTH-1:0]  next_addr;
  logic [LEN_WIDTH-1:0]   next_len;
  logic [LEN_WIDTH-1:0]   next_rem;
  always_comb begin
    if (fixed_q) begin
      next_addr = base_addr_q;
      next_len  = LEN_WIDTH'(1);
      next_rem  = rem_q - LEN_WIDTH'(1);
    end else begin                 // WRAP second segment
      next_addr = wrap_base_q;
      next_len  = rem_q;
      next_rem  = LEN_WIDTH'(0);
    end
  end

  // ------------------------------------------------------------------------
  // State register + transaction context latch
  // ------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      state          <= S_IDLE;
      last_was_write <= 1'b0;
      id_q           <= '0;
      read_q         <= 1'b0;
      reg_q          <= 1'b0;
      fixed_q        <= 1'b0;
      base_addr_q    <= '0;
      wrap_base_q    <= '0;
      addr_q         <= '0;
      seg_len_q      <= '0;
      rem_q          <= '0;
      seg_left       <= '0;
      err_q          <= 1'b0;
    end else begin
      unique case (state)
        S_IDLE: begin
          if (grant_write) begin           // awready == grant_write, so AW beat is accepted now
            id_q           <= awid;
            read_q         <= 1'b0;
            reg_q          <= aw_reg;
            base_addr_q    <= aw_word;
            wrap_base_q    <= aw_wbase;
            err_q          <= ~aw_size_ok;                  // SLVERR on narrow (unsupported) beat size
            last_was_write <= 1'b1;
            state          <= S_CMD;
            if (awburst == AXI_BURST_WRAP) begin
              fixed_q   <= 1'b0;
              addr_q    <= aw_word;
              seg_len_q <= aw_len1;
              rem_q     <= LEN_WIDTH'(aw_off);              // second segment length
            end else if (awburst == AXI_BURST_FIXED) begin
              fixed_q   <= 1'b1;
              addr_q    <= aw_word;
              seg_len_q <= LEN_WIDTH'(1);
              rem_q     <= aw_n - LEN_WIDTH'(1);
            end else begin                                  // INCR (and reserved) -> single linear run
              fixed_q   <= 1'b0;
              addr_q    <= aw_word;
              seg_len_q <= aw_n;
              rem_q     <= LEN_WIDTH'(0);
            end
          end else if (grant_read) begin   // arready == grant_read
            id_q           <= arid;
            read_q         <= 1'b1;
            reg_q          <= ar_reg;
            base_addr_q    <= ar_word;
            wrap_base_q    <= ar_wbase;
            err_q          <= ~ar_size_ok;
            last_was_write <= 1'b0;
            state          <= S_CMD;
            if (arburst == AXI_BURST_WRAP) begin
              fixed_q   <= 1'b0;
              addr_q    <= ar_word;
              seg_len_q <= ar_len1;
              rem_q     <= LEN_WIDTH'(ar_off);
            end else if (arburst == AXI_BURST_FIXED) begin
              fixed_q   <= 1'b1;
              addr_q    <= ar_word;
              seg_len_q <= LEN_WIDTH'(1);
              rem_q     <= ar_n - LEN_WIDTH'(1);
            end else begin
              fixed_q   <= 1'b0;
              addr_q    <= ar_word;
              seg_len_q <= ar_n;
              rem_q     <= LEN_WIDTH'(0);
            end
          end
        end

        S_CMD: begin
          if (cmd_ready) begin             // cmd_valid held high in this state
            seg_left <= seg_len_q;
            state    <= read_q ? S_RDATA : S_WDATA;
          end
        end

        S_WDATA: begin
          if (err_underrun) err_q <= 1'b1;
          if (wr_valid & wr_ready) begin
            if (seg_left == LEN_WIDTH'(1)) begin   // last word of this native segment
              if (rem_q != LEN_WIDTH'(0)) begin    // more segments (FIXED repeat / WRAP seg1)
                addr_q    <= next_addr;
                seg_len_q <= next_len;
                rem_q     <= next_rem;
                state     <= S_CMD;
              end else begin
                state <= S_BRESP;
              end
            end
            seg_left <= seg_left - LEN_WIDTH'(1);
          end
        end

        S_BRESP: begin
          if (bready) begin                // bvalid held high in this state
            state <= S_IDLE;
          end
        end

        S_RDATA: begin
          if (err_timeout) err_q <= 1'b1;
          if (rd_valid & rready & rd_last) begin // native segment finished
            if (rem_q != LEN_WIDTH'(0)) begin
              addr_q    <= next_addr;
              seg_len_q <= next_len;
              rem_q     <= next_rem;
              state     <= S_CMD;
            end else begin
              state <= S_IDLE;
            end
          end
        end

        default: state <= S_IDLE;
      endcase
    end
  end

  // ------------------------------------------------------------------------
  // Combinational output drive
  // ------------------------------------------------------------------------
  // AXI address-channel readies: accept exactly the granted beat in IDLE (Low during reset).
  assign awready = (state == S_IDLE) & grant_write & ~rst;
  assign arready = (state == S_IDLE) & grant_read  & ~rst;

  // Native command channel (valid must not depend on cmd_ready). Always linear; the WRAP/FIXED
  // decomposition above already produced AXI-correct linear segments.
  assign cmd_valid = (state == S_CMD) & ~rst;
  assign cmd_read  = read_q;
  assign cmd_reg   = reg_q;
  assign cmd_wrap  = 1'b0;
  assign cmd_addr  = addr_q;
  assign cmd_len   = seg_len_q;

  // Write-data: pass AXI W beats through in S_WDATA. wr_last marks the current native segment's
  // final word (the controller uses cmd_len as authoritative; wr_last is informational).
  assign wr_valid = (state == S_WDATA) & wvalid;
  assign wready   = (state == S_WDATA) & wr_ready & ~rst;
  assign wr_data  = wdata;
  assign wr_strb  = wstrb;
  assign wr_last  = (seg_left == LEN_WIDTH'(1));

  // Write response.
  assign bvalid = (state == S_BRESP) & ~rst;
  assign bid    = id_q;
  assign bresp  = err_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;

  // Read-data: pass native read words through in S_RDATA. rlast only on the final beat of the
  // final segment (native rd_last while no segments remain).
  assign rvalid   = (state == S_RDATA) & rd_valid & ~rst;
  assign rd_ready = (state == S_RDATA) & rready;
  assign rdata    = rd_data;
  assign rlast    = rd_last & (rem_q == LEN_WIDTH'(0));
  assign rid      = id_q;
  assign rresp    = err_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;

  // ------------------------------------------------------------------------
  // Intentionally unused inputs: the within-word byte-offset address bit(s) (word-granular
  // access) and AXI wlast (native segment length is authoritative for control).
  // ------------------------------------------------------------------------
  /* verilator lint_off UNUSEDSIGNAL */
  logic _unused;
  assign _unused = &{1'b0, wlast, awaddr[BYTE_BITS-1:0], araddr[BYTE_BITS-1:0]};
  /* verilator lint_on UNUSEDSIGNAL */

endmodule
