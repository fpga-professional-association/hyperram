// hyperbus_avalon — Avalon-MM slave front-end for the HyperBus master IP.
//
// Thin bus adapter: translates one Avalon-MM burst-capable slave port into the
// native command / write-data / read-data valid-ready channels of hyperbus_ctrl
// (see docs/INTERFACES.md §hyperbus_avalon, docs/DESIGN.md §3). It adds no
// buffering and no protocol semantics beyond the Avalon<->native mapping.
//
// Address convention (DESIGN.md §3 "Address-space selection"): the Avalon
// address is a WORD address; its MSB selects register space and drives cmd_reg.
// The remaining bits (MSB cleared) form the native word address. All bursts are
// LINEAR (cmd_wrap = 0), per the frozen interface.
//
// Native channel timing (SPEC_DIGEST §5, INTERFACES.md §hyperbus_ctrl): the
// controller accepts one command (cmd handshake) and, for a write, only asserts
// wr_ready later during the data phase — cmd_ready and wr_ready are NOT
// coincident. This front-end therefore issues the command first (holding the
// Avalon write beat via waitrequest) and streams write words afterward. Exactly
// avs_burstcount words are transferred per burst; wr_last marks the final one.
//
// Fully synchronous to clk, synchronous active-high rst. No vendor primitives,
// so it simulates cleanly. Everything derives from hyperbus_pkg; no magic numbers.

module hyperbus_avalon
    import hyperbus_pkg::*;
#(
    parameter int unsigned DQ_WIDTH   = HB_DQ_WIDTH_DEFAULT,
    parameter int unsigned DATA_WIDTH = 2 * DQ_WIDTH,
    parameter int unsigned ADDR_WIDTH = HB_ADDR_WIDTH,
    parameter int unsigned LEN_WIDTH  = HB_LEN_WIDTH_DEFAULT
) (
    input  logic                     clk,
    input  logic                     rst,

    // ---- Avalon-MM slave ---------------------------------------------------
    input  logic [ADDR_WIDTH-1:0]    avs_address,      // WORD address; MSB = register-space select
    input  logic                     avs_read,
    input  logic                     avs_write,
    input  logic [DATA_WIDTH-1:0]    avs_writedata,
    input  logic [DATA_WIDTH/8-1:0]  avs_byteenable,
    input  logic [LEN_WIDTH-1:0]     avs_burstcount,   // words in burst (>=1)
    output logic [DATA_WIDTH-1:0]    avs_readdata,
    output logic                     avs_readdatavalid,
    output logic                     avs_waitrequest,

    // ---- native master (to hyperbus_ctrl) ----------------------------------
    output logic                     cmd_valid,
    input  logic                     cmd_ready,
    output logic                     cmd_read,
    output logic                     cmd_reg,
    output logic                     cmd_wrap,
    output logic [ADDR_WIDTH-1:0]    cmd_addr,
    output logic [LEN_WIDTH-1:0]     cmd_len,

    output logic                     wr_valid,
    input  logic                     wr_ready,
    output logic [DATA_WIDTH-1:0]    wr_data,
    output logic [DATA_WIDTH/8-1:0]  wr_strb,
    output logic                     wr_last,

    input  logic                     rd_valid,
    output logic                     rd_ready,
    input  logic [DATA_WIDTH-1:0]    rd_data,
    input  logic                     rd_last,

    // -- wrapped-burst enable (issue #13): drives cmd_wrap for the wrap-probe burst. Quasi-static,
    //    same `clk` domain. POR-legacy 0 => cmd_wrap=0 = today's linear-only front end. NO port
    //    default value (Verilator rejects them); every instantiation ties it (legacy 0). --
    input  logic                     wrap_en,

    // -- DEBUG tap (bring-up only; leave unconnected in normal instantiations) --
    output logic [1:0]               dbg_state       // front-end FSM state (IDLE/WR_DATA/RD_WAIT)
);

    // MSB of the Avalon address selects register space (DESIGN.md §3).
    localparam int unsigned REG_SEL_BIT = ADDR_WIDTH - 1;

    // Transaction FSM. Single outstanding transaction: waitrequest holds off any
    // new request until the current read drains / write burst completes.
    //   IDLE    : accept a read or write command from the Avalon side.
    //   WR_DATA : stream the write-burst data words to the native wr channel.
    //   RD_WAIT : forward returning read words to the Avalon side until rd_last.
    typedef enum logic [1:0] {IDLE, WR_DATA, RD_WAIT} state_e;
    state_e state;

    assign dbg_state = 2'(state);

    // Remaining write words to transfer in the current burst (loaded when the
    // write command is accepted; wr_last asserts on the final word).
    logic [LEN_WIDTH-1:0] wr_words_left;

    // Remaining read words to return in the current burst. avs_burstcount is the AUTHORITATIVE beat
    // count (Avalon: exactly burstcount readdatavalid beats per read burst), so the burst completes
    // once exactly that many beats have been returned. The controller's rd_last hint is deliberately
    // NOT used to terminate the burst: on the AXC3000, under the device's back-to-back over-streamed
    // read delivery, rd_last proved UNRELIABLE at this boundary (on-chip capture: for one 16-word burst
    // it never asserted — front-end hung waiting for a burst-end that never came; for the next it
    // asserted EARLY at ~beat 5 — front-end finished early and the bench then hung waiting for the
    // missing beats). Counting the requested beats is deterministic and makes multi-burst reads robust.
    logic [LEN_WIDTH-1:0] rd_words_left;
    wire                  rd_beat = rd_valid & (state == RD_WAIT);   // a returned read beat this cycle
    wire                  rd_burst_end = rd_beat & (rd_words_left == LEN_WIDTH'(1));

    // ------------------------------------------------------------------------
    // Combinational outputs
    // ------------------------------------------------------------------------
    always_comb begin
        // Command channel: memory/register from the address MSB; linear bursts.
        // The native word address is the Avalon address with the select MSB
        // cleared so it never leaks into the CA row/column field.
        cmd_valid = 1'b0;
        cmd_read  = 1'b0;
        cmd_reg   = avs_address[REG_SEL_BIT];
        cmd_wrap  = wrap_en;                                  // issue #13: 1 = wrapped probe burst (else linear)
        cmd_addr  = {1'b0, avs_address[ADDR_WIDTH-2:0]};
        cmd_len   = avs_burstcount;

        // Write-data channel (active only while streaming a write burst).
        wr_valid  = 1'b0;
        wr_data   = avs_writedata;
        wr_strb   = avs_byteenable;                          // 1 = write byte
        wr_last   = (wr_words_left == LEN_WIDTH'(1));

        // Read-data channel: accept returning words only during a read.
        rd_ready          = (state == RD_WAIT);
        avs_readdata      = rd_data;
        avs_readdatavalid = rd_valid & (state == RD_WAIT);

        // Default: stall the Avalon master (busy / not ready to accept).
        avs_waitrequest = 1'b1;

        unique case (state)
            IDLE: begin
                if (avs_read) begin
                    // Issue the read command; accept the Avalon read beat on the
                    // cycle the controller takes the command.
                    cmd_valid       = 1'b1;
                    cmd_read        = 1'b1;
                    avs_waitrequest = ~cmd_ready;
                end else if (avs_write) begin
                    // Issue the write command but hold the Avalon write beat
                    // (waitrequest high) until the data phase — cmd_ready and
                    // wr_ready are not coincident on the native side.
                    cmd_valid       = 1'b1;
                    cmd_read        = 1'b0;
                    avs_waitrequest = 1'b1;
                end
            end

            WR_DATA: begin
                // Stream write words; the master holds each writedata beat while
                // waitrequest is high. Consume one word per wr handshake.
                wr_valid        = avs_write;
                avs_waitrequest = ~wr_ready;
            end

            RD_WAIT: begin
                // Draining read data; block new requests until the burst ends.
                avs_waitrequest = 1'b1;
            end

            default: begin
                avs_waitrequest = 1'b1;
            end
        endcase
    end

    // ------------------------------------------------------------------------
    // Sequential state
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state         <= IDLE;
            wr_words_left <= '0;
            rd_words_left <= '0;
        end else begin
            unique case (state)
                IDLE: begin
                    // A command transfers when cmd_valid & cmd_ready; cmd_valid
                    // is asserted combinationally for the pending request.
                    if (avs_read && cmd_ready) begin
                        state         <= RD_WAIT;
                        rd_words_left <= avs_burstcount;     // >=1 (authoritative read-beat count)
                    end else if (avs_write && cmd_ready) begin
                        state         <= WR_DATA;
                        wr_words_left <= avs_burstcount;     // >=1
                    end
                end

                WR_DATA: begin
                    if (wr_valid && wr_ready) begin
                        if (wr_words_left == LEN_WIDTH'(1)) begin
                            state <= IDLE;                   // final word accepted
                        end
                        wr_words_left <= wr_words_left - LEN_WIDTH'(1);
                    end
                end

                RD_WAIT: begin
                    // Complete on the controller's rd_last OR once avs_burstcount beats have been
                    // returned (whichever first) — robust to a missing/mistimed rd_last (see above).
                    if (rd_beat)      rd_words_left <= rd_words_left - LEN_WIDTH'(1);
                    if (rd_burst_end) state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
