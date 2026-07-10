// hyperbus_pkg — shared parameters, typedefs and functions for the HyperBus master IP.
//
// Normative source: Infineon/Cypress HyperBus Specification 001-99253 Rev *H, distilled in
// docs/SPEC_DIGEST.md. Every module in this IP imports this package; nothing here instantiates
// logic, so it is fully Verilator-simulable and vendor-agnostic.
//
// Conventions frozen here (see docs/INTERFACES.md):
//   * DQ bus is DQ_WIDTH bits wide (default 8). One HyperBus "word" = 2*DQ_WIDTH bits = DATA_WIDTH.
//     The DDR bus moves one DQ_WIDTH byte per clock EDGE, i.e. one DATA_WIDTH word per clock CYCLE.
//   * Within a word, "byte A" = word[DATA_WIDTH-1 : DQ_WIDTH] is the FIRST edge (CK rising / RWDS
//     rising); "byte B" = word[DQ_WIDTH-1 : 0] is the SECOND edge. Register space is always
//     big-endian; this A=high / B=low mapping is the big-endian order (SPEC_DIGEST §4).
//   * Addresses on every interface are WORD addresses (units of DATA_WIDTH/16-bit words), per the
//     spec's word-addressed CA field (SPEC_DIGEST §2).
`ifndef HYPERBUS_PKG_SV
`define HYPERBUS_PKG_SV
package hyperbus_pkg;

  // ------------------------------------------------------------------------
  // Bus geometry (defaults; modules re-expose these as overridable parameters)
  // ------------------------------------------------------------------------
  localparam int unsigned HB_DQ_WIDTH_DEFAULT   = 8;                       // DQ pins
  localparam int unsigned HB_DATA_WIDTH_DEFAULT = 2 * HB_DQ_WIDTH_DEFAULT; // 16-bit HyperBus word
  localparam int unsigned HB_ADDR_WIDTH         = 32;                      // word-address space (CA max)
  localparam int unsigned HB_LEN_WIDTH_DEFAULT  = 16;                      // burst length counter (words)

  localparam int unsigned HB_CA_BITS = 48;                                 // Command-Address width

  // ------------------------------------------------------------------------
  // Latency defaults (SPEC_DIGEST §3). Latency is measured in CLOCK CYCLES from
  // CA1 capture to first data. DEFAULT is the POR / max-frequency-safe value.
  // ------------------------------------------------------------------------
  localparam int unsigned HB_LATENCY_CLOCKS_DEFAULT = 6;   // CR0[7:4]=0001
  localparam bit          HB_FIXED_LATENCY_DEFAULT  = 1'b1; // CR0[3]=1 (fixed, POR default)

  // ------------------------------------------------------------------------
  // Default burst / wrap (SPEC_DIGEST §7, CR0[1:0]/CR0[2])
  // ------------------------------------------------------------------------
  localparam int unsigned HB_BURST_WORDS_DEFAULT = 16;    // default user burst length (words)

  // ------------------------------------------------------------------------
  // Runtime PHY read-eye CALIBRATION port widths (docs/INTERFACES.md v9). The frozen `hyperbus_phy`
  // contract carries four *mandatory* cal_* inputs so a host can retune the read eye by a CSR write
  // (REG_CAL, see hyperram_bw_test) with no recompile. These widths size the two multi-bit knobs:
  //   * cal_preamble_skip : leading rwds-rise edges to discard as read-strobe preamble (SDR PHY).
  //                         3 bits => 0..7 (fixed width; do NOT derive from a parameter's value).
  //   * cal_rx_tap        : RWDS eye-centring delay-line tap index (Agilex DDIO PHY). 5 bits =>
  //                         0..31 ⊇ the 17 valid tap indices of RX_STROBE_MAX_TAPS+1 @ default.
  // (cal_capture_phase and cal_pair_skew are 1 bit each and need no width localparam.)
  localparam int unsigned HB_CAL_PREAMBLE_SKIP_WIDTH = 3;
  localparam int unsigned HB_CAL_RX_TAP_WIDTH        = 5;

  // ------------------------------------------------------------------------
  // Device register-space word addresses + reset values.
  // [device, not generic spec] — W957D8NB / HyperRAM family, cross-checked against
  // agilex_3_ai_benchmarks/sim/hyperbus/w957d8nb_bfm.sv (SPEC_DIGEST §8.4).
  // ------------------------------------------------------------------------
  localparam logic [HB_ADDR_WIDTH-1:0] HB_REG_ID0 = 32'h0000_0000;
  localparam logic [HB_ADDR_WIDTH-1:0] HB_REG_ID1 = 32'h0000_0001;
  localparam logic [HB_ADDR_WIDTH-1:0] HB_REG_CR0 = 32'h0000_0800;
  localparam logic [HB_ADDR_WIDTH-1:0] HB_REG_CR1 = 32'h0000_0801;

  localparam logic [15:0] HB_ID0_RESET = 16'h0C81; // mfr nibble 0001
  localparam logic [15:0] HB_ID1_RESET = 16'h0000; // device-type 0000 = HyperRAM
  localparam logic [15:0] HB_CR0_RESET = 16'h0008; // bit3=1 -> fixed latency
  localparam logic [15:0] HB_CR1_RESET = 16'h0000;

  // ------------------------------------------------------------------------
  // Enumerated CA field meanings (documentation typedefs; 1-bit each)
  // ------------------------------------------------------------------------
  typedef enum logic {HB_WRITE = 1'b0, HB_READ    = 1'b1} hb_rw_e;       // CA[47]
  typedef enum logic {HB_MEM   = 1'b0, HB_REG     = 1'b1} hb_space_e;    // CA[46]
  typedef enum logic {HB_WRAP  = 1'b0, HB_LINEAR  = 1'b1} hb_burst_e;    // CA[45]

  typedef logic [HB_CA_BITS-1:0] hb_ca_t;

  // ------------------------------------------------------------------------
  // CA pack / unpack (SPEC_DIGEST §2, Table 3.2)
  //   CA[47]    = R/W#         (1 = read)
  //   CA[46]    = Address Space(1 = register)
  //   CA[45]    = Burst Type   (1 = linear, 0 = wrapped)
  //   CA[44:16] = word address bits A31..A3
  //   CA[15:3]  = reserved (host writes 0)
  //   CA[2:0]   = word address bits A2..A0
  // ------------------------------------------------------------------------
  function automatic hb_ca_t hb_pack_ca(input logic                       rd_notwr,
                                        input logic                       reg_space,
                                        input logic                       linear,
                                        input logic [HB_ADDR_WIDTH-1:0]   word_addr);
    hb_ca_t ca;
    ca           = '0;
    ca[47]       = rd_notwr;
    ca[46]       = reg_space;
    ca[45]       = linear;
    ca[44:16]    = word_addr[31:3];
    ca[2:0]      = word_addr[2:0];
    return ca;
  endfunction

  function automatic logic                     hb_ca_read  (input hb_ca_t ca); return ca[47]; endfunction
  function automatic logic                     hb_ca_reg   (input hb_ca_t ca); return ca[46]; endfunction
  function automatic logic                     hb_ca_linear(input hb_ca_t ca); return ca[45]; endfunction

  /* verilator lint_off UNUSEDSIGNAL */
  function automatic logic [HB_ADDR_WIDTH-1:0] hb_ca_addr(input hb_ca_t ca);
    // CA[47:45] control + CA[15:3] reserved are intentionally dropped
    return {ca[44:16], ca[2:0]};
  endfunction
  /* verilator lint_on UNUSEDSIGNAL */

  // ------------------------------------------------------------------------
  // Latency code <-> clock-count mapping (SPEC_DIGEST §3, Table 5.3).
  // NOTE the two low codes sit at the TOP of the field, out of numeric order:
  //   0000..1011 => 5..16 clocks ; 1110 => 3 ; 1111 => 4 ; 1100/1101 reserved.
  // ------------------------------------------------------------------------
  function automatic int unsigned hb_latency_code_to_clocks(input logic [3:0] code);
    unique case (code)
      4'b1110: return 3;
      4'b1111: return 4;
      4'b0000: return 5;
      4'b0001: return 6;
      4'b0010: return 7;
      4'b0011: return 8;
      4'b0100: return 9;
      4'b0101: return 10;
      4'b0110: return 11;
      4'b0111: return 12;
      4'b1000: return 13;
      4'b1001: return 14;
      4'b1010: return 15;
      4'b1011: return 16;
      default: return 6; // 1100/1101 reserved -> safe default
    endcase
  endfunction

  function automatic logic [3:0] hb_clocks_to_latency_code(input int unsigned clocks);
    unique case (clocks)
      3:       return 4'b1110;
      4:       return 4'b1111;
      5:       return 4'b0000;
      6:       return 4'b0001;
      7:       return 4'b0010;
      8:       return 4'b0011;
      9:       return 4'b0100;
      10:      return 4'b0101;
      11:      return 4'b0110;
      12:      return 4'b0111;
      13:      return 4'b1000;
      14:      return 4'b1001;
      15:      return 4'b1010;
      16:      return 4'b1011;
      default: return 4'b0001; // -> 6 clocks
    endcase
  endfunction

  // ------------------------------------------------------------------------
  // Wrap-boundary decode (SPEC_DIGEST §7). CR0[1:0]: 00=128B,01=64B,10=16B,11=32B.
  // Returns the wrap group size in WORDS (bytes/2).
  // ------------------------------------------------------------------------
  function automatic int unsigned hb_wrap_words(input logic [1:0] cr0_burst);
    unique case (cr0_burst)
      2'b00: return 64; // 128 B
      2'b01: return 32; //  64 B
      2'b10: return 8;  //  16 B
      2'b11: return 16; //  32 B (default)
      default: return 16;
    endcase
  endfunction

endpackage
`endif
