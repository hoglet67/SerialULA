`define BOARD_REV_01
//`define BOARD_REV_02
//`define BOARD_GODIL


`define MODEL_FERRANTI
//`define MODEL_VLSI
//`define MODEL_JUMPERED

`define HIGH_TONE_THRESHOLD_VLSI     445
`define HIGH_TONE_THRESHOLD_FERRANTI 962

`ifdef MODEL_VLSI
`define HIGH_TONE_BITS 9
`else
`define HIGH_TONE_BITS 10
`endif

module serialula
  (
   // Fast clock (16/13 MHz)
   input            clk,

   // Mode Jumper (used to enable VLSI_SERPROC mode)
   input            jp1,   // off/1=Ferranti on/0=VLSI

   // Interface to 6502
   input            E,
   input [7:0]      Data,
   input            nCS,

   // Interface to Cassette Port
   output           CasMotor,
   input            CasIn,
`ifdef BOARD_GODIL
   output [2:0]     CasOut,
`else
   output [1:0]     CasOut,
`endif

   // Interface to ACIA
   output           TxC,
   input            TxD,
   output           RxC,
   output           RxD,
   output           DCD,
   input            RTSI,
   output           CTSO,

   // Interface to RS423 Port
   input            Din,
   output           Dout,
   input            CTSI,
   output           RTSO
   );

`ifdef MODEL_FERRANTI
   wire         vlsi_mode = 1'b0;
`endif
`ifdef MODEL_VLSI
   wire         vlsi_mode = 1'b1;
`endif
`ifdef MODEL_JUMPERED
   wire         vlsi_mode = !jp1;
`endif

   reg [7:0]        control;

   wire [2:0]       ctrl_tx_baud       = control[2:0];
   wire [2:0]       ctrl_rx_baud       = control[5:3];
   wire             ctrl_reverse_tones = control[3] & vlsi_mode;
   wire             ctrl_rs423_sel     = control[6];
   wire             ctrl_motor_on      = control[7];

   reg [9:0]        clk_divider;
   reg              tx_clk;
   reg              rx_clk;
   wire [2:0]       sine_in;
   reg [2:0]        burst_counter;
   reg [`HIGH_TONE_BITS-1:0] high_tone_counter;
   wire [`HIGH_TONE_BITS-1:0] high_tone_threshold;
   reg              high_tone_detect;
   reg              txd_s;
   reg              enable_s;
   reg              cas_clk_recovered;
   reg              cas_din_recovered;
   reg              cas_din_synchronized;
   reg              cas_din_filtered;
   reg              cas_din_edge;
   reg [1:0]        filter_counter;
   reg [7:0]        bit_counter;
   wire             burst0;
   wire             burst1;
   reg              is_long;
   reg              is_long_last;

   // =================================================
   // Control reguster
   // =================================================

   // Update the control register on the falling edge of the 2MHz clock
   // Note: reads do seem to corrupt this register

   always @(negedge E) begin
      if (!nCS) begin
         control <= Data;
      end
   end

   // =================================================
   // Master clock divider
   // =================================================

   always @(posedge clk) begin
      clk_divider <= clk_divider + 1'b1;
   end

   // =================================================
   // Transmit baud rate generator
   // =================================================

   always @(*) begin
      case (ctrl_tx_baud)
        3'b000: tx_clk = clk;            // 19200 baud
        3'b100: tx_clk = clk_divider[0]; //  9600 baud
        3'b010: tx_clk = clk_divider[1]; //  4800 baud
        3'b110: tx_clk = clk_divider[2]; //  2400 baud
        3'b001: tx_clk = clk_divider[3]; //  1200 baud
        3'b101: tx_clk = clk_divider[5]; //   300 baud
        3'b011: tx_clk = clk_divider[6]; //   150 baud
        3'b111: tx_clk = clk_divider[7]; //    75 baud
      endcase
   end

   // =================================================
   // Receive baud rate generator
   // =================================================

   always @(*) begin
      case (ctrl_rx_baud)
        3'b000: rx_clk = clk;            // 19200 baud
        3'b100: rx_clk = clk_divider[0]; //  9600 baud
        3'b010: rx_clk = clk_divider[1]; //  4800 baud
        3'b110: rx_clk = clk_divider[2]; //  2400 baud
        3'b001: rx_clk = clk_divider[3]; //  1200 baud
        3'b101: rx_clk = clk_divider[5]; //   300 baud
        3'b011: rx_clk = clk_divider[6]; //   150 baud
        3'b111: rx_clk = clk_divider[7]; //    75 baud
      endcase
   end

   // =================================================
   // Synchronise/filter raw CasIn and detect edges
   // =================================================

   // We don't have any evidance (yet) that the real ULA
   // does any filtering of the input

   always @(posedge clk) begin
      if (clk_divider[0]) begin
         cas_din_edge <= 1'b0;
         cas_din_synchronized <= CasIn;
         if (cas_din_filtered == cas_din_synchronized) begin
            filter_counter <= 0;
         end else begin
            filter_counter <= filter_counter + 1;
            if (&filter_counter) begin
               cas_din_filtered <= cas_din_synchronized;
               cas_din_edge <= 1'b1;
            end
         end
      end
   end

   // =================================================
   // Cassette Data Seperator
   // =================================================

   assign burst0 = (bit_counter == 8'h08); // 13us after the edge
   assign burst1 = (bit_counter == 8'hB0); // 260us after the edge

   always @(posedge clk) begin
      if (clk_divider[0]) begin

         // Measure the gap between edges with an 8-bit saturating counter
         if (cas_din_edge) begin
            bit_counter <= 0;
         end else if (!(&bit_counter)) begin
            bit_counter <= bit_counter + 1'b1;
         end

         // Clock recovery, generate a burst of 4 clock pulses
         if (burst0 || burst1 || |burst_counter) begin
            burst_counter <= burst_counter + 1'b1;
         end
         if (|burst_counter) begin
            cas_clk_recovered <= !burst_counter[0];
         end else begin
            cas_clk_recovered <= 1'b1;
         end

         // Track the length of the last two gaps between edges
         if (cas_din_edge) begin
            is_long <= 1'b0;
            is_long_last <= is_long;
         end else if (burst1) begin
            is_long <= 1'b1;
         end

         // Data recovery, make the data decision on each edge
         if (cas_din_edge) begin
            if (is_long) begin
               // last gap long: output a zero
               cas_din_recovered <= ctrl_reverse_tones;
               // last two gaps short: output a one
            end else if (!is_long_last) begin
               cas_din_recovered <= !ctrl_reverse_tones;
            end
         end
      end
   end

   // =================================================
   // High Tone Run-in Detect
   // =================================================

   assign high_tone_threshold = vlsi_mode ? `HIGH_TONE_THRESHOLD_VLSI :
                                `HIGH_TONE_THRESHOLD_FERRANTI;

   always @(posedge clk) begin
      if (&clk_divider[7:0]) begin
         if (!cas_din_recovered || !ctrl_motor_on) begin
            high_tone_counter <= 0;
         end else if (!(&high_tone_counter)) begin
            high_tone_counter <= high_tone_counter + 1;
         end
         high_tone_detect <= (high_tone_counter == high_tone_threshold);
      end
   end

   // =================================================
   // Sine Wave Synthesis
   // =================================================

   // The Ferranti Serial ULA produces 4 discrete levels:
   //
   //               1200Hz 2400Hz
   // 00: 2.95V for 208us  104us
   // 01: 3.36V for 104us   52us
   // 10: 4.52V for 104us   52us
   // 11: 4.92V for 208us  104us
   // 10: 4.52V for 104us   52us
   // 01: 3.36V for 104us   52us
   //               -----  -----
   // etc           832us  416us
   //
   // At 1200 baud:
   //    TxD = 0 -> one cycle  of 1200 Hz
   //    TxD = 1 -> two cycles of 2400 Hz

   assign sine_in = txd_s ? clk_divider[8:6] : clk_divider[9:7];

   always @(posedge clk) begin
      // Sample TxD and Enable once per bit period
      if (&clk_divider[9:0]) begin
         txd_s <= TxD ^ ctrl_reverse_tones;
         enable_s <= !ctrl_rs423_sel & !RTSI;
      end
   end

   // Note: the polarity doesn't matter (the Ferranti and
   // VLSI parts actually have opposite polarities).
   //
   // Note: bit transitions (between low and high tones)
   // should happen at the zero crossings.
   //         V                                V
   //                             +-------+
   //                             |       |
   //                         +---+       +---+
   //                         |               |
   //                         |               |
   //                         |               |
   //         |               |
   //         |               |
   //         |               |
   //         +---+       +---+
   //             |       |
   //             +-------+
   //
   // Sine_in |000|001|010|011|100|101|110|111|

`ifdef BOARD_REV_01

   // Note: this change fixes a 90 degree phase shift error
   // in previous commits.
   //
   // Uses open drain drivers and a pullup (R1) to 5V
   //
   // Sine_in |000|001|010|011|100|101|110|111|
   // CasOut1 | 0 | 0 | 0 | 0 | Z | Z | Z | Z | (Pin 37 = 1K8)
   // CasOut0 | Z | 0 | 0 | Z | 0 | Z | Z | 0 | (Pin 38 = 10K)
   //
   // Output 00 when !enable_s so CasOut sits at lowest voltage

   assign CasOut[1] = (enable_s &    sine_in[2]   ) ? 1'bZ : 1'b0;
   assign CasOut[0] = (enable_s & !(^sine_in[2:0])) ? 1'bZ : 1'b0;

`endif

`ifdef BOARD_REV_02

   // Uses push-pull drivers and a bias voltage of 1.65V
   //
   // Sine_in |000|001|010|011|100|101|110|111|
   // CasOut1 | 0 | 0 | 0 | 0 | 1 | 1 | 1 | 1 | (Pin 37 = 1K8)
   // CasOut0 | Z | 0 | 0 | Z | Z | 1 | 1 | Z | (Pin 38 = 510R)
   //
   // Output ZZ when !enable_s so CasOut sits at 1.65V

   // Normal phase sine wave
   assign CasOut[1] = (enable_s                  ) ? sine_in[2] : 1'bZ;
   assign CasOut[0] = (enable_s & (^sine_in[1:0])) ? sine_in[2] : 1'bZ;

   // Normal phase square wave
   // assign CasOut[1] = (enable_s                  ) ? sine_in[2] : 1'bZ;
   // assign CasOut[0] = (enable_s                  ) ? sine_in[2] : 1'bZ;

   // Phase shifted sine wave
   // assign CasOut[1] = (enable_s                   ) ? (^sine_in[2:1]) : 1'bZ;
   // assign CasOut[0] = (enable_s & !(^sine_in[1:0])) ? (^sine_in[2:1]) : 1'bZ;

   // Phase shifted square wave
   // assign CasOut[1] = (enable_s                   ) ? (^sine_in[2:1]) : 1'bZ;
   // assign CasOut[0] = (enable_s                   ) ? (^sine_in[2:1]) : 1'bZ;

`endif


`ifdef BOARD_GODIL

   // Uses three open connector drivers with diodes and variable resistors
   //
   // Sine_in |000|001|010|011|100|101|110|111|
   // CasOut2 | Z | Z | Z | Z | 0 | Z | Z | 0 | (Pin 70 = 10K Variable) [ Intermediate high ]
   // CasOut1 | 0 | Z | Z | 0 | Z | Z | Z | Z | (Pin 67 = 4K7 Variable) [ Intermediate low ]
   // CasOut0 | Z | 0 | 0 | Z | Z | Z | Z | Z | (Pin 65 = 1K0 Variable) [ Peak low ]
   // CasOut/Common                           | (Pin 10 = 4K7 Variable) [ Peak high ]
   //
   // Output ZZ when !enable_s so CasOut goes high

   assign CasOut[2] = enable_s && (sine_in == 3'b100 || sine_in == 3'b111) ? 1'b0 : 1'bZ;
   assign CasOut[1] = enable_s && (sine_in == 3'b000 || sine_in == 3'b011) ? 1'b0 : 1'bZ;
   assign CasOut[0] = enable_s && (sine_in == 3'b001 || sine_in == 3'b010) ? 1'b0 : 1'bZ;

`endif

   // =================================================
   // Output Multiplexers
   // =================================================

   assign Dout = !TxD;
   assign TxC  = tx_clk;
   assign DCD  = ctrl_rs423_sel ? 1'b0 : high_tone_detect;
   assign RxC  = ctrl_rs423_sel ? rx_clk : cas_clk_recovered;
   assign RxD  = ctrl_rs423_sel ? !Din : cas_din_recovered;
   assign RTSO = ctrl_rs423_sel ? !RTSI : 1'b0;
   assign CTSO = ctrl_rs423_sel ? !CTSI : 1'b0;
   assign CasMotor = ctrl_motor_on;

endmodule
