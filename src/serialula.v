`define HIGH_TONE_THRESHOLD 445

module serialula
  (
   // Fast clock (16/13 MHz)
   input            clk,

   // Interface to 6502
   input            E,
   input [7:0]      Data,
   input            nCS,

   // Interface to Cassette Port
   output           CasMotor,
   input            CasIn,
   output [1:0]     CasOut,

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

   reg [7:0]        control;

   wire [2:0]       ctrl_tx_baud       = control[2:0];
   wire [2:0]       ctrl_rx_baud       = control[5:3];
   wire             ctrl_reverse_tones = control[3];
   wire             ctrl_rs423_sel     = control[6];
   wire             ctrl_motor_on      = control[7];

   reg [9:0]        clk_divider;
   reg              tx_clk;
   reg              rx_clk;
   wire [2:0]       sine_in;
   reg [2:0]        burst_counter;
   reg [8:0]        high_tone_counter;
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
   reg [1:0]        sine_out;

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

   always @(posedge clk) begin
      if (&clk_divider[7:0]) begin
         if (!cas_din_recovered) begin
            high_tone_counter <= 0;
         end else if (!(&high_tone_counter)) begin
            high_tone_counter <= high_tone_counter + 1;
         end
         high_tone_detect <= (high_tone_counter == `HIGH_TONE_THRESHOLD);
      end
   end

   // =================================================
   // Sine Wave Synthesis
   // =================================================

   // Produces 4 discrete levels:
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
      // TODO: does this needs to be better synchronised?
      if (&clk_divider[9:0]) begin
         txd_s <= TxD ^ ctrl_reverse_tones;
         enable_s <= !ctrl_rs423_sel & !RTSI;
      end
      if (enable_s) begin
         case (sine_in)
           3'b000: sine_out <= 2'b00;
           3'b001: sine_out <= 2'b01;
           3'b010: sine_out <= 2'b10;
           3'b011: sine_out <= 2'b11;
           3'b100: sine_out <= 2'b11;
           3'b101: sine_out <= 2'b10;
           3'b110: sine_out <= 2'b01;
           3'b111: sine_out <= 2'b00;
         endcase
      end else begin
         sine_out <= 2'b00;
      end
   end

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
   assign CasOut[1] = sine_out[1] ? 1'bZ : 1'b0;
   assign CasOut[0] = sine_out[0] ? 1'bZ : 1'b0;

endmodule
