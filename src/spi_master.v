// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: spi_master
// Description:
// SPI Master module for communication with an external SPI RAM.
// - Supports SPI Mode 0 (CPOL=0, CPHA=0).
// - Sends READ (0x03) or WRITE (0x02) commands.
// - Sends a 16-bit address (MSB first).
// - Transfers 1 or 2 data bytes sequentially under a single chip select.
// - Generates SPI clock (sclk) from the core clock.
// -----------------------------------------------------------------------------



module spi_master #(
  parameter SPI_MODE = 0, // Default to SPI Mode 0 (CPOL=0, CPHA=0)
                           // Note: This implementation is hardcoded for Mode 0 logic.
  parameter CLOCK_DIVIDER = 4 // Core clock cycles for one SCLK half period
                               // SCLK frequency = clk_core / (2 * CLOCK_DIVIDER)
                               // Ensure CLOCK_DIVIDER >= 2
) (
  // CPU Core Interface
  input  wire                       clk_core_i,
  input  wire                       rst_n_i,
  input  wire                       start_transaction_i,       // Pulse to initiate
  input  wire [15:0]                address_i,                 // Memory address
  input  wire [7:0]                 data_to_write_i,           // Data for WRITE op
  input  wire                       read_not_write_i,          // 1 for READ, 0 for WRITE
  input  wire [1:0]                 num_bytes_to_transfer_i,   // 2'b01 for 1 byte, 2'b10 for 2 bytes
  output reg  [7:0]                 data_read_byte1_o,
  output reg  [7:0]                 data_read_byte2_o,
  output reg                        transaction_done_o,        // Pulsed high for 1 clk_core_i cycle
  output reg                        busy_o,                    // High during transaction

  // SPI Bus Interface
  output reg                        spi_sclk_o,
  output reg                        spi_mosi_o,
  input  wire                       spi_miso_i,
  output reg                        spi_cs_o                   // Active low
);

  // SPI Commands (as per 23LC512 and spi-ram-emu)
  localparam CMD_READ  = 8'h03;
  localparam CMD_WRITE = 8'h02;

  // State machine states
  localparam ST_IDLE                 = 4'h0;
  localparam ST_START_TRANSACTION    = 4'h1;
  localparam ST_SEND_COMMAND         = 4'h2;
  localparam ST_SEND_ADDR_BYTE1      = 4'h3;
  localparam ST_SEND_ADDR_BYTE2      = 4'h4;
  localparam ST_SEND_DATA_BYTE1      = 4'h5; // For WRITE operations
  // ST_SEND_DATA_BYTE2 could be added for multi-byte writes if needed
  localparam ST_RECEIVE_DATA_BYTE1   = 4'h6; // For READ operations
  localparam ST_RECEIVE_DATA_BYTE2   = 4'h7; // For 2-byte READ operations
  localparam ST_END_TRANSACTION      = 4'h8;
  localparam ST_DONE_PULSE           = 4'h9; // To ensure transaction_done_o is a pulse

  reg [3:0] current_state_q, next_state_d;

  // Registers to latch transaction parameters
  reg [15:0] address_r;
  reg [7:0]  data_to_write_r;
  reg        read_not_write_r;
  reg [1:0]  num_bytes_to_transfer_r;
  reg [7:0]  command_r;

  // SCLK generation
  reg [($clog2(CLOCK_DIVIDER*2))-1:0] sclk_divider_cnt_q;
  reg internal_sclk_q; // Represents the SPI SCLK state

  // Bit counting for SPI transfers
  reg [2:0] bit_counter_q; // Counts 8 bits

  // Shift registers for MOSI and MISO
  reg [7:0] mosi_shift_reg_q;
  reg [7:0] miso_shift_reg_q;

  // Internal registers for read data
  reg [7:0] data_read_byte1_internal_r;
  reg [7:0] data_read_byte2_internal_r;
  
  wire sclk_rising_edge;
  wire sclk_falling_edge;
  wire sclk_tick; // Indicates one SCLK half period is done

  // SCLK Generation Logic
  // sclk_divider_cnt_q counts from 0 to (CLOCK_DIVIDER - 1) for each SCLK phase (half period)
  // internal_sclk_q toggles when sclk_divider_cnt_q reaches CLOCK_DIVIDER-1
  // sclk_tick is asserted for one clk_core_i cycle when internal_sclk_q is about to toggle
  
  assign sclk_tick = (sclk_divider_cnt_q == (CLOCK_DIVIDER - 1));

  always @(posedge clk_core_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
      sclk_divider_cnt_q <= 0;
      internal_sclk_q    <= 1'b0; // SCLK idle low for Mode 0
      spi_sclk_o         <= 1'b0;
    end else begin
      if (busy_o && current_state_q != ST_IDLE && current_state_q != ST_START_TRANSACTION && current_state_q != ST_END_TRANSACTION && current_state_q != ST_DONE_PULSE) begin
        if (sclk_divider_cnt_q == (CLOCK_DIVIDER*2 - 1)) begin
          sclk_divider_cnt_q <= 0;
        end else begin
          sclk_divider_cnt_q <= sclk_divider_cnt_q + 1;
        end

        // Toggle internal_sclk_q at half period
        if (sclk_divider_cnt_q == (CLOCK_DIVIDER - 1)) begin // End of first half (e.g. high phase)
            internal_sclk_q <= ~internal_sclk_q;
        end else if (sclk_divider_cnt_q == (CLOCK_DIVIDER*2 - 1)) begin // End of second half (e.g. low phase)
            internal_sclk_q <= ~internal_sclk_q;
        end
      end else begin
        sclk_divider_cnt_q <= 0; // Reset counter when not actively clocking
        internal_sclk_q    <= 1'b0; // Keep SCLK idle low
      end
      spi_sclk_o <= internal_sclk_q; // Output the generated SCLK
    end
  end
  
  // Detect SCLK edges based on internal_sclk_q state for SPI Mode 0 (CPOL=0, CPHA=0)
  // Data is shifted out on falling edge, sampled on rising edge.
  reg internal_sclk_prev_q;
  always @(posedge clk_core_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
        internal_sclk_prev_q <= 1'b0;
    end else begin
        internal_sclk_prev_q <= internal_sclk_q;
    end
  end

  assign sclk_rising_edge  = internal_sclk_q & ~internal_sclk_prev_q;
  assign sclk_falling_edge = ~internal_sclk_q & internal_sclk_prev_q;

  // State Machine - Sequential Part
  always @(posedge clk_core_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
      current_state_q <= ST_IDLE;
      busy_o          <= 1'b0;
      spi_cs_o        <= 1'b1; // Chip select inactive high
      bit_counter_q   <= 3'd7; // For MSB first
      // Outputs reset
      data_read_byte1_o <= 8'h00;
      data_read_byte2_o <= 8'h00;
      transaction_done_o <= 1'b0;
    end else begin
      current_state_q   <= next_state_d;
      busy_o            <= (next_state_d != ST_IDLE && next_state_d != ST_DONE_PULSE);
      spi_cs_o          <= (next_state_d == ST_IDLE || next_state_d == ST_DONE_PULSE) ? 1'b1 : 1'b0;
      transaction_done_o<= (current_state_q == ST_END_TRANSACTION && next_state_d == ST_DONE_PULSE); // Pulse for one cycle

      // Latch inputs on start
      if (current_state_q == ST_IDLE && next_state_d == ST_START_TRANSACTION) begin
        address_r               <= address_i;
        data_to_write_r         <= data_to_write_i;
        read_not_write_r        <= read_not_write_i;
        num_bytes_to_transfer_r <= num_bytes_to_transfer_i;
        command_r               <= read_not_write_i ? CMD_READ : CMD_WRITE;
      end

      // Bit counter logic (active during shifting states)
      if (sclk_rising_edge && 
         (current_state_q == ST_SEND_COMMAND || 
          current_state_q == ST_SEND_ADDR_BYTE1 || 
          current_state_q == ST_SEND_ADDR_BYTE2 ||
          current_state_q == ST_SEND_DATA_BYTE1 ||
          current_state_q == ST_RECEIVE_DATA_BYTE1 ||
          current_state_q == ST_RECEIVE_DATA_BYTE2)) begin
        if (bit_counter_q == 3'd0) begin
          // Counter will be reset when transitioning to next data byte/phase
        end else begin
          bit_counter_q <= bit_counter_q - 1;
        end
      end
      
      // MOSI shift register logic (data loaded before state, shifted on falling edge for mode 0)
      if (next_state_d == ST_SEND_COMMAND && current_state_q == ST_START_TRANSACTION) begin
        mosi_shift_reg_q <= read_not_write_r ? CMD_READ : CMD_WRITE;
      end else if (next_state_d == ST_SEND_ADDR_BYTE1 && current_state_q == ST_SEND_COMMAND) begin
        mosi_shift_reg_q <= address_r[15:8];
      end else if (next_state_d == ST_SEND_ADDR_BYTE2 && current_state_q == ST_SEND_ADDR_BYTE1) begin
        mosi_shift_reg_q <= address_r[7:0];
      end else if (next_state_d == ST_SEND_DATA_BYTE1 && current_state_q == ST_SEND_ADDR_BYTE2) begin
        mosi_shift_reg_q <= data_to_write_r;
      end else if (sclk_falling_edge && 
                  (current_state_q == ST_SEND_COMMAND || 
                   current_state_q == ST_SEND_ADDR_BYTE1 || 
                   current_state_q == ST_SEND_ADDR_BYTE2 ||
                   current_state_q == ST_SEND_DATA_BYTE1)) begin
        mosi_shift_reg_q <= {mosi_shift_reg_q[6:0], 1'b0}; // Shift left, MSB sent first
      end
      
      // MISO shift register logic & data latching (sampled on rising edge for mode 0)
      if (sclk_rising_edge && 
         (current_state_q == ST_RECEIVE_DATA_BYTE1 || current_state_q == ST_RECEIVE_DATA_BYTE2)) begin
        miso_shift_reg_q <= {miso_shift_reg_q[6:0], spi_miso_i};
      end
      
      // Latch received bytes
      if (current_state_q == ST_RECEIVE_DATA_BYTE1 && bit_counter_q == 3'd0 && sclk_rising_edge) begin
        data_read_byte1_internal_r <= miso_shift_reg_q;
      end
      if (current_state_q == ST_RECEIVE_DATA_BYTE2 && bit_counter_q == 3'd0 && sclk_rising_edge) begin
        data_read_byte2_internal_r <= miso_shift_reg_q;
      end
      
      // Assign to outputs
      if (current_state_q == ST_END_TRANSACTION && next_state_d == ST_DONE_PULSE) begin
        data_read_byte1_o <= data_read_byte1_internal_r;
        data_read_byte2_o <= data_read_byte2_internal_r;
      end else if (next_state_d == ST_IDLE) begin // Clear when going back to idle after pulse
        data_read_byte1_o <= 8'h00;
        data_read_byte2_o <= 8'h00;
      end
    end
  end

  // State Machine - Combinational Part (Next State Logic)
  always @* begin
    next_state_d = current_state_q; // Default: stay in current state
    spi_mosi_o   = mosi_shift_reg_q[7]; // MSB out for MOSI

    case (current_state_q)
      ST_IDLE: begin
        if (start_transaction_i) begin
          next_state_d = ST_START_TRANSACTION;
        end
      end
      ST_START_TRANSACTION: begin
        next_state_d  = ST_SEND_COMMAND;
        // bit_counter_q reset handled by state transition logic below
      end
      ST_SEND_COMMAND: begin
        if (sclk_rising_edge && bit_counter_q == 3'd0) begin // 8 bits sent
          next_state_d = ST_SEND_ADDR_BYTE1;
          // bit_counter_q reset handled by state transition logic below
        end
      end
      ST_SEND_ADDR_BYTE1: begin
        if (sclk_rising_edge && bit_counter_q == 3'd0) begin // 8 bits sent
          next_state_d = ST_SEND_ADDR_BYTE2;
          // bit_counter_q reset handled by state transition logic below
        end
      end
      ST_SEND_ADDR_BYTE2: begin
        if (sclk_rising_edge && bit_counter_q == 3'd0) begin // 8 bits sent
          if (read_not_write_r) begin
            next_state_d = ST_RECEIVE_DATA_BYTE1;
          end else begin // WRITE
            next_state_d = ST_SEND_DATA_BYTE1;
          end
          // bit_counter_q reset handled by state transition logic below
        end
      end
      ST_SEND_DATA_BYTE1: begin // For WRITE
        if (sclk_rising_edge && bit_counter_q == 3'd0) begin // 8 bits sent
          // Assuming 1 byte write based on num_bytes_to_transfer_r for now
          // If num_bytes_to_transfer_r == 2 for write, more states needed
          next_state_d = ST_END_TRANSACTION;
        end
      end
      ST_RECEIVE_DATA_BYTE1: begin // For READ
        if (sclk_rising_edge && bit_counter_q == 3'd0) begin // 8 bits received
          if (num_bytes_to_transfer_r == 2'b01 || num_bytes_to_transfer_r == 2'b00 ) begin // 1 byte total
            next_state_d = ST_END_TRANSACTION;
          end else if (num_bytes_to_transfer_r == 2'b10) begin // 2 bytes total
            next_state_d = ST_RECEIVE_DATA_BYTE2;
            // bit_counter_q reset handled by state transition logic below
          end else begin // Should not happen with 2-bit num_bytes
            next_state_d = ST_END_TRANSACTION; 
          end
        end
      end
      ST_RECEIVE_DATA_BYTE2: begin // For 2-byte READ
        if (sclk_rising_edge && bit_counter_q == 3'd0) begin // 8 bits received
          next_state_d = ST_END_TRANSACTION;
        end
      end
      ST_END_TRANSACTION: begin
        next_state_d = ST_DONE_PULSE;
      end
      ST_DONE_PULSE: begin
        next_state_d = ST_IDLE;
      end
      default: begin
        next_state_d = ST_IDLE;
      end
    endcase
    
    // Reset bit counter for next byte/phase, done in combinational logic based on next_state_d
    // to ensure it's ready for the new state's first bit.
    if ((next_state_d == ST_SEND_COMMAND && current_state_q != ST_SEND_COMMAND) ||
        (next_state_d == ST_SEND_ADDR_BYTE1 && current_state_q != ST_SEND_ADDR_BYTE1) ||
        (next_state_d == ST_SEND_ADDR_BYTE2 && current_state_q != ST_SEND_ADDR_BYTE2) ||
        (next_state_d == ST_SEND_DATA_BYTE1 && current_state_q != ST_SEND_DATA_BYTE1) ||
        (next_state_d == ST_RECEIVE_DATA_BYTE1 && current_state_q != ST_RECEIVE_DATA_BYTE1) ||
        (next_state_d == ST_RECEIVE_DATA_BYTE2 && current_state_q != ST_RECEIVE_DATA_BYTE2)) begin
      // This assignment will be seen by bit_counter_q on the next clock edge
      // However, for immediate use in the *next* cycle's bit shifting,
      // this needs careful thought or bit_counter is directly assigned based on next_state_d.
      // The sequential block already handles bit_counter_q update.
      // The primary role here is to ensure that when a new shifting state is entered,
      // the bit counter will be at its starting position (7 for MSB first).
      // The sequential block already resets to 7 when not actively decrementing.
      // Let's ensure it gets set to 7 when transitioning TO a shifting state.
      // This is implicitly handled if bit_counter_q is only decremented when active.
      // If we reset bit_counter_q in the sequential block upon state change to a shifting state:
      // This logic might be redundant or could be simplified in the sequential part.
    end
  end
  
  // Sequential assignment for bit_counter_q based on state transitions
  always @(posedge clk_core_i or negedge rst_n_i) begin
      if(!rst_n_i) begin
          bit_counter_q <= 3'd7;
      end else begin
          if ((current_state_q == ST_IDLE && next_state_d == ST_START_TRANSACTION) || // About to start sending cmd
              (current_state_q == ST_SEND_COMMAND && next_state_d == ST_SEND_ADDR_BYTE1) || // Cmd sent, about to send addr1
              (current_state_q == ST_SEND_ADDR_BYTE1 && next_state_d == ST_SEND_ADDR_BYTE2) || // Addr1 sent, about to send addr2
              (current_state_q == ST_SEND_ADDR_BYTE2 && (next_state_d == ST_SEND_DATA_BYTE1 || next_state_d == ST_RECEIVE_DATA_BYTE1)) || // Addr2 sent, about to send/recv data1
              (current_state_q == ST_RECEIVE_DATA_BYTE1 && next_state_d == ST_RECEIVE_DATA_BYTE2) // Data1 recvd, about to recv data2
             ) begin
              bit_counter_q <= 3'd7; // Reset for next 8-bit phase
          end else if (sclk_rising_edge && 
             (current_state_q == ST_SEND_COMMAND || 
              current_state_q == ST_SEND_ADDR_BYTE1 || 
              current_state_q == ST_SEND_ADDR_BYTE2 ||
              current_state_q == ST_SEND_DATA_BYTE1 ||
              current_state_q == ST_RECEIVE_DATA_BYTE1 ||
              current_state_q == ST_RECEIVE_DATA_BYTE2)) begin
            if (bit_counter_q != 3'd0) begin // Only decrement if not already zero
              bit_counter_q <= bit_counter_q - 1;
            end
          end else if (next_state_d == ST_IDLE) begin // When going to IDLE fully
             bit_counter_q <= 3'd7; 
          end
      end
  end

endmodule