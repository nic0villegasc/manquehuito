verilog
`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>
// -----------------------------------------------------------------------------
// Module: manquehuito_domain
// Description: Top-level SoC integrating the manquehuito core with an SPI
//              master to access external instruction and data memory.
// -----------------------------------------------------------------------------
module manquehuito_domain #() (
  input  logic clk_core_i,
  input  logic rst_n_i,

  output logic spi_sclk_o,
  output logic spi_mosi_o,
  output logic spi_cs_o,
  input  logic spi_miso_i
);

  // --- Internal Wires and Registers ---
  logic [7:0]  pc_out;
  [cite_start]logic [7:0]  reg_a_out, reg_b_out; [cite: 90]
  [cite_start]logic [7:0]  mux_a_out, mux_b_out, mux_d_out; [cite: 90]
  [cite_start]logic [7:0]  alu_out; [cite: 90]
  [cite_start]logic [3:0]  alu_zncv; [cite: 90]
  [cite_start]logic [11:0] ctrl_out; [cite: 91]
  [cite_start]logic [3:0]  status_out; [cite: 91]


  // SPI Master Interface Signals
  logic        spi_start;
  logic [15:0] spi_address;
  logic [7:0]  spi_data_write;
  logic        spi_read_not_write;
  logic [1:0]  spi_num_bytes;
  wire  [7:0]  spi_data_read_byte1;
  wire  [7:0]  spi_data_read_byte2;
  wire         spi_done;
  wire         spi_busy;

  // CPU Control & Instruction Fetch Logic
  logic        cpu_stall_o; // Stalls the PC while waiting for memory
  reg [14:0]   current_instruction_r; // Holds the fetched instruction

  // State machine for fetching 2 bytes for one instruction
  typedef enum logic [1:0] {
    IFETCH_IDLE,
    IFETCH_START_READ,
    IFETCH_WAIT_DONE
  } ifetch_state_t;
  ifetch_state_t ifetch_state_q, ifetch_state_d;


  // --- Sub-module Instantiations ---

  // PC now has a stall input to pause it during memory access
  pc i_pc (
    .clk_i(clk_core_i),
    .rst_n_i(rst_n_i), Reset connection
    .stall_i(cpu_stall_o), Stall connection
    .load_i(ctrl_out[11]),
    .im_i(current_instruction_r[7:0]), // Jump address from new instruction reg
    .pc_o(pc_out)
  );

  // Control unit now fed from the new instruction register
  control_unit i_ctrl (
    .opcode_i(current_instruction_r[14:8]),
    .zncv_i(status_out),
    .out_o(ctrl_out)
  );

  // Registers A and B are unchanged
  register i_reg_a (
    .clk_i(clk_core_i),
    .load_i(ctrl_out[8]),
    .data_i(alu_out),
    .out_o(reg_a_out)
  );

  register i_reg_b (
    .clk_i(clk_core_i),
    .load_i(ctrl_out[7]),
    .data_i(alu_out),
    .out_o(reg_b_out)
  );

  // MUXes A and D are modified to use the new instruction register
  muxA i_mux_a (
    .e0_i(reg_a_out),
    .e1_i(8'h01),
    .e2_i(8'h00),
    .e3_i(reg_b_out),
    .sel_i(ctrl_out[6:5]),
    .out_o(mux_a_out)
  );

  // MUX B sources are changed
  muxB i_mux_b (
    .e0_i(reg_b_out),
    .e1_i(spi_data_read_byte1),      // Data from SPI for LOADs
    .e2_i(current_instruction_r[7:0]), // Immediate from new instruction reg
    .e3_i(8'h00),
    .sel_i(ctrl_out[4:3]),
    .out_o(mux_b_out)
  );

  // MUX D source is changed
  muxD i_mux_d (
    .e0_i(current_instruction_r[7:0]), // Address from new instruction reg
    .e1_i(reg_b_out),
    .sel_i(ctrl_out[9]),
    .out_o(mux_d_out)
  );

  // ALU and Status are unchanged
  alu i_alu (
    .a_i(mux_a_out),
    .b_i(mux_b_out),
    .sel_i(ctrl_out[2:0]),
    .out_o(alu_out),
    .zncv_o(alu_zncv)
  );

  status i_status (
    .clk_i(clk_core_i),
    .zncv_i(alu_zncv),
    .out_o(status_out)
  );


  // SPI Master instantiation
  spi_master #(
    .CLOCK_DIVIDER(4)
  ) i_spi_master (
    .clk_core_i               (clk_core_i),
    .rst_n_i                  (rst_n_i),
    .start_transaction_i      (spi_start),
    .address_i                (spi_address),
    .data_to_write_i          (spi_data_write),
    .read_not_write_i         (spi_read_not_write),
    .num_bytes_to_transfer_i  (spi_num_bytes),
    .data_read_byte1_o        (spi_data_read_byte1),
    .data_read_byte2_o        (spi_data_read_byte2),
    .transaction_done_o       (spi_done),
    .busy_o                   (spi_busy),
    .spi_sclk_o               (spi_sclk_o),
    .spi_mosi_o               (spi_mosi_o),
    .spi_miso_i               (spi_miso_i),
    .spi_cs_o                 (spi_cs_o)
  );

  // Instruction Fetch Control Logic
  // This logic is responsible for fetching instructions from SPI RAM.
  // A more complete design would multiplex this with data access logic for LOAD/STORE.
  assign spi_address = {8'd0, pc_out}; // For now, only PC drives the address bus

  // Sequential part of the Instruction Fetch FSM
  always_ff @(posedge clk_core_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
      ifetch_state_q <= IFETCH_IDLE;
    end else begin
      ifetch_state_q <= ifetch_state_d;
    end
  end

  // Combinational part of the Instruction Fetch FSM
  always_comb begin
    // Default assignments
    ifetch_state_d     = ifetch_state_q;
    spi_start          = 1'b0;
    spi_num_bytes      = 2'b10; // Fetch 2 bytes to form one 15-bit instruction
    spi_read_not_write = 1'b1;  // Instruction fetch is always a read
    cpu_stall_o        = 1'b1;  // Default to stalling the CPU while fetching

    // Placeholder for data memory access
    // TO-DO: A higher-level FSM will decide if the memory access is for
    // an instruction fetch or a data LOAD/STORE operation.
    // For LOAD/STORE, spi_address would come from alu_out, spi_read_not_write
    // would be controlled by ctrl_out, etc.
    assign spi_data_write = alu_out; // Data for STORE operations comes from ALU

    case (ifetch_state_q)
      IFETCH_IDLE: begin
        cpu_stall_o = 1'b0; // CPU is not stalled, ready to execute/start fetch
        ifetch_state_d = IFETCH_START_READ;
      end

      IFETCH_START_READ: begin
        // Start a 2-byte read from the address given by the PC
        spi_start = 1'b1;
        if (spi_busy) begin // Once transaction starts, move to waiting state
          ifetch_state_d = IFETCH_WAIT_DONE;
        end
      end

      IFETCH_WAIT_DONE: begin
        // Wait for the spi_master to signal the transaction is complete
        if (spi_done) begin
          ifetch_state_d = IFETCH_IDLE; // Done, go back to idle
        end
      end

      default: begin
        ifetch_state_d = IFETCH_IDLE;
      end
    endcase
  end

  // Instruction Assembly Logic
  always_ff @(posedge clk_core_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
      current_instruction_r <= 15'b0;
    end else begin
      // When the SPI master is done with its 2-byte read, latch the result
      // into the 15-bit instruction register.
      if (spi_done) begin
        current_instruction_r[14:7] <= spi_data_read_byte1;
        current_instruction_r[6:0]  <= spi_data_read_byte2[6:0]; // Use 7 bits of the second byte
      end
    end
  end

endmodule : manquehuito_domain
