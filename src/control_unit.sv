`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: control_unit
// Description: Sequential FSM control unit. Manages instruction fetch,
//              decode, execution, and SPI-based memory access.
// -----------------------------------------------------------------------------
module control_unit (
  input logic         clk_i,
  input logic         rst_n_i,
  input logic [7:0]   pc_i,
  input logic [7:0]   reg_b_in_i,
  input logic [7:0]   mux_d_out_i,
  input logic [3:0]   zncv_i,

  input logic         spi_done_i,
  input logic         spi_busy_i,
  input logic [7:0]   spi_data_read_byte1_i,
  input logic [7:0]   spi_data_read_byte2_i,
  output logic        spi_start_o,
  output logic [15:0] spi_address_o,
  output logic [7:0]  spi_data_write_o,
  output logic        spi_read_not_write_o,
  output logic [1:0]  spi_num_bytes_o,

  output logic        cpu_stall_o,
  output logic        pc_load_o,
  output logic        reg_a_load_o,
  output logic        reg_b_load_o,
  output logic [1:0]  mux_a_sel_o,
  output logic [1:0]  mux_b_sel_o,
  output logic        mux_d_sel_o,
  output logic [2:0]  alu_sel_o
);

   // Define parameters for ALU operations for readability
   localparam ALU_ADD = 3'b000;
   localparam ALU_SUB = 3'b001;
   localparam ALU_AND = 3'b010;
   localparam ALU_OR  = 3'b011;
   localparam ALU_NOT = 3'b100;
   localparam ALU_XOR = 3'b101;
   localparam ALU_SHL = 3'b110;
   localparam ALU_SHR = 3'b111;

   // Define parameters for MUX A selections for readability
   localparam MUX_A_SEL_REGA = 2'b00; // Selects output of Register A
   localparam MUX_A_SEL_CONST_1 = 2'b01; // Selects constant value 1
   localparam MUX_A_SEL_CONST_0 = 2'b10; // Selects constant value 0
   localparam MUX_A_SEL_REGB = 2'b11; // Unused in this instruction set

   typedef enum logic [2:0] {
                            RESET,
                            IFETCH_START,
                            IFETCH_WAIT,
                            DECODE_EXECUTE,
                            MEM_ACCESS_START,
                            MEM_ACCESS_WAIT
                            } state_t;

   state_t state_q, state_d;

   reg [14:0] instruction_r;
   reg [15:0] sp_q, sp_d;

   always_ff @(posedge clk_i or negedge rst_n_i) begin
     if(!rst_n_i) begin
       state_q       <= RESET;
       sp_q          <= 16'hFFFF;
       instruction_r <= 15'b0;

     end else begin
       state_q <= state_d;
       sp_q    <= sp_d;

       if (state_q == IFETCH_WAIT && spi_done_i) begin
         instruction_r[14:7] <= spi_data_read_byte1_i;
         instruction_r[6:0]  <= spi_data_read_byte2_i[6:0];

       end
     end

   end // always_ff @ (posedge clk_i or negedge rst_n_i)

   always_comb begin
     state_d              = state_q;
     sp_d                 = sp_q;

     cpu_stall_o          = 1'b0;
     pc_load_o            = 1'b0;

     reg_a_load_o         = 1'b0;
     reg_b_load_o         = 1'b0;
     mux_a_sel_o          = 2'b00;
     mux_b_sel_o          = 2'b00;
     mux_d_sel_o          = 1'b0;
     alu_sel_o            = 3'b000;
     spi_start_o          = 1'b0;
     spi_address_o        = 16'b0;
     spi_data_write_o     = 8'b0;
     spi_read_not_write_o = 1'b1; // Default to read
     spi_num_bytes_o      = 2'b00;

     case (state_q)
       RESET: begin
         cpu_stall_o = 1'b1;
         state_d     = IFETCH_START;
       end

       IFETCH_START: begin
         cpu_stall_o          = 1'b1;
         spi_start_o          = 1'b1;
         spi_address_o        = {8'd0, pc_i};
         spi_num_bytes_o      = 2'b10;
         spi_read_not_write_o = 1'b1;

         if (!spi_busy_i) begin
           state_d = IFETCH_WAIT;
         end
       end

       IFETCH_WAIT: begin
         cpu_stall_o = 1'b1;

         if (spi_done_i) begin
           state_d = DECODE_EXECUTE;
         end
       end
       DECODE_EXECUTE: begin
         state_d = IFETCH_START;

         case (instruction_r[14:8])

           // --- Category 1: ALU/Register Ops ---
           // ALU and Register Operations
           7'b0000000: begin // MOV A, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b10; // Input A from Constant 0
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b000; // OP: A + B
           end
           7'b0000001: begin // MOV B, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b11; // Input B from Constant 0
             alu_sel_o    = 3'b000; // OP: A + B
           end
           7'b0000010: begin // MOV A, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b10; // Input A from Constant 0
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b000; // OP: A + B
           end
           7'b0000011: begin // MOV B, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b10; // Input A from Constant 0
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b000; // OP: A + B
           end
           7'b0000100: begin // ADD A, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b000; // OP: A + B
           end
           7'b0000101: begin // ADD B, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b000; // OP: A + B
           end
           7'b0000110: begin // ADD A, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b000; // OP: A + B
           end
           7'b0000111: begin // ADD B, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b000; // OP: A + B
           end
           7'b0001000: begin // SUB A, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b001; // OP: A - B
           end
           7'b0001001: begin // SUB B, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b001; // OP: A - B
           end
           7'b0001010: begin // SUB A, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b001; // OP: A - B
           end
           7'b0001011: begin // SUB B, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b001; // OP: A - B
           end
           7'b0001100: begin // AND A, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b010; // OP: A & B
           end
           7'b0001101: begin // AND B, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b010; // OP: A & B
           end
           7'b0001110: begin // AND A, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b010; // OP: A & B
           end
           7'b0001111: begin // AND B, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b010; // OP: A & B
           end
           7'b0010000: begin // OR A, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b011; // OP: A | B
           end
           7'b0010001: begin // OR B, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b011; // OP: A | B
           end
           7'b0010010: begin // OR A, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b011; // OP: A | B
           end
           7'b0010011: begin // OR B, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b011; // OP: A | B
           end
           7'b0010100: begin // NOT A, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b100; // OP: ~A
           end
           7'b0010101: begin // NOT A, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b100; // OP: ~A
           end
           7'b0010110: begin // NOT B, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b100; // OP: ~A
           end
           7'b0010111: begin // NOT B, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b100; // OP: ~A
           end
           7'b0011000: begin // XOR A, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b101; // OP: A ^ B
           end
           7'b0011001: begin // XOR B, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b101; // OP: A ^ B
           end
           7'b0011010: begin // XOR A, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b101; // OP: A ^ B
           end
           7'b0011011: begin // XOR B, Lit
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'b10; // Input B from Immediate
             alu_sel_o    = 3'b101; // OP: A ^ B
           end
           7'b0011100: begin // SHL A, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b110; // OP: A << 1
           end
           7'b0011101: begin // SHL A, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b110; // OP: A << 1
           end
           7'b0011110: begin // SHL B, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b110; // OP: A << 1
           end
           7'b0011111: begin // SHL B, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b110; // OP: A << 1
           end
           7'b0100000: begin // SHR A, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b111; // OP: A >> 1
           end
           7'b0100001: begin // SHR A, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b1;
             reg_b_load_o = 1'b0;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b111; // OP: A >> 1
           end
           7'b0100010: begin // SHR B, A
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b00; // Input A from Register A
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b111; // OP: A >> 1
           end
           7'b0100011: begin // SHR B, B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b11; // Input A from Register B
             mux_b_sel_o  = 2'bXX; // Input B is Don't Care
             alu_sel_o    = 3'b111; // OP: A >> 1
           end
           7'b0100100: begin // INC B
             pc_load_o    = 1'b0;
             reg_a_load_o = 1'b0;
             reg_b_load_o = 1'b1;
             mux_a_sel_o  = 2'b01; // Input A from Constant 1
             mux_b_sel_o  = 2'b00; // Input B from Register B
             alu_sel_o    = 3'b000; // OP: A + B
           end

           // --- Category 2: Memory Ops ---
           // 7'b0100101: begin // MOV A,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0100110: begin // MOV B,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0100111: begin // MOV (Dir),A
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0101000: begin // MOV (Dir),B
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0101001: begin // MOV A,(B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0101010: begin // MOV B,(B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0101011: begin // MOV (B),A
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0101100: begin // ADD A,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0101101: begin // ADD B,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0101110: begin // ADD A,(B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0101111: begin // ADD (Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0110000: begin // SUB A,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0110001: begin // SUB B,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0110010: begin // SUB A,(B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0110011: begin // SUB (Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0110100: begin // AND A,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0110101: begin // AND B,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0110110: begin // AND A,(B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0110111: begin // AND (Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0111000: begin // OR A,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0111001: begin // OR B,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0111010: begin // OR A,(B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0111011: begin // OR (Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0111100: begin // NOT (Dir),A
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0111101: begin // NOT (Dir),B
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0111110: begin // NOT (B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b0111111: begin // XOR A,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1000000: begin // XOR B,(Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1000001: begin // XOR A,(B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1000010: begin // XOR (Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1000011: begin // SHL (Dir),A
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1000100: begin // SHL (Dir),B
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1000101: begin // SHL (B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1000110: begin // SHR (Dir),A
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1000111: begin // SHR (Dir),B
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1001000: begin // SHR (B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1001001: begin // INC (Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1001010: begin // INC (B)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1001011: begin // RST (Dir)
           //   state_d = MEM_ACCESS_START;
           // end
           // 7'b1001100: begin // RST (B)
           //   state_d = MEM_ACCESS_START;
           // end


           // --- Category 3: Jump/Branch Ops ---

           default: begin
             state_d = IFETCH_START;
           end
         endcase

       end
       MEM_ACCESS_START: begin
         cpu_stall_o   = 1'b1; // Stall the PC
         spi_start_o   = 1'b1; // Start SPI transaction

         // The address comes from the datapath's Mux D
         spi_address_o = {8'd0, mux_d_out_i};

         case (instruction_r[14:8])
           // --- MOV Instructions ---
           7'b0100101: begin // MOV A,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0100110: begin // MOV B,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0100111: begin // MOV (Dir),A -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
           end
           7'b0101000: begin // MOV (Dir),B -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
           end
           7'b0101001: begin // MOV A,(B) -> Load
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0101010: begin // MOV B,(B) -> Load
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0101011: begin // MOV (B),A -> Store
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
           end

           // --- ADD Instructions ---
           7'b0101100: begin // ADD A,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0101101: begin // ADD B,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0101110: begin // ADD A,(B) -> Load
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0101111: begin // ADD (Dir) -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result, not handled here
           end

           // --- SUB Instructions ---
           7'b0110000: begin // SUB A,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0110001: begin // SUB B,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0110010: begin // SUB A,(B) -> Load
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0110011: begin // SUB (Dir) -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end

           // --- AND Instructions ---
           7'b0110100: begin // AND A,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0110101: begin // AND B,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0110110: begin // AND A,(B) -> Load
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0110111: begin // AND (Dir) -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end

           // --- OR Instructions ---
           7'b0111000: begin // OR A,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0111001: begin // OR B,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0111010: begin // OR A,(B) -> Load
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b0111011: begin // OR (Dir) -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end

           // --- NOT Instructions ---
           7'b0111100: begin // NOT (Dir),A -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end
           7'b0111101: begin // NOT (Dir),B -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end
           7'b0111110: begin // NOT (B) -> Store
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end

           // --- XOR Instructions ---
           7'b0111111: begin // XOR A,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b1000000: begin // XOR B,(Dir) -> Load
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b1000001: begin // XOR A,(B) -> Load
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b1; // DW = 0 (Load)
           end
           7'b1000010: begin // XOR (Dir) -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end

           // --- SHL Instructions ---
           7'b1000011: begin // SHL (Dir),A -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end
           7'b1000100: begin // SHL (Dir),B -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end
           7'b1000101: begin // SHL (B) -> Store
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end

           // --- SHR Instructions ---
           7'b1000110: begin // SHR (Dir),A -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end
           7'b1000111: begin // SHR (Dir),B -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end
           7'b1001000: begin // SHR (B) -> Store
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // spi_data_write_o is driven by ALU result
           end

           // --- INC Instructions ---
           7'b1001001: begin // INC (Dir) -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // This is a read-modify-write operation.
             // spi_data_write_o is driven by ALU result.
           end
           7'b1001010: begin // INC (B) -> Store
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
             // This is a read-modify-write operation.
             // spi_data_write_o is driven by ALU result.
           end

           // --- RST Instructions ---
           7'b1001011: begin // RST (Dir) -> Store
             mux_d_sel_o          = 1'b0; // SD0 = Lit
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
           end
           7'b1001100: begin // RST (B) -> Store
             mux_d_sel_o          = 1'b1; // SD0 = B
             spi_read_not_write_o = 1'b0; // DW = 1 (Store)
           end

           // --- Default Case ---
           // For any opcodes not related to memory access, the default
           // assignments at the top of the always block will be used.
           default: begin
             // Keep default assignments
           end
         endcase // case (instruction_r[14:8])

         if (!spi_busy_i) begin
           state_d = MEM_ACCESS_WAIT;
         end
       end
       MEM_ACCESS_WAIT: begin
         cpu_stall_o = 1'b1;

         reg_a_load_o = 1'b0;
         reg_b_load_o = 1'b0;
         alu_sel_o    = ALU_ADD; // Default to ADD (or another safe operation)
         mux_a_sel_o  = MUX_A_SEL_CONST_0; // Default to 0
         mux_b_sel_o  = 2'b01; // This is fixed to select memory data input for all these ops

         if (spi_done_i) begin
           // The data transfer is done, so we can finish the instruction
           // and go back to fetch the next one.
           state_d = IFETCH_START;

           case (instruction_r[14:8])
             7'b0100101: begin // MOV A,(Dir)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_CONST_0; // SA0=Z
             end
             7'b0100110: begin // MOV B,(Dir)
               reg_b_load_o = 1'b1; // LB=1
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_CONST_0; // SA0=Z
             end
             7'b0100111: begin // MOV (Dir),A
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b0101000: begin // MOV (Dir),B
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_CONST_0; // SA0=Z
             end
             7'b0101001: begin // MOV A,(B)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_CONST_0; // SA0=Z
             end
             7'b0101010: begin // MOV B,(B)
               reg_b_load_o = 1'b1; // LB=1
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_CONST_0; // SA0=Z
             end
             7'b0101011: begin // MOV (B),A
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end

             // --- ADD Instructions ---
             7'b0101100: begin // ADD A,(Dir)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b0101101: begin // ADD B,(Dir)
               reg_b_load_o = 1'b1; // LB=1
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_REGB; // SA0=B
             end
             7'b0101110: begin // ADD A,(B)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b0101111: begin // ADD (Dir)
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end

             // --- SUB Instructions ---
             7'b0110000: begin // SUB A,(Dir)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_SUB;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b0110001: begin // SUB B,(Dir)
               reg_b_load_o = 1'b1; // LB=1
               alu_sel_o    = ALU_SUB;
               mux_a_sel_o  = MUX_A_SEL_REGB; // SA0=B
             end
             7'b0110010: begin // SUB A,(B)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_SUB;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b0110011: begin // SUB (Dir)
               alu_sel_o    = ALU_SUB;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end

             // --- AND Instructions ---
             7'b0110100: begin // AND A,(Dir)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_AND;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b0110101: begin // AND B,(Dir)
               reg_b_load_o = 1'b1; // LB=1
               alu_sel_o    = ALU_AND;
               mux_a_sel_o  = MUX_A_SEL_REGB; // SA0=B
             end
             7'b0110110: begin // AND A,(B)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_AND;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b0110111: begin // AND (Dir)
               alu_sel_o    = ALU_AND;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end

             // --- OR Instructions ---
             7'b0111000: begin // OR A,(Dir)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_OR;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b0111001: begin // OR B,(Dir)
               reg_b_load_o = 1'b1; // LB=1
               alu_sel_o    = ALU_OR;
               mux_a_sel_o  = MUX_A_SEL_REGB; // SA0=B
             end
             7'b0111010: begin // OR A,(B)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_OR;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b0111011: begin // OR (Dir)
               alu_sel_o    = ALU_OR;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end

             // --- NOT Instructions ---
             7'b0111100: begin // NOT (Dir),A
               alu_sel_o    = ALU_NOT;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b0111101: begin // NOT (Dir),B
               alu_sel_o    = ALU_NOT;
               mux_a_sel_o  = MUX_A_SEL_REGB; // SA0=B
             end
             7'b0111110: begin // NOT (B)
               alu_sel_o    = ALU_NOT;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end

             // --- XOR Instructions ---
             7'b0111111: begin // XOR A,(Dir)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_XOR;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b1000000: begin // XOR B,(Dir)
               reg_b_load_o = 1'b1; // LB=1
               alu_sel_o    = ALU_XOR;
               mux_a_sel_o  = MUX_A_SEL_REGB; // SA0=B
             end
             7'b1000001: begin // XOR A,(B)
               reg_a_load_o = 1'b1; // LA=1
               alu_sel_o    = ALU_XOR;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b1000010: begin // XOR (Dir)
               alu_sel_o    = ALU_XOR;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end

             // --- SHL Instructions ---
             7'b1000011: begin // SHL (Dir),A
               alu_sel_o    = ALU_SHL;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b1000100: begin // SHL (Dir),B
               alu_sel_o    = ALU_SHL;
               mux_a_sel_o  = MUX_A_SEL_REGB; // SA0=B
             end
             7'b1000101: begin // SHL (B)
               alu_sel_o    = ALU_SHL;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end

             // --- SHR Instructions ---
             7'b1000110: begin // SHR (Dir),A
               alu_sel_o    = ALU_SHR;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end
             7'b1000111: begin // SHR (Dir),B
               alu_sel_o    = ALU_SHR;
               mux_a_sel_o  = MUX_A_SEL_REGB; // SA0=B
             end
             7'b1001000: begin // SHR (B)
               alu_sel_o    = ALU_SHR;
               mux_a_sel_o  = MUX_A_SEL_REGA; // SA0=A
             end

             // --- INC Instructions ---
             7'b1001001: begin // INC (Dir)
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_CONST_1; // SA0=U (Unit/1)
             end
             7'b1001010: begin // INC (B)
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_CONST_1; // SA0=U (Unit/1)
             end

             // --- RST Instructions ---
             7'b1001011: begin // RST (Dir)
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_CONST_0; // SA0=Z (Zero)
             end
             7'b1001100: begin // RST (B)
               alu_sel_o    = ALU_ADD;
               mux_a_sel_o  = MUX_A_SEL_CONST_0; // SA0=Z (Zero)
             end

             default: begin

             end
           endcase
         end
       end

       default: begin
         state_d = RESET;
       end
     endcase // case (state_q)

   end // always_comb

endmodule
