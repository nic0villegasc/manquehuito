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
  output logic        spi_read_not_write_o,
  output logic [1:0]  spi_num_bytes_o,

  output logic        cpu_stall_o,
  output logic        pc_load_o,
  output logic        reg_a_load_o,
  output logic        reg_b_load_o,
  output logic        reg_mdr_load_o,
  output logic        status_load_o,
  output logic [1:0]  mux_a_sel_o,
  output logic [1:0]  mux_b_sel_o,
  output logic        mux_d_sel_o,
  output logic [2:0]  alu_sel_o,
  output logic [14:0] instruction_o
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

   // Define parameters for MUX B selections for readability
   localparam MUX_B_SEL_REGB    = 2'b00; // Selects Register B
   localparam MUX_B_SEL_DMEM    = 2'b01; // Selects Data Memory
   localparam MUX_B_SEL_IMEM    = 2'b10; // Selects Instruction Memory
   localparam MUX_B_SEL_CONST_0 = 2'b11; // Selects Constant 0

   // Define parameters for MUX D selections for readability
   localparam MUX_D_SEL_IMEM = 1'b0; // Selects Instruction Memory
   localparam MUX_D_SEL_REGB = 1'b1; // Selects Register B

   typedef enum logic [3:0] {
                            RESET,
                            IFETCH_START,
                            IFETCH_WAIT,
                            DECODE_EXECUTE,
                            MEM_ACCESS_START,
                            MEM_ACCESS_WAIT,
                            MEM_WRITE_START,
                            MEM_WRITE_WAIT,
                            EXECUTE_MEM_OP
                            } state_t;

   state_t state_q, state_d;

   reg [14:0] instruction_r;
   reg [15:0] sp_q, sp_d;

   assign instruction_o = instruction_r;

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
     reg_mdr_load_o       = 1'b0;
     status_load_o        = 1'b0;
     mux_a_sel_o          = 2'b00;
     mux_b_sel_o          = 2'b00;
     mux_d_sel_o          = 1'b0;
     alu_sel_o            = 3'b000;
     spi_start_o          = 1'b0;
     spi_address_o        = 16'b0;
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
         state_d     = IFETCH_START;

         case (instruction_r[14:8])

           // --- Category 1: ALU/Register Ops ---
           // ALU and Register Operations
           7'b0000000: begin // MOV A, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             mux_a_sel_o   = 2'b10; // Input A from Constant 0
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b000; // OP: A + B
           end
           7'b0000001: begin // MOV B, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b11; // Input B from Constant 0
             alu_sel_o     = 3'b000; // OP: A + B
           end
           7'b0000010: begin // MOV A, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             mux_a_sel_o   = 2'b10; // Input A from Constant 0
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b000; // OP: A + B
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
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b000; // OP: A + B
           end
           7'b0000101: begin // ADD B, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b000; // OP: A + B
           end
           7'b0000110: begin // ADD A, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b000; // OP: A + B
           end
           7'b0000111: begin // ADD B, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b000; // OP: A + B
           end
           7'b0001000: begin // SUB A, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b001; // OP: A - B
           end
           7'b0001001: begin // SUB B, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b001; // OP: A - B
           end
           7'b0001010: begin // SUB A, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = MUX_A_SEL_REGA; // Input A from Register A
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b001; // OP: A - B
           end
           7'b0001011: begin // SUB B, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b001; // OP: A - B
           end
           7'b0001100: begin // AND A, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b010; // OP: A & B
           end
           7'b0001101: begin // AND B, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b010; // OP: A & B
           end
           7'b0001110: begin // AND A, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b010; // OP: A & B
           end
           7'b0001111: begin // AND B, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b010; // OP: A & B
           end
           7'b0010000: begin // OR A, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b011; // OP: A | B
           end
           7'b0010001: begin // OR B, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b011; // OP: A | B
           end
           7'b0010010: begin // OR A, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b011; // OP: A | B
           end
           7'b0010011: begin // OR B, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b011; // OP: A | B
           end
           7'b0010100: begin // NOT A, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b100; // OP: ~A
           end
           7'b0010101: begin // NOT A, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b100; // OP: ~A
           end
           7'b0010110: begin // NOT B, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b100; // OP: ~A
           end
           7'b0010111: begin // NOT B, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b100; // OP: ~A
           end
           7'b0011000: begin // XOR A, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b101; // OP: A ^ B
           end
           7'b0011001: begin // XOR B, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b101; // OP: A ^ B
           end
           7'b0011010: begin // XOR A, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b101; // OP: A ^ B
           end
           7'b0011011: begin // XOR B, Lit
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'b10; // Input B from Immediate
             alu_sel_o     = 3'b101; // OP: A ^ B
           end
           7'b0011100: begin // SHL A, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b110; // OP: A << 1
           end
           7'b0011101: begin // SHL A, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b110; // OP: A << 1
           end
           7'b0011110: begin // SHL B, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b110; // OP: A << 1
           end
           7'b0011111: begin // SHL B, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b110; // OP: A << 1
           end
           7'b0100000: begin // SHR A, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b111; // OP: A >> 1
           end
           7'b0100001: begin // SHR A, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b1;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b111; // OP: A >> 1
           end
           7'b0100010: begin // SHR B, A
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b00; // Input A from Register A
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b111; // OP: A >> 1
           end
           7'b0100011: begin // SHR B, B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b11; // Input A from Register B
             mux_b_sel_o   = 2'bXX; // Input B is Don't Care
             alu_sel_o     = 3'b111; // OP: A >> 1
           end
           7'b0100100: begin // INC B
             pc_load_o     = 1'b0;
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b1;
             status_load_o = 1'b1;

             mux_a_sel_o   = 2'b01; // Input A from Constant 1
             mux_b_sel_o   = 2'b00; // Input B from Register B
             alu_sel_o     = 3'b000; // OP: A + B
           end

           // --- Category 2: Memory Ops ---
           7'b0100101: begin // MOV A,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0100110: begin // MOV B,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0100111: begin // MOV (Dir),A
             state_d = MEM_ACCESS_START;
           end
           7'b0101000: begin // MOV (Dir),B
             state_d = MEM_ACCESS_START;
           end
           7'b0101001: begin // MOV A,(B)
             state_d = MEM_ACCESS_START;
           end
           7'b0101010: begin // MOV B,(B)
             state_d = MEM_ACCESS_START;
           end
           7'b0101011: begin // MOV (B),A
             state_d = MEM_ACCESS_START;
           end
           7'b0101100: begin // ADD A,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0101101: begin // ADD B,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0101110: begin // ADD A,(B)
             state_d = MEM_ACCESS_START;
           end
           7'b0101111: begin // ADD (Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0110000: begin // SUB A,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0110001: begin // SUB B,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0110010: begin // SUB A,(B)
             state_d = MEM_ACCESS_START;
           end
           7'b0110011: begin // SUB (Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0110100: begin // AND A,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0110101: begin // AND B,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0110110: begin // AND A,(B)
             state_d = MEM_ACCESS_START;
           end
           7'b0110111: begin // AND (Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0111000: begin // OR A,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0111001: begin // OR B,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0111010: begin // OR A,(B)
             state_d = MEM_ACCESS_START;
           end
           7'b0111011: begin // OR (Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b0111100: begin // NOT (Dir),A
             state_d = MEM_ACCESS_START;
           end
           7'b0111101: begin // NOT (Dir),B
             state_d = MEM_ACCESS_START;
           end
           7'b0111110: begin // NOT (B)
             state_d = MEM_ACCESS_START;
           end
           7'b0111111: begin // XOR A,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b1000000: begin // XOR B,(Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b1000001: begin // XOR A,(B)
             state_d = MEM_ACCESS_START;
           end
           7'b1000010: begin // XOR (Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b1000011: begin // SHL (Dir),A
             state_d = MEM_ACCESS_START;
           end
           7'b1000100: begin // SHL (Dir),B
             state_d = MEM_ACCESS_START;
           end
           7'b1000101: begin // SHL (B)
             state_d = MEM_ACCESS_START;
           end
           7'b1000110: begin // SHR (Dir),A
             state_d = MEM_ACCESS_START;
           end
           7'b1000111: begin // SHR (Dir),B
             state_d = MEM_ACCESS_START;
           end
           7'b1001000: begin // SHR (B)
             state_d = MEM_ACCESS_START;
           end
           7'b1001001: begin // INC (Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b1001010: begin // INC (B)
             state_d = MEM_ACCESS_START;
           end
           7'b1001011: begin // RST (Dir)
             state_d = MEM_ACCESS_START;
           end
           7'b1001100: begin // RST (B)
             state_d = MEM_ACCESS_START;
           end

           // --- Category 3: Compare Ops ---
           7'b1001101: begin // CMP A,B
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = MUX_A_SEL_REGA; // SA0 = A
             mux_b_sel_o   = MUX_B_SEL_REGB; // SB0 = B
             alu_sel_o     = ALU_SUB;        // S0 = -
           end
           7'b1001110: begin // CMP A,Lit
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = MUX_A_SEL_REGA; // SA0 = A
             mux_b_sel_o   = MUX_B_SEL_IMEM; // SB0 = Lit
             alu_sel_o     = ALU_SUB;        // S0 = -
           end
           7'b1001111: begin // CMP B,Lit
             reg_a_load_o  = 1'b0;
             reg_b_load_o  = 1'b0;
             status_load_o = 1'b1;

             mux_a_sel_o   = MUX_A_SEL_REGB; // SA0 = B
             mux_b_sel_o   = MUX_B_SEL_IMEM; // SB0 = Lit
             alu_sel_o     = ALU_SUB;        // S0 = -
           end

           // For memory-based CMP, we start the memory access cycle
           7'b1010000, // CMP A,(Dir)
             7'b1010001, // CMP B,(Dir)
             7'b1010010: // CMP A,(B)
               state_d = MEM_ACCESS_START;

           // --- Category 4: Jump/Branch Ops ---
           7'b1010011: begin // JMP Dir
             pc_load_o = 1'b1; // LPC = 1
           end
           7'b1010100: begin // JEQ Dir
             if (zncv_i[3]) begin // Z flag
               pc_load_o = 1'b1;
             end
           end
           7'b1010101: begin // JNE Dir
             if (!zncv_i[3]) begin // Not Z flag
               pc_load_o = 1'b1;
             end
           end
           7'b1010110: begin // JGT Dir
             if (!zncv_i[2] && !zncv_i[3]) begin // Not N and Not Z
               pc_load_o = 1'b1;
             end
           end
           7'b1010111: begin // JLT Dir
             if (zncv_i[2]) begin // N flag
               pc_load_o = 1'b1;
             end
           end
           7'b11000: begin // JGE Dir
             if (!zncv_i[2]) begin // Not N flag
               pc_load_o = 1'b1;
             end
           end
           7'b11001: begin // JLE Dir
             if (zncv_i[2] || zncv_i[3]) begin // N or Z flag
               pc_load_o = 1'b1;
             end
           end
           7'b11010: begin // JCR Dir
             if (zncv_i[1]) begin // C flag
               pc_load_o = 1'b1;
             end
           end
           7'b11011: begin // JOV Dir
             if (zncv_i[0]) begin // V flag
               pc_load_o = 1'b1;
             end
           end

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
           7'b0100101: begin // MOV A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0100110: begin // MOV B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0100111: begin // MOV (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (pass-through)
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through A)
           end
           7'b0101000: begin // MOV (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0101001: begin // MOV A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0101010: begin // MOV B,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0101011: begin // MOV (B),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (pass-through)
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through A)
           end

           // --- ADD Instructions ---
           7'b0101100: begin // ADD A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b0101101: begin // ADD B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b0101110: begin // ADD A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b0101111: begin // ADD (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end

           // --- SUB Instructions ---
           7'b0110000: begin // SUB A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end
           7'b0110001: begin // SUB B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end
           7'b0110010: begin // SUB A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end
           7'b0110011: begin // SUB (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end

           // --- AND Instructions ---
           7'b0110100: begin // AND A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end
           7'b0110101: begin // AND B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end
           7'b0110110: begin // AND A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end
           7'b0110111: begin // AND (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end

           // --- OR Instructions ---
           7'b0111000: begin // OR A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end
           7'b0111001: begin // OR B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end
           7'b0111010: begin // OR A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end
           7'b0111011: begin // OR (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end

           // --- NOT Instructions ---
           7'b0111100: begin // NOT (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_NOT;              // ALU Op = ~
           end
           7'b0111101: begin // NOT (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_NOT;              // ALU Op = ~
           end
           7'b0111110: begin // NOT (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_NOT;              // ALU Op = ~
           end

           // --- XOR Instructions ---
           7'b0111111: begin // XOR A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end
           7'b1000000: begin // XOR B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end
           7'b1000001: begin // XOR A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end
           7'b1000010: begin // XOR (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end

           // --- SHL Instructions ---
           7'b1000011: begin // SHL (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHL;              // ALU Op = <<
           end
           7'b1000100: begin // SHL (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHL;              // ALU Op = <<
           end
           7'b1000101: begin // SHL (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHL;              // ALU Op = <<
           end

           // --- SHR Instructions ---
           7'b1000110: begin // SHR (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHR;              // ALU Op = >>
           end
           7'b1000111: begin // SHR (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHR;              // ALU Op = >>
           end
           7'b1001000: begin // SHR (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHR;              // ALU Op = >>
           end

           // --- INC Instructions ---
           7'b1001001: begin // INC (Dir)
             spi_read_not_write_o = 1'b1;                 // (Read first)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_CONST_1;   // SA0 = U (Constant 1)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b1001010: begin // INC (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_1;   // SA0 = U (Constant 1)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end

           // --- RST Instructions ---
           7'b1001011: begin // RST (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z
             alu_sel_o            = ALU_ADD;              // ALU Op = + (0+0=0)
           end
           7'b1001100: begin // RST (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z
             alu_sel_o            = ALU_ADD;              // ALU Op = + (0+0=0)
           end

           // --- CMP Memory Instructions ---
           7'b1010000: begin // CMP A,(Dir)
             spi_read_not_write_o = 1'b1;           // Read from memory
             mux_d_sel_o          = MUX_D_SEL_IMEM;  // SD0 = Lit (Address from immediate)
             reg_mdr_load_o       = 1'b1;           // Load value into MDR
           end
           7'b1010001: begin // CMP B,(Dir)
             spi_read_not_write_o = 1'b1;           // Read from memory
             mux_d_sel_o          = MUX_D_SEL_IMEM;  // SD0 = Lit (Address from immediate)
             reg_mdr_load_o       = 1'b1;           // Load value into MDR
           end
           7'b1010010: begin // CMP A,(B)
             spi_read_not_write_o = 1'b1;           // Read from memory
             mux_d_sel_o          = MUX_D_SEL_REGB;  // SD0 = B (Address from Register B)
             reg_mdr_load_o       = 1'b1;           // Load value into MDR
           end

           // --- Default Case ---
           default: begin
             spi_read_not_write_o = 1'b1;                 // Default to read
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // Default to literal
             reg_a_load_o         = 1'b0;                 // Default: no load
             reg_b_load_o         = 1'b0;                 // Default: no load
             reg_mdr_load_o       = 1'b0;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // Default
             mux_b_sel_o          = MUX_B_SEL_REGB;      // Default
             alu_sel_o            = ALU_ADD;              // Default
           end
         endcase // case (instruction_r[14:8])

         if (!spi_busy_i) begin
           state_d = MEM_ACCESS_WAIT;
         end
       end

       MEM_ACCESS_WAIT: begin
         cpu_stall_o = 1'b1; // Keep the CPU stalleds

         case (instruction_r[14:8])
           // --- MOV Instructions ---
           7'b0100101: begin // MOV A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0100110: begin // MOV B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0100111: begin // MOV (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (pass-through)
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through A)
           end
           7'b0101000: begin // MOV (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0101001: begin // MOV A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0101010: begin // MOV B,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0101011: begin // MOV (B),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (pass-through)
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through A)
           end

           // --- ADD Instructions ---
           7'b0101100: begin // ADD A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b0101101: begin // ADD B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b0101110: begin // ADD A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b0101111: begin // ADD (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end

           // --- SUB Instructions ---
           7'b0110000: begin // SUB A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end
           7'b0110001: begin // SUB B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end
           7'b0110010: begin // SUB A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end
           7'b0110011: begin // SUB (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end

           // --- AND Instructions ---
           7'b0110100: begin // AND A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end
           7'b0110101: begin // AND B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end
           7'b0110110: begin // AND A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end
           7'b0110111: begin // AND (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end

           // --- OR Instructions ---
           7'b0111000: begin // OR A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end
           7'b0111001: begin // OR B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end
           7'b0111010: begin // OR A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end
           7'b0111011: begin // OR (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end

           // --- NOT Instructions ---
           7'b0111100: begin // NOT (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_NOT;              // ALU Op = ~
           end
           7'b0111101: begin // NOT (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_NOT;              // ALU Op = ~
           end
           7'b0111110: begin // NOT (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_NOT;              // ALU Op = ~
           end

           // --- XOR Instructions ---
           7'b0111111: begin // XOR A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end
           7'b1000000: begin // XOR B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             // reg_a_load_o         = 1'b0;                 // LA = 0
             // reg_b_load_o         = 1'b1;                 // LB = 1
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end
           7'b1000001: begin // XOR A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             // reg_a_load_o         = 1'b1;                 // LA = 1
             // reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end
           7'b1000010: begin // XOR (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end

           // --- SHL Instructions ---
           7'b1000011: begin // SHL (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHL;              // ALU Op = <<
           end
           7'b1000100: begin // SHL (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHL;              // ALU Op = <<
           end
           7'b1000101: begin // SHL (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHL;              // ALU Op = <<
           end

           // --- SHR Instructions ---
           7'b1000110: begin // SHR (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHR;              // ALU Op = >>
           end
           7'b1000111: begin // SHR (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHR;              // ALU Op = >>
           end
           7'b1001000: begin // SHR (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHR;              // ALU Op = >>
           end

           // --- INC Instructions ---
           7'b1001001: begin // INC (Dir)
             spi_read_not_write_o = 1'b1;                 // (First read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             reg_mdr_load_o       = 1'b1;
             mux_a_sel_o          = MUX_A_SEL_CONST_1;   // SA0 = U (Constant 1)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b1001010: begin // INC (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_1;   // SA0 = U (Constant 1)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end

           // --- RST Instructions ---
           7'b1001011: begin // RST (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z
             alu_sel_o            = ALU_ADD;              // ALU Op = + (0+0=0)
           end
           7'b1001100: begin // RST (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z
             alu_sel_o            = ALU_ADD;              // ALU Op = + (0+0=0)
           end

           7'b1010000: begin // CMP A,(Dir)
             spi_read_not_write_o = 1'b1;           // Read from memory
             mux_d_sel_o          = MUX_D_SEL_IMEM;  // SD0 = Lit (Address from immediate)
             reg_mdr_load_o       = 1'b1;           // Load value into MDR
           end
           7'b1010001: begin // CMP B,(Dir)
             spi_read_not_write_o = 1'b1;           // Read from memory
             mux_d_sel_o          = MUX_D_SEL_IMEM;  // SD0 = Lit (Address from immediate)
             reg_mdr_load_o       = 1'b1;           // Load value into MDR
           end
           7'b1010010: begin // CMP A,(B)
             spi_read_not_write_o = 1'b1;           // Read from memory
             mux_d_sel_o          = MUX_D_SEL_REGB;  // SD0 = B (Address from Register B)
             reg_mdr_load_o       = 1'b1;           // Load value into MDR
           end

           // --- Default Case ---
           default: begin
             spi_read_not_write_o = 1'b1;                 // Default to read
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // Default to literal
             reg_a_load_o         = 1'b0;                 // Default: no load
             reg_b_load_o         = 1'b0;                 // Default: no load
             reg_mdr_load_o       = 1'b0;
             mux_a_sel_o          = MUX_A_SEL_REGA;      // Default
             mux_b_sel_o          = MUX_B_SEL_REGB;      // Default
             alu_sel_o            = ALU_ADD;              // Default
           end
         endcase // case (instruction_r[14:8])

         if (spi_done_i) begin
           
           case (instruction_r[14:8])
             7'b1001001,
             7'b1001010: begin
               state_d        = MEM_WRITE_START;
             end
             default:
               state_d = EXECUTE_MEM_OP;
           endcase
         end
       end // case: MEM_ACCESS_WAIT

       MEM_WRITE_START: begin
         cpu_stall_o          = 1'b1;
         spi_start_o          = 1'b1;         // Start the SPI write transaction
         spi_num_bytes_o      = 2'b01;         // We are writing one byte

         case (instruction_r[14:8])
           7'b1001001: begin // INC (Dir)
             spi_read_not_write_o = 1'b0;                 // (Second write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_CONST_1;   // SA0 = U (Constant 1)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end

           7'b1001010: begin // INC (B)
             mux_d_sel_o   = MUX_D_SEL_REGB; // Address is from Register B
             status_load_o = 1'b1;
           end

         endcase // case (instruction_r[14:8])

         spi_address_o = {8'd0, mux_d_out_i};

         if (!spi_busy_i) begin
           state_d = MEM_WRITE_WAIT;
         end
       end // case: MEM_WRITE_START

       MEM_WRITE_WAIT: begin
         cpu_stall_o = 1'b1;

         case (instruction_r[14:8])
           7'b1001001: begin // INC (Dir)
             spi_read_not_write_o = 1'b0;                 // (Second write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_CONST_1;   // SA0 = U (Constant 1)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end

           7'b1001010: begin // INC (B)
             mux_d_sel_o   = MUX_D_SEL_REGB; // Address is from Register B
             status_load_o = 1'b1;
           end
         endcase // case (instruction_r[14:8])

         if (spi_done_i)
           state_d = IFETCH_START;

       end // case: MEM_WRITE_WAIT

       EXECUTE_MEM_OP: begin
         cpu_stall_o = 1'b1;

        case (instruction_r[14:8])
           // --- MOV Instructions ---
           7'b0100101: begin // MOV A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b1;                 // LA = 1
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0100110: begin // MOV B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b1;                 // LB = 1
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0100111: begin // MOV (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (pass-through)
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through A)
           end
           7'b0101000: begin // MOV (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0101001: begin // MOV A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b1;                 // LA = 1
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0101010: begin // MOV B,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b1;                 // LB = 1
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z (pass-through)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through B)
           end
           7'b0101011: begin // MOV (B),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (pass-through)
             alu_sel_o            = ALU_ADD;              // ALU Op = + (pass-through A)
           end

           // --- ADD Instructions ---
           7'b0101100: begin // ADD A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b1;                 // LA = 1
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b0101101: begin // ADD B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b1;                 // LB = 1
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b0101110: begin // ADD A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b1;                 // LA = 1
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b0101111: begin // ADD (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end

           // --- SUB Instructions ---
           7'b0110000: begin // SUB A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b1;                 // LA = 1
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end
           7'b0110001: begin // SUB B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b1;                 // LB = 1
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end
           7'b0110010: begin // SUB A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b1;                 // LA = 1
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end
           7'b0110011: begin // SUB (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_SUB;              // ALU Op = -
           end

           // --- AND Instructions ---
           7'b0110100: begin // AND A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b1;                 // LA = 1
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end
           7'b0110101: begin // AND B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b1;                 // LB = 1
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end
           7'b0110110: begin // AND A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b1;                 // LA = 1
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end
           7'b0110111: begin // AND (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_AND;              // ALU Op = &
           end

           // --- OR Instructions ---
           7'b0111000: begin // OR A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b1;                 // LA = 1
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end
           7'b0111001: begin // OR B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b1;                 // LB = 1
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end
           7'b0111010: begin // OR A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b1;                 // LA = 1
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end
           7'b0111011: begin // OR (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_OR;               // ALU Op = |
           end

           // --- NOT Instructions ---
           7'b0111100: begin // NOT (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_NOT;              // ALU Op = ~
           end
           7'b0111101: begin // NOT (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_NOT;              // ALU Op = ~
           end
           7'b0111110: begin // NOT (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_NOT;              // ALU Op = ~
           end

           // --- XOR Instructions ---
           7'b0111111: begin // XOR A,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b1;                 // LA = 1
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end
           7'b1000000: begin // XOR B,(Dir)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o         = 1'b0;                 // LA = 0
             reg_b_load_o         = 1'b1;                 // LB = 1
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end
           7'b1000001: begin // XOR A,(B)
             spi_read_not_write_o = 1'b1;                 // DW = 0 (Read)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o         = 1'b1;                 // LA = 1
             reg_b_load_o         = 1'b0;                 // LB = 0
             status_load_o        = 1'b1;

             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end
           7'b1000010: begin // XOR (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_REGB;      // SB0 = B
             alu_sel_o            = ALU_XOR;              // ALU Op = ^
           end

           // --- SHL Instructions ---
           7'b1000011: begin // SHL (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHL;              // ALU Op = <<
           end
           7'b1000100: begin // SHL (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHL;              // ALU Op = <<
           end
           7'b1000101: begin // SHL (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHL;              // ALU Op = <<
           end

           // --- SHR Instructions ---
           7'b1000110: begin // SHR (Dir),A
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHR;              // ALU Op = >>
           end
           7'b1000111: begin // SHR (Dir),B
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGB;      // SA0 = B
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHR;              // ALU Op = >>
           end
           7'b1001000: begin // SHR (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_REGA;      // SA0 = A
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z (Unused)
             alu_sel_o            = ALU_SHR;              // ALU Op = >>
           end

           // --- INC Instructions ---
           7'b1001001: begin // INC (Dir)
             spi_read_not_write_o = 1'b1;                 // (First read)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_1;   // SA0 = U (Constant 1)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end
           7'b1001010: begin // INC (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_1;   // SA0 = U (Constant 1)
             mux_b_sel_o          = MUX_B_SEL_DMEM;      // SB0 = Mem
             alu_sel_o            = ALU_ADD;              // ALU Op = +
           end

           // --- RST Instructions ---
           7'b1001011: begin // RST (Dir)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // SD0 = Lit
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z
             alu_sel_o            = ALU_ADD;              // ALU Op = + (0+0=0)
           end
           7'b1001100: begin // RST (B)
             spi_read_not_write_o = 1'b0;                 // DW = 1 (Write)
             mux_d_sel_o          = MUX_D_SEL_REGB;      // SD0 = B
             reg_a_load_o           = 1'b0;                 // LA = 0
             reg_b_load_o           = 1'b0;                 // LB = 0
             mux_a_sel_o          = MUX_A_SEL_CONST_0;   // SA0 = Z
             mux_b_sel_o          = MUX_B_SEL_CONST_0;   // SB0 = Z
             alu_sel_o            = ALU_ADD;              // ALU Op = + (0+0=0)
           end

          // --- CMP Memory Instructions ---
          7'b1010000: begin // CMP A,(Dir)
            reg_a_load_o  = 1'b0;            // LA = 0 (no write-back)
            reg_b_load_o  = 1'b0;            // LB = 0
            status_load_o = 1'b1;
            
            mux_a_sel_o   = MUX_A_SEL_REGA;  // SA0 = A
            mux_b_sel_o   = MUX_B_SEL_DMEM;  // SB0 = Mem (from MDR)
            alu_sel_o     = ALU_SUB;         // S0 = -
          end
          7'b1010001: begin // CMP B,(Dir)
            reg_a_load_o  = 1'b0;            // LA = 0
            reg_b_load_o  = 1'b0;            // LB = 0
            status_load_o = 1'b1;

            mux_a_sel_o   = MUX_A_SEL_REGB;  // SA0 = B
            mux_b_sel_o   = MUX_B_SEL_DMEM;  // SB0 = Mem (from MDR)
            alu_sel_o     = ALU_SUB;         // S0 = -
          end
          7'b1010010: begin // CMP A,(B)
            reg_a_load_o  = 1'b0;            // LA = 0
            reg_b_load_o  = 1'b0;            // LB = 0
            status_load_o = 1'b1;

            mux_a_sel_o   = MUX_A_SEL_REGA;  // SA0 = A
            mux_b_sel_o   = MUX_B_SEL_DMEM;  // SB0 = Mem (from MDR)
            alu_sel_o     = ALU_SUB;         // S0 = -
          end

           // --- Default Case ---
           default: begin
             spi_read_not_write_o = 1'b1;                 // Default to read
             mux_d_sel_o          = MUX_D_SEL_IMEM;      // Default to literal
             reg_a_load_o           = 1'b0;                 // Default: no load
             reg_b_load_o           = 1'b0;                 // Default: no load
             mux_a_sel_o          = MUX_A_SEL_REGA;      // Default
             mux_b_sel_o          = MUX_B_SEL_REGB;      // Default
             alu_sel_o            = ALU_ADD;              // Default
           end
         endcase // case (instruction_r[14:8])

         state_d = IFETCH_START;
       end
       default: begin
         state_d = RESET;
       end
     endcase // case (state_q)

   end // always_comb

endmodule
