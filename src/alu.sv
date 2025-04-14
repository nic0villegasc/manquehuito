`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: alu
// Description: 8-bit ALU with arithmetic, logic and shift operations.
//              Outputs result and ZNCV flags: Zero, Negative, Carry, Overflow.
// -----------------------------------------------------------------------------
module alu (
  input  logic [7:0]  a_i,    // Operand A
  input  logic [7:0]  b_i,    // Operand B
  input  logic [2:0]  sel_i,  // Operation select
  output logic [7:0]  out_o,  // Result
  output logic [3:0]  zncv_o  // Flags: Z, N, C, V
);

  logic [8:0] sub_ext;

  always_comb begin
    unique case (sel_i)
      3'b000: out_o = a_i + b_i;
      3'b001: out_o = a_i - b_i;
      3'b010: out_o = a_i & b_i;
      3'b011: out_o = a_i | b_i;
      3'b100: out_o = ~a_i;
      3'b101: out_o = a_i ^ b_i;
      3'b110: out_o = a_i << 1;
      3'b111: out_o = a_i >> 1;
    endcase

    sub_ext      = a_i - b_i;
    zncv_o[3]    = (sub_ext[7:0] == 8'b0);              // Z: result is zero
    zncv_o[2]    = sub_ext[7];                          // N: negative result
    zncv_o[1]    = sub_ext[8];                          // C: carry out
    zncv_o[0]    = (a_i[7] == b_i[7]) && (a_i[7] != sub_ext[7]); // V: signed overflow
  end

endmodule : alu
