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
  input  logic [7:0]  a_i,
  input  logic [7:0]  b_i,
  input  logic [2:0]  sel_i,
  output logic [7:0]  out_o,
  output logic [3:0]  zncv_o
);

  logic [8:0] result_ext;
  logic       carry, overflow;

  always_comb begin
    // Default values
    result_ext = 9'b0;
    carry      = 1'b0;
    overflow   = 1'b0;
    out_o      = 8'b0;

    unique case (sel_i)
      3'b000: begin // ADD
        result_ext = a_i + b_i;
        out_o      = result_ext[7:0];
        carry      = result_ext[8];
        overflow   = ~(a_i[7] ^ b_i[7]) & (a_i[7] ^ out_o[7]);
      end

      3'b001: begin // SUB
        result_ext = {1'b0, a_i} - {1'b0, b_i};
        out_o      = result_ext[7:0];
        carry      = result_ext[8];
        overflow   = (a_i[7] ^ b_i[7]) & (a_i[7] ^ out_o[7]);
      end

      3'b010: out_o = a_i & b_i;
      3'b011: out_o = a_i | b_i;
      3'b100: out_o = ~a_i;
      3'b101: out_o = a_i ^ b_i;
      3'b110: out_o = a_i << 1;
      3'b111: out_o = a_i >> 1;
    endcase

    // Flags
    zncv_o[3] = (out_o == 8'b0);   // Z
    zncv_o[2] = out_o[7];          // N
    zncv_o[1] = carry;             // C
    zncv_o[0] = overflow;          // V
  end


endmodule