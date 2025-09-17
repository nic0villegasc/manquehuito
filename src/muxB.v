`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: muxB
// Description: 4-to-1 multiplexer for 8-bit inputs. Used to select between
//              RegB, DataMem, InstructionMem, and zero depending on 2-bit control.
// -----------------------------------------------------------------------------
module muxB (
  input [7:0] e0_i,   // Option 0: Register B
  input [7:0] e1_i,   // Option 1: Data Memory
  input [7:0] e2_i,   // Option 2: Instruction Memory
  input [7:0] e3_i,   // Option 3: Constant 0
  input [1:0] sel_i,  // Select signal
  output reg [7:0] out_o   // Selected output
);

  always begin
    case (sel_i)
      2'b00: out_o = e0_i;
      2'b01: out_o = e1_i;
      2'b10: out_o = e2_i;
      2'b11: out_o = e3_i;
    endcase
  end

endmodule : muxB
