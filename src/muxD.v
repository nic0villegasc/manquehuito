`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: muxD
// Description: 2-to-1 multiplexer for 8-bit inputs. Selects between instruction
//              memory and register B based on 1-bit control.
// -----------------------------------------------------------------------------
module muxD (
  input [7:0] e0_i,   // Option 0: Instruction Memory
  input [7:0] e1_i,   // Option 1: Register B
  input       sel_i,  // Select signal
  output reg [7:0] out_o   // Selected output
);

  always begin
    case (sel_i)
      1'b0: out_o = e0_i;
      1'b1: out_o = e1_i;
    endcase
  end

endmodule : muxD
