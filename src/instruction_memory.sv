`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: instruction_memory
// Description: 256x15-bit ROM used for instruction fetch.
// -----------------------------------------------------------------------------
module instruction_memory (
  input  logic  [7:0]  addr_i,  // Address input
  output logic [14:0]  out_o    // Instruction output
);

  logic [14:0] mem [0:255];

  assign out_o = mem[addr_i];

endmodule : instruction_memory
