`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: status
// Description: Latches the ZNCV flags on rising edge of clk_i
// -----------------------------------------------------------------------------
module status (
  input  logic        clk_i,   // Clock input
  input  logic [3:0]  zncv_i,  // Input flags: Zero, Negative, Carry, Overflow
  output logic [3:0]  out_o    // Latched output flags
);

  always_ff @(posedge clk_i) begin
    out_o <= zncv_i;
  end

endmodule : status