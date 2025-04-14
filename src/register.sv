`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: register
// Description: 8-bit register with load enable and synchronous reset to 0
// -----------------------------------------------------------------------------
module register (
  input  logic        clk_i,   // Clock input
  input  logic        load_i,  // Load enable
  input  logic [7:0]  data_i,  // Input data
  output logic [7:0]  out_o    // Output register value
);

  always_ff @(posedge clk_i) begin
    if (load_i) begin
      out_o <= data_i;
    end
  end

endmodule : register