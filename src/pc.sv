`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: pc
// Description: 8-bit Program Counter with load enable and increment logic
// -----------------------------------------------------------------------------
module pc (
  input  logic        clk_i,   // Clock input
  input  logic        load_i,  // Load enable
  input  logic [7:0]  im_i,    // Immediate input for load
  output logic [7:0]  pc_o     // Program counter output
);

  always_ff @(posedge clk_i) begin
    if (load_i) begin
      pc_o <= im_i;
    end else begin
      pc_o <= pc_o + 1;
    end
  end

endmodule : pc