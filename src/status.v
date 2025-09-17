`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: status
// Description: Latches the ZNCV flags only when load_en_i is asserted.
// -----------------------------------------------------------------------------
module status (
               input        clk_i,     // Clock input
               input        load_en_i, // NEW: Load enable signal
               input [3:0]  zncv_i,    // Input flags: Zero, Negative, Carry, Overflow
               output reg [3:0] out_o      // Latched output flags
               );

   always @(posedge clk_i) begin
     // Only update the flags if the load enable is high
     if (load_en_i) begin
       out_o <= zncv_i;
     end
   end

endmodule : status
