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
           input logic        clk_i,   // Clock input
           input logic        rst_n_i, // Asynchronous reset
           input logic        stall_i, // Stall signal
           input logic        load_i,  // Load enable (for jumps)
           input logic [7:0]  im_i,    // Immediate input for load
           output logic [7:0] pc_o     // Program counter output
           );

   // Note: Added negedge rst_n_i for proper asynchronous reset
   always_ff @(posedge clk_i or negedge rst_n_i) begin
     if(!rst_n_i) begin
       pc_o <= 8'b0;
     end else begin
       // A load (jump) has the highest priority
       if (load_i) begin
         pc_o <= im_i;
         // NEW: If not loading, check if we should stall
       end else if (stall_i) begin
         pc_o <= pc_o; // Do nothing, hold the current value
         // If not loading and not stalling, do the default increment
       end else begin
         pc_o <= pc_o + 1;
       end
     end
   end

endmodule : pc
