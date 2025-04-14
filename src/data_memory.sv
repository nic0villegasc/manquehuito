`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: data_memory
// Description: 256x8-bit synchronous memory with write enable.
// -----------------------------------------------------------------------------
module data_memory (
  input  logic        clk_i,      // Clock input
  input  logic        we_i,       // Write enable
  input  logic [7:0]  addr_i,     // Address input
  input  logic [7:0]  data_in_i,  // Data input (write)
  output logic [7:0]  data_out_o  // Data output (read)
);

  logic [7:0] mem [0:255];

  always_ff @(posedge clk_i) begin
    if (we_i) begin
      mem[addr_i] <= data_in_i;
    end
  end

  assign data_out_o = mem[addr_i];

endmodule : data_memory
