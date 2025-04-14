`timescale 1ns/1ps

// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>

// -----------------------------------------------------------------------------
// Module: manquehuito_domain
// Description: Top-level SoC integrating program counter, instruction memory,
//              control unit, ALU, registers, multiplexers, data memory and status.
// -----------------------------------------------------------------------------
module manquehuito_domain (
  input logic clk_i  // Global clock input
);

  logic [7:0]  pc_out;
  logic [14:0] im_out;
  logic [7:0]  reg_a_out, reg_b_out;
  logic [7:0]  mux_a_out, mux_b_out, mux_d_out;
  logic [7:0]  alu_out;
  logic [3:0]  alu_zncv;
  logic [11:0] ctrl_out;
  logic [3:0]  status_out;
  logic [7:0]  data_mem_out;

  pc i_pc (
    .clk_i(clk_i),
    .load_i(ctrl_out[11]),
    .im_i(im_out[7:0]),
    .pc_o(pc_out)
  );

  instruction_memory i_imem (
    .addr_i(pc_out),
    .out_o(im_out)
  );

  control_unit i_ctrl (
    .opcode_i(im_out[14:8]),
    .zncv_i(status_out),
    .out_o(ctrl_out)
  );

  register i_reg_a (
    .clk_i(clk_i),
    .load_i(ctrl_out[8]),
    .data_i(alu_out),
    .out_o(reg_a_out)
  );

  register i_reg_b (
    .clk_i(clk_i),
    .load_i(ctrl_out[7]),
    .data_i(alu_out),
    .out_o(reg_b_out)
  );

  muxA i_mux_a (
    .e0_i(reg_a_out),
    .e1_i(8'h01),
    .e2_i(8'h00),
    .e3_i(reg_b_out),
    .sel_i(ctrl_out[6:5]),
    .out_o(mux_a_out)
  );

  muxB i_mux_b (
    .e0_i(reg_b_out),
    .e1_i(data_mem_out),
    .e2_i(im_out[7:0]),
    .e3_i(8'h00),
    .sel_i(ctrl_out[4:3]),
    .out_o(mux_b_out)
  );

  muxD i_mux_d (
    .e0_i(im_out[7:0]),
    .e1_i(reg_b_out),
    .sel_i(ctrl_out[9]),
    .out_o(mux_d_out)
  );

  alu i_alu (
    .a_i(mux_a_out),
    .b_i(mux_b_out),
    .sel_i(ctrl_out[2:0]),
    .out_o(alu_out),
    .zncv_o(alu_zncv)
  );

  data_memory i_dmem (
    .clk_i(clk_i),
    .we_i(ctrl_out[10]),
    .addr_i(mux_d_out),
    .data_in_i(alu_out),
    .data_out_o(data_mem_out)
  );

  status i_status (
    .clk_i(clk_i),
    .zncv_i(alu_zncv),
    .out_o(status_out)
  );

endmodule : computer
