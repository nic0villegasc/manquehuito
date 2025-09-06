`timescale 1ns/1ps
// Copyright 2025 Universidad de los Andes, Chile
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors:
// - Nicolás Villegas <navillegas@miuandes.cl>
// -----------------------------------------------------------------------------
// Module: manquehuito_domain
// Description: Top-level SoC integrating the manquehuito core with an SPI
//              master to access external instruction and data memory.
// -----------------------------------------------------------------------------
module manquehuito_domain (
  input clk_core_i,
  input rst_n_i,

  output wire spi_sclk_o,
  output wire spi_mosi_o,
  output wire spi_cs_o,
  input spi_miso_i
);

  // --- Internal Wires and Registers ---
  wire [7:0]  pc_out;
  wire [7:0]  reg_a_out, reg_b_out, mdr_out;
  wire [7:0]  mux_a_out, mux_b_out, mux_d_out;
  wire [7:0]  alu_out;
  wire [3:0]  alu_zncv;
  wire [3:0]  status_out;

   wire       cpu_stall_o;
   wire       pc_load_o;
   wire       reg_a_load_o, reg_b_load_o, reg_mdr_load_o, status_load_o;
   wire [1:0] mux_a_sel_o, mux_b_sel_o;
   wire       mux_d_sel_o;
   wire [2:0] alu_sel_o;
   wire [14:0] current_instruction;

  // SPI Master Interface Signals
  wire        spi_start;
  wire [15:0] spi_address;
  wire        spi_read_not_write;
  wire [1:0]  spi_num_bytes;
  wire  [7:0]  spi_data_read_byte1;
  wire  [7:0]  spi_data_read_byte2;
  wire         spi_done;
  wire         spi_busy;

  // --- Sub-module Instantiations ---
  pc i_pc (
    .clk_i(clk_core_i),
    .rst_n_i(rst_n_i),
    .stall_i(cpu_stall_o),
    .load_i(pc_load_o),
    .im_i(current_instruction[7:0]), // Jump address from new instruction reg
    .pc_o(pc_out)
  );

  // Control unit now fed from the new instruction register
   control_unit i_ctrl (
                        // Core Interface
                        .clk_i(clk_core_i),
                        .rst_n_i(rst_n_i),
                        .pc_i(pc_out),
                        .reg_b_in_i(reg_b_out),
                        .mux_d_out_i(mux_d_out),
                        .zncv_i(status_out),

                        // SPI Master Interface
                        .spi_done_i(spi_done),
                        .spi_busy_i(spi_busy),
                        .spi_data_read_byte1_i(spi_data_read_byte1),
                        .spi_data_read_byte2_i(spi_data_read_byte2),
                        .spi_start_o(spi_start),
                        .spi_address_o(spi_address),
                        .spi_read_not_write_o(spi_read_not_write),
                        .spi_num_bytes_o(spi_num_bytes),

                        // Control Outputs to Datapath
                        .cpu_stall_o(cpu_stall_o),
                        .pc_load_o(pc_load_o),
                        .reg_a_load_o(reg_a_load_o),
                        .reg_b_load_o(reg_b_load_o),
                        .reg_mdr_load_o(reg_mdr_load_o),
                        .status_load_o(status_load_o),
                        .mux_a_sel_o(mux_a_sel_o),
                        .mux_b_sel_o(mux_b_sel_o),
                        .mux_d_sel_o(mux_d_sel_o),
                        .alu_sel_o(alu_sel_o),
                        .instruction_r(current_instruction)
                        );

  // Registers A and B are unchanged
  register i_reg_a (
    .clk_i(clk_core_i),
    .load_i(reg_a_load_o),
    .data_i(alu_out),
    .out_o(reg_a_out)
  );

  register i_reg_b (
    .clk_i(clk_core_i),
    .load_i(reg_b_load_o),
    .data_i(alu_out),
    .out_o(reg_b_out)
  );

  register i_mdr (
    .clk_i(clk_core_i),
    .load_i(reg_mdr_load_o),
    .data_i(spi_data_read_byte1), // Input is the SPI data
    .out_o(mdr_out)
  );

  // MUXes A and D are modified to use the new instruction register
  muxA i_mux_a (
    .e0_i(reg_a_out),
    .e1_i(8'h01),
    .e2_i(8'h00),
    .e3_i(reg_b_out),
    .sel_i(mux_a_sel_o),
    .out_o(mux_a_out)
  );

  // MUX B sources are changed
  muxB i_mux_b (
    .e0_i(reg_b_out),
    .e1_i(mdr_out),      // Data from SPI for LOADs
    .e2_i(current_instruction[7:0]), // Immediate from new instruction reg
    .e3_i(8'h00),
    .sel_i(mux_b_sel_o),
    .out_o(mux_b_out)
  );

  // MUX D source is changed
  muxD i_mux_d (
    .e0_i(current_instruction[7:0]), // Address from new instruction reg
    .e1_i(reg_b_out),
    .sel_i(mux_d_sel_o),
    .out_o(mux_d_out)
  );

  // ALU and Status are unchanged
  alu i_alu (
    .a_i(mux_a_out),
    .b_i(mux_b_out),
    .sel_i(alu_sel_o),
    .out_o(alu_out),
    .zncv_o(alu_zncv)
  );

  status i_status (
    .clk_i(clk_core_i),
    .load_en_i(status_load_o),
    .zncv_i(alu_zncv),
    .out_o(status_out)
  );


  // SPI Master instantiation
  spi_master #(
    .CLOCK_DIVIDER(4)
  ) i_spi_master (
    .clk_core_i               (clk_core_i),
    .rst_n_i                  (rst_n_i),
    .start_transaction_i      (spi_start),
    .address_i                (spi_address),
    .data_to_write_i          (alu_out),
    .read_not_write_i         (spi_read_not_write),
    .num_bytes_to_transfer_i  (spi_num_bytes),
    .data_read_byte1_o        (spi_data_read_byte1),
    .data_read_byte2_o        (spi_data_read_byte2),
    .transaction_done_o       (spi_done),
    .busy_o                   (spi_busy),
    .spi_sclk_o               (spi_sclk_o),
    .spi_mosi_o               (spi_mosi_o),
    .spi_miso_i               (spi_miso_i),
    .spi_cs_o                 (spi_cs_o)
  );
endmodule : manquehuito_domain
