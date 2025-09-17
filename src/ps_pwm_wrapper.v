module ps_pwm_wrapper (
                       // --- CPU Interface ---
                       input       clk_i,
                       input       rst_n_i,
                       input       cs_i,       // Chip Select (active when CPU targets 0xF4-F6)
                       input       write_en_i, // Write enable from CPU
                       input [1:0] addr_lsb_i, // Selects which register (0, 1, or 2)
                       input [7:0] data_in_i,  // Data from CPU's ALU output

                       // --- Physical PWM Outputs ---
                       output      pmos1_o,
                       output      nmos2_o,
                       output      pmos2_o,
                       output      nmos1_o
                       ;)

  // Internal registers to hold PWM configuration
  reg [6:0] d1_reg;
   reg [6:0] d2_reg;
   reg [5:0] control_reg; // Holds dt[4:0] and enable[5]

   // Logic to update registers on a CPU write
   always @(posedge clk_i or negedge rst_n_i) begin
     if (!rst_n_i) begin
       d1_reg <= 7'd0;
       d2_reg <= 7'd0;
       control_reg <= 6'd0;
     end else if (cs_i && write_en_i) begin
       // The address decoder in the main CPU file will only assert cs_i
       // for the correct address range. We use the LSBs to pick the register.
       case (addr_lsb_i) // 0xF4 -> 00, 0xF5 -> 01, 0xF6 -> 10
         2'b00: d1_reg <= data_in_i[6:0];
         2'b01: d2_reg <= data_in_i[6:0];
         2'b10: control_reg <= data_in_i[5:0];
         default:;
       endcase
     end
   end // always @ (posedge clk_i or negedge rst_n_i)

   // Connect MMIO registers to the PWM core logic inputs
   wire [4:0] dt;
   wire       enable_output;
   
   assign dt = control_reg[4:0];
   assign enable_output = control_reg[5];

   wire [6:0] triangular_0;
   Signal_Generator_0phase Signal_Generator_1_0phase_inst(
                                                          CLK_PRIMARY,
                                                          RST,
                                                          triangular_0
                                                          );

   wire [6:0] triangular_180;
   Signal_Generator_180phase Signal_Generator_1_180phase_inst(
                                                              CLK_PRIMARY,
                                                              RST,
                                                              triangular_180
                                                              );

   wire Output_Comparison_1;
   Comparator Comparator_Inst_1(
                                d1,
                                triangular_0,
                                Output_Comparison_1
                                );

   wire Output_Comparison_2;
   Comparator Comparator_Inst_2(
                                d2,
                                triangular_180,
                                Output_Comparison_2
                                );

   wire pmos1_prev; 
   Dead_Time_Generator Dead_Time_Generator_inst_1(
                                                  CLK_PRIMARY,
                                                  dt,
                                                  Output_Comparison_1,
                                                  pmos1_prev
                                                  );

   wire Not_Output_Comparison_1;
   wire nmos2_prev;
   assign Not_Output_Comparison_1 = ~Output_Comparison_1;
   Dead_Time_Generator Dead_Time_Generator_inst_2(
                                                  CLK_PRIMARY,
                                                  dt,
                                                  Not_Output_Comparison_1,
                                                  nmos2_prev
                                                  );

   wire pmos2_prev;
   Dead_Time_Generator Dead_Time_Generator_inst_3(
                                                  CLK_PRIMARY,
                                                  dt,
                                                  Output_Comparison_2,
                                                  pmos2_prev
                                                  );

   wire Not_Output_Comparison_2;
   wire nmos1_prev;
   assign Not_Output_Comparison_2 = ~Output_Comparison_2;
   Dead_Time_Generator Dead_Time_Generator_inst_4(
                                                  CLK_PRIMARY,
                                                  dt,
                                                  Not_Output_Comparison_2,
                                                  nmos1_prev
                                                  );

   assign PMOS1 = ENABLE_OUTPUT ? ~pmos1_prev : 1;
   assign NMOS2 = ENABLE_OUTPUT ? nmos2_prev : 0;
   assign PMOS2 = ENABLE_OUTPUT ? ~pmos2_prev : 1;
   assign NMOS1 = ENABLE_OUTPUT ? nmos1_prev : 0;

endmodule
