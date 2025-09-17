`timescale 1ns / 1ps

module Signal_Generator_0phase #(parameter WIDTH_TRIANG = 7)(
                                                             input                         clk,  // Reloj de entrada
                                                             input                         rst,  // Senal de reinicio
                                                             output reg [WIDTH_TRIANG-1:0] count // Salida triangular de 12 bits
                                                             );

   //reg [WIDTH_TRIANG-1:0] count; // contador de 12 bits
   reg direction;

   always @(posedge clk or posedge rst) begin
     if (rst) begin
       count <= 0;
       direction <= 1;
     end
     else begin
       if (direction) begin
         if (count == 7'b1111111) begin // 64 en binario
           direction <= 0;
           count <= 7'b1111110 ; end 
         else
           count <= count + 1;
       end
       else begin
         if (count == 7'b0000000) begin// 0 en binario
           direction <= 1;
           count <= 7'b0000001; end
         else
           count <= count - 1;
       end
     end
   end

   //assign triangular_out = count;

endmodule
