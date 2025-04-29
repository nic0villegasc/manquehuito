`default_nettype none

module tb;
  logic [7:0] a_i, b_i;
  logic [2:0] sel_i;
  logic [7:0] out_o;
  logic [3:0] zncv_o;

  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0, tb);
  end

  alu dut (
    .a_i(a_i),
    .b_i(b_i),
    .sel_i(sel_i),
    .out_o(out_o),
    .zncv_o(zncv_o)
  );
endmodule
