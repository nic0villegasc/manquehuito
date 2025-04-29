`default_nettype none

module tb;
  logic clk_i;
  logic load_i;
  logic [7:0] data_i;
  logic [7:0] out_o;

  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0, tb);
  end

  // Clock generation
  initial begin
    clk_i = 0;
    forever #5 clk_i = ~clk_i; // 10ns period clock
  end

  // Instantiate DUT
  register dut (
    .clk_i(clk_i),
    .load_i(load_i),
    .data_i(data_i),
    .out_o(out_o)
  );

endmodule
