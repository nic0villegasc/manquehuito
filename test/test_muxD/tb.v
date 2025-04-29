`default_nettype none

module tb;
  logic [7:0] e0_i, e1_i;
  logic sel_i;
  logic [7:0] out_o;

  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0, tb);
  end

  // Instantiate DUT (muxD)
  muxD dut (
    .e0_i(e0_i),
    .e1_i(e1_i),
    .sel_i(sel_i),
    .out_o(out_o)
  );

endmodule
