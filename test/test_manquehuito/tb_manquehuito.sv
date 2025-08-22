`default_nettype none

// =================================================================
// --- Behavioral SPI Slave RAM Model ---
// Copied from your spi_master testbench (tb_spi.txt)
// =================================================================
module spi_slave_ram_model #(
    parameter ADDR_WIDTH = 16
)(
  input wire spi_rst_n_i,
  input wire spi_sclk_i,
  input wire spi_cs_i,
  input wire spi_mosi_i,
  output reg spi_miso_o
);

    // --- Command Opcodes ---
    localparam CMD_WRITE = 8'h02;
    localparam CMD_READ  = 8'h03;

    // --- FSM States ---
    localparam S_IDLE       = 3'd0;
    localparam S_GET_CMD    = 3'd1;
    localparam S_GET_ADDR   = 3'd2;
    localparam S_READ_DATA  = 3'd3;
    localparam S_WRITE_DATA = 3'd4;

    // --- Internal Registers ---
    reg [2:0] state_reg; // No longer need state_next
    reg [2:0] bit_count;
    reg       addr_byte_count;
    reg [7:0] mosi_shifter;
    reg [7:0] miso_shifter;
    reg [7:0] cmd_reg;
    reg [ADDR_WIDTH-1:0] addr_reg;

    // --- Memory Array ---
    reg [7:0] memory [0:(2**ADDR_WIDTH)-1];

   reg [ADDR_WIDTH-1:0] addr_temp_reg;

   // Inside spi_slave_ram_model module
   initial begin
     for (integer i = 0; i < (2**ADDR_WIDTH); i = i + 1) begin
       memory[i] = 8'hCA;
     end
   end

    // --- FSM and Datapath Logic (Single Clocked Block) ---
    always @(posedge spi_sclk_i or negedge spi_rst_n_i or posedge spi_cs_i) begin
        if (!spi_rst_n_i) begin
            // Reset all stateful registers
            state_reg       <= S_IDLE;
            bit_count       <= 3'd0;
            addr_byte_count <= 1'b0;
            cmd_reg         <= 8'h00;
            addr_reg        <= 0;
        end else begin
            // If Chip Select is high, always reset the transaction state
            if (spi_cs_i) begin
                state_reg <= S_IDLE;
            end else begin
                // Otherwise, execute the FSM logic clocked by spi_sclk_i
                case (state_reg)
                    S_IDLE: begin
                        // This is the first clock cycle with CS low
                      state_reg <= S_GET_CMD;
                      bit_count <= 3'd0;
                      mosi_shifter <= {mosi_shifter[6:0], spi_mosi_i};
                    end

                    S_GET_CMD: begin
                        mosi_shifter <= {mosi_shifter[6:0], spi_mosi_i};
                        if (bit_count == 3'd7) begin
                            state_reg <= S_GET_ADDR;
                            cmd_reg <= mosi_shifter[7:0];
                            addr_byte_count <= 1'b0;
                            bit_count <= 3'd0;
                        end else begin
                          bit_count <= bit_count + 1;
                        end
                    end

                    S_GET_ADDR: begin
                      mosi_shifter <= {mosi_shifter[6:0], spi_mosi_i};
                      bit_count    <= bit_count + 1;

                      if (bit_count == 3'd6 && cmd_reg == CMD_READ && addr_byte_count == 1'b1) begin
                        miso_shifter  <= memory[{addr_reg[ADDR_WIDTH-1:8], {mosi_shifter[6:0], spi_mosi_i}}];
                        addr_temp_reg <= {addr_reg[ADDR_WIDTH-1:8], {mosi_shifter[6:0], spi_mosi_i}};
                        spi_miso_o <= memory[{addr_reg[ADDR_WIDTH-1:8], {mosi_shifter[6:0], spi_mosi_i}}][7];
                      end

                        if (bit_count == 3'd7) begin
                            bit_count <= 3'd0;
                            if (addr_byte_count == 1'b0) begin // First (MSB) byte
                                addr_reg[ADDR_WIDTH-1:8] <= mosi_shifter[7:0];
                                addr_byte_count <= 1'b1;
                            end else begin // Second (LSB) byte
                                addr_reg[7:0] <= mosi_shifter[7:0];
                                // Now transition to the next state based on the command
                                case (cmd_reg)
                                    CMD_READ: begin
                                      state_reg    <= S_READ_DATA;
                                      miso_shifter <= {miso_shifter[6:0], 1'b0};
                                    end
                                      CMD_WRITE: state_reg <= S_WRITE_DATA;
                                      default:   state_reg <= S_IDLE;
                                endcase
                            end
                        end
                    end

                    S_WRITE_DATA: begin
                        mosi_shifter <= {mosi_shifter[6:0], spi_mosi_i};
                        bit_count <= bit_count + 1;
                        if (bit_count == 3'd7) begin
                            memory[addr_reg] <= {mosi_shifter[6:0], spi_mosi_i};
                            addr_reg <= addr_reg + 1;
                        end
                    end

                    S_READ_DATA: begin
                    end

                    default: begin
                        state_reg <= S_IDLE;
                    end
                endcase
            end
        end
    end // always @ (posedge spi_sclk_i or negedge spi_rst_n_i)

   // --- MISO Output Logic (Falling Edge Triggered) ---
   always @(negedge spi_sclk_i or negedge spi_rst_n_i) begin
     if (!spi_rst_n_i) begin
       spi_miso_o <= 1'b1; // Default state during reset
     end else if (spi_cs_i) begin
       spi_miso_o <= 1'b1; // When not selected, MISO should be high (simulating Hi-Z)
     end else begin
       // If we are in the read state, output the MSB of the shifter.
       // The shifter itself is updated on the RISING edge.
       if (state_reg == S_READ_DATA) begin
         if (bit_count == 3'b0) begin
           miso_shifter <= {miso_shifter[6:0], 1'b0};
         end else begin
           miso_shifter <= {miso_shifter[6:0], 1'b0};
         end
         bit_count <= bit_count + 1;

         spi_miso_o <= miso_shifter[7];

           if (bit_count == 3'd7) begin
             addr_reg     <= addr_reg + 1;
             miso_shifter <= {memory[addr_reg + 1][6:0], 1'b0};
             spi_miso_o   <= memory[addr_reg + 1][7];
             bit_count    <= 3'b0;
           end
       end else if (state_reg == S_WRITE_DATA) begin
         if (bit_count == 3'd7) begin
           memory[addr_reg] <= mosi_shifter[7:0];
           addr_reg         <= addr_reg + 1;
         end
       end
     end
   end

endmodule

// =================================================================
// --- Testbench for the full manquehuito_domain ---
// =================================================================
module tb;

   // Clock and Reset for the DUT
   logic clk_core_i;
   logic rst_n_i;

   // SPI Bus signals - these are now wires between the DUT and the Slave model
   wire spi_sclk_o;
   wire spi_mosi_o;
   wire spi_cs_o;
   wire spi_miso_i; // Driven by the slave model

   // Instantiate the Device Under Test (DUT)
   manquehuito_domain dut (
                           .clk_core_i(clk_core_i),
                           .rst_n_i(rst_n_i),
                           .spi_sclk_o(spi_sclk_o),
                           .spi_mosi_o(spi_mosi_o),
                           .spi_cs_o(spi_cs_o),
                           .spi_miso_i(spi_miso_i)
                           );

   // NEW: Instantiate the behavioral SPI Slave model
   spi_slave_ram_model #(
    .ADDR_WIDTH(16)
   ) SLAVE (
     .spi_rst_n_i(rst_n_i),    // Connect to the main testbench reset
     .spi_sclk_i(spi_sclk_o),  // Master's SCLK output is slave's input
     .spi_cs_i(spi_cs_o),      // Master's CS output is slave's input
     .spi_mosi_i(spi_mosi_o),   // Master's MOSI output is slave's input
     .spi_miso_o(spi_miso_i)    // Slave's MISO output is master's input
   );


   // Standard VCD dump for waveform viewing
   initial begin
     $dumpfile("dump.vcd");
     $dumpvars(0, tb);
   end

endmodule
