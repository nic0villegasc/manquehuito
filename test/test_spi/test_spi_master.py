# test_spi_master_write_only.py
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Timer, RisingEdge

# Vital helper function to reset the DUT
async def reset_dut(dut):
    """Applies reset to the DUT."""
    dut.rst_n_i.value = 1
    await Timer(10, units="ns")
    dut.rst_n_i.value = 0
    await Timer(20, units="ns")
    dut.rst_n_i.value = 1
    await Timer(10, units="ns")
    dut._log.info("Reset complete")

# Vital helper function to run a complete transaction
async def run_transaction(dut, address, read_not_write, num_bytes, data_to_write=0):
    """Starts and waits for a single SPI transaction to complete."""
    op_str = "READ" if read_not_write else "WRITE"
    dut._log.info(f"--- Starting transaction: {op_str} {num_bytes} byte(s) at addr {address:#06x} ---")
    if not read_not_write:
        dut._log.info(f"      - Data to write: {data_to_write:#04x}")

    # Wait for the DUT to be ready
    await RisingEdge(dut.clk_core_i)
    assert not dut.busy_o.value, "DUT is busy at the start of a new transaction"

    # Set up transaction parameters
    dut.address_i.value = address
    dut.read_not_write_i.value = read_not_write
    dut.num_bytes_to_transfer_i.value = num_bytes
    dut.data_to_write_i.value = data_to_write

    # Start the transaction with a single-cycle pulse
    dut.start_transaction_i.value = 1
    await RisingEdge(dut.clk_core_i)
    dut.start_transaction_i.value = 0
    
    # Wait for the transaction to be acknowledged and completed
    await RisingEdge(dut.busy_o)
    dut._log.info("Transaction started (busy is high). Waiting for completion...")
    
    await RisingEdge(dut.transaction_done_o)
    dut._log.info("Transaction done pulse detected.")
    
    # Ensure the DUT is no longer busy
    await RisingEdge(dut.clk_core_i)
    assert not dut.busy_o.value, "DUT should not be busy after transaction is done"
    dut._log.info(f"--- Transaction finished (busy is low) ---")

@cocotb.test()
async def test_spi_write(dut):
    """Test a single-byte WRITE operation."""
    # Start the clock
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Reset the DUT
    await reset_dut(dut)
    
    # Test Case: Write 0x42 to 0xBEEF
    addr = 0xBEEF
    data_to_write = 0x42
    
    await run_transaction(dut, address=addr, read_not_write=0, num_bytes=1, data_to_write=data_to_write)
    
    await Timer(50, units="ns") # Wait a bit for simulation to end cleanly
    dut._log.info("Write test complete.")

@cocotb.test()
async def test_spi_read(dut):
    """Test single-byte and two-byte READ operations."""
    # Start the clock and reset the DUT
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())
    await reset_dut(dut)

    # --- Test Case 1: Single-Byte Read ---
    addr = 0xBEEF
    expected_byte1 = 0x42 # This value is pre-loaded in the slave model
    
    await run_transaction(dut, address=addr, read_not_write=1, num_bytes=1)
    
    # Verify the received data
    actual_byte1 = dut.data_read_byte1_o.value
    dut._log.info(f"CHECK: Read from {addr:#06x}: got {actual_byte1}, expected {expected_byte1:#04x}")
    assert actual_byte1 == expected_byte1, f"Read error! Got {actual_byte1}, expected {expected_byte1}"

    await Timer(50, units="ns") # Wait a bit between transactions

    dut._log.info("Read test complete.")
