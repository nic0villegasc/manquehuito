# test_spi_master.py
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Timer
from cocotb.triggers import RisingEdge

# Make sure you have your reset_dut and SlaveMemory helpers available
from test_helpers import SlaveMemory, reset_dut, run_transaction

@cocotb.test()
async def backdoor_memory(dut):
    # 1. Start the clock
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())
    dut._log.info("Clock started")

    # 2. Reset the DUT to put it in a known state
    await reset_dut(dut)
    dut._log.info("DUT Reset")

    # 3. Now, you can safely interact with the design
    slave_mem = SlaveMemory(memory_signal=dut.SLAVE.memory, log=dut._log)

    data_to_write = [0xDE, 0xAD, 0xBE, 0xEF]
    slave_mem.write(0x1000, data_to_write)

    # Let simulation advance a tiny bit to be safe
    await Timer(1, units="ns")

    read_back_data = slave_mem.read(0x1000, 4)

    # This assertion should now pass
    assert read_back_data == data_to_write
    dut._log.info("SUCCESS: Read-back data matches written data.")

@cocotb.test()
async def test_read_preloaded(dut):
    """
    Test 1: Verify the DUT's READ command against a known value
              that was set via the backdoor.
    """
    # --- Test Setup ---
    # 1. Start the clock
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())
    dut._log.info("Clock started for test_read_preloaded")

    # 2. Reset the DUT
    await reset_dut(dut)

    # 3. Initialize the backdoor memory helper
    slave_mem = SlaveMemory(memory_signal=dut.SLAVE.memory, log=dut._log)

    # --- Parameterized Test Cases ---
    # A list of dictionaries, where each dictionary defines one test run.
    # The provided run_transaction helper supports reading up to 2 bytes.
    test_cases = [
        {"addr": 0x1234, "data": [0xAB]},
        {"addr": 0x5000, "data": [0xCA, 0xFE]},
        {"addr": 0x0000, "data": [0x55]},
        {"addr": 0xFFFE, "data": [0x88, 0x99]}, # Test near the top of memory
        {"addr": 0x2040, "data": [0x01, 0x02]},
    ]

    # --- Run Test Loop ---
    for i, case in enumerate(test_cases):
        test_addr = case["addr"]
        data_to_preload = case["data"]
        num_bytes_to_read = len(data_to_preload)

        dut._log.info(f"--- Running Test Case #{i}: Read {num_bytes_to_read} byte(s) from addr {test_addr:#06x} ---")

        # 1. Use the backdoor to write a known value into the slave's memory
        slave_mem.write(test_addr, data_to_preload)
        dut._log.info(f"Preloaded memory at {test_addr:#06x} with {[hex(d) for d in data_to_preload]}")

        # Add a small delay for the write to settle before starting the read
        await RisingEdge(dut.clk_core_i)

        # 2. Use the DUT to READ from that same address via the SPI master interface
        read_data_from_dut = await run_transaction(
            dut,
            address=test_addr,
            read_bytes=num_bytes_to_read
        )

        # 3. Assert that the data returned by the DUT matches the original value
        assert read_data_from_dut == data_to_preload, \
            f"Mismatch in test case #{i}! Expected {[hex(d) for d in data_to_preload]}, Got {[hex(d) for d in read_data_from_dut]}"

        dut._log.info(f"SUCCESS for test case #{i}: Read data matches preloaded value.")

        # Wait a few cycles between transactions for cleaner waves
        for _ in range(5):
            await RisingEdge(dut.clk_core_i)

@cocotb.test()
async def test_write_and_backdoor_check(dut):
    """
    Test 3: Verify the DUT's WRITE command by checking with backdoor access.

    This test commands the DUT to write data, then uses backdoor access
    to the slave's memory to verify the data was written correctly.
    This isolates the DUT's write functionality for testing.
    """
    # --- Test Setup ---
    # 1. Start the clock
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())
    dut._log.info("Clock started for test_write_and_backdoor_check")

    # 2. Reset the DUT
    await reset_dut(dut)

    # 3. Initialize the backdoor memory helper
    slave_mem = SlaveMemory(memory_signal=dut.SLAVE.memory, log=dut._log)
    dut._log.info("DUT Reset and SlaveMemory helper initialized.")

    # --- Parameterized Test Cases ---
    test_cases = [
        {"addr": 0x1000, "data": [0xAA, 0x55]},
        {"addr": 0xBEEF, "data": [0x12, 0x34, 0x56, 0x78]},
        {"addr": 0x0000, "data": [0x01]},
        {"addr": 0xFFF0, "data": [0xF0, 0xE1, 0xD2, 0xC3]},
    ]

    # --- Run Test Loop ---
    for i, case in enumerate(test_cases):
        start_addr = case["addr"]
        data_to_write = case["data"]
        num_bytes = len(data_to_write)

        dut._log.info(f"--- Running Test Case #{i}: DUT Write at addr {start_addr:#06x}, verify with backdoor ---")

        # 1. Use the DUT to WRITE data using single-byte transactions
        for byte_index, byte_value in enumerate(data_to_write):
            current_addr = start_addr + byte_index
            dut._log.info(f"  DUT Writing byte {byte_index+1}/{num_bytes}: {hex(byte_value)} to addr {current_addr:#06x}")
            await run_transaction(
                dut,
                address=current_addr,
                write_data=[byte_value]
            )
            await RisingEdge(dut.clk_core_i)

        # Wait a moment for writes to settle
        await Timer(1, units="ns")

        # 2. Use the BACKDOOR to read the data directly from the slave's memory
        dut._log.info(f"  Backdoor reading {num_bytes} bytes from start addr {start_addr:#06x}")
        read_back_data = slave_mem.read(start_addr, num_bytes)

        # 3. Assert that the data read via the backdoor matches the data written by the DUT
        assert read_back_data == data_to_write, \
            f"MISMATCH in test case #{i}! DUT wrote {[hex(d) for d in data_to_write]}, but backdoor read {[hex(d) for d in read_back_data]}"

        dut._log.info(f"SUCCESS for test case #{i}: Backdoor data matches DUT-written data.")

        # Wait a few cycles before the next test case
        for _ in range(5):
            await RisingEdge(dut.clk_core_i)


@cocotb.test()
async def test_write_read_cycle(dut):
    """
    Test 2: Verify the full round-trip WRITE then READ cycle.

    This test uses the DUT's SPI master interface for both writing
    and reading. It respects the DUT constraint of writing only one
    byte per transaction by sequencing multi-byte writes.
    """
    # --- Test Setup ---
    # 1. Start the clock
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())
    dut._log.info("Clock started for test_write_read_cycle")

    # 2. Reset the DUT
    await reset_dut(dut)
    dut._log.info("DUT Reset")

    # --- Parameterized Test Cases ---
    # A list of dictionaries, where each dictionary defines one test run.
    # Each case specifies a start address and the data to write/read.
    test_cases = [
        {"addr": 0x2000, "data": [0xDE, 0xAD]},
        {"addr": 0xCAFE, "data": [0xBE, 0xEF]},
        {"addr": 0x0001, "data": [0xFF]},
        {"addr": 0xFFFF, "data": [0x00]}, # Test edges of memory map
        {"addr": 0x8765, "data": [0x43, 0x21]},
    ]

    # --- Run Test Loop ---
    for i, case in enumerate(test_cases):
        start_addr = case["addr"]
        data_to_write = case["data"]
        num_bytes = len(data_to_write)

        dut._log.info(f"--- Running Test Case #{i}: Write/Read {num_bytes} byte(s) at start addr {start_addr:#06x} ---")

        # 1. Use the DUT to WRITE a known value to the slave via the SPI master interface.
        # Since the DUT can only write one byte per transaction, we loop for multi-byte data,
        # incrementing the address for each byte.
        dut._log.info(f"DUT Write to {start_addr:#06x} with data {[hex(d) for d in data_to_write]}")
        for byte_index, byte_value in enumerate(data_to_write):
            current_addr = start_addr + byte_index
            dut._log.info(f"  Writing byte {byte_index+1}/{num_bytes}: {hex(byte_value)} to addr {current_addr:#06x}")
            await run_transaction(
                dut,
                address=current_addr,
                write_data=[byte_value]  # Pass a list containing the single byte
            )
            # Small delay between consecutive write transactions
            await RisingEdge(dut.clk_core_i)


        # Wait a few clock cycles for the transaction sequence to fully complete and
        # to create a clean visual separation in the waveform dump before the read.
        for _ in range(10):
            await RisingEdge(dut.clk_core_i)

        # 2. Use the DUT to READ from the start address via the SPI master interface.
        # The read operation can fetch all the bytes written in the previous steps.
        dut._log.info(f"DUT Read {num_bytes} bytes from start addr {start_addr:#06x}")
        read_data_from_dut = await run_transaction(
            dut,
            address=start_addr,
            read_bytes=num_bytes
        )

        # 3. Assert that the data read back by the DUT matches the original written data
        assert read_data_from_dut == data_to_write, \
            f"MISMATCH in test case #{i}! Wrote {[hex(d) for d in data_to_write]}, but read {[hex(d) for d in read_data_from_dut]}"

        dut._log.info(f"SUCCESS for test case #{i}: Read-back data matches written data.")

        # Wait a few more cycles before the next test case
        for _ in range(5):
            await RisingEdge(dut.clk_core_i)
