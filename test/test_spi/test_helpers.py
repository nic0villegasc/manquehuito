# test_helpers.py

import cocotb
from cocotb.triggers import Timer, RisingEdge

class SlaveMemory:
    """
    Helper class to provide backdoor access to a simulated memory array.
    This makes tests cleaner by abstracting the direct memory manipulation.
    """
    def __init__(self, memory_signal, log):
        """
        Initializes the SlaveMemory helper.

        Args:
            memory_signal: A direct handle to the memory array signal in the DUT
                           (e.g., dut.SLAVE.memory).
            log: The logger instance to use (e.g., dut._log).
        """
        self._memory = memory_signal
        self._log = log
        self._log.info("SlaveMemory helper initialized.")

    def write(self, address, data_list):
        """
        Writes a list of bytes to the memory via backdoor access.

        Args:
            address (int): The starting memory address.
            data_list (list): A list of integer byte values to write.
        """
        self._log.info(f"BACKDOOR WRITE of {len(data_list)} bytes to address {address:#06x}")
        for i, data_byte in enumerate(data_list):
            target_addr = address + i
            # Assign to the '.value' attribute for cocotb handles
            self._memory[target_addr].value = data_byte
            self._log.debug(f"  - Wrote {data_byte:#04x} to address {target_addr:#06x}")

    def read(self, address, num_bytes):
        """
        Reads a number of bytes from the memory via backdoor access.

        Args:
            address (int): The starting memory address.
            num_bytes (int): The number of bytes to read.

        Returns:
            list: A list of integer byte values read from memory.
        """
        self._log.info(f"BACKDOOR READ of {num_bytes} bytes from address {address:#06x}")
        read_data = []
        for i in range(num_bytes):
            target_addr = address + i
            # Convert the read LogicValue to an integer
            data_val = int(self._memory[target_addr].value)
            read_data.append(data_val)
            self._log.debug(f"  - Read {data_val:#04x} from address {target_addr:#06x}")
        return read_data

# Generic helper function to reset the DUT
async def reset_dut(dut):
    """Applies an active-low reset to the DUT."""
    dut._log.info("Applying reset...")
    # Drive reset high initially to avoid X->0 transition
    dut.rst_n_i.value = 1
    await Timer(10, units="ns")
    # Drive reset low
    dut.rst_n_i.value = 0
    await Timer(20, units="ns") # Keep reset active for a couple of cycles
    # Release reset
    dut.rst_n_i.value = 1
    await Timer(10, units="ns")
    dut._log.info("Reset complete.")

async def run_transaction(dut, address, *, write_data=None, read_bytes=0):
    """
    Runs a single SPI master transaction (either a write or a read).

    This function is the primary interface for controlling the DUT.

    Args:
        dut: The DUT object.
        address (int): The 16-bit address for the transaction.
        write_data (list[int], optional): A list of bytes to write.
                                          If provided, performs a WRITE.
        read_bytes (int, optional): The number of bytes to read.
                                    If > 0, performs a READ.

    Returns:
        list[int] or None: A list of bytes read from the DUT for a read operation,
                           otherwise None.
    """
    # --- 1. Argument validation and setup ---
    is_a_write = write_data is not None
    is_a_read = read_bytes > 0

    assert is_a_write != is_a_read, "Specify either 'write_data' or 'read_bytes', not both or neither."

    if is_a_write:
        num_bytes = len(write_data)
        data_to_send = write_data[0] # DUT has one 8-bit data input port
        op_str = "WRITE"
        if num_bytes > 1:
            dut._log.warning(f"This DUT only supports writing a single byte per transaction. Sending the first byte: {data_to_send:#04x}")
    else:  # is_a_read
        num_bytes = read_bytes
        data_to_send = 0  # Value is ignored for reads
        op_str = "READ"

    dut._log.info(f"--- Starting transaction: {op_str} {num_bytes} byte(s) at addr {address:#06x} ---")

    # --- 2. Wait for DUT to be ready and drive inputs ---
    await RisingEdge(dut.clk_core_i)
    assert not dut.busy_o.value, "DUT is busy at the start of a new transaction"

    dut.address_i.value = address
    dut.read_not_write_i.value = not is_a_write
    dut.num_bytes_to_transfer_i.value = num_bytes
    dut.data_to_write_i.value = data_to_send

    # --- 3. Start transaction with a single-cycle pulse ---
    dut.start_transaction_i.value = 1
    await RisingEdge(dut.clk_core_i)
    dut.start_transaction_i.value = 0

    # --- 4. Wait for the transaction to complete ---
    await RisingEdge(dut.busy_o)
    dut._log.info("Transaction started (busy is high). Waiting for completion...")

    await RisingEdge(dut.transaction_done_o)
    dut._log.info("Transaction done pulse detected.")
    
    # Wait one more cycle for busy to de-assert
    await RisingEdge(dut.clk_core_i)
    assert not dut.busy_o.value, "DUT should not be busy after transaction is done"

    # --- 5. Collect and return data for READ operations ---
    if is_a_read:
        read_results = []
        if num_bytes >= 1:
            read_results.append(int(dut.data_read_byte1_o.value))
        if num_bytes >= 2:
            read_results.append(int(dut.data_read_byte2_o.value))
        # Extend here if your DUT supports reading more bytes in one transaction
        # if num_bytes >= 3:
        #     read_results.append(int(dut.data_read_byte3_o.value))

        dut._log.info(f"Read data: {[hex(val) for val in read_results]}")
        dut._log.info("--- Transaction finished ---")
        return read_results

    # For WRITE operations, we're done
    dut._log.info("--- Transaction finished ---")
    return None
