import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
from cocotb.result import TestFailure


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

# --- The Tests ---

#@cocotb.test()
async def test_instruction_fetch(dut):
    """Test that the control unit can fetch a 2-byte instruction via SPI."""

    # 1. Define a sample instruction to be fetched.
    # Opcode 'MOV A,Lit' (0000010) with literal 0xAB.
    opcode = 0b0000010
    literal = 0xAB
    instruction_word = (opcode << 8) | literal

    # Split into two bytes for memory
    byte1 = (instruction_word >> 7) & 0xFF
    byte2 = instruction_word & 0x7F

    dut._log.info(f"Test instruction: {instruction_word:#06x}")
    dut._log.info(f"  - Byte @ addr 0: {byte1:#04x}")
    dut._log.info(f"  - Byte @ addr 1: {byte2:#04x}")

    # 2. NEW: Initialize the backdoor memory helper
    # The path 'dut.SLAVE.memory' matches the instance names in tb_manquehuito.sv
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)

    # 3. Start the clock
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())
    # REMOVED: No longer need to start the Python slave model.

    # 4. Reset the DUT
    await reset_dut(dut)

    # 5. NEW: Pre-load the slave's memory using the backdoor
    # The control unit will fetch from PC=0, so write the instruction there.
    slave_mem.write(0, [byte1, byte2])

    # 6. Wait for the fetch to complete.
    # The DUT FSM will start automatically after reset.
    # We wait for the 'transaction_done_o' signal from the spi_master inside the DUT.
    await RisingEdge(dut.dut.i_spi_master.transaction_done_o)

    dut._log.info("SPI transaction reported done by the master.")

    # 7. Verify the result
    # Give it one clock cycle for the control unit to latch the instruction.
    await RisingEdge(dut.clk_core_i)
    
    await RisingEdge(dut.clk_core_i)

    actual_instruction = dut.dut.i_ctrl.instruction_r.value.integer

    dut._log.info(f"Expected instruction: {instruction_word:#06x}")
    dut._log.info(f"Actual instruction:   {actual_instruction:#06x}")

    assert actual_instruction == instruction_word, \
        f"Instruction mismatch! Expected {instruction_word:#06x}, got {actual_instruction:#06x}"

    dut._log.info("Instruction fetch test passed!")


@cocotb.test()
async def test_add_instruction(dut):
    """Tests fetching and executing an 'ADD A, B' instruction."""

    # 1. Define instruction and test values
    # Opcode 'ADD A, B' is 7'b0000100
    opcode = 0b0000100
    # The literal/immediate field is unused for this instruction
    instruction_word = (opcode << 8) | 0x00

    # Split the 15-bit instruction into two bytes for memory
    byte1 = (instruction_word >> 7) & 0xFF
    byte2 = instruction_word & 0x7F

    # Define initial register values and the expected result
    reg_a_val = 0x11
    reg_b_val = 0x22
    expected_sum = reg_a_val + reg_b_val

    dut._log.info(f"Test instruction: ADD A, B ({instruction_word:#06x})")
    dut._log.info(f"  - Instruction Bytes: [{byte1:#04x}, {byte2:#04x}]")
    dut._log.info(f"Initial Reg A: {reg_a_val:#04x}, Reg B: {reg_b_val:#04x}")
    dut._log.info(f"Expected Sum in Reg A: {expected_sum:#04x}")

    # 2. Initialize test environment
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # 3. Reset the DUT
    await reset_dut(dut)

    # 4. Pre-load state using backdoor access
    # Load the instruction into the slave memory at address 0
    slave_mem.write(0, [byte1, byte2])
    dut._log.info("Wrote instruction to simulated memory.")

    # Directly set the initial values of registers A and B
    # Note: This requires knowledge of the register's internal signal name ('reg_q').
    # This is a common and efficient method for test setup.
    dut.dut.reg_a_out.value = reg_a_val
    dut.dut.reg_b_out.value = reg_b_val
    await RisingEdge(dut.clk_core_i) # Allow values to propagate through the design
    dut._log.info("Pre-loaded registers A and B via backdoor.")

    # 5. Wait for the fetch and execute cycles
    # The processor automatically starts fetching from PC=0 after reset
    # Wait for the SPI master to signal the fetch is complete
    await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
    dut._log.info("SPI transaction (instruction fetch) is done.")

    # After the fetch, the FSM needs two clock cycles to:
    # 1st cycle: Transition to DECODE_EXECUTE and calculate the sum.
    # 2nd cycle: Latch the ALU result into Register A.
    await RisingEdge(dut.clk_core_i)
    await RisingEdge(dut.clk_core_i)
    await RisingEdge(dut.clk_core_i)

    # 6. Verify the results
    # a) Verify that the correct instruction was latched by the control unit
    actual_instruction = dut.dut.i_ctrl.instruction_r.value.integer
    dut._log.info(f"Fetched instruction: {actual_instruction:#06x}")
    assert actual_instruction == instruction_word, \
        f"Instruction fetch mismatch! Expected {instruction_word:#06x}, got {actual_instruction:#06x}"

    # b) Verify that Register A now holds the sum of the initial values
    final_reg_a_val = dut.dut.i_reg_a.out_o.value.integer
    dut._log.info(f"Final Reg A value: {final_reg_a_val:#04x}")
    assert final_reg_a_val == expected_sum, \
        f"ALU result mismatch! Expected {expected_sum:#04x}, got {final_reg_a_val:#04x}"

    dut._log.info("ADD A, B instruction fetch and execute test passed!")

@cocotb.test()
async def test_store_direct_instruction(dut):
    """Tests a direct memory store operation: MOV (Dir), A."""

    # 1. ⚙️ Setup: Define the instruction and test values
    # Opcode 'MOV (Dir),A' is 7'b0100111
    opcode = 0b0100111
    # The literal (Dir) is the memory address to write to.
    mem_address = 0x60
    instruction_word = (opcode << 8) | mem_address

    # Define the data in Register A that we want to store in memory.
    data_to_store = 0xAA

    # Split the instruction into bytes for memory storage
    byte1 = (instruction_word >> 7) & 0xFF
    byte2 = instruction_word & 0x7F

    dut._log.info(f"Test instruction: MOV ({mem_address:#04x}), A -> {instruction_word:#06x}")
    dut._log.info(f"  - Instruction Bytes: [{byte1:#04x}, {byte2:#04x}]")
    dut._log.info(f"Initial Reg A value (data to store): {data_to_store:#04x}")
    dut._log.info(f"Expected value at Mem[{mem_address:#04x}]: {data_to_store:#04x}")

    # 2. Pre-load State: Set up the memory and register
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Write the instruction to PC=0
    slave_mem.write(0, [byte1, byte2])

    # Pre-load Register A with the value we want to store
    dut.dut.reg_a_out.value = data_to_store

    # 3. Execution: Reset the DUT and let it run
    await reset_dut(dut)

    # The processor will now perform two sequential SPI transactions:
    # 1. Fetch the instruction from address 0x00.
    await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
    dut._log.info("Instruction Fetch complete.")

    # 2. Write the data from Register A to the specified memory address.
    await RisingEdge(dut.clk_core_i) # Cycle for FSM to move to MEM_ACCESS_START
    await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
    dut._log.info("Data Store to memory complete.")
    await RisingEdge(dut.clk_core_i)

    # 4. Verification: Check the final memory content
    # Use the backdoor to read the value from the target memory address
    stored_value = slave_mem.read(mem_address, 1) # Read 1 byte
    final_mem_val = stored_value[0]

    dut._log.info(f"Final value in Mem[{mem_address:#04x}]: {final_mem_val:#04x}")

    assert final_mem_val == data_to_store, \
        f"Memory Store failed! Expected {data_to_store:#04x}, got {final_mem_val:#04x}"

    dut._log.info("MOV (Dir), A instruction test passed!")

@cocotb.test()
async def test_increment_memory_direct(dut):
    """Tests a read-modify-write operation: INC (Dir)."""

    # 1. Setup: Define the instruction and test values
    # Opcode 'INC (Dir)' is 7'b1001001
    opcode = 0b1001001
    # The literal (Dir) is the memory address to read from and write to.
    mem_address = 0x70
    instruction_word = (opcode << 8) | mem_address

    # Define the initial value in memory and the expected final value.
    initial_mem_val = 0x41
    expected_mem_val = initial_mem_val + 1

    # Split the instruction into bytes for memory storage
    byte1 = (instruction_word >> 7) & 0xFF
    byte2 = instruction_word & 0x7F

    dut._log.info(f"Test instruction: INC ({mem_address:#04x}) -> {instruction_word:#06x}")
    dut._log.info(f"  - Instruction Bytes: [{byte1:#04x}, {byte2:#04x}]")
    dut._log.info(f"Initial value at Mem[{mem_address:#04x}]: {initial_mem_val:#04x}")
    dut._log.info(f"Expected final value at Mem[{mem_address:#04x}]: {expected_mem_val:#04x}")

    # 2. Pre-load State: Set up the memory
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Write the instruction to PC=0 and the initial data to the target address
    slave_mem.write(0, [byte1, byte2])
    slave_mem.write(mem_address, [initial_mem_val])

    # 3. Execution: Reset the DUT and let it run
    await reset_dut(dut)

    # This operation requires THREE sequential SPI transactions:
    # 1. Fetch the instruction (Read).
    await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
    dut._log.info("1. Instruction Fetch complete (Read).")
    
    # 2. Read the data from the memory address (Read).
    await RisingEdge(dut.clk_core_i)
    await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
    dut._log.info("2. Data Load from memory complete (Read).")

    # 3. Write the incremented data back to the same memory address (Write).
    await RisingEdge(dut.clk_core_i)
    await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
    dut._log.info("3. Incremented data store to memory complete (Write).")
    await RisingEdge(dut.clk_core_i)

    # 4. Verification: Check the final memory content
    # Use the backdoor to read the value from the target memory address
    stored_value = slave_mem.read(mem_address, 1) # Read 1 byte
    final_mem_val = stored_value[0]

    dut._log.info(f"Final value in Mem[{mem_address:#04x}]: {final_mem_val:#04x}")

    assert final_mem_val == expected_mem_val, \
        f"INC (Dir) failed! Expected {expected_mem_val:#04x}, got {final_mem_val:#04x}"

    dut._log.info("INC (Dir) instruction test passed!")

def twos_comp_to_signed(val, bits):
    """Converts a two's complement integer to a signed integer."""
    if (val & (1 << (bits - 1))) != 0: # if the sign bit is set
        val = val - (1 << bits)        # compute the negative value
    return val

@cocotb.test()
async def test_program_flow(dut):
    """
    Tests a 4-instruction program flow:
    1. MOV B, 0x10      ; Load immediate value into Reg B
    2. MOV A, (0x50)    ; Load value from memory into Reg A
    3. ADD A, B         ; Add Reg B to Reg A
    4. MOV (0x51), A    ; Store result from Reg A into memory
    """

    # 1. Setup: Assemble the program and define the test data

    # Define memory locations for data
    data_addr_a = 0x50
    result_addr = 0x51

    # Define test values
    immediate_val_b = 0x10
    initial_mem_val_a = 0x25
    expected_result = immediate_val_b + initial_mem_val_a  # 0x10 + 0x25 = 0x35

    # Assemble the program instruction by instruction
    # [Opcode, Literal/Address]
    program = [
        [0b0000011, immediate_val_b], # MOV B, Lit
        [0b0100101, data_addr_a],   # MOV A, (Dir)
        [0b0000100, 0x00],           # ADD A, B (no literal)
        [0b0100111, result_addr],    # MOV (Dir), A
    ]

    # Convert the program into a stream of bytes for memory
    program_bytes = []
    for opcode, operand in program:
        instruction_word = (opcode << 8) | operand
        byte1 = (instruction_word >> 7) & 0xFF
        byte2 = instruction_word & 0x7F
        program_bytes.extend([byte1, byte2])

    dut._log.info("--- Test Program Flow ---")
    dut._log.info(f"Program will be loaded at address 0x00.")
    dut._log.info(f"Initial data at Mem[{data_addr_a:#04x}]: {initial_mem_val_a:#04x}")
    dut._log.info(f"Immediate value for Reg B: {immediate_val_b:#04x}")
    dut._log.info(f"Expected result at Mem[{result_addr:#04x}]: {expected_result:#04x}")

    # 2. Pre-load State: Write the program and initial data to memory
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Write program starting at PC = 0
    slave_mem.write(0, program_bytes)
    # Write the initial data value
    slave_mem.write(data_addr_a, [initial_mem_val_a])
    # Ensure the result location is initially zero to prevent false positives
    slave_mem.write(result_addr, [0x00])

    # 3. Execution: Reset the DUT and let it run the full program
    await reset_dut(dut)

    # We expect 6 SPI transactions in total for this program:
    # - 4 Instruction Fetches
    # - 1 Data Read (for MOV A, (Dir))
    # - 1 Data Write (for MOV (Dir), A)
    num_transactions = 6
    for i in range(num_transactions):
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Transaction {i+1}/{num_transactions} complete.")
    
    # Wait a few extra cycles for the final write-back to stabilize
    for _ in range(5):
        await RisingEdge(dut.clk_core_i)

    # 4. Verification: Read back the result from memory and check it
    final_value_in_mem = slave_mem.read(result_addr, 1)[0]
    dut._log.info(f"Final value read from Mem[{result_addr:#04x}]: {final_value_in_mem:#04x}")

    assert final_value_in_mem == expected_result, \
        f"Program flow failed! Expected {expected_result:#04x}, got {final_value_in_mem:#04x}"

    dut._log.info("Program flow test passed!")

@cocotb.test()
async def test_load_and_add_direct(dut):
    """
    Tests a two-instruction sequence:
    1. MOV A, Lit      ; Load an initial value into Register A.
    2. ADD A, (Dir)    ; Read from memory, add to A, store result in A.
    """

    # 1. Setup: Define the instructions and test values
    # Define opcodes
    opcode_mov_a_lit = 0b0000010
    opcode_add_a_dir = 0b0101100

    # Define memory address for the ADD operation
    mem_address = 0x50

    # Define the initial values for the operation
    initial_reg_a = 0x12
    value_in_mem = 0x34
    expected_sum = initial_reg_a + value_in_mem # 0x12 + 0x34 = 0x46

    # Assemble the program instruction by instruction
    # [Opcode, Literal/Address]
    program = [
        [opcode_mov_a_lit, initial_reg_a], # MOV A, 0x12
        [opcode_add_a_dir, mem_address]    # ADD A, (0x50)
    ]

    # Convert the program into a stream of bytes for memory
    program_bytes = []
    for opcode, operand in program:
        instruction_word = (opcode << 8) | operand
        byte1 = (instruction_word >> 7) & 0xFF
        byte2 = instruction_word & 0x7F
        program_bytes.extend([byte1, byte2])

    dut._log.info("--- Test: MOV A, Lit -> ADD A, (Dir) ---")
    dut._log.info(f"Program will load {initial_reg_a:#04x} into Reg A, then add value from Mem[{mem_address:#04x}].")
    dut._log.info(f"Value at Mem[{mem_address:#04x}]: {value_in_mem:#04x}")
    dut._log.info(f"Expected final Reg A value (Sum): {expected_sum:#04x}")

    # 2. Pre-load State: Set up the memory
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Write the program to PC=0 and the data to the target address
    slave_mem.write(0, program_bytes)
    slave_mem.write(mem_address, [value_in_mem])

    # 3. Execution: Reset the DUT and let it run
    await reset_dut(dut)

    # The processor will now perform three sequential SPI transactions:
    # 1. Fetch the 'MOV A, Lit' instruction.
    # 2. Fetch the 'ADD A, (Dir)' instruction.
    # 3. Read the data from the specified memory address for the ADD.
    num_transactions = 3
    for i in range(num_transactions):
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Transaction {i+1}/{num_transactions} complete.")

    # A few extra clock cycles are needed for the final ALU operation and write-back.
    for _ in range(5):
        await RisingEdge(dut.clk_core_i)

    # 4. Verification: Check the final result in Register A
    final_reg_a_val = dut.dut.i_reg_a.out_o.value.integer
    dut._log.info(f"Final Reg A value: {final_reg_a_val:#04x}")

    assert final_reg_a_val == expected_sum, \
        f"Program sequence failed! Expected {expected_sum:#04x}, got {final_reg_a_val:#04x}"

    dut._log.info("MOV/ADD program sequence test passed!")

@cocotb.test()
async def test_load_and_and_direct(dut):
    """
    Tests a two-instruction sequence:
    1. MOV A, Lit      ; Load an initial value into Register A.
    2. AND A, (Dir)    ; Read from memory, AND with A, store result in A.
    """

    # 1. ⚙️ Setup: Define the instructions and test values
    # Define opcodes
    opcode_mov_a_lit = 0b0000010
    opcode_and_a_dir = 0b0110100 # Opcode for AND A, (Dir)

    # Define memory address for the AND operation
    mem_address = 0x50

    # Define the initial values for the operation
    initial_reg_a = 0b11001100  # 0xCC
    value_in_mem  = 0b10101010  # 0xAA
    expected_result = initial_reg_a & value_in_mem # 0xCC & 0xAA = 0x88 (0b10001000)

    # Assemble the program instruction by instruction
    # [Opcode, Literal/Address]
    program = [
        [opcode_mov_a_lit, initial_reg_a], # MOV A, 0xCC
        [opcode_and_a_dir, mem_address]    # AND A, (0x50)
    ]

    # Convert the program into a stream of bytes for memory
    program_bytes = []
    for opcode, operand in program:
        instruction_word = (opcode << 8) | operand
        byte1 = (instruction_word >> 7) & 0xFF
        byte2 = instruction_word & 0x7F
        program_bytes.extend([byte1, byte2])

    dut._log.info("--- Test: MOV A, Lit -> AND A, (Dir) ---")
    dut._log.info(f"Program will load {initial_reg_a:#04x} into Reg A, then AND with value from Mem[{mem_address:#04x}].")
    dut._log.info(f"Value at Mem[{mem_address:#04x}]: {value_in_mem:#04x}")
    dut._log.info(f"Expected final Reg A value (Result): {expected_result:#04x}")

    # 2. Pre-load State: Set up the memory
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Write the program to PC=0 and the data to the target address
    slave_mem.write(0, program_bytes)
    slave_mem.write(mem_address, [value_in_mem])

    # 3. Execution: Reset the DUT and let it run
    await reset_dut(dut)

    # The processor will now perform three sequential SPI transactions:
    # 1. Fetch the 'MOV A, Lit' instruction.
    # 2. Fetch the 'AND A, (Dir)' instruction.
    # 3. Read the data from the specified memory address for the AND.
    num_transactions = 3
    for i in range(num_transactions):
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Transaction {i+1}/{num_transactions} complete.")

    # A few extra clock cycles are needed for the final ALU operation and write-back.
    for _ in range(5):
        await RisingEdge(dut.clk_core_i)

    # 4. Verification: Check the final result in Register A
    final_reg_a_val = dut.dut.i_reg_a.out_o.value.integer
    dut._log.info(f"Final Reg A value: {final_reg_a_val:#04x}")

    assert final_reg_a_val == expected_result, \
        f"Program sequence failed! Expected {expected_result:#04x}, got {final_reg_a_val:#04x}"

    dut._log.info("MOV/AND program sequence test passed!")

@cocotb.test()
async def test_load_and_sub_direct(dut):
    """
    Tests a two-instruction sequence:
    1. MOV B, Lit      ; Load an initial value into Register B.
    2. SUB B, (Dir)    ; Read from memory, subtract from B, store result in B.
    """

    # 1. Setup: Define the instructions and test values
    # Define opcodes
    opcode_mov_b_lit = 0b0000011
    opcode_sub_b_dir = 0b0110001 # Opcode for SUB B, (Dir)

    # Define memory address for the SUB operation
    mem_address = 0x60

    # Define the initial values for the operation
    initial_reg_b = 0x40
    value_in_mem  = 0x15
    expected_result = initial_reg_b - value_in_mem # 0x40 - 0x15 = 0x2B

    # Assemble the program instruction by instruction
    # [Opcode, Literal/Address]
    program = [
        [opcode_mov_b_lit, initial_reg_b], # MOV B, 0x40
        [opcode_sub_b_dir, mem_address]    # SUB B, (0x60)
    ]

    # Convert the program into a stream of bytes for memory
    program_bytes = []
    for opcode, operand in program:
        instruction_word = (opcode << 8) | operand
        byte1 = (instruction_word >> 7) & 0xFF
        byte2 = instruction_word & 0x7F
        program_bytes.extend([byte1, byte2])

    dut._log.info("--- Test: MOV B, Lit -> SUB B, (Dir) ---")
    dut._log.info(f"Program will load {initial_reg_b:#04x} into Reg B, then SUB with value from Mem[{mem_address:#04x}].")
    dut._log.info(f"Value at Mem[{mem_address:#04x}]: {value_in_mem:#04x}")
    dut._log.info(f"Expected final Reg B value (Result): {expected_result:#04x}")

    # 2. Pre-load State: Set up the memory
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Write the program to PC=0 and the data to the target address
    slave_mem.write(0, program_bytes)
    slave_mem.write(mem_address, [value_in_mem])

    # 3. Execution: Reset the DUT and let it run
    await reset_dut(dut)

    # The processor will now perform three sequential SPI transactions:
    # 1. Fetch the 'MOV B, Lit' instruction.
    # 2. Fetch the 'SUB B, (Dir)' instruction.
    # 3. Read the data from the specified memory address for the SUB.
    num_transactions = 3
    for i in range(num_transactions):
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Transaction {i+1}/{num_transactions} complete.")

    # A few extra clock cycles are needed for the final ALU operation and write-back.
    for _ in range(5):
        await RisingEdge(dut.clk_core_i)

    # 4. Verification: Check the final result in Register B
    final_reg_b_val = dut.dut.i_reg_b.out_o.value.integer
    dut._log.info(f"Final Reg B value: {final_reg_b_val:#04x}")

    assert final_reg_b_val == expected_result, \
        f"Program sequence failed! Expected {expected_result:#04x}, got {final_reg_b_val:#04x}"

    dut._log.info("MOV/SUB program sequence test passed!")

@cocotb.test()
async def test_load_and_xor_indirect(dut):
    """
    Tests a three-instruction sequence using indirect addressing:
    1. MOV B, Lit      ; Load a memory address into Register B.
    2. MOV A, Lit      ; Load an initial value into Register A.
    3. XOR A, (B)      ; Read from Mem[B], XOR with A, store result in A.
    """

    # 1. Setup: Define the instructions and test values
    # Define opcodes
    opcode_mov_b_lit = 0b0000011
    opcode_mov_a_lit = 0b0000010
    opcode_xor_a_b_indirect = 0b1000001 # Opcode for XOR A, (B)

    # Define memory address and initial values
    address_in_b = 0x70
    initial_reg_a = 0b11110000  # 0xF0
    value_in_mem  = 0b01010101  # 0x55
    expected_result = initial_reg_a ^ value_in_mem # 0xF0 ^ 0x55 = 0xA5

    # Assemble the program instruction by instruction
    # [Opcode, Literal/Address]
    program = [
        [opcode_mov_b_lit, address_in_b],  # MOV B, 0x70
        [opcode_mov_a_lit, initial_reg_a], # MOV A, 0xF0
        [opcode_xor_a_b_indirect, 0x00]    # XOR A, (B) (operand is unused)
    ]

    # Convert the program into a stream of bytes for memory
    program_bytes = []
    for opcode, operand in program:
        instruction_word = (opcode << 8) | operand
        byte1 = (instruction_word >> 7) & 0xFF
        byte2 = instruction_word & 0x7F
        program_bytes.extend([byte1, byte2])

    dut._log.info("--- Test: MOV B, MOV A -> XOR A, (B) ---")
    dut._log.info(f"Program will load address {address_in_b:#04x} into Reg B.")
    dut._log.info(f"Program will load value {initial_reg_a:#04x} into Reg A.")
    dut._log.info(f"Value at Mem[{address_in_b:#04x}]: {value_in_mem:#04x}")
    dut._log.info(f"Expected final Reg A value (Result): {expected_result:#04x}")

    # 2. Pre-load State: Set up the memory
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Write the program to PC=0 and the data to the target address
    slave_mem.write(0, program_bytes)
    slave_mem.write(address_in_b, [value_in_mem])

    # 3. Execution: Reset the DUT and let it run
    await reset_dut(dut)

    # The processor will now perform four sequential SPI transactions:
    # 1. Fetch the 'MOV B, Lit' instruction.
    # 2. Fetch the 'MOV A, Lit' instruction.
    # 3. Fetch the 'XOR A, (B)' instruction.
    # 4. Read data from Mem[B] for the XOR operation.
    num_transactions = 4
    for i in range(num_transactions):
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Transaction {i+1}/{num_transactions} complete.")

    # A few extra clock cycles are needed for the final ALU operation and write-back.
    for _ in range(5):
        await RisingEdge(dut.clk_core_i)

    # 4. Verification: Check the final result in Register A
    final_reg_a_val = dut.dut.i_reg_a.out_o.value.integer
    dut._log.info(f"Final Reg A value: {final_reg_a_val:#04x}")

    assert final_reg_a_val == expected_result, \
        f"Program sequence failed! Expected {expected_result:#04x}, got {final_reg_a_val:#04x}"

    dut._log.info("MOV/XOR indirect program sequence test passed!")


@cocotb.test()
async def test_store_add_direct(dut):
    """
    Tests a register-to-memory ALU operation:
    1. MOV A, Lit      ; Load an initial value into Register A.
    2. MOV B, Lit      ; Load an initial value into Register B.
    3. ADD (Dir)       ; Calculate A + B and store the result in Mem[Dir].
    """

    # 1. Setup: Define the instructions and test values
    # Define opcodes
    opcode_mov_a_lit = 0b0000010
    opcode_mov_b_lit = 0b0000011
    opcode_add_dir = 0b0101111 # Opcode for ADD (Dir)

    # Define memory address for the result
    result_address = 0x80

    # Define the initial values for the operation
    initial_reg_a = 0x11
    initial_reg_b = 0x22
    expected_result = initial_reg_a + initial_reg_b # 0x11 + 0x22 = 0x33

    # Assemble the program instruction by instruction
    # [Opcode, Literal/Address]
    program = [
        [opcode_mov_a_lit, initial_reg_a],  # MOV A, 0x11
        [opcode_mov_b_lit, initial_reg_b],  # MOV B, 0x22
        [opcode_add_dir, result_address]    # ADD (0x80)
    ]

    # Convert the program into a stream of bytes for memory
    program_bytes = []
    for opcode, operand in program:
        instruction_word = (opcode << 8) | operand
        byte1 = (instruction_word >> 7) & 0xFF
        byte2 = instruction_word & 0x7F
        program_bytes.extend([byte1, byte2])

    dut._log.info("--- Test: MOV A, MOV B -> ADD (Dir) ---")
    dut._log.info(f"Program will load {initial_reg_a:#04x} into Reg A and {initial_reg_b:#04x} into Reg B.")
    dut._log.info(f"It will then store A+B at Mem[{result_address:#04x}].")
    dut._log.info(f"Expected final memory value: {expected_result:#04x}")

    # 2. Pre-load State: Set up the memory
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Write the program to PC=0 and ensure the result location is clear
    slave_mem.write(0, program_bytes)
    slave_mem.write(result_address, [0x00]) # Pre-clear to avoid false positives

    # 3. Execution: Reset the DUT and let it run
    await reset_dut(dut)

    # The processor will now perform four sequential SPI transactions:
    # 1. Fetch the 'MOV A, Lit' instruction.
    # 2. Fetch the 'MOV B, Lit' instruction.
    # 3. Fetch the 'ADD (Dir)' instruction.
    # 4. Write the result of A+B to Mem[Dir].
    num_transactions = 4
    for i in range(num_transactions):
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Transaction {i+1}/{num_transactions} complete.")

    # A few extra clock cycles for the FSM to return to idle.
    for _ in range(5):
        await RisingEdge(dut.clk_core_i)

    # 4. Verification: Read back the result from memory and check it
    final_mem_val = slave_mem.read(result_address, 1)[0]
    dut._log.info(f"Final value at Mem[{result_address:#04x}]: {final_mem_val:#04x}")

    assert final_mem_val == expected_result, \
        f"Program sequence failed! Expected {expected_result:#04x}, got {final_mem_val:#04x}"

    dut._log.info("Register-to-memory ADD program sequence test passed!")

@cocotb.test()
async def test_compare_immediate_equal(dut):
    """
    Tests a single case for the CMP A, Lit instruction: Equality.
    It verifies that the Z (Zero) flag is set when A equals the literal.

    Program:
    1. MOV A, 0x42
    2. CMP A, 0x42
    Expected outcome: Z flag = 1
    """
    # 1. Setup: Define opcodes and test values for the equality test
    opcode_mov_a_lit = 0b0000010
    opcode_cmp_a_lit = 0b1001110

    reg_a_val = 0x42
    literal_val = 0x42
    expected_z = 1
    expected_n = 0

    dut._log.info("--- Test Scenario: CMP A, Lit (Equality) ---")
    dut._log.info(f"Program: MOV A, {reg_a_val:#04x}; CMP A, {literal_val:#04x}")
    dut._log.info(f"Expected Flags: Z={expected_z}, N={expected_n}")

    # Initialize the clock and memory simulation
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # 2. Assemble and Pre-load State
    program = [
        [opcode_mov_a_lit, reg_a_val],    # MOV A, 0x42
        [opcode_cmp_a_lit, literal_val]  # CMP A, 0x42
    ]

    program_bytes = []
    for opcode, operand in program:
        instruction_word = (opcode << 8) | operand
        byte1 = (instruction_word >> 7) & 0xFF
        byte2 = instruction_word & 0x7F
        program_bytes.extend([byte1, byte2])

    # Write the program to memory at PC=0
    slave_mem.write(0, program_bytes)

    # 3. Execution: Reset and run the DUT
    await reset_dut(dut)

    # The processor will perform two SPI transactions to fetch the two instructions.
    num_transactions = 2
    for i in range(num_transactions):
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Fetch {i+1}/{num_transactions} complete.")

    # Give the ALU and status register a moment to update after the last fetch.
    for _ in range(3):
        await RisingEdge(dut.clk_core_i)

    # 4. Verification: Read the status flags from the DUT
    # zncv_i is wired to status_out in the top-level module.
    # zncv_o[3]=Z, zncv_o[2]=N
    status_flags = dut.dut.i_status.out_o.value
    z_flag = (status_flags >> 3) & 1
    n_flag = (status_flags >> 2) & 1

    dut._log.info(f"Observed Flags: Z={z_flag}, N={n_flag}")

    # Assert that the flags match the expected outcome
    assert z_flag == expected_z, \
        f"Z flag mismatch! Expected {expected_z}, got {z_flag}"
    assert n_flag == expected_n, \
        f"N flag mismatch! Expected {expected_n}, got {n_flag}"

    dut._log.info("CMP A, Lit (Equality) test passed successfully!")

@cocotb.test()
async def test_flags_latching(dut):
    """
    Tests if the status flags are correctly latched across instructions.
    It runs a program where a jump instruction depends on a flag set by a
    CMP instruction that is separated by another unrelated instruction.

    Program:
    1. MOV A, 0x55
    2. CMP A, 0x55  ; Sets Z=1
    3. MOV B, 0xFF  ; Does this overwrite the Z flag?
    4. JEQ 0x80     ; Should jump if Z is still 1
    """
    # 1. Setup: Define opcodes and test values
    opcode_mov_a_lit = 0b0000010
    opcode_mov_b_lit = 0b0000011
    opcode_cmp_a_lit = 0b1001110
    opcode_jeq_dir   = 0b1010100

    reg_a_val = 0x55
    unrelated_val = 0xFF
    jump_address = 0x80
    expected_pc = jump_address

    dut._log.info("--- Test: Status Flag Latching ---")
    dut._log.info(f"Program will CMP, execute a MOV, then JEQ to {jump_address:#04x}.")
    dut._log.info(f"If flags are latched, final PC should be {expected_pc:#04x}.")

    # 2. Assemble and Pre-load State
    program = [
        [opcode_mov_a_lit, reg_a_val],     # MOV A, 0x55
        [opcode_cmp_a_lit, reg_a_val],     # CMP A, 0x55
        [opcode_mov_b_lit, unrelated_val], # MOV B, 0xFF
        [opcode_jeq_dir, jump_address]     # JEQ 0x80
    ]

    program_bytes = []
    for opcode, operand in program:
        instruction_word = (opcode << 8) | operand
        byte1 = (instruction_word >> 7) & 0xFF
        byte2 = instruction_word & 0x7F
        program_bytes.extend([byte1, byte2])

    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())
    slave_mem.write(0, program_bytes)

    # 3. Execution: Reset and run the DUT
    await reset_dut(dut)

    # We need to wait long enough for all instructions to execute.
    # Each instruction takes at least one fetch cycle.
    # We'll wait for 4 transaction_done signals.
    num_transactions = 4
    for i in range(num_transactions):
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Fetch {i+1}/{num_transactions} complete.")

    # Add a generous wait for the last instruction to fully execute and update PC
    await Timer(100, units="ns")

    # 4. Verification: Check the final value of the Program Counter
    final_pc = dut.dut.i_pc.pc_o.value
    dut._log.info(f"Final PC value: {int(final_pc):#04x}")

    assert final_pc == expected_pc, \
        f"Flag latching test failed! PC is {final_pc:#04x}, expected {expected_pc:#04x}. " \
        "This indicates the flags were overwritten before the JEQ instruction."

    dut._log.info("Flag latching test passed! Status flags are preserved correctly.")

@cocotb.test()
async def test_jump_if_equal(dut):
    """
    Tests a conditional jump instruction: JEQ (Jump if Equal).

    This test verifies that a jump occurs when the Z flag is set.
    The program flow is:
    1. MOV A, 0xAA
    2. CMP A, 0xAA  ; This sets the Z flag to 1.
    3. JEQ 0x50     ; This jump should be taken.
    4. ADD A, 0x01  ; This instruction should be skipped.
    """
    # 1. Setup: Define opcodes and test values
    opcode_mov_a_lit = 0b0000010
    opcode_cmp_a_lit = 0b1001110
    opcode_jeq_dir   = 0b1010100
    opcode_add_a_lit = 0b0000110 # Opcode for the instruction that should be skipped

    reg_a_val    = 0xAA
    jump_address = 0x50
    expected_pc  = jump_address

    dut._log.info("--- Test: Conditional Jump (JEQ) ---")
    dut._log.info(f"Program will CMP and then JEQ to {jump_address:#04x}.")
    dut._log.info(f"The instruction at PC=0x06 should be skipped.")
    dut._log.info(f"Expected final PC value: {expected_pc:#04x}")

    # 2. Assemble and Pre-load State
    program = [
        [opcode_mov_a_lit, reg_a_val],      # PC=0x00: MOV A, 0xAA
        [opcode_cmp_a_lit, reg_a_val],      # PC=0x02: CMP A, 0xAA
        [opcode_jeq_dir, jump_address],     # PC=0x04: JEQ 0x50
        [opcode_add_a_lit, 0x01]            # PC=0x06: ADD A, 0x01 (should be skipped)
    ]

    program_bytes = []
    for opcode, operand in program:
        instruction_word = (opcode << 8) | operand
        byte1 = (instruction_word >> 7) & 0xFF
        byte2 = instruction_word & 0x7F
        program_bytes.extend([byte1, byte2])

    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())
    slave_mem.write(0, program_bytes)

    # 3. Execution: Reset and run the DUT
    await reset_dut(dut)

    # The processor will fetch and execute the first three instructions.
    # The third instruction (JEQ) will alter the PC, so the 4th fetch never happens.
    num_transactions = 3
    for i in range(num_transactions):
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Fetch {i+1}/{num_transactions} complete.")

    # Add a brief wait for the PC to be loaded after the JEQ executes.
    await RisingEdge(dut.clk_core_i)
    await RisingEdge(dut.clk_core_i)
    await RisingEdge(dut.clk_core_i)

    # 4. Verification: Check the final value of the Program Counter
    final_pc = dut.dut.i_pc.pc_o.value
    dut._log.info(f"Final PC value: {int(final_pc):#04x}")

    assert int(final_pc) == expected_pc, \
        f"JEQ instruction failed! PC is {int(final_pc):#04x}, expected {expected_pc:#04x}."

    dut._log.info("Conditional jump (JEQ) test passed successfully!")

@cocotb.test()
async def test_compare_memory_direct(dut):
    """
    Tests a memory-based compare instruction: CMP A,(Dir).

    This test verifies the multi-cycle operation of reading from memory
    and then performing the comparison, checking the N (Negative) flag.
    Program:
    1. MOV A, 0x10
    2. CMP A, (0x90)  ; Where Mem[0x90] is pre-loaded with 0x20
    Expected outcome: N flag = 1, Z flag = 0
    """
    # 1. Setup: Define opcodes and test values
    opcode_mov_a_lit = 0b0000010
    opcode_cmp_a_dir = 0b1010000

    reg_a_val   = 0x10
    mem_address = 0x90
    mem_val     = 0x20  # Value in memory is greater than in register A

    expected_z = 0
    expected_n = 1

    dut._log.info("--- Test: Memory-based Compare (CMP A,(Dir)) ---")
    dut._log.info(f"Program will compare Reg A ({reg_a_val:#04x}) with Mem[{mem_address:#04x}] ({mem_val:#04x}).")
    dut._log.info(f"Expected Flags: Z={expected_z}, N={expected_n}")

    # 2. Assemble and Pre-load State
    program = [
        [opcode_mov_a_lit, reg_a_val],   # PC=0x00: MOV A, 0x10
        [opcode_cmp_a_dir, mem_address]  # PC=0x02: CMP A, (0x90)
    ]

    program_bytes = []
    for opcode, operand in program:
        instruction_word = (opcode << 8) | operand
        byte1 = (instruction_word >> 7) & 0xFF
        byte2 = instruction_word & 0x7F
        program_bytes.extend([byte1, byte2])

    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Write the program to instruction memory and the data to data memory
    slave_mem.write(0, program_bytes)
    slave_mem.write(mem_address, [mem_val])

    # 3. Execution: Reset and run the DUT
    await reset_dut(dut)

    # The processor will perform three sequential SPI transactions:
    # 1. Fetch the 'MOV A, Lit' instruction.
    # 2. Fetch the 'CMP A,(Dir)' instruction.
    # 3. Read the data from Mem[0x90] for the CMP operation.
    num_transactions = 3
    for i in range(num_transactions):
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Transaction {i+1}/{num_transactions} complete.")

    # Add a brief wait for the EXECUTE_MEM_OP state to complete and update flags.
    await RisingEdge(dut.clk_core_i)
    await RisingEdge(dut.clk_core_i)
    await RisingEdge(dut.clk_core_i)

    # 4. Verification: Read the status flags from the DUT
    status_flags = dut.dut.i_status.out_o.value
    z_flag = (status_flags >> 3) & 1
    n_flag = (status_flags >> 2) & 1

    dut._log.info(f"Observed Flags: Z={z_flag}, N={n_flag}")

    # Assert that the flags match the expected outcome
    assert z_flag == expected_z, \
        f"Z flag mismatch! Expected {expected_z}, got {z_flag}"
    assert n_flag == expected_n, \
        f"N flag mismatch! Expected {expected_n}, got {n_flag}"

    dut._log.info("Memory-based compare (CMP A,(Dir)) test passed successfully!")

@cocotb.test()
async def test_complex_program_flow(dut):
    """
    Tests a complex 11-instruction program flow involving a loop,
    arithmetic, bitwise operations, and conditional jumps.

    --- Assembly Program Logic ---
    1.  Initialize a counter (Reg B) from a value in memory (5).
    2.  Initialize an accumulator (Reg A) to 0.
    3.  Start a loop that sums the numbers from 5 down to 1 into Reg A.
        - The loop decrements the counter (Reg B) each iteration.
        - It uses CMP and JNE to check if the counter has reached zero.
    4.  After the loop, perform bitwise operations (XOR, SHL) on the result.
    5.  Store the final calculated value into a designated memory location.
    6.  Enter an infinite loop to halt execution.

    --- Expected Calculation ---
    - Summation: 5 + 4 + 3 + 2 + 1 = 15 (0x0F)
    - XOR: 15 (0b00001111) XOR 255 (0b11111111) = 240 (0b11110000)
    - SHL: 240 (0b11110000) << 1 = 224 (0b11100000) (assuming 8-bit register)
    - Final Expected Result: 0xE0
    """

    # 1. Setup: Assemble the program and define test data

    # Define memory locations for data
    counter_addr = 0x60
    result_addr = 0x61

    # Define test values
    initial_counter_val = 5
    expected_result = 0xE0 # See calculation in docstring

    # Assemble the program instruction by instruction
    # Each instruction is 2 bytes: [Opcode, Literal/Address]
    # Program addresses are calculated assuming each instruction takes 2 bytes.
    program = [
        # Opcode,      Operand,    # Address | Instruction        ; Comment
        [0b0100110, counter_addr],  # 0x00    | MOV B, (0x60)      ; Load counter value into B
        [0b0000010, 0x00],          # 0x02    | MOV A, 0           ; Clear accumulator A
        [0b0000100, 0x00],          # 0x04    | ADD A, B           ; LOOP_START: Add B to A
        [0b0001011, 0x01],          # 0x06    | SUB B, 1           ; Decrement counter B
        [0b1001111, 0x00],          # 0x08    | CMP B, 0           ; Compare B with 0
        [0b1010101, 0x04],          # 0x0A    | JNE 0x04           ; If B != 0, jump to LOOP_START
        [0b0011010, 0xFF],          # 0x0C    | XOR A, 0xFF        ; Invert all bits in the result
        [0b0011100, 0x00],          # 0x0E    | SHL A, A           ; Shift result left by 1
        [0b0100111, result_addr],   # 0x10    | MOV (0x61), A      ; Store final result in memory
        [0b0100100, 0x00],          # 0x12    | INC B              ; Extra instruction (B becomes 1)
        [0b1010011, 0x14],          # 0x14    | JMP 0x14           ; Halt processor in infinite loop
    ]

    # Convert the program into a stream of bytes for memory
    # Assuming a simple encoding: [byte_opcode, byte_operand]
    program_bytes = []
    for opcode, operand in program:
        program_bytes.extend([opcode, operand])

    dut._log.info("--- Test Complex Program Flow ---")
    dut._log.info(f"Program will be loaded at address 0x00.")
    dut._log.info(f"Initial counter at Mem[{counter_addr:#04x}]: {initial_counter_val:#04x}")
    dut._log.info(f"Expected result at Mem[{result_addr:#04x}]: {expected_result:#04x}")

    # 2. Pre-load State: Write the program and initial data to memory

    # This assumes your DUT has a memory model accessible via `dut.SLAVE.memory`
    # You will need to adapt this to your specific DUT hierarchy.
    # The placeholder class is used here for demonstration.
    slave_mem = SlaveMemory(dut.SLAVE.memory, dut._log)
    cocotb.start_soon(Clock(dut.clk_core_i, 10, units="ns").start())

    # Write program starting at PC = 0
    slave_mem.write(0, program_bytes)
    # Write the initial counter value
    slave_mem.write(counter_addr, [initial_counter_val])
    # Ensure the result location is initially zero to prevent false positives
    slave_mem.write(result_addr, [0x00])

    # 3. Execution: Reset the DUT and let it run until the result is stored

    await reset_dut(dut)

    # We expect a specific number of SPI transactions for this program to complete
    # before it enters the final halt loop.
    # Calculation:
    # - MOV B, (dir): 1 fetch + 1 read = 2
    # - MOV A, lit: 1 fetch = 1
    # - Loop (5 iterations of 4 instructions): 5 * 4 fetches = 20
    # - XOR: 1 fetch = 1
    # - SHL: 1 fetch = 1
    # - MOV (dir), A: 1 fetch + 1 write = 2
    # - INC B: 1 fetch = 1
    # TOTAL = 2 + 1 + 20 + 1 + 1 + 2 + 1 = 28 transactions
    num_transactions = 28

    dut._log.info(f"Waiting for {num_transactions} SPI transactions to complete...")
    for i in range(num_transactions):
        # This trigger needs to be adapted to your specific DUT's signal
        # that indicates a memory/bus transaction has finished.
        await RisingEdge(dut.dut.i_spi_master.transaction_done_o)
        dut._log.info(f"SPI Transaction {i+1}/{num_transactions} complete.")

    # Wait a few extra clock cycles for the final write-back to stabilize in memory
    for _ in range(10):
        await RisingEdge(dut.clk_core_i)

    # 4. Verification: Read back the result from memory and check it

    final_value_in_mem = slave_mem.read(result_addr, 1)[0]
    dut._log.info(f"Final value read from Mem[{result_addr:#04x}]: {final_value_in_mem:#04x}")

    assert final_value_in_mem == expected_result, \
        f"Complex program flow failed! Expected {expected_result:#04x}, but got {final_value_in_mem:#04x}"

    dut._log.info("Complex program flow test passed successfully! ✅")
