import cocotb
from cocotb.triggers import RisingEdge, Timer

@cocotb.test()
async def test_register_load_and_hold(dut):
    """Test register load and hold behavior."""
    # Load data into register
    dut.load_i.value = 1
    dut.data_i.value = 0xAB
    await RisingEdge(dut.clk_i)
    await Timer(1, units="ns")
    assert dut.out_o.value == 0xAB, f"Register failed to load value, got {dut.out_o.value}"

    # Try to change data with load=0, should not change output
    dut.load_i.value = 0
    dut.data_i.value = 0xCD
    await RisingEdge(dut.clk_i)
    await Timer(1, units="ns")
    assert dut.out_o.value == 0xAB, f"Register changed output without load! got {dut.out_o.value}"

    # Load new value
    dut.load_i.value = 1
    dut.data_i.value = 0x55
    await RisingEdge(dut.clk_i)
    await Timer(1, units="ns")
    assert dut.out_o.value == 0x55, f"Register failed to load new value, got {dut.out_o.value}"
