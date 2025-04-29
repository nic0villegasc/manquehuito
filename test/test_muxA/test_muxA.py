import cocotb
from cocotb.triggers import Timer

@cocotb.test()
async def test_mux_basic(dut):
    """Test muxA selection logic."""
    # Setup known values
    dut.e0_i.value = 0x11
    dut.e1_i.value = 0x22
    dut.e2_i.value = 0x33
    dut.e3_i.value = 0x44

    for sel in range(4):
        dut.sel_i.value = sel
        await Timer(1, units="ns")

        expected = [0x11, 0x22, 0x33, 0x44][sel]
        actual = dut.out_o.value.integer
        assert actual == expected, f"MUXA sel={sel}: got {actual:#04x}, expected {expected:#04x}"