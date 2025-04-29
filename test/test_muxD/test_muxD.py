import cocotb
from cocotb.triggers import Timer

@cocotb.test()
async def test_mux_basic(dut):
    """Test muxD selection logic."""
    # Setup known values
    dut.e0_i.value = 0x0F
    dut.e1_i.value = 0xF0

    for sel in range(2):
        dut.sel_i.value = sel
        await Timer(1, units="ns")

        expected = [0x0F, 0xF0][sel]
        actual = dut.out_o.value.integer
        assert actual == expected, f"MUXD sel={sel}: got {actual:#04x}, expected {expected:#04x}"
