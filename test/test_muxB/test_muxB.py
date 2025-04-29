import cocotb
from cocotb.triggers import Timer

@cocotb.test()
async def test_mux_basic(dut):
    """Test muxB selection logic."""
    # Setup known values
    dut.e0_i.value = 0x55
    dut.e1_i.value = 0xAA
    dut.e2_i.value = 0x77
    dut.e3_i.value = 0x88

    for sel in range(4):
        dut.sel_i.value = sel
        await Timer(1, units="ns")

        expected = [0x55, 0xAA, 0x77, 0x88][sel]
        actual = dut.out_o.value.integer
        assert actual == expected, f"MUXB sel={sel}: got {actual:#04x}, expected {expected:#04x}"