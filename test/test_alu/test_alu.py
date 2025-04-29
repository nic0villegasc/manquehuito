import cocotb
from cocotb.triggers import Timer
from cocotb.result import TestFailure

def print_zncv(label, a, b, expected, zncv_val):
    zncv_bits = zncv_val.integer & 0xF
    z = (zncv_bits >> 3) & 1
    n = (zncv_bits >> 2) & 1
    c = (zncv_bits >> 1) & 1
    v = (zncv_bits >> 0) & 1
    print(f"[{label}] a={a:#04x}, b={b:#04x}, result={expected:#04x}, Z={z}, N={n}, C={c}, V={v}")

@cocotb.test()
async def test_all_operations(dut):
    """Test all ALU operations and flags"""
    ops = {
        0b000: ("ADD", lambda a, b: (a + b) & 0xFF),
        0b001: ("SUB", lambda a, b: (a - b) & 0xFF),
        0b010: ("AND", lambda a, b: a & b),
        0b011: ("OR",  lambda a, b: a | b),
        0b100: ("NOT", lambda a, b: ~a & 0xFF),  # unary
        0b101: ("XOR", lambda a, b: a ^ b),
        0b110: ("SHL", lambda a, b: (a << 1) & 0xFF),
        0b111: ("SHR", lambda a, b: (a >> 1) & 0xFF)
    }

    tests = [
        (0x00, 0x00),
        (0x0F, 0xF0),
        (0x80, 0x01),
        (0x7F, 0x01),
        (0xFF, 0x01),
        (0xAA, 0x55),
        (0x00, 0xFF),
        (0xFF, 0xFF),
    ]

    for op, (label, func) in ops.items():
        for a, b in tests:
            dut.a_i.value = a
            dut.b_i.value = b
            dut.sel_i.value = op
            await Timer(1, units="ns")

            expected = func(a, b)
            actual = dut.out_o.value.integer
            assert actual == expected, f"{label} failed: {a=:#04x}, {b=:#04x}, got {actual=:#04x}, expected {expected=:#04x}"

            # SUB-specific flag checks
            if op == 0b001:
                z = (expected == 0)
                n = (expected >> 7) & 1
                c = (a & 0xFF) < (b & 0xFF)
                v = ((a ^ b) & (a ^ expected)) & 0x80 != 0
                
                print(f"[DEBUG] a={a:#04x}, b={b:#04x}, expected={expected:#04x}, "f"ZNCV={dut.zncv_o.value.binstr}")
                print_zncv("SUB", a, b, expected, dut.zncv_o.value)

                assert dut.zncv_o.value[0] == z, f"Z flag error for SUB {a:#x} - {b:#x}: {dut.zncv_o.value[3]} != {z}"
                assert dut.zncv_o.value[1] == n, f"N flag error for SUB {a:#x} - {b:#x}: {dut.zncv_o.value[2]} != {n}"
                assert dut.zncv_o.value[2] == c, f"C flag error for SUB {a:#x} - {b:#x}: {dut.zncv_o.value[1]} != {c}"
                assert dut.zncv_o.value[3] == v, f"V flag error for SUB {a:#x} - {b:#x}: {dut.zncv_o.value[0]} != {v}"