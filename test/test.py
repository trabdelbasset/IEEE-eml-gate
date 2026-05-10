import math
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles
from programs_list import programs

CLOCK_UNIT = "unit" if cocotb.__version__.startswith("2") else "units"

Q6_14_MAX = 31.99993896484375
Q6_14_MIN = -32.0

OP_EML = 0x00
OP_MUL = 0x01

_PI = math.pi

def float_to_q6_14(value):
    if isinstance(value, complex):
        value = value.real
    if math.isnan(value):
        return 0x7FFFE
    if value == math.inf or value > Q6_14_MAX:
        return 0x7FFFF
    if value == -math.inf or value < Q6_14_MIN:
        return 0x80001
    scaled = round(value * 16384.0)
    if scaled > 524287:
        return 0x7FFFF
    if scaled < -524288:
        return 0x80001
    if scaled < 0:
        scaled = (1 << 20) + scaled
    return scaled & 0xFFFFF

def q6_14_to_float(value):
    value = value & 0xFFFFF
    if value == 0x7FFFF:
        return math.inf
    if value == 0x80001:
        return -math.inf
    if value == 0x7FFFE:
        return math.nan
    if value & 0x80000:
        value -= 1 << 20
    return value / 16384.0

def as_complex(value):
    if isinstance(value, complex):
        return value
    return complex(value)

def uo_bits(dut):
    return int(dut.uo_out.value)

async def spi_transfer(dut, data_bytes):
    ui_value = 0x04
    dut.ui_in.value = ui_value
    await ClockCycles(dut.clk, 2)
    ui_value &= ~0x04
    dut.ui_in.value = ui_value
    await ClockCycles(dut.clk, 2)

    captured_bits = 0
    for byte_value in data_bytes:
        for bit_index in range(7, -1, -1):
            mosi_bit = (byte_value >> bit_index) & 1
            ui_value = (ui_value & ~0x03) | mosi_bit
            dut.ui_in.value = ui_value
            await ClockCycles(dut.clk, 2)

            ui_value |= 0x02
            dut.ui_in.value = ui_value
            await ClockCycles(dut.clk, 2)

            miso_bit = int(dut.uo_out.value) & 1
            captured_bits = (captured_bits << 1) | miso_bit

            ui_value &= ~0x02
            dut.ui_in.value = ui_value
            await ClockCycles(dut.clk, 2)

    ui_value |= 0x04
    dut.ui_in.value = ui_value
    await ClockCycles(dut.clk, 2)
    return captured_bits

async def reset_dut(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0x04
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

async def wait_for_done(dut, limit=12000):
    for _ in range(limit):
        if (int(dut.uo_out.value) >> 2) & 1:
            return
        await ClockCycles(dut.clk, 1)
    raise AssertionError("Timed out waiting for done")

async def chip_call(dut, opcode, x_value, y_value):
    x_bits = float_to_q6_14(x_value)
    y_bits = float_to_q6_14(y_value)
    command_byte = 0x80 | (opcode & 0x03)
    frame = [
        command_byte,
        (x_bits >> 16) & 0xFF,
        (x_bits >> 8) & 0xFF,
        x_bits & 0xFF,
        (y_bits >> 16) & 0xFF,
        (y_bits >> 8) & 0xFF,
        y_bits & 0xFF,
    ]
    await spi_transfer(dut, frame)
    await wait_for_done(dut)
    response = await spi_transfer(dut, [0] * 7)
    status = (response >> 52) & 0x7
    primary_bits = (response >> 24) & 0xFFFFF
    secondary_bits = response & 0xFFFFF
    return q6_14_to_float(primary_bits), q6_14_to_float(secondary_bits), status

async def chip_eml(dut, x_value, y_value):
    result, _, _ = await chip_call(dut, OP_EML, x_value, y_value)
    return result

async def chip_mul(dut, x_value, y_value):
    result, _, _ = await chip_call(dut, OP_MUL, x_value, y_value)
    return result

_program_map = {name: (program, arity) for name, program, arity, _ in programs}

async def host_log_real(value):
    if value > 0:
        return complex(math.log(value), 0.0)
    if value == 0:
        return complex(-math.inf, 0.0)
    return complex(math.log(-value), 0.0)

async def run_program_chip(dut, program, x_value, y_value, dbg=False, call_stack=None):
    if call_stack is None:
        call_stack = []
    evaluation_stack = []
    chip_node_count = 0

    for token_index, token in enumerate(program):
        if token == "1":
            evaluation_stack.append(complex(1.0, 0.0))
            if dbg:
                dut._log.info(f"  [{token_index:3d}] PUSH 1.0")
        elif token == "x":
            evaluation_stack.append(complex(x_value, 0.0))
            if dbg:
                dut._log.info(f"  [{token_index:3d}] PUSH x={x_value}")
        elif token == "y":
            evaluation_stack.append(complex(y_value, 0.0))
            if dbg:
                dut._log.info(f"  [{token_index:3d}] PUSH y={y_value}")
        elif token == "E":
            b_value = evaluation_stack.pop()
            a_value = evaluation_stack.pop()
            if dbg:
                dut._log.info(f"  [{token_index:3d}] EML a={fmt(a_value)} b={fmt(b_value)}")
            result = await complex_eml_chip(dut, a_value, b_value, debug=dbg, call_stack=call_stack)
            chip_node_count += 1
            if dbg:
                dut._log.info(f"        -> {fmt(result)}")
            evaluation_stack.append(result)

    if evaluation_stack:
        return evaluation_stack[-1], chip_node_count
    return complex(0.0, 0.0), chip_node_count

async def _run_named_program(dut, name, x_value=0.0, y_value=0.0, call_stack=None):
    if call_stack is None:
        call_stack = []
    if name in call_stack:
        cycle_trace = " -> ".join(call_stack + [name])
        raise RuntimeError(f"Recursive program cycle detected: {cycle_trace}")

    if name == "CONST_PI":
        return complex(_PI, 0.0)

    program, _ = _program_map[name]
    result, _ = await run_program_chip(dut, program, x_value, y_value, dbg=False, call_stack=call_stack + [name])
    return result

async def complex_eml_chip(dut, a_value, b_value, debug=False, call_stack=None):
    if call_stack is None:
        call_stack = []
    a_complex = as_complex(a_value)
    b_complex = as_complex(b_value)

    ar = a_complex.real
    ai = a_complex.imag
    br = b_complex.real
    bi = b_complex.imag

    if debug:
        dut._log.info(f"  CEML: a={ar:.4f}+{ai:.4f}j, b={br:.4f}+{bi:.4f}j")

    if abs(ai) < 1e-9 and abs(bi) < 1e-9 and br > 0:
        result = await chip_eml(dut, ar, br)
        return complex(result, 0.0)

    if math.isinf(ar) and ar < 0:
        exp_real = 0.0
        exp_imag = 0.0
    elif math.isinf(ar) and ar > 0:
        exp_real = math.inf
        exp_imag = 0.0
    else:
        exp_ar = await chip_eml(dut, ar, 1.0)
        if abs(ai) > 1e-9:
            if math.isinf(ai) or math.isnan(ai):
                cos_ai = 0.0
                sin_ai = 0.0
            else:
                reduced_angle = ai % (2 * _PI)
                if reduced_angle > _PI:
                    reduced_angle -= 2 * _PI
                if reduced_angle < -_PI:
                    reduced_angle += 2 * _PI

                cosine_sign = 1.0
                sine_sign = 1.0

                if reduced_angle > _PI / 2:
                    reduced_angle = _PI - reduced_angle
                    cosine_sign = -1.0
                elif reduced_angle < -_PI / 2:
                    reduced_angle = -_PI - reduced_angle
                    cosine_sign = -1.0
                    sine_sign = -1.0

                cos_ai = math.cos(reduced_angle) * cosine_sign
                sin_ai = math.sin(reduced_angle) * sine_sign

            exp_real = await chip_mul(dut, exp_ar, cos_ai)
            exp_imag = await chip_mul(dut, exp_ar, sin_ai) 
        else:
            exp_real = exp_ar
            exp_imag = 0.0

    b_essentially_real = abs(bi) < max(abs(br) * 0.05, 1e-6)

    if b_essentially_real:
        if br > 0:
            ln_complex = await host_log_real(br)
            ln_real = ln_complex.real
            ln_imag = 0.0
        elif abs(br) < 1e-9:
            ln_real = -math.inf
            ln_imag = 0.0
        else:
            ln_complex = await host_log_real(-br)
            ln_real = ln_complex.real
            ln_imag = -_PI
    else:
        hypot_complex = await _run_named_program(dut, "HYPOT", br, bi, call_stack=call_stack + ["HYPOT"])
        abs_b = hypot_complex.real

        if abs_b < 1e-9:
            ln_real = -math.inf
        else:
            ln_complex = await host_log_real(abs_b)
            ln_real = ln_complex.real

        if abs(br) > 1e-9:
            ratio_complex = await _run_named_program(dut, "DIV", bi, br, call_stack=call_stack + ["DIV"])
            ratio = ratio_complex.real
            atan_complex = await _run_named_program(dut, "ATAN", ratio, 0.0, call_stack=call_stack + ["ATAN"])
            base_angle = atan_complex.real
            if br < 0:
                if bi >= 0:
                    ln_imag = base_angle + _PI
                else:
                    ln_imag = base_angle - _PI
            else:
                ln_imag = base_angle
        elif bi > 0:
            ln_imag = _PI / 2.0
        elif bi < 0:
            ln_imag = -_PI / 2.0
        else:
            ln_imag = 0.0

    result_real = exp_real - ln_real
    result_imag = exp_imag - ln_imag
    return complex(result_real, result_imag)

@cocotb.test()
async def test_protocol_basic(dut):
    cocotb.start_soon(Clock(dut.clk, 20, **{CLOCK_UNIT: "ns"}).start())
    await reset_dut(dut)
    command_byte = 0x80 | (OP_MUL & 0x03)
    frame = [command_byte, 0, 0, 0, 0, 0, 0]
    await spi_transfer(dut, frame)
    dut.ui_in.value = 0x00
    await ClockCycles(dut.clk, 2)
    dut.ui_in.value = 0x04
    await ClockCycles(dut.clk, 2)
    await ClockCycles(dut.clk, 10)
    assert (uo_bits(dut) >> 3) & 1 == 1, "starting while busy should set error"
    await wait_for_done(dut)
    dut._log.info("PASS test_protocol_basic")

@cocotb.test()
async def test_chip_eml_scalar(dut):
    cocotb.start_soon(Clock(dut.clk, 20, **{CLOCK_UNIT: "ns"}).start())
    await reset_dut(dut)
    got = await chip_eml(dut, 0.5, 0.5)
    expected = math.exp(0.5) - math.log(0.5)
    dut._log.info(f"eml(0.5,0.5): got={got:.4f} expected={expected:.4f}")
    assert abs(got - expected) <= 0.05
    dut._log.info("PASS test_chip_eml_scalar")

@cocotb.test()
async def test_chip_mul(dut):
    cocotb.start_soon(Clock(dut.clk, 20, **{CLOCK_UNIT: "ns"}).start())
    await reset_dut(dut)
    got = await chip_mul(dut, 2.5, 3.0)
    dut._log.info(f"mul(2.5, 3.0): got={got:.4f} expected=7.5")
    assert abs(got - 7.5) <= 0.05
    dut._log.info("PASS test_chip_mul")

def fmt(value):
    complex_value = as_complex(value)
    if abs(complex_value.imag) <= 1e-6:
        return f"{complex_value.real:.4f}"
    return f"{complex_value.real:.4f}{complex_value.imag:+.4f}j"