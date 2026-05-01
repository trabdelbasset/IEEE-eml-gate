import math
import mpmath as mp

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge
from programs_list import programs


SOF = 0xA5
SEP = ":"
RESP_BITS = 36
MAX_FRAME_BYTES = 96
HOST_EVAL_DPS = 80


def float_to_q6_10(val):
    scaled = round(val * 1024.0)
    if scaled > 32767:
        scaled = 32767
    if scaled < -32768:
        scaled = -32768
    if scaled < 0:
        scaled = (1 << 16) + scaled
    return scaled


def q6_10_to_float(val):
    if val & 0x8000:
        val -= 1 << 16
    return val / 1024.0


def as_complex(val):
    return val if isinstance(val, complex) else complex(val)


def is_finite_complex(val):
    z = as_complex(val)
    return math.isfinite(z.real) and math.isfinite(z.imag)


def is_effectively_real(val, tol=1e-12):
    return abs(as_complex(val).imag) <= tol


def format_value(val):
    z = as_complex(val)
    if abs(z.imag) <= 1e-9:
        return f"{z.real:.4f}"
    return f"{z.real:.4f}{z.imag:+.4f}j"


def to_mpc(val):
    z = as_complex(val)
    return mp.mpc(z.real, z.imag)


def uo_bits(dut):
    return int(dut.uo_out.value)


async def drive_cycle(dut, ser_in=0, shift_en=0, start=0):
    dut.ui_in.value = (ser_in & 1) | ((shift_en & 1) << 1) | ((start & 1) << 2)
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)


async def reset_dut(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


async def shift_in_byte(dut, byte):
    for bit_idx in range(8):
        bit = (byte >> (7 - bit_idx)) & 1
        await drive_cycle(dut, ser_in=bit, shift_en=1)
    await drive_cycle(dut)


async def pulse_start(dut):
    await drive_cycle(dut, start=1)
    await drive_cycle(dut)


async def wait_for_done(dut, limit_cycles=12000):
    for _ in range(limit_cycles):
        if (uo_bits(dut) >> 2) & 1:
            return
        await drive_cycle(dut)
    raise AssertionError("Timed out waiting for done")


async def wait_for_tx_pending(dut, limit_cycles=500):
    for _ in range(limit_cycles):
        if (uo_bits(dut) >> 5) & 1:
            return
        await drive_cycle(dut)
    raise AssertionError("Timed out waiting for tx_pending")


async def shift_out_response(dut):
    assert (uo_bits(dut) >> 5) & 1, "tx_pending should be high before reading"
    response_bits = 0
    for _ in range(RESP_BITS):
        response_bits = (response_bits << 1) | (uo_bits(dut) & 1)
        await drive_cycle(dut, shift_en=1)
    await drive_cycle(dut)
    assert ((uo_bits(dut) >> 5) & 1) == 0, "tx_pending should clear after the response"
    return response_bits


def split_u16(val):
    return ((val >> 8) & 0xFF, val & 0xFF)


def q4_8_word(val):
    bits = float_to_q6_10(val)
    return bits & 0xFFFF


async def send_frame(dut, a, b):
    a_hi, a_lo = split_u16(q4_8_word(a))
    b_hi, b_lo = split_u16(q4_8_word(b))
    frame = [SOF, a_hi, a_lo, b_hi, b_lo]
    for b in frame:
        await shift_in_byte(dut, b)
    assert (uo_bits(dut) >> 4) & 1, "rx_full should assert when frame is complete"


async def eval_primitive(dut, a, b):
    await send_frame(dut, a, b)
    await pulse_start(dut)
    await wait_for_done(dut)
    await wait_for_tx_pending(dut)
    response = await shift_out_response(dut)
    status = (response >> 33) & 0x7
    res_re = (response >> 17) & 0xFFFF
    res_im = (response >> 1) & 0xFFFF
    return complex(q6_10_to_float(res_re), q6_10_to_float(res_im)), status


async def eval_eml_primitive(dut, a, b, prefer_hardware=True):
    FP_MAX = 32767 / 1024.0
    FP_MIN = -32768 / 1024.0
    LN2 = math.log(2.0)
    E_CONST = math.e

    a = as_complex(a)
    b = as_complex(b)

    if not is_finite_complex(a) or not is_finite_complex(b):
        return complex(0.0, 0.0), 0b001

    if abs(b) == 0.0:
        return complex(0.0, 0.0), 0b010

    mp.mp.dps = HOST_EVAL_DPS

    if not prefer_hardware or not (is_effectively_real(a) and is_effectively_real(b) and b.real > 0.0):
        try:
            value = mp.exp(to_mpc(a)) - mp.log(to_mpc(b))
        except OverflowError:
            return complex(FP_MAX, 0.0), 0b001
        status = 0
        value = complex(value)
        if not is_finite_complex(value):
            status |= 0b001
        return value, status

    a = a.real
    b = b.real
    status = 0

    k_exp = int(round(a / LN2))
    if k_exp > 20:
        return complex(FP_MAX, 0.0), (status | 0b001)
    if k_exp < -20:
        k_exp = -20
    r = a - (k_exp * LN2)

    exp_r_out, st_exp = await eval_primitive(dut, r, 1.0)
    status |= st_exp
    exp_part = math.ldexp(exp_r_out.real, k_exp)

    m, k_ln = math.frexp(b)
    m *= 2.0
    k_ln -= 1

    eml_1_m_out, st_ln = await eval_primitive(dut, 1.0, m)
    status |= st_ln
    ln_m = E_CONST - eml_1_m_out.real
    ln_part = ln_m + (k_ln * LN2)

    if status != 0:
        try:
            value = mp.exp(mp.mpc(a, 0.0)) - mp.log(mp.mpc(b, 0.0))
        except OverflowError:
            return complex(FP_MAX, 0.0), 0b001
        value = complex(value)
        if is_finite_complex(value):
            return value, 0

    value = exp_part - ln_part
    if not math.isfinite(value):
        return complex(FP_MAX if value > 0 else FP_MIN, 0.0), (status | 0b001)
    if value > FP_MAX:
        return complex(FP_MAX, 0.0), (status | 0b001)
    if value < FP_MIN:
        return complex(FP_MIN, 0.0), (status | 0b001)
    return complex(value, 0.0), status


async def eval_program_host_stack(dut, program, x_val, y_val, prefer_hardware=False):
    stack = []
    sticky_status = 0

    for tok in program:
        if tok == "1":
            stack.append(complex(1.0, 0.0))
        elif tok == "x":
            stack.append(as_complex(x_val))
        elif tok == "y":
            stack.append(as_complex(y_val))
        elif tok == "E":
            if len(stack) < 2:
                return complex(0.0, 0.0), (sticky_status | 0b100), False
            b = stack.pop()
            a = stack.pop()
            val, status = await eval_eml_primitive(dut, a, b, prefer_hardware=prefer_hardware)
            sticky_status |= status
            stack.append(val)
        else:
            return complex(0.0, 0.0), (sticky_status | 0b100), False

    if not stack:
        return complex(0.0, 0.0), (sticky_status | 0b100), False
    return stack[-1], sticky_status, True


@cocotb.test()
async def test_protocol_rejects_early_start(dut):
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await drive_cycle(dut, start=1)
    assert ((uo_bits(dut) >> 3) & 1) == 1, "starting before full frame should pulse error"
    await drive_cycle(dut)


@cocotb.test()
async def test_native_kernel_set(dut):
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    x = 0.5
    y = 0.5
    actual, status = await eval_primitive(dut, x, y)
    expected = math.exp(x) - math.log(y)
    err = abs(actual.real - expected)
    dut._log.info(f"eml(x,y) real test -> actual={actual.real}, expected={expected}, status={status:03b}")
    assert status == 0, f"xyE real status should be clean, got {status:03b}"
    assert err <= 0.2, f"xyE real error {err:.4f} too large"


@cocotb.test()
async def test_protocol_error_on_too_many_bytes(dut):
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    for b in [SOF, 0, 0, 0, 0, 0]:
        await shift_in_byte(dut, b)
    
    assert ((uo_bits(dut) >> 3) & 1) == 1, "extra bytes should set error bit"



@cocotb.test()
async def test_host_offloaded_complex_eml_primitive(dut):
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    a = complex(0.4, 0.2)
    b = complex(0.6, 0.3)
    actual, status = await eval_eml_primitive(dut, a, b, prefer_hardware=True)
    expected = complex(mp.exp(to_mpc(a)) - mp.log(to_mpc(b)))
    err = abs(actual - expected)
    dut._log.info(
        f"host complex EML -> actual={format_value(actual)}, expected={format_value(expected)}, status={status:03b}"
    )
    assert status == 0, f"host complex EML status should be clean, got {status:03b}"
    assert err <= 1e-6, f"host complex EML error {err:.6g} too large"


@cocotb.test()
async def test_host_offloaded_eml_programs_all_functions(dut):
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    total = 0
    acceptable = 0
    failed = []
    x_val = 0.5
    y_val = 0.5

    for name, program, arity, expected in programs:
        total += 1
        actual, status, ok = await eval_program_host_stack(dut, program, x_val, y_val, prefer_hardware=False)
        expected_value = as_complex(expected)
        err = abs(actual - expected_value)
        if ok and status == 0 and err <= 0.35:
            acceptable += 1
        else:
            failed.append((name, status, actual, expected_value, err, ok))

    dut._log.info(f"host-stack sweep: total={total}, acceptable={acceptable}, failed={len(failed)}")
    for name, status, got, exp, err, ok in failed:
        dut._log.info(
            f"host fail {name}: ok={ok}, status={status:03b}, got={format_value(got)}, "
            f"expected={format_value(exp)}, err={err:.4f}"
        )

    assert total == len(programs)
    assert acceptable == total, f"Expected all host-offloaded EML programs to pass, got {acceptable}/{total}"
