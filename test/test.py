# EML Gate — cocotb test suite. All exp/ln use chip hardware.
import math
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles
from programs_list import programs

CLOCK_UNIT = "unit" if cocotb.__version__.startswith("2") else "units"

Q6_14_MAX = 31.99993896484375
Q6_14_MIN = -32.0
PROG_TOL = 0.15

OP_EML = 0x00
OP_MUL = 0x01

PI_CONST = 3.14159265

def float_to_q6_14(val):
    if isinstance(val, complex): val = val.real
    if math.isnan(val): return 0x7FFFE
    if val == math.inf or val > Q6_14_MAX: return 0x7FFFF
    if val == -math.inf or val < Q6_14_MIN: return 0x80001
    scaled = round(val * 16384.0)
    if scaled > 524287: return 0x7FFFF
    if scaled < -524288: return 0x80001
    if scaled < 0: scaled = (1 << 20) + scaled
    return scaled & 0xFFFFF

def q6_14_to_float(val):
    val = val & 0xFFFFF
    if val == 0x7FFFF: return math.inf
    if val == 0x80001: return -math.inf
    if val == 0x7FFFE: return math.nan
    if val & 0x80000: val -= 1 << 20
    return val / 16384.0

def as_complex(val):
    return val if isinstance(val, complex) else complex(val)

def uo_bits(dut):
    return int(dut.uo_out.value)

async def spi_transfer(dut, data_bytes):
    ui_val = 0x04
    dut.ui_in.value = ui_val
    await ClockCycles(dut.clk, 2)
    ui_val &= ~0x04
    dut.ui_in.value = ui_val
    await ClockCycles(dut.clk, 2)
    miso_data = 0
    for b in data_bytes:
        for i in range(7, -1, -1):
            mosi_bit = (b >> i) & 1
            ui_val = (ui_val & ~0x03) | mosi_bit
            dut.ui_in.value = ui_val
            await ClockCycles(dut.clk, 2)
            ui_val |= 0x02
            dut.ui_in.value = ui_val
            await ClockCycles(dut.clk, 2)
            miso_bit = int(dut.uo_out.value) & 1
            miso_data = (miso_data << 1) | miso_bit
            ui_val &= ~0x02
            dut.ui_in.value = ui_val
            await ClockCycles(dut.clk, 2)
    ui_val |= 0x04
    dut.ui_in.value = ui_val
    await ClockCycles(dut.clk, 2)
    return miso_data

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

async def chip_call(dut, opcode, x_f, y_f):
    x_bits = float_to_q6_14(x_f)
    y_bits = float_to_q6_14(y_f)
    cmd_byte = 0x80 | (opcode & 0x03)
    frame = [cmd_byte,
             (x_bits >> 16) & 0xFF, (x_bits >> 8) & 0xFF, x_bits & 0xFF,
             (y_bits >> 16) & 0xFF, (y_bits >> 8) & 0xFF, y_bits & 0xFF]
    await spi_transfer(dut, frame)
    await wait_for_done(dut)
    response = await spi_transfer(dut, [0]*7)
    primary_bits = (response >> 24) & 0xFFFFF
    return q6_14_to_float(primary_bits)

async def chip_eml(dut, x, y):
    return await chip_call(dut, OP_EML, x, y)

async def chip_mul(dut, x, y):
    return await chip_call(dut, OP_MUL, x, y)

async def chip_exp(dut, x):
    return await chip_eml(dut, x, 1.0)

async def chip_ln(dut, y):
    if y <= 0.0:
        return -math.inf
    r = await chip_eml(dut, 0.0, y)
    return 1.0 - r

async def complex_eml_chip(dut, a, b, debug=False):
    a_c = as_complex(a)
    b_c = as_complex(b)
    ar, ai = a_c.real, a_c.imag
    br, bi = b_c.real, b_c.imag

    if debug:
        dut._log.info(f"  CEML: a={ar:.4f}+{ai:.4f}j, b={br:.4f}+{bi:.4f}j")

    if abs(ai) < 1e-9 and abs(bi) < 1e-9 and br > 0:
        r = await chip_eml(dut, ar, br)
        return complex(r, 0.0)

    if math.isinf(ar) and ar < 0:
        exp_real, exp_imag = 0.0, 0.0
    elif math.isinf(ar) and ar > 0:
        exp_real, exp_imag = math.inf, 0.0
    else:
        exp_ar = await chip_exp(dut, ar)
        if abs(ai) > 1e-9:
            cos_ai = math.cos(ai)
            sin_ai = math.sin(ai)
            exp_real = await chip_mul(dut, exp_ar, cos_ai)
            exp_imag = await chip_mul(dut, exp_ar, sin_ai)
        else:
            exp_real = exp_ar
            exp_imag = 0.0

    if abs(b_c) < 1e-12:
        ln_real = -math.inf
        ln_imag = 0.0
    elif abs(bi) < 1e-9 and br > 0:
        ln_real = await chip_ln(dut, br)
        ln_imag = 0.0
    else:
        abs_b = abs(b_c)
        if abs_b > 0.01:
            ln_real = await chip_ln(dut, abs_b)
        else:
            ln_real = -math.inf
        ln_imag = math.atan2(bi, br)

    result_real = exp_real - ln_real
    result_imag = exp_imag - ln_imag
    return complex(result_real, result_imag)

async def run_program_chip(dut, program, x_val, y_val, dbg=False):
    stack = []
    chip_nodes = 0
    for i, tok in enumerate(program):
        if tok == "1":
            stack.append(complex(1.0, 0.0))
        elif tok == "x":
            stack.append(complex(x_val, 0.0))
        elif tok == "y":
            stack.append(complex(y_val, 0.0))
        elif tok == "E":
            b_val = stack.pop()
            a_val = stack.pop()
            if dbg:
                dut._log.info(f"  [{i:3d}] EML a={fmt(a_val)} b={fmt(b_val)}")
            result = await complex_eml_chip(dut, a_val, b_val, debug=dbg)
            chip_nodes += 1
            if dbg:
                dut._log.info(f"        -> {fmt(result)}")
            stack.append(result)
    return stack[-1] if stack else complex(0), chip_nodes



@cocotb.test()
async def test_protocol_basic(dut):
    cocotb.start_soon(Clock(dut.clk, 20, **{CLOCK_UNIT: "ns"}).start())
    await reset_dut(dut)
    cmd_byte = 0x80 | (OP_MUL & 0x03)
    frame = [cmd_byte, 0, 0, 0, 0, 0, 0]
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

@cocotb.test()
async def test_chip_exp_ln_sweep(dut):
    cocotb.start_soon(Clock(dut.clk, 20, **{CLOCK_UNIT: "ns"}).start())
    await reset_dut(dut)

    ABS_TOL = 0.05
    REL_TOL = 0.03
    fails = 0
    total = 0

    dut._log.info("=" * 60)
    dut._log.info("PURE ASIC SWEEP: exp(x) and ln(x)")
    dut._log.info("=" * 60)


    dut._log.info("-- exp(x) sweep --")
    for x_10 in range(-30, 41, 5):
        x = x_10 / 10.0
        got = await chip_exp(dut, x)
        ref = math.exp(x)
        ae = abs(got - ref)
        re = ae / max(abs(ref), 1e-9)
        ok = (ae < ABS_TOL) or (re < REL_TOL)
        total += 1
        if not ok:
            fails += 1
        status = "ok" if ok else "FAIL"
        dut._log.info(f"  exp({x:+5.1f}): got={got:+10.4f} ref={ref:+10.4f} |err|={ae:.2e} {status}")


    dut._log.info("-- ln(x) sweep --")
    ln_points = [0.1, 0.2, 0.5, 1.0, 1.5, 2.0, 2.71828, 5.0, 10.0, 20.0]
    for x in ln_points:
        got = await chip_ln(dut, x)
        ref = math.log(x)
        ae = abs(got - ref)
        re = ae / max(abs(ref), 1e-9)
        ok = (ae < ABS_TOL) or (re < REL_TOL)
        total += 1
        if not ok:
            fails += 1
        status = "ok" if ok else "FAIL"
        dut._log.info(f"  ln({x:+7.3f}): got={got:+10.4f} ref={ref:+10.4f} |err|={ae:.2e} {status}")

    dut._log.info(f"-- Sweep result: {total - fails}/{total} within tolerance --")
    assert fails <= 3, f"Too many ASIC sweep failures: {fails}/{total}"
    dut._log.info("PASS test_chip_exp_ln_sweep")

@cocotb.test()
async def test_all_38_functions(dut):
    cocotb.start_soon(Clock(dut.clk, 20, **{CLOCK_UNIT: "ns"}).start())
    await reset_dut(dut)

    x_val, y_val = 0.5, 0.5
    accurate, degraded, poor = [], [], []
    total_chip_nodes = 0

    dut._log.info("=" * 70)
    dut._log.info("38-FUNCTION EML SWEEP — LEVEL B HONEST (x=0.5, y=0.5)")
    dut._log.info("  exp/ln: chip hardware | cos/sin/atan2: host (real HW limit)")
    dut._log.info("=" * 70)

    for name, program, arity, expected_raw in programs:
        expected = as_complex(expected_raw)
        actual, chip_nodes = await run_program_chip(
            dut, program, x_val, y_val, dbg=False
        )
        total_chip_nodes += chip_nodes

        if abs(expected.imag) < 1e-6:
            actual_cmp = complex(actual.real, 0)
        else:
            actual_cmp = actual

        err = abs(actual_cmp - expected)
        if abs(expected) < 0.01:
            rel_err = err
        else:
            rel_err = err / max(abs(expected), 1e-6)
        entry = (name, actual, expected, rel_err, chip_nodes)

        if rel_err <= PROG_TOL:
            accurate.append(entry)
            dut._log.info(
                f"  ACC  {name:15s}: got={fmt(actual):16s} "
                f"exp={fmt(expected):16s} err={rel_err:6.1%} nodes={chip_nodes}"
            )
        elif rel_err <= 1.0:
            degraded.append(entry)
            dut._log.info(
                f"  DEG  {name:15s}: got={fmt(actual):16s} "
                f"exp={fmt(expected):16s} err={rel_err:6.1%} nodes={chip_nodes}"
            )
        else:
            poor.append(entry)
            dut._log.info(
                f"  FAIL {name:15s}: got={fmt(actual):16s} "
                f"exp={fmt(expected):16s} err={rel_err:6.1%} nodes={chip_nodes}"
            )

    dut._log.info("=" * 70)
    dut._log.info("EML ARCHITECTURE VERIFICATION REPORT — LEVEL B")
    dut._log.info("=" * 70)
    dut._log.info(f"  Total chip EML/MUL calls      : {total_chip_nodes}")
    dut._log.info(f"  Accurate  (< 15% error)       : {len(accurate)}/{len(programs)}")
    dut._log.info(f"  Degraded  (15-100% error)     : {len(degraded)}/{len(programs)}")
    dut._log.info(f"  Poor      (> 100% error)      : {len(poor)}/{len(programs)}")
    dut._log.info("-" * 70)
    dut._log.info("  Chip computes: exp(x), ln(y), x*y")
    dut._log.info("  Host computes: cos/sin for complex Euler rotation")
    dut._log.info("  Host computes: atan2 for complex phase angle")
    dut._log.info("  No cmath.exp or cmath.log used anywhere")
    dut._log.info("=" * 70)

    assert total_chip_nodes == sum(
        e[4] for e in accurate + degraded + poor
    )
    assert len(accurate) >= 18, (
        f"Only {len(accurate)}/18 minimum accurate. "
        f"Poor: {[e[0] for e in poor]}"
    )
    dut._log.info("PASS test_all_38_functions")

def fmt(val):
    z = as_complex(val)
    if abs(z.imag) <= 1e-6:
        return f"{z.real:.4f}"
    return f"{z.real:.4f}{z.imag:+.4f}j"