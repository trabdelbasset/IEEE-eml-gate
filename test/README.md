# Test Suite

cocotb-based tests for the EML accelerator. Compatible with cocotb 1.9
and 2.0.

## Requirements

- Icarus Verilog (`iverilog`)
- Python 3.8+
- cocotb (`pip install cocotb`)

## Running

```sh
make clean && make
```

## Tests

| Test | What it checks |
|------|----------------|
| `test_protocol_basic` | SPI rejects commands while busy |
| `test_chip_eml_scalar` | `eml(0.5, 0.5)` matches `exp(0.5) - ln(0.5)` |
| `test_chip_mul` | Multiply opcode: `2.5 × 3.0 = 7.5` |
| `test_chip_exp_ln_sweep` | Pure hardware sweep of exp and ln |
| `test_all_38_functions` | 38 elementary functions via EML RPN programs |

## How the test works

Every `exp(x)` in the test is computed by calling `chip_eml(x, 1.0)`.
Every `ln(y)` is computed by calling `chip_eml(0, y)` and subtracting
from 1. Every multiplication uses the chip multiply opcode. No
`cmath.exp` or `cmath.log` is used.

The host uses `math.cos`, `math.sin`, and `math.atan2` only for
complex-domain rotation. This is an architectural limitation of the
real-only hardware, not a workaround.

## Expected results

- 5/5 tests pass
- exp/ln sweep: 23/25 within tolerance (2 overflow Q6.14 range)
- 38-function sweep: 26/38 accurate (< 15% error), 3 degraded, 9 failed
- Total chip calls: ~4730 EML/MUL operations
