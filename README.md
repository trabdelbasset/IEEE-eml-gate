![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# EML Gate for Tiny Tapeout

A hardware implementation of the EML (Exp-Minus-Log) operator from
[arXiv:2603.21852](https://arxiv.org/abs/2603.21852v2).

The chip computes `eml(x, y) = exp(x) - ln(y)` in Q6.14 fixed-point
arithmetic using a shared sequential multiplier and hyperbolic CORDIC.
A host controller composes this single primitive into higher functions
(sin, cos, sqrt, pow, etc.) via RPN programs.

## Architecture

| Layer | Role |
|-------|------|
| **Chip** | Real-only `eml(x,y)` and `mul(x,y)` via SPI |
| **Host** | RPN stack interpreter, complex arithmetic |

## Pin Map

| Pin | Direction | Function |
|-----|-----------|----------|
| `ui_in[0]` | Input | MOSI |
| `ui_in[1]` | Input | SCLK |
| `ui_in[2]` | Input | CS_N |
| `uo_out[0]` | Output | MISO |
| `uo_out[1]` | Output | Busy |
| `uo_out[2]` | Output | Done |
| `uo_out[3]` | Output | Error |

## Running Tests

```sh
cd test
make clean && make
```

5 tests: protocol, eml scalar, multiply, exp/ln sweep, 38-function composition.

## Resources

- [Project details](docs/info.md)
- [Test documentation](test/README.md)
- [Tiny Tapeout](https://tinytapeout.com)
