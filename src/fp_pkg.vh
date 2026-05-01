`ifndef FP_PKG_VH
`define FP_PKG_VH

`define Q_INT    6
`define Q_FRAC   10
`define Q_WIDTH  (`Q_INT + `Q_FRAC)

`define FP_ZERO      16'sd0
`define FP_ONE       16'sd1024
`define FP_TWO       16'sd2048
`define FP_HALF      16'sd512
`define FP_LN2       16'sd710
`define FP_INV_LN2   16'sd1477

`define FP_SHIFT_SAT_POS   16'sd16383
`define FP_SHIFT_SAT_NEG  -16'sd16384
`define FP_POS_MAX         16'sd32767
`define FP_NEG_MAX        -16'sd32768

`define CORDIC_N  12
`define CORDIC_INV_GAIN_HYP   16'sd1236

`endif
