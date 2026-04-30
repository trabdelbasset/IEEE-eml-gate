// ============================================================================
// fp_pkg.vh
//
// Format: Q4.8 signed (12-bit total)
//   • Range  : -8.0 .. +7.996
//   • LSB    : 2^-8 ~= 3.9e-3
// ============================================================================

`ifndef FP_PKG_VH
`define FP_PKG_VH

// ---------------------------------------------------------------------------
// Word geometry
// ---------------------------------------------------------------------------
`define Q_INT    4                         // integer bits (incl. sign)
`define Q_FRAC   8                         // fractional bits
`define Q_WIDTH  (`Q_INT + `Q_FRAC)        // total = 12

// ---------------------------------------------------------------------------
// Useful fixed-point constants   (value × 2^8, rounded to nearest)
// ---------------------------------------------------------------------------
`define FP_ZERO      12'sd0
`define FP_ONE       12'sd256                // 1.0
`define FP_TWO       12'sd512                // 2.0
`define FP_HALF      12'sd128                // 0.5
`define FP_E         12'sd696                // e  ~= 2.71828
`define FP_LN2       12'sd177                // ln2 ~= 0.69315
`define FP_INV_LN2   12'sd369                // 1/ln2 ~= 1.44269
`define FP_HALF_LN2  12'sd89                 // ln2/2 ~= 0.34657

// Saturation limits for the exp left-shift loop
`define FP_SHIFT_SAT_POS   12'sd1023
`define FP_SHIFT_SAT_NEG  -12'sd1024
`define FP_POS_MAX         12'sd2047
`define FP_NEG_MAX        -12'sd2048

// ---------------------------------------------------------------------------
// Hyperbolic CORDIC constants
// ---------------------------------------------------------------------------
// Number of CORDIC iterations
`define CORDIC_N  9

// Inverse of hyperbolic CORDIC gain: 1/K_h ≈ 1.20749
`define CORDIC_INV_GAIN_HYP   12'sd309

// Circular CORDIC: 1/K_circ = 1/1.64676 ≈ 0.60725
`define CORDIC_INV_GAIN_CIRC  12'sd155

`endif // FP_PKG_VH
