// ============================================================================
// fp_mul_seq.v
//
// Computes:  result = (a × b) >> Q_FRAC   (Q6.10 signed)
//
// Uses a shift-and-add algorithm taking `Q_WIDTH` clock cycles.
// Drastically reduces area compared to a combinational multiplier.
// ============================================================================

`include "fp_pkg.vh"

module fp_mul_seq (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire signed [`Q_WIDTH-1:0] a,
    input  wire signed [`Q_WIDTH-1:0] b,
    output reg  signed [`Q_WIDTH-1:0] result,
    output reg                        done
);

    localparam S_IDLE = 2'd0;
    localparam S_CALC = 2'd1;
    localparam S_DONE = 2'd2;

    (* fsm_encoding = "binary" *) reg [1:0] state;

    localparam COUNT_WIDTH = 4; // Q_WIDTH=16 needs counts 0..15.
    reg [COUNT_WIDTH-1:0] count;

    // We need an accumulator of size 2 * Q_WIDTH to hold the full product
    reg signed [2*`Q_WIDTH-1:0] p_reg;     // Product accumulator
    reg signed [2*`Q_WIDTH-1:0] a_reg;     // Multiplicand shifted
    reg signed [`Q_WIDTH-1:0]   b_reg;     // Multiplier shift register

    // Saturation constants
    localparam signed [`Q_WIDTH-1:0] SAT_POS = {1'b0, {(`Q_WIDTH-1){1'b1}}};
    localparam signed [`Q_WIDTH-1:0] SAT_NEG = {1'b1, {(`Q_WIDTH-1){1'b0}}};

    wire signed [2*`Q_WIDTH-1:0] p_shifted = p_reg >>> `Q_FRAC;
    wire p_overflow = (p_shifted[2*`Q_WIDTH-1:`Q_WIDTH] !=
                       {`Q_WIDTH{p_shifted[`Q_WIDTH-1]}});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= S_IDLE;
            result <= `FP_ZERO;
            done   <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        // Initialize:
                        // a_reg is sign-extended to 48 bits
                        a_reg <= { {(`Q_WIDTH){a[`Q_WIDTH-1]}}, a };
                        b_reg <= b;
                        p_reg <= 0;
                        count <= 0;
                        state <= S_CALC;
                    end
                end

                S_CALC: begin
                    if (count == `Q_WIDTH - 1) begin
                        // Last step: if the multiplier is negative, the last bit
                        // represents the sign bit (-2^N), so we SUBTRACT a_reg.
                        if (b_reg[0])
                            p_reg <= p_reg - a_reg;
                        
                        state <= S_DONE;
                    end else begin
                        if (b_reg[0])
                            p_reg <= p_reg + a_reg;

                        a_reg <= a_reg << 1;
                        b_reg <= {b_reg[`Q_WIDTH-1], b_reg[`Q_WIDTH-1:1]}; 
                        count <= count + 1;
                    end
                end

                S_DONE: begin
                    // Extract result with saturation after the Q_FRAC realignment.
                    if (p_overflow)
                        result <= p_shifted[2*`Q_WIDTH-1] ? SAT_NEG : SAT_POS;
                    else
                        result <= p_shifted[`Q_WIDTH-1:0];
                    
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
