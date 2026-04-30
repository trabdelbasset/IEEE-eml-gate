// ============================================================================
// cordic_hyp.v
//
// Hyperbolic iteration schedule: [1,2,3,4,4,5,6,7,8]  (9 steps)
// Circular  iteration schedule:  [0,1,2,3,4,5,6,7,8]  (9 steps)
// ============================================================================

`include "fp_pkg.vh"

module cordic_hyp (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire [1:0]                 mode,   // [1]=circ/hyp, [0]=vec/rot
    input  wire signed [`Q_WIDTH-1:0] x_in,
    input  wire signed [`Q_WIDTH-1:0] y_in,
    input  wire signed [`Q_WIDTH-1:0] z_in,
    output reg  signed [`Q_WIDTH-1:0] x_out,
    output reg  signed [`Q_WIDTH-1:0] y_out,
    output reg  signed [`Q_WIDTH-1:0] z_out,
    output reg                        done
);

    // -----------------------------------------------------------------------
    // Hyperbolic LUT: atanh(2^-i) for i=1..8, Q4.8
    // -----------------------------------------------------------------------
    function signed [`Q_WIDTH-1:0] get_atanh;
        input [3:0] i;
        case (i)
            4'd1:  get_atanh = 12'sd141;
            4'd2:  get_atanh = 12'sd65;
            4'd3:  get_atanh = 12'sd32;
            4'd4:  get_atanh = 12'sd16;
            4'd5:  get_atanh = 12'sd8;
            4'd6:  get_atanh = 12'sd4;
            4'd7:  get_atanh = 12'sd2;
            4'd8:  get_atanh = 12'sd1;
            default: get_atanh = 12'sd0;
        endcase
    endfunction

    // -----------------------------------------------------------------------
    // Circular LUT: atan(2^-i) for i=0..8, Q4.8
    // -----------------------------------------------------------------------
    function signed [`Q_WIDTH-1:0] get_atan;
        input [3:0] i;
        case (i)
            4'd0:  get_atan = 12'sd201;   // atan(1) = pi/4
            4'd1:  get_atan = 12'sd119;
            4'd2:  get_atan = 12'sd63;
            4'd3:  get_atan = 12'sd32;
            4'd4:  get_atan = 12'sd16;
            4'd5:  get_atan = 12'sd8;
            4'd6:  get_atan = 12'sd4;
            4'd7:  get_atan = 12'sd2;
            4'd8:  get_atan = 12'sd1;
            default: get_atan = 12'sd0;
        endcase
    endfunction

    // -----------------------------------------------------------------------
    // Iteration schedule: hyperbolic [1,2,3,4,4,5,6,7,8]
    //                     circular   [0,1,2,3,4,5,6,7,8]
    // -----------------------------------------------------------------------
    function [3:0] get_shift_hyp;
        input [3:0] s;
        case (s)
            4'd0:  get_shift_hyp = 4'd1;
            4'd1:  get_shift_hyp = 4'd2;
            4'd2:  get_shift_hyp = 4'd3;
            4'd3:  get_shift_hyp = 4'd4;
            4'd4:  get_shift_hyp = 4'd4;
            4'd5:  get_shift_hyp = 4'd5;
            4'd6:  get_shift_hyp = 4'd6;
            4'd7:  get_shift_hyp = 4'd7;
            4'd8:  get_shift_hyp = 4'd8;
            default: get_shift_hyp = 4'd0;
        endcase
    endfunction

    localparam [3:0] TOTAL_STEPS = `CORDIC_N;

    localparam S_IDLE    = 2'd0;
    localparam S_ITERATE = 2'd1;
    localparam S_DONE    = 2'd2;

    (* fsm_encoding = "binary" *) reg [1:0] state;
    reg [3:0]                     step;
    reg signed [`Q_WIDTH-1:0]     x_reg, y_reg, z_reg;
    reg [1:0]                     mode_reg;

    wire is_circ    = mode_reg[1];
    wire is_vector  = mode_reg[0];

    wire [3:0] cur_shift = is_circ ? step : get_shift_hyp(step);

    wire signed [`Q_WIDTH-1:0] angle =
        is_circ ? get_atan(cur_shift) : get_atanh(cur_shift);

    wire signed [`Q_WIDTH-1:0] x_shifted = x_reg >>> cur_shift;
    wire signed [`Q_WIDTH-1:0] y_shifted = y_reg >>> cur_shift;

    wire sigma = is_vector ? y_reg[`Q_WIDTH-1] : ~z_reg[`Q_WIDTH-1];

    wire signed [`Q_WIDTH-1:0] x_delta = sigma ? y_shifted : -y_shifted;
    wire signed [`Q_WIDTH-1:0] y_delta = sigma ? x_shifted : -x_shifted;

    wire signed [`Q_WIDTH-1:0] x_next =
        is_circ ? (x_reg - x_delta) : (x_reg + x_delta);
    wire signed [`Q_WIDTH-1:0] y_next = y_reg + y_delta;
    wire signed [`Q_WIDTH-1:0] z_next = sigma ? (z_reg - angle) : (z_reg + angle);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            x_out <= `FP_ZERO; y_out <= `FP_ZERO; z_out <= `FP_ZERO;
            done  <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        x_reg    <= x_in;
                        y_reg    <= y_in;
                        z_reg    <= z_in;
                        mode_reg <= mode;
                        step     <= 4'd0;
                        state    <= S_ITERATE;
                    end
                end

                S_ITERATE: begin
                    x_reg <= x_next;
                    y_reg <= y_next;
                    z_reg <= z_next;
                    if (step == TOTAL_STEPS - 1)
                        state <= S_DONE;
                    else
                        step <= step + 4'd1;
                end

                S_DONE: begin
                    x_out <= x_reg; y_out <= y_reg; z_out <= z_reg;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
