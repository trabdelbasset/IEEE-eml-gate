// ============================================================================
// eml_gate_top.v 
//
// This module merges eml_gate.v into the top-level dispatcher to ensure 
// exactly ONE sequential multiplier and ONE CORDIC engine are used.
//
// Native kernels:
//   EXP, LOG, SIN, COS, ATAN, ADD, SUB, MUL, RAW_EML
// ============================================================================

`include "fp_pkg.vh"

module eml_gate_top (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire [3:0]                 func_id,
    input  wire signed [`Q_WIDTH-1:0] x_in,
    input  wire signed [`Q_WIDTH-1:0] y_in,

    output wire signed [`Q_WIDTH-1:0] result,
    output wire                       done,
    output wire                       busy,
    output reg                        error,
    output reg                        domain_error,
    output reg                        overflow
);

    localparam [3:0] FUNC_EXP     = 4'd0;
    localparam [3:0] FUNC_LOG     = 4'd1;
    localparam [3:0] FUNC_SIN     = 4'd2;
    localparam [3:0] FUNC_COS     = 4'd3;
    localparam [3:0] FUNC_ATAN    = 4'd4;
    localparam [3:0] FUNC_ADD     = 4'd5;
    localparam [3:0] FUNC_SUB     = 4'd6;
    localparam [3:0] FUNC_MUL     = 4'd7;
    localparam [3:0] FUNC_RAW_EML = 4'd15;

    // -----------------------------------------------------------------------
    // FSM States
    // -----------------------------------------------------------------------
    localparam [3:0] S_IDLE            = 4'd0;
    localparam [3:0] S_EML_NORM        = 4'd1;
    localparam [3:0] S_EML_PREP_EXP    = 4'd2;
    localparam [3:0] S_EML_MUL_LN      = 4'd3;
    localparam [3:0] S_EML_MUL_EXP     = 4'd4;
    localparam [3:0] S_EML_PREP_CORDIC = 4'd5;
    localparam [3:0] S_EML_CORDIC_EXP  = 4'd6;
    localparam [3:0] S_EML_EXP_SHIFT   = 4'd7;
    localparam [3:0] S_EML_FINISH      = 4'd8;
    localparam [3:0] S_WAIT_MUL        = 4'd9;
    localparam [3:0] S_WAIT_CORDIC     = 4'd10;
    localparam [3:0] S_DONE            = 4'd11;

    reg [3:0] state;
    reg [3:0] func_reg;
    reg       busy_r;
    reg       done_r;
    reg signed [`Q_WIDTH-1:0] result_r;

    // -----------------------------------------------------------------------
    // Shared Work Registers
    // -----------------------------------------------------------------------
    reg signed [`Q_WIDTH-1:0] reg_x;
    reg signed [`Q_WIDTH-1:0] reg_work_0; 
    reg signed [`Q_WIDTH-1:0] reg_work_1; 
    reg signed [`Q_WIDTH-1:0] reg_work_2; 
    reg signed [7:0]          reg_k_0;
    reg signed [7:0]          reg_k_1;
    reg                       flag_0;
    reg                       flag_1;

    // -----------------------------------------------------------------------
    // Shared Sequential Multiplier
    // -----------------------------------------------------------------------
    reg                        mul_start_r;
    reg  signed [`Q_WIDTH-1:0] mul_a_r;
    reg  signed [`Q_WIDTH-1:0] mul_b_r;
    wire signed [`Q_WIDTH-1:0] mul_result;
    wire                       mul_done;

    fp_mul_seq u_shared_mul (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (mul_start_r),
        .a      (mul_a_r),
        .b      (mul_b_r),
        .result (mul_result),
        .done   (mul_done)
    );

    // -----------------------------------------------------------------------
    // Shared Hyperbolic CORDIC
    // -----------------------------------------------------------------------
    reg                        cordic_start_r;
    reg  [1:0]                 cordic_mode_r;
    reg  signed [`Q_WIDTH-1:0] cordic_x_in_r;
    reg  signed [`Q_WIDTH-1:0] cordic_y_in_r;
    reg  signed [`Q_WIDTH-1:0] cordic_z_in_r;
    wire signed [`Q_WIDTH-1:0] cordic_x_out;
    wire signed [`Q_WIDTH-1:0] cordic_y_out;
    wire signed [`Q_WIDTH-1:0] cordic_z_out;
    wire                       cordic_done;

    cordic_hyp u_shared_cordic (
        .clk   (clk),
        .rst_n (rst_n),
        .start (cordic_start_r),
        .mode  (cordic_mode_r),
        .x_in  (cordic_x_in_r),
        .y_in  (cordic_y_in_r),
        .z_in  (cordic_z_in_r),
        .x_out (cordic_x_out),
        .y_out (cordic_y_out),
        .z_out (cordic_z_out),
        .done  (cordic_done)
    );

    // -----------------------------------------------------------------------
    // Helper logic
    // -----------------------------------------------------------------------
    wire signed [`Q_WIDTH:0] add_wide =
        $signed({x_in[`Q_WIDTH-1], x_in}) + $signed({y_in[`Q_WIDTH-1], y_in});
    wire signed [`Q_WIDTH:0] sub_wide =
        $signed({x_in[`Q_WIDTH-1], x_in}) - $signed({y_in[`Q_WIDTH-1], y_in});

    // EML special wide logic
    wire signed [`Q_WIDTH:0] exp_sum_wide =
        $signed({cordic_x_out[`Q_WIDTH-1], cordic_x_out}) +
        $signed({cordic_y_out[`Q_WIDTH-1], cordic_y_out});

    wire signed [`Q_WIDTH:0] ln_term_wide =
        ($signed({reg_work_0[`Q_WIDTH-1], reg_work_0}) <<< 1) +
        $signed({reg_work_2[`Q_WIDTH-1], reg_work_2});

    wire signed [`Q_WIDTH:0] final_result_wide =
        $signed({reg_work_1[`Q_WIDTH-1], reg_work_1}) - ln_term_wide;

    // LOG logic ( ln(x) = e - EML(1.0, x) )
    wire signed [`Q_WIDTH:0] log_wide =
        $signed({1'b0, `FP_E}) - final_result_wide;

    function signed [`Q_WIDTH-1:0] sat_wide_to_fp;
        input signed [`Q_WIDTH:0] value;
        begin
            if (value > $signed({1'b0, `FP_POS_MAX}))
                sat_wide_to_fp = `FP_POS_MAX;
            else if (value < $signed({1'b1, `FP_NEG_MAX}))
                sat_wide_to_fp = `FP_NEG_MAX;
            else
                sat_wide_to_fp = value[`Q_WIDTH-1:0];
        end
    endfunction

    function wide_overflow;
        input signed [`Q_WIDTH:0] value;
        begin
            wide_overflow =
                (value > $signed({1'b0, `FP_POS_MAX})) ||
                (value < $signed({1'b1, `FP_NEG_MAX}));
        end
    endfunction

    assign result = result_r;
    assign done   = done_r;
    assign busy   = busy_r;

    // -----------------------------------------------------------------------
    // Master FSM
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            busy_r          <= 1'b0;
            done_r          <= 1'b0;
            error           <= 1'b0;
            domain_error    <= 1'b0;
            overflow        <= 1'b0;
            result_r        <= `FP_ZERO;
            func_reg        <= 4'd0;
            mul_start_r     <= 1'b0;
            cordic_start_r  <= 1'b0;
        end else begin
            done_r          <= 1'b0;
            mul_start_r     <= 1'b0;
            cordic_start_r  <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        busy_r       <= 1'b1;
                        func_reg     <= func_id;
                        domain_error <= 1'b0;
                        overflow     <= 1'b0;
                        error        <= 1'b0;
                        
                        case (func_id)
                            FUNC_ADD: begin
                                result_r <= sat_wide_to_fp(add_wide);
                                overflow <= wide_overflow(add_wide);
                                state    <= S_DONE;
                            end
                            FUNC_SUB: begin
                                result_r <= sat_wide_to_fp(sub_wide);
                                overflow <= wide_overflow(sub_wide);
                                state    <= S_DONE;
                            end
                            FUNC_MUL: begin
                                mul_a_r     <= x_in;
                                mul_b_r     <= y_in;
                                mul_start_r <= 1'b1;
                                state       <= S_WAIT_MUL;
                            end
                            FUNC_SIN: begin
                                cordic_mode_r <= 2'b10; // Circular rotation
                                cordic_x_in_r <= `CORDIC_INV_GAIN_CIRC;
                                cordic_y_in_r <= `FP_ZERO;
                                cordic_z_in_r <= x_in;
                                cordic_start_r<= 1'b1;
                                state         <= S_WAIT_CORDIC;
                            end
                            FUNC_COS: begin
                                cordic_mode_r <= 2'b10; // Circular rotation
                                cordic_x_in_r <= `CORDIC_INV_GAIN_CIRC;
                                cordic_y_in_r <= `FP_ZERO;
                                cordic_z_in_r <= x_in;
                                cordic_start_r<= 1'b1;
                                state         <= S_WAIT_CORDIC;
                            end
                            FUNC_ATAN: begin
                                cordic_mode_r <= 2'b11; // Circular vectoring
                                cordic_x_in_r <= `FP_ONE;
                                cordic_y_in_r <= x_in;
                                cordic_z_in_r <= `FP_ZERO;
                                cordic_start_r<= 1'b1;
                                state         <= S_WAIT_CORDIC;
                            end
                            FUNC_EXP, FUNC_LOG, FUNC_RAW_EML: begin
                                if (func_id == FUNC_LOG && x_in <= `FP_ZERO) begin
                                    result_r      <= `FP_NEG_MAX;
                                    domain_error  <= 1'b1;
                                    state         <= S_DONE;
                                end else begin
                                    // Setup for EML core
                                    reg_x      <= (func_id == FUNC_LOG) ? `FP_ONE : x_in;
                                    reg_work_0 <= (func_id == FUNC_EXP) ? `FP_ONE : 
                                                  (func_id == FUNC_LOG) ? x_in     : y_in;
                                    reg_k_0    <= 8'sd0;
                                    flag_0     <= 1'b0; // norm_done
                                    flag_1     <= 1'b0; // mul_finished
                                    
                                    // Start scaling multiplier: x_in * FP_INV_LN2
                                    mul_a_r    <= (func_id == FUNC_LOG) ? `FP_ONE : x_in;
                                    mul_b_r    <= `FP_INV_LN2;
                                    mul_start_r<= 1'b1;
                                    state      <= S_EML_NORM;
                                end
                            end
                            default: begin
                                result_r <= `FP_ZERO;
                                error    <= 1'b1;
                                state    <= S_DONE;
                            end
                        endcase
                    end
                end

                S_WAIT_MUL: begin
                    if (mul_done) begin
                        result_r <= mul_result;
                        overflow <= (mul_result == `FP_POS_MAX) || (mul_result == `FP_NEG_MAX);
                        state    <= S_DONE;
                    end
                end

                S_WAIT_CORDIC: begin
                    if (cordic_done) begin
                        if (func_reg == FUNC_SIN)  result_r <= cordic_y_out;
                        if (func_reg == FUNC_COS)  result_r <= cordic_x_out;
                        if (func_reg == FUNC_ATAN) result_r <= cordic_z_out;
                        state <= S_DONE;
                    end
                end

                // --- EML Core States ---

                S_EML_NORM: begin
                    if (mul_done) flag_1 <= 1'b1;
                    if (!flag_0) begin
                        if (reg_work_0 <= `FP_ZERO) begin
                            reg_work_0 <= `FP_ONE;
                            reg_k_0    <= -8'sd31;
                            flag_0     <= 1'b1;
                        end else if (reg_work_0 >= `FP_TWO) begin
                            reg_work_0 <= reg_work_0 >>> 1;
                            reg_k_0    <= reg_k_0 + 8'sd1;
                        end else if (reg_work_0 < `FP_ONE) begin
                            reg_work_0 <= reg_work_0 <<< 1;
                            reg_k_0    <= reg_k_0 - 8'sd1;
                        end else if (reg_k_0 >= 8'sd31 || reg_k_0 <= -8'sd31) begin
                            flag_0     <= 1'b1;
                        end else begin
                            flag_0     <= 1'b1;
                        end
                    end
                    if (flag_0 && (flag_1 || mul_done)) begin
                        reg_work_1 <= mul_result; // x_scaled
                        state      <= S_EML_PREP_EXP;
                    end
                end

                S_EML_PREP_EXP: begin
                    // Round x_scaled to k_exp
                    reg_k_1 <= (reg_work_1[`Q_WIDTH-1] ? (reg_work_1 - `FP_HALF) : (reg_work_1 + `FP_HALF)) >>> `Q_FRAC;
                    
                    // Start ln vectoring
                    cordic_mode_r <= 2'b01; // Vectoring
                    cordic_x_in_r <= reg_work_0 + `FP_ONE;
                    cordic_y_in_r <= reg_work_0 - `FP_ONE;
                    cordic_z_in_r <= `FP_ZERO;
                    cordic_start_r<= 1'b1;
                    flag_0        <= 1'b0; // cordic_finished
                    
                    // Start ln integer mult: k_ln * FP_LN2
                    mul_a_r       <= (reg_k_0 > 8'sd15) ? `FP_POS_MAX : (reg_k_0 < -8'sd16) ? `FP_NEG_MAX : ($signed(reg_k_0) <<< `Q_FRAC);
                    mul_b_r       <= `FP_LN2;
                    mul_start_r   <= 1'b1;
                    flag_1        <= 1'b0; // mul_finished
                    
                    state <= S_EML_MUL_LN;
                end

                S_EML_MUL_LN: begin
                    if (cordic_done) flag_0 <= 1'b1;
                    if (mul_done)    flag_1 <= 1'b1;
                    if ((flag_0 || cordic_done) && (flag_1 || mul_done)) begin
                        reg_work_0 <= cordic_z_out;
                        reg_work_2 <= mul_result;
                        // Start exp integer mult: k_exp * FP_LN2
                        mul_a_r    <= (reg_k_1 > 8'sd15) ? `FP_POS_MAX : (reg_k_1 < -8'sd16) ? `FP_NEG_MAX : ($signed(reg_k_1) <<< `Q_FRAC);
                        mul_b_r    <= `FP_LN2;
                        mul_start_r<= 1'b1;
                        state      <= S_EML_MUL_EXP;
                    end
                end

                S_EML_MUL_EXP: begin
                    if (mul_done) begin
                        reg_work_1 <= reg_x - mul_result; // r
                        state      <= S_EML_PREP_CORDIC;
                    end
                end

                S_EML_PREP_CORDIC: begin
                    cordic_mode_r <= 2'b00; // Rotation
                    cordic_x_in_r <= `CORDIC_INV_GAIN_HYP;
                    cordic_y_in_r <= `FP_ZERO;
                    cordic_z_in_r <= reg_work_1;
                    cordic_start_r<= 1'b1;
                    state         <= S_EML_CORDIC_EXP;
                end

                S_EML_CORDIC_EXP: begin
                    if (cordic_done) begin
                        reg_work_1 <= sat_wide_to_fp(exp_sum_wide);
                        state      <= S_EML_EXP_SHIFT;
                    end
                end

                S_EML_EXP_SHIFT: begin
                    if (reg_k_1 > 0) begin
                        if (reg_work_1 > `FP_SHIFT_SAT_POS) reg_work_1 <= `FP_POS_MAX;
                        else if (reg_work_1 < `FP_SHIFT_SAT_NEG) reg_work_1 <= `FP_NEG_MAX;
                        else reg_work_1 <= reg_work_1 <<< 1;
                        reg_k_1 <= reg_k_1 - 8'sd1;
                    end else if (reg_k_1 < 0) begin
                        reg_work_1 <= reg_work_1 >>> 1;
                        reg_k_1    <= reg_k_1 + 8'sd1;
                    end else begin
                        state <= S_EML_FINISH;
                    end
                end

                S_EML_FINISH: begin
                    if (func_reg == FUNC_LOG) begin
                        result_r <= sat_wide_to_fp(log_wide);
                        overflow <= wide_overflow(log_wide);
                    end else begin
                        result_r <= sat_wide_to_fp(final_result_wide);
                    end
                    state    <= S_DONE;
                end

                S_DONE: begin
                    done_r  <= 1'b1;
                    busy_r  <= 1'b0;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase

            if (start && busy_r) error <= 1'b1;
        end
    end

endmodule
