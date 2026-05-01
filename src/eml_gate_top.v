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

    localparam [3:0] FUNC_RAW_EML = 4'd15;

    localparam [3:0] S_IDLE            = 4'd0;
    localparam [3:0] S_EML_NORM        = 4'd1;
    localparam [3:0] S_EML_PREP_EXP    = 4'd2;
    localparam [3:0] S_EML_MUL_LN      = 4'd3;
    localparam [3:0] S_EML_MUL_EXP     = 4'd4;
    localparam [3:0] S_EML_PREP_CORDIC = 4'd5;
    localparam [3:0] S_EML_CORDIC_EXP  = 4'd6;
    localparam [3:0] S_EML_EXP_SHIFT   = 4'd7;
    localparam [3:0] S_EML_FINISH      = 4'd8;
    localparam [3:0] S_DONE            = 4'd9;

    reg [3:0] state;

    reg signed [`Q_WIDTH-1:0] reg_x;
    reg signed [`Q_WIDTH-1:0] reg_work_0; 
    reg signed [`Q_WIDTH-1:0] reg_work_1; 
    reg signed [5:0]          reg_k;
    reg                       flag_0;
    reg                       flag_1;

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

    wire signed [`Q_WIDTH+2:0] exp_sum_wide =
        $signed({{3{cordic_x_out[`Q_WIDTH-1]}}, cordic_x_out}) +
        $signed({{3{cordic_y_out[`Q_WIDTH-1]}}, cordic_y_out});

    wire signed [`Q_WIDTH+2:0] ln_full_wide =
        ($signed({{3{cordic_z_out[`Q_WIDTH-1]}}, cordic_z_out}) <<< 1) +
        $signed({{3{mul_result[`Q_WIDTH-1]}}, mul_result});

    wire signed [`Q_WIDTH+2:0] final_result_wide =
        $signed({{3{reg_work_1[`Q_WIDTH-1]}}, reg_work_1}) - 
        $signed({{3{reg_work_0[`Q_WIDTH-1]}}, reg_work_0});

    function signed [`Q_WIDTH-1:0] sat_wide_to_fp;
        input signed [`Q_WIDTH+2:0] value;
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
        input signed [`Q_WIDTH+2:0] value;
        begin
            wide_overflow =
                (value > $signed({1'b0, `FP_POS_MAX})) ||
                (value < $signed({1'b1, `FP_NEG_MAX}));
        end
    endfunction

    assign result = reg_work_1;
    assign done   = (state == S_DONE);
    assign busy   = (state != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            error           <= 1'b0;
            domain_error    <= 1'b0;
            overflow        <= 1'b0;
            mul_start_r     <= 1'b0;
            cordic_start_r  <= 1'b0;
            reg_work_1      <= `FP_ZERO;
        end else begin
            mul_start_r     <= 1'b0;
            cordic_start_r  <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        domain_error <= 1'b0;
                        overflow     <= 1'b0;
                        error        <= 1'b0;
                        
                        if (func_id != FUNC_RAW_EML) begin
                            reg_work_1 <= `FP_ZERO;
                            error    <= 1'b1;
                            state    <= S_DONE;
                        end else if (y_in <= `FP_ZERO) begin
                            reg_work_1    <= `FP_NEG_MAX;
                            domain_error  <= 1'b1;
                            state         <= S_DONE;
                        end else begin
                            reg_x      <= x_in;
                            reg_work_0 <= y_in;
                            reg_k      <= 6'sd0;
                            flag_0     <= 1'b0;
                            flag_1     <= 1'b0;

                            mul_a_r    <= x_in;
                            mul_b_r    <= `FP_INV_LN2;
                            mul_start_r<= 1'b1;
                            state      <= S_EML_NORM;
                        end
                    end
                end

                S_EML_NORM: begin
                    if (mul_done) flag_1 <= 1'b1;
                    if (!flag_0) begin
                        if (reg_work_0 <= `FP_ZERO) begin
                            reg_work_0 <= `FP_ONE;
                            reg_k      <= -6'sd31;
                            flag_0     <= 1'b1;
                        end else if (reg_work_0 >= `FP_TWO) begin
                            reg_work_0 <= reg_work_0 >>> 1;
                            reg_k      <= reg_k + 6'sd1;
                        end else if (reg_work_0 < `FP_ONE) begin
                            reg_work_0 <= reg_work_0 <<< 1;
                            reg_k      <= reg_k - 6'sd1;
                        end else if (reg_k >= 6'sd31 || reg_k <= -6'sd31) begin
                            flag_0     <= 1'b1;
                        end else begin
                            flag_0     <= 1'b1;
                        end
                    end
                    if (flag_0 && (flag_1 || mul_done)) begin
                        reg_work_1 <= mul_result;
                        state      <= S_EML_PREP_EXP;
                    end
                end

                S_EML_PREP_EXP: begin
                    reg_k <= (reg_work_1[`Q_WIDTH-1] ? (reg_work_1 - `FP_HALF) : (reg_work_1 + `FP_HALF)) >>> `Q_FRAC;
                    
                    cordic_mode_r <= 2'b01;
                    cordic_x_in_r <= reg_work_0 + `FP_ONE;
                    cordic_y_in_r <= reg_work_0 - `FP_ONE;
                    cordic_z_in_r <= `FP_ZERO;
                    cordic_start_r<= 1'b1;
                    flag_0        <= 1'b0;
                    
                    mul_a_r       <= ($signed(reg_k) <<< `Q_FRAC);
                    mul_b_r       <= `FP_LN2;
                    mul_start_r   <= 1'b1;
                    flag_1        <= 1'b0;
                    
                    state <= S_EML_MUL_LN;
                end

                S_EML_MUL_LN: begin
                    if (cordic_done) flag_0 <= 1'b1;
                    if (mul_done)    flag_1 <= 1'b1;
                    if ((flag_0 || cordic_done) && (flag_1 || mul_done)) begin
                        reg_work_0 <= sat_wide_to_fp(ln_full_wide);
                        
                        mul_a_r    <= ($signed(reg_k) <<< `Q_FRAC);
                        mul_b_r    <= `FP_LN2;
                        mul_start_r<= 1'b1;
                        state      <= S_EML_MUL_EXP;
                    end
                end

                S_EML_MUL_EXP: begin
                    if (mul_done) begin
                        reg_work_1 <= reg_x - mul_result;
                        state      <= S_EML_PREP_CORDIC;
                    end
                end

                S_EML_PREP_CORDIC: begin
                    cordic_mode_r <= 2'b00;
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
                    if (reg_k > 0) begin
                        if (reg_work_1 > `FP_SHIFT_SAT_POS) reg_work_1 <= `FP_POS_MAX;
                        else if (reg_work_1 < `FP_SHIFT_SAT_NEG) reg_work_1 <= `FP_NEG_MAX;
                        else reg_work_1 <= reg_work_1 <<< 1;
                        reg_k <= reg_k - 6'sd1;
                    end else if (reg_k < 0) begin
                        reg_work_1 <= reg_work_1 >>> 1;
                        reg_k    <= reg_k + 6'sd1;
                    end else begin
                        state <= S_EML_FINISH;
                    end
                end

                S_EML_FINISH: begin
                    reg_work_1 <= sat_wide_to_fp(final_result_wide);
                    overflow   <= wide_overflow(final_result_wide);
                    state    <= S_DONE;
                end

                S_DONE: begin
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase


            if (start && (state != S_IDLE)) error <= 1'b1;
        end
    end

endmodule
