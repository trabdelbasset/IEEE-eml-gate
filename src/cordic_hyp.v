`include "fp_pkg.vh"

module cordic_hyp (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire [1:0]                 mode,
    input  wire signed [`Q_WIDTH-1:0] x_in,
    input  wire signed [`Q_WIDTH-1:0] y_in,
    input  wire signed [`Q_WIDTH-1:0] z_in,
    output wire signed [`Q_WIDTH-1:0] x_out,
    output wire signed [`Q_WIDTH-1:0] y_out,
    output wire signed [`Q_WIDTH-1:0] z_out,
    output wire                       done
);

    function signed [`Q_WIDTH-1:0] get_atanh;
        input [3:0] i;
        case (i)
            4'd1:  get_atanh = 16'sd562;
            4'd2:  get_atanh = 16'sd262;
            4'd3:  get_atanh = 16'sd129;
            4'd4:  get_atanh = 16'sd64;
            4'd5:  get_atanh = 16'sd32;
            4'd6:  get_atanh = 16'sd16;
            4'd7:  get_atanh = 16'sd8;
            4'd8:  get_atanh = 16'sd4;
            4'd9:  get_atanh = 16'sd2;
            4'd10: get_atanh = 16'sd1;
            4'd11: get_atanh = 16'sd1;
            default: get_atanh = 16'sd0;
        endcase
    endfunction

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
            4'd9:  get_shift_hyp = 4'd9;
            4'd10: get_shift_hyp = 4'd10;
            4'd11: get_shift_hyp = 4'd11;
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

    wire is_vector  = mode_reg[0];

    wire [3:0] cur_shift = get_shift_hyp(step);
    wire signed [`Q_WIDTH-1:0] angle = get_atanh(cur_shift);

    wire signed [`Q_WIDTH-1:0] x_shifted = x_reg >>> cur_shift;
    wire signed [`Q_WIDTH-1:0] y_shifted = y_reg >>> cur_shift;

    wire sigma = is_vector ? y_reg[`Q_WIDTH-1] : ~z_reg[`Q_WIDTH-1];

    wire signed [`Q_WIDTH-1:0] x_delta = sigma ? y_shifted : -y_shifted;
    wire signed [`Q_WIDTH-1:0] y_delta = sigma ? x_shifted : -x_shifted;

    wire signed [`Q_WIDTH-1:0] x_next = x_reg + x_delta;
    wire signed [`Q_WIDTH-1:0] y_next = y_reg + y_delta;
    wire signed [`Q_WIDTH-1:0] z_next = sigma ? (z_reg - angle) : (z_reg + angle);

    assign x_out = x_reg;
    assign y_out = y_reg;
    assign z_out = z_reg;
    assign done  = (state == S_DONE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            x_reg <= 0; y_reg <= 0; z_reg <= 0;
            step  <= 0;
            mode_reg <= 0;
        end else begin
            case (state)
                S_IDLE: begin
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
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
