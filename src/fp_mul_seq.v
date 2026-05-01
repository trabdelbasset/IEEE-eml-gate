`include "fp_pkg.vh"

module fp_mul_seq (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire signed [`Q_WIDTH-1:0] a,
    input  wire signed [`Q_WIDTH-1:0] b,
    output wire signed [`Q_WIDTH-1:0] result,
    output wire                       done
);

    localparam S_IDLE = 2'd0;
    localparam S_CALC = 2'd1;
    localparam S_DONE = 2'd2;

    (* fsm_encoding = "binary" *) reg [1:0] state;

    reg [3:0] count;

    reg signed [`Q_WIDTH:0]     p_reg;
    reg signed [`Q_WIDTH-1:0]   p_low_reg;
    reg signed [`Q_WIDTH-1:0]   a_reg;
    reg signed [`Q_WIDTH-1:0]   b_reg;

    localparam signed [`Q_WIDTH-1:0] SAT_POS = {1'b0, {(`Q_WIDTH-1){1'b1}}};
    localparam signed [`Q_WIDTH-1:0] SAT_NEG = {1'b1, {(`Q_WIDTH-1){1'b0}}};

    wire signed [2*`Q_WIDTH-1:0] full_p = {p_reg[`Q_WIDTH-1:0], p_low_reg};
    wire signed [2*`Q_WIDTH-1:0] p_shifted = full_p >>> `Q_FRAC;
    
    wire p_overflow = (p_shifted[2*`Q_WIDTH-1:`Q_WIDTH] !=
                       {`Q_WIDTH{p_shifted[`Q_WIDTH-1]}});

    assign result = p_overflow ? (p_shifted[2*`Q_WIDTH-1] ? SAT_NEG : SAT_POS) : p_shifted[`Q_WIDTH-1:0];
    assign done   = (state == S_DONE);

    wire signed [`Q_WIDTH:0] p_plus_a = p_reg + {a_reg[`Q_WIDTH-1], a_reg};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= S_IDLE;
            p_reg  <= 0;
            p_low_reg <= 0;
            a_reg  <= 0;
            b_reg  <= 0;
            count  <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        a_reg <= a;
                        b_reg <= b;
                        p_reg <= 0;
                        p_low_reg <= 0;
                        count <= 0;
                        state <= S_CALC;
                    end
                end

                S_CALC: begin
                    if (count == `Q_WIDTH - 1) begin
                        if (b_reg[0])
                            p_reg <= p_reg - {a_reg[`Q_WIDTH-1], a_reg};
                        state <= S_DONE;
                    end else begin
                        if (b_reg[0]) begin
                            p_reg <= p_plus_a >>> 1;
                            p_low_reg <= { p_plus_a[0], p_low_reg[`Q_WIDTH-1:1] };
                        end else begin
                            p_reg <= p_reg >>> 1;
                            p_low_reg <= { p_reg[0], p_low_reg[`Q_WIDTH-1:1] };
                        end
                        b_reg <= {b_reg[`Q_WIDTH-1], b_reg[`Q_WIDTH-1:1]}; 
                        count <= count + 1;
                    end
                end

                S_DONE: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
