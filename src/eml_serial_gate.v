// ============================================================================
// eml_serial_gate.v
//
// Request (28 bits):
//   [27:24] func_id
//   [23:12] x operand (12-bit)
//   [11:0]  y operand (12-bit)
//
// Response (15 bits):
//   [14]    error
//   [13]    domain_error
//   [12]    overflow
//   [11:0]  result (12-bit)
// ============================================================================

`include "fp_pkg.vh"

module eml_serial_gate (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       ser_in,
    input  wire       shift_en,
    input  wire       start,

    output wire       ser_out,
    output wire       busy,
    output wire       done,
    output reg        error,
    output wire       rx_full,
    output wire       tx_pending
);

    localparam [4:0] RX_BITS = 5'd28;
    localparam [3:0] TX_BITS = 4'd15;

    reg [27:0] rx_shift_reg;
    reg [4:0]  rx_count_reg;
    reg [14:0] tx_shift_reg;
    reg [3:0]  tx_count_reg;
    reg        tx_pending_reg;

    wire [3:0] gate_func = rx_shift_reg[27:24];
    wire signed [`Q_WIDTH-1:0] gate_x = rx_shift_reg[23:12];
    wire signed [`Q_WIDTH-1:0] gate_y = rx_shift_reg[11:0];
    wire signed [`Q_WIDTH-1:0] gate_result;
    wire gate_done;
    wire gate_busy;
    wire gate_start;
    wire gate_error;
    wire gate_domain_error;
    wire gate_overflow;

    assign gate_start = start & rx_full & ~gate_busy & ~shift_en;
    assign ser_out    = tx_pending ? tx_shift_reg[14] : 1'b0;
    assign busy       = gate_busy;
    assign done       = gate_done;
    assign rx_full    = (rx_count_reg == RX_BITS);
    assign tx_pending = tx_pending_reg;

    eml_gate_top u_eml_gate_top (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (gate_start),
        .func_id      (gate_func),
        .x_in         (gate_x),
        .y_in         (gate_y),
        .result       (gate_result),
        .done         (gate_done),
        .busy         (gate_busy),
        .error        (gate_error),
        .domain_error (gate_domain_error),
        .overflow     (gate_overflow)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_shift_reg   <= 28'd0;
            rx_count_reg   <= 5'd0;
            tx_shift_reg   <= 15'd0;
            tx_count_reg   <= 4'd0;
            tx_pending_reg <= 1'b0;
            error          <= 1'b0;
        end else begin
            // Clear error only on a fresh start pulse (if we aren't busy)
            if (start && !gate_busy) begin
                error <= 1'b0;
            end

            if (gate_done) begin
                tx_shift_reg   <= {gate_error, gate_domain_error, gate_overflow, gate_result};
                tx_count_reg   <= TX_BITS;
                tx_pending_reg <= 1'b1;
                if (gate_error || gate_domain_error || gate_overflow)
                    error <= 1'b1;
            end

            if (shift_en && start) begin
                error <= 1'b1;
            end else if (shift_en) begin
                if (gate_busy) begin
                    error <= 1'b1;
                end else if (tx_pending_reg) begin
                    tx_shift_reg <= {tx_shift_reg[13:0], 1'b0};
                    if (tx_count_reg != 4'd0) begin
                        tx_count_reg <= tx_count_reg - 4'd1;
                    end
                    if (tx_count_reg == 4'd1) begin
                        tx_pending_reg <= 1'b0;
                    end
                end else if (!rx_full) begin
                    rx_shift_reg <= {rx_shift_reg[26:0], ser_in};
                    rx_count_reg <= rx_count_reg + 5'd1;
                end else begin
                    error <= 1'b1;
                end
            end

            if (start) begin
                if (!rx_full || gate_busy || shift_en) begin
                    error <= 1'b1;
                end else begin
                    rx_count_reg <= 5'd0;
                end
            end
        end
    end

endmodule
