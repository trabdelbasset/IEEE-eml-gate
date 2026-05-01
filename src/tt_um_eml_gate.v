`default_nettype none

module tt_um_eml_gate (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    wire ser_in   = ui_in[0];
    wire shift_en = ui_in[1];
    wire start_op = ui_in[2];

    wire ser_out;
    wire busy_ext;
    wire done_ext;
    wire error_ext;
    wire rx_full_ext;
    wire tx_pending_ext;

    assign uo_out[0] = ser_out;
    assign uo_out[1] = busy_ext;
    assign uo_out[2] = done_ext;
    assign uo_out[3] = error_ext;
    assign uo_out[4] = rx_full_ext;
    assign uo_out[5] = tx_pending_ext;
    assign uo_out[7:6] = 2'b00;

    assign uio_out = 8'd0;
    assign uio_oe  = 8'd0;

    wire _unused = &{ena, clk, rst_n, ui_in[7:3], uio_in, 1'b0};

    eml_serial_gate u_eml_serial_gate (
        .clk    (clk),
        .rst_n  (rst_n),
        .ser_in (ser_in),
        .shift_en(shift_en),
        .start  (start_op),
        .ser_out(ser_out),
        .busy   (busy_ext),
        .done   (done_ext),
        .error  (error_ext),
        .rx_full(rx_full_ext),
        .tx_pending(tx_pending_ext)
    );

endmodule
