// Streaming 5/6 (1.2x) downscaler: 2x2 window plus bilinear interpolation.
// Emits 5 of every 6 pixels in x and 5 of every 6 rows in y.

`timescale 1ns / 1ps

module scaling #(
    parameter int PIXEL_WIDTH = 6,
    parameter int IMG_WIDTH   = 640,
    parameter int IMG_HEIGHT  = 480
) (
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic                   pixel_valid,
    input  logic [PIXEL_WIDTH-1:0] pixel_in,

    output logic                   scaled_valid,
    output logic [PIXEL_WIDTH-1:0] pixel_out
);

    logic [11:0] in_x, in_y;
    logic [2:0]  mod_x, mod_y;
    logic [11:0] win_x1, win_y1;
    logic [2:0]  win_frac_x, win_frac_y;

    // 2x2 window from window_buffer
    logic [PIXEL_WIDTH-1:0] win [1:0][1:0];
    logic                   win_valid;

    window_buffer #(
        .DATA_WIDTH(PIXEL_WIDTH),
        .LINE_WIDTH(IMG_WIDTH),
        .WINDOW_SIZE(2)
    ) u_win_buf (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(pixel_valid),
        .in_data(pixel_in),
        .out_valid(win_valid),
        .win(win)
    );

    // Raster coordinate tracking aligned with the 1-cycle window_buffer output
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            in_x       <= '0;
            in_y       <= '0;
            mod_x      <= '0;
            mod_y      <= '0;
            win_x1     <= '0;
            win_y1     <= '0;
            win_frac_x <= '0;
            win_frac_y <= '0;
        end else if (pixel_valid) begin
            win_x1     <= in_x;
            win_y1     <= in_y;
            win_frac_x <= mod_x;
            win_frac_y <= mod_y;

            if (in_x == 12'(IMG_WIDTH - 1)) begin
                in_x  <= '0;
                mod_x <= '0;
                if (in_y == 12'(IMG_HEIGHT - 1)) begin
                    in_y  <= '0;
                    mod_y <= '0;
                end else if (in_y == 12'd0) begin
                    in_y  <= 12'd1;
                    mod_y <= 3'd0;
                end else begin
                    in_y  <= in_y + 12'd1;
                    mod_y <= (mod_y == 3'd5) ? 3'd0 : mod_y + 3'd1;
                end
            end else if (in_x == 12'd0) begin
                in_x  <= 12'd1;
                mod_x <= 3'd0;
            end else begin
                in_x  <= in_x + 12'd1;
                mod_x <= (mod_x == 3'd5) ? 3'd0 : mod_x + 3'd1;
            end
        end
    end

    // Valid when the 2x2 window does not wrap across rows/frames and is not on the 6th step
    logic interp_valid_in;
    assign interp_valid_in = win_valid && (win_x1 >= 12'd1) && (win_y1 >= 12'd1) &&
                             (win_frac_x != 3'd5) && (win_frac_y != 3'd5);

    bilinear_interpolator #(
        .PIXEL_WIDTH(PIXEL_WIDTH)
    ) u_interpolator (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(interp_valid_in),
        .pixel_00(win[1][1]), // Top-left
        .pixel_01(win[1][0]), // Top-right
        .pixel_10(win[0][1]), // Bottom-left
        .pixel_11(win[0][0]), // Bottom-right
        .frac_x(win_frac_x),
        .frac_y(win_frac_y),
        .valid_out(scaled_valid),
        .pixel_out(pixel_out)
    );

endmodule
