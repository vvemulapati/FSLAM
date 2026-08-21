//------------------------------------------------------------------------------
// File: scaling.sv
// Description: Streaming 5/6 image downsampler (1.2x reduction).
//              Uses a 2-row line window buffer and bilinear interpolation.
//              Outputs resampled pixels every 5 out of 6 cycles in X and Y.
//------------------------------------------------------------------------------

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
    logic [2:0]  scale_x, scale_y;

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

    // Raster coordinate tracking
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_x    <= '0;
            in_y    <= '0;
            scale_x <= '0;
            scale_y <= '0;
        end else if (pixel_valid) begin
            // X coordinate & modulo 6 counter
            if (in_x == IMG_WIDTH - 1) begin
                in_x    <= '0;
                scale_x <= '0;
                // Y coordinate & modulo 6 counter
                if (in_y == IMG_HEIGHT - 1) begin
                    in_y    <= '0;
                    scale_y <= '0;
                end else begin
                    in_y    <= in_y + 1;
                    scale_y <= (scale_y == 3'd5) ? 3'd0 : scale_y + 1;
                end
            end else begin
                in_x    <= in_x + 1;
                scale_x <= (scale_x == 3'd5) ? 3'd0 : scale_x + 1;
            end
        end
    end

    // Pixel is valid when within active 2x2 window region and not in 6th cycle/row
    logic interp_valid_in;
    assign interp_valid_in = win_valid && (scale_x != 3'd5) && (scale_y != 3'd5) && (in_y >= 1);

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
        .frac_x(scale_x),
        .frac_y(scale_y),
        .valid_out(scaled_valid),
        .pixel_out(pixel_out)
    );

endmodule
