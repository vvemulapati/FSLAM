//------------------------------------------------------------------------------
// File: bilinear_interpolator.sv
// Description: Bilinear interpolator for 5/6 (1.2x) downsampling.
//              Computes weighted sum of 2x2 neighborhood based on (x%6, y%6).
//              Normalizes by dividing by 25 using fixed-point multiplication:
//              pixel_out = (weighted_sum * 41 + 512) >> 10
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module bilinear_interpolator #(
    parameter int PIXEL_WIDTH = 6
) (
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic                   valid_in,

    // 2x2 neighborhood: [row][col] -> [y][x]
    input  logic [PIXEL_WIDTH-1:0] pixel_00, // (y0, x0) top-left
    input  logic [PIXEL_WIDTH-1:0] pixel_01, // (y0, x1) top-right
    input  logic [PIXEL_WIDTH-1:0] pixel_10, // (y1, x0) bottom-left
    input  logic [PIXEL_WIDTH-1:0] pixel_11, // (y1, x1) bottom-right

    // Fractional coordinates (0 to 4, representing modulo 6 position)
    input  logic [2:0]             frac_x,
    input  logic [2:0]             frac_y,

    output logic                   valid_out,
    output logic [PIXEL_WIDTH-1:0] pixel_out
);

    logic [2:0] inv_x, inv_y;
    logic [5:0] w00, w01, w10, w11;
    logic [PIXEL_WIDTH+5:0] wp00, wp01, wp10, wp11;
    logic [PIXEL_WIDTH+6:0] weighted_sum;
    logic [PIXEL_WIDTH+16:0] scaled_sum;

    always_comb begin
        inv_x = 3'd5 - frac_x;
        inv_y = 3'd5 - frac_y;

        // Weights: sum of w00+w01+w10+w11 = 25
        w00 = inv_x * inv_y;
        w01 = frac_x * inv_y;
        w10 = inv_x * frac_y;
        w11 = frac_x * frac_y;

        wp00 = pixel_00 * w00;
        wp01 = pixel_01 * w01;
        wp10 = pixel_10 * w10;
        wp11 = pixel_11 * w11;

        weighted_sum = wp00 + wp01 + wp10 + wp11;
        // Fixed-point division by 25: (weighted_sum * 41 + 512) >> 10
        scaled_sum   = (weighted_sum * 17'd41) + 17'd512;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            pixel_out <= '0;
        end else begin
            valid_out <= valid_in;
            if (valid_in) begin
                pixel_out <= scaled_sum[PIXEL_WIDTH+9:10];
            end
        end
    end

endmodule
