// Bilinear interpolation for the 5/6 (1.2x) downscaler. The 2x2 weights come
// from (x % 6, y % 6) and sum to 25; the divide by 25 is done as
// (weighted_sum * 41 + 512) >> 10.

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

    localparam int SCALED_W = PIXEL_WIDTH + 17;

    // Weight table for a, b in 0..5
    localparam logic [4:0] WEIGHT_LUT [0:7][0:7] = '{
        '{5'd0, 5'd0,  5'd0,  5'd0,  5'd0,  5'd0,  5'd0, 5'd0},
        '{5'd0, 5'd1,  5'd2,  5'd3,  5'd4,  5'd5,  5'd0, 5'd0},
        '{5'd0, 5'd2,  5'd4,  5'd6,  5'd8,  5'd10, 5'd0, 5'd0},
        '{5'd0, 5'd3,  5'd6,  5'd9,  5'd12, 5'd15, 5'd0, 5'd0},
        '{5'd0, 5'd4,  5'd8,  5'd12, 5'd16, 5'd20, 5'd0, 5'd0},
        '{5'd0, 5'd5,  5'd10, 5'd15, 5'd20, 5'd25, 5'd0, 5'd0},
        '{5'd0, 5'd0,  5'd0,  5'd0,  5'd0,  5'd0,  5'd0, 5'd0},
        '{5'd0, 5'd0,  5'd0,  5'd0,  5'd0,  5'd0,  5'd0, 5'd0}
    };

    logic [2:0] inv_x, inv_y;
    logic [5:0] w00, w01, w10, w11;
    logic [PIXEL_WIDTH+5:0] wp00, wp01, wp10, wp11;
    logic [PIXEL_WIDTH+6:0] weighted_sum;
    logic [SCALED_W-1:0]    scaled_sum;

    always_comb begin
        inv_x = (frac_x <= 3'd5) ? (3'd5 - frac_x) : 3'd0;
        inv_y = (frac_y <= 3'd5) ? (3'd5 - frac_y) : 3'd0;

        // Weights: sum of w00+w01+w10+w11 = 25
        w00 = {1'b0, WEIGHT_LUT[inv_y][inv_x]};
        w01 = {1'b0, WEIGHT_LUT[inv_y][frac_x]};
        w10 = {1'b0, WEIGHT_LUT[frac_y][inv_x]};
        w11 = {1'b0, WEIGHT_LUT[frac_y][frac_x]};

        wp00 = pixel_00 * w00;
        wp01 = pixel_01 * w01;
        wp10 = pixel_10 * w10;
        wp11 = pixel_11 * w11;

        weighted_sum = wp00 + wp01 + wp10 + wp11;

        // Fixed-point division by 25: (weighted_sum * 41 + 512) >> 10
        scaled_sum   = (SCALED_W'(weighted_sum) << 5)
                     + (SCALED_W'(weighted_sum) << 3)
                     + SCALED_W'(weighted_sum)
                     + SCALED_W'(512);
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
