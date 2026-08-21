// Rotates a BRIEF test point (x, y) by the keypoint angle using 64-sector
// sin/cos tables (Q1.7):
//   x' = (x*cos(theta) - y*sin(theta)) >>> 7
//   y' = (x*sin(theta) + y*cos(theta)) >>> 7

`timescale 1ns / 1ps

module rotator #(
    parameter int COORD_WIDTH = 6,
    parameter int ANGLE_BITS  = 6
) (
    input  logic                             clk,
    input  logic                             rst_n,
    input  logic                             valid_in,

    input  logic signed [COORD_WIDTH-1:0]    x_in,
    input  logic signed [COORD_WIDTH-1:0]    y_in,
    input  logic [ANGLE_BITS-1:0]            angle,

    output logic signed [COORD_WIDTH-1:0]    x_out,
    output logic signed [COORD_WIDTH-1:0]    y_out,
    output logic                             valid_out
);

    // 64-sector Cosine and Sine LUTs in Q1.7 format (scaled by 127)
    localparam logic signed [7:0] COS_LUT [0:63] = '{
         8'sd127,  8'sd126,  8'sd125,  8'sd122,  8'sd117,  8'sd112,  8'sd106,  8'sd98,
         8'sd90,   8'sd81,   8'sd71,   8'sd60,   8'sd49,   8'sd37,   8'sd25,   8'sd12,
         8'sd0,   -8'sd12,  -8'sd25,  -8'sd37,  -8'sd49,  -8'sd60,  -8'sd71,  -8'sd81,
        -8'sd90,  -8'sd98,  -8'sd106, -8'sd112, -8'sd117, -8'sd122, -8'sd125, -8'sd126,
        -8'sd127, -8'sd126, -8'sd125, -8'sd122, -8'sd117, -8'sd112, -8'sd106, -8'sd98,
        -8'sd90,  -8'sd81,  -8'sd71,  -8'sd60,  -8'sd49,  -8'sd37,  -8'sd25,  -8'sd12,
         8'sd0,    8'sd12,   8'sd25,   8'sd37,   8'sd49,   8'sd60,   8'sd71,   8'sd81,
         8'sd90,   8'sd98,   8'sd106,  8'sd112,  8'sd117,  8'sd122,  8'sd125,  8'sd126
    };

    localparam logic signed [7:0] SIN_LUT [0:63] = '{
         8'sd0,    8'sd12,   8'sd25,   8'sd37,   8'sd49,   8'sd60,   8'sd71,   8'sd81,
         8'sd90,   8'sd98,   8'sd106,  8'sd112,  8'sd117,  8'sd122,  8'sd125,  8'sd126,
         8'sd127,  8'sd126,  8'sd125,  8'sd122,  8'sd117,  8'sd112,  8'sd106,  8'sd98,
         8'sd90,   8'sd81,   8'sd71,   8'sd60,   8'sd49,   8'sd37,   8'sd25,   8'sd12,
         8'sd0,   -8'sd12,  -8'sd25,  -8'sd37,  -8'sd49,  -8'sd60,  -8'sd71,  -8'sd81,
        -8'sd90,  -8'sd98,  -8'sd106, -8'sd112, -8'sd117, -8'sd122, -8'sd125, -8'sd126,
        -8'sd127, -8'sd126, -8'sd125, -8'sd122, -8'sd117, -8'sd112, -8'sd106, -8'sd98,
        -8'sd90,  -8'sd81,  -8'sd71,  -8'sd60,  -8'sd49,  -8'sd37,  -8'sd25,  -8'sd12
    };

    logic signed [7:0] cos_val, sin_val;
    assign cos_val = COS_LUT[angle];
    assign sin_val = SIN_LUT[angle];

    logic signed [COORD_WIDTH+8:0] rot_x, rot_y;

    always_comb begin
        rot_x = (x_in * cos_val) - (y_in * sin_val);
        rot_y = (x_in * sin_val) + (y_in * cos_val);
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            x_out     <= '0;
            y_out     <= '0;
        end else begin
            valid_out <= valid_in;
            if (valid_in) begin
                x_out <= rot_x >>> 7;
                y_out <= rot_y >>> 7;
            end
        end
    end

endmodule
