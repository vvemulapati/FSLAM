//------------------------------------------------------------------------------
// File: rotator.sv
// Description: BRIEF coordinate rotator module using 64-sector sin/cos LUT.
//              Rotates coordinate pair (x, y) relative to keypoint center:
//                x' = (x*cos(θ) - y*sin(θ)) >>> 7
//                y' = (x*sin(θ) + y*cos(θ)) >>> 7
//------------------------------------------------------------------------------

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
        8'sd127, 8'sd126, 8'sd124, 8'sd120, 8'sd114, 8'sd108, 8'sd99,  8'sd90,
        8'sd80,  8'sd69,  8'sd57,  8'sd45,  8'sd32,  8'sd19,  8'sd6,   -8'sd6,
        -8'sd19, -8'sd32, -8'sd45, -8'sd57, -8'sd69, -8'sd80, -8'sd90, -8'sd99,
        -8'sd108,-8'sd114,-8'sd120,-8'sd124,-8'sd126,-8'sd127,-8'sd127,-8'sd126,
        -8'sd124,-8'sd120,-8'sd114,-8'sd108,-8'sd99, -8'sd90, -8'sd80, -8'sd69,
        -8'sd57, -8'sd45, -8'sd32, -8'sd19, -8'sd6,   8'sd6,   8'sd19,  8'sd32,
        8'sd45,  8'sd57,  8'sd69,  8'sd80,  8'sd90,  8'sd99,  8'sd108, 8'sd114,
        8'sd120, 8'sd124, 8'sd126, 8'sd127, 8'sd127, 8'sd126, 8'sd124, 8'sd120
    };

    localparam logic signed [7:0] SIN_LUT [0:63] = '{
        8'sd0,   8'sd12,  8'sd25,  8'sd37,  8'sd49,  8'sd60,  8'sd71,  8'sd80,
        8'sd90,  8'sd99,  8'sd108, 8'sd114, 8'sd120, 8'sd124, 8'sd126, 8'sd127,
        8'sd127, 8'sd126, 8'sd124, 8'sd120, 8'sd114, 8'sd108, 8'sd99,  8'sd90,
        8'sd80,  8'sd71,  8'sd60,  8'sd49,  8'sd37,  8'sd25,  8'sd12,  8'sd0,
        -8'sd12, -8'sd25, -8'sd37, -8'sd49, -8'sd60, -8'sd71, -8'sd80, -8'sd90,
        -8'sd99, -8'sd108,-8'sd114,-8'sd120,-8'sd124,-8'sd126,-8'sd127,-8'sd127,
        -8'sd126,-8'sd124,-8'sd120,-8'sd114,-8'sd108,-8'sd99, -8'sd90, -8'sd80,
        -8'sd71, -8'sd60, -8'sd49, -8'sd37, -8'sd25, -8'sd12, -8'sd0,  -8'sd0
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
