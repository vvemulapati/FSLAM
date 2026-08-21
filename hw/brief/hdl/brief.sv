// rBRIEF descriptor (256 bits). On a keypoint, freezes the 37x37 blurred patch,
// rotates the 256 test pairs by the keypoint angle and compares the sampled
// pixel pairs. Keypoint x, y and score are passed through with descriptor_valid.

`timescale 1ns / 1ps

module brief #(
    parameter int PIXEL_WIDTH     = 6,
    parameter int WINDOW_SIZE     = 37,
    parameter int DESCRIPTOR_BITS = 256,
    parameter int ANGLE_BITS      = 6,
    parameter int COORD_BITS      = 6
) (
    input  logic                                  clk,
    input  logic                                  rst_n,
    input  logic                                  patch_latch,
    input  logic                                  start,

    input  logic [15:0]                           keypoint_score_in,
    input  logic [11:0]                           keypoint_x_in,
    input  logic [11:0]                           keypoint_y_in,
    input  logic [ANGLE_BITS-1:0]                 orientation,
    input  logic [PIXEL_WIDTH-1:0]                window_buffer [WINDOW_SIZE-1:0][WINDOW_SIZE-1:0],
    input  logic                                  window_valid,

    // 256 test pairs (x1, y1, x2, y2), coordinates in [-13, 12]
    input  logic signed [COORD_BITS-1:0]          pattern_coords [0:DESCRIPTOR_BITS-1][0:3],

    output logic [DESCRIPTOR_BITS-1:0]            descriptor,
    output logic [15:0]                           keypoint_score_out,
    output logic [11:0]                           keypoint_x_out,
    output logic [11:0]                           keypoint_y_out,
    output logic [ANGLE_BITS-1:0]                 orientation_out,
    output logic                                  descriptor_valid,
    output logic                                  busy
);

    localparam int WINDOW_CENTER = WINDOW_SIZE / 2; // 18

    // 64-sector Cosine and Sine LUTs (Q1.7 format)
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

    // Latched patch buffer and metadata
    logic [PIXEL_WIDTH-1:0] frozen_win [WINDOW_SIZE-1:0][WINDOW_SIZE-1:0];
    logic [ANGLE_BITS-1:0]  active_orientation;
    logic [15:0]            latched_score;
    logic [11:0]            latched_x, latched_y;
    logic                   s1_valid;

    // Active sin/cos
    logic signed [7:0] cos_val, sin_val;
    assign cos_val = COS_LUT[active_orientation];
    assign sin_val = SIN_LUT[active_orientation];

    // Rotate all 256 test pairs and compare sampled pixels from frozen_win.
    // frozen_win[0][0] is bottom-right (+18, +18), so index = WINDOW_CENTER - rot.
    logic [DESCRIPTOR_BITS-1:0] desc_comb;

    always_comb begin
        desc_comb = '0;
        for (int i = 0; i < DESCRIPTOR_BITS; i++) begin
            automatic logic signed [15:0] px1_cos, py1_sin, px1_sin, py1_cos;
            automatic logic signed [15:0] px2_cos, py2_sin, px2_sin, py2_cos;
            automatic logic signed [7:0]  rx1, ry1, rx2, ry2;
            automatic logic signed [7:0]  wx1, wy1, wx2, wy2;
            automatic logic [PIXEL_WIDTH-1:0] p1, p2;

            px1_cos = 16'($signed(pattern_coords[i][0])) * 16'($signed(cos_val));
            py1_sin = 16'($signed(pattern_coords[i][1])) * 16'($signed(sin_val));
            px1_sin = 16'($signed(pattern_coords[i][0])) * 16'($signed(sin_val));
            py1_cos = 16'($signed(pattern_coords[i][1])) * 16'($signed(cos_val));

            px2_cos = 16'($signed(pattern_coords[i][2])) * 16'($signed(cos_val));
            py2_sin = 16'($signed(pattern_coords[i][3])) * 16'($signed(sin_val));
            px2_sin = 16'($signed(pattern_coords[i][2])) * 16'($signed(sin_val));
            py2_cos = 16'($signed(pattern_coords[i][3])) * 16'($signed(cos_val));

            rx1 = 8'($signed(px1_cos - py1_sin) >>> 7);
            ry1 = 8'($signed(px1_sin + py1_cos) >>> 7);
            rx2 = 8'($signed(px2_cos - py2_sin) >>> 7);
            ry2 = 8'($signed(px2_sin + py2_cos) >>> 7);

            wx1 = 8'(WINDOW_CENTER) - rx1;
            wy1 = 8'(WINDOW_CENTER) - ry1;
            wx2 = 8'(WINDOW_CENTER) - rx2;
            wy2 = 8'(WINDOW_CENTER) - ry2;

            p1 = (wx1 >= 0 && wx1 < 8'(WINDOW_SIZE) && wy1 >= 0 && wy1 < 8'(WINDOW_SIZE)) ?
                 frozen_win[wy1[5:0]][wx1[5:0]] : '0;
            p2 = (wx2 >= 0 && wx2 < 8'(WINDOW_SIZE) && wy2 >= 0 && wy2 < 8'(WINDOW_SIZE)) ?
                 frozen_win[wy2[5:0]][wx2[5:0]] : '0;

            desc_comb[i] = (p1 < p2);
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            active_orientation <= '0;
            latched_score      <= '0;
            latched_x          <= '0;
            latched_y          <= '0;
            s1_valid           <= 1'b0;
            descriptor         <= '0;
            keypoint_score_out <= '0;
            keypoint_x_out     <= '0;
            keypoint_y_out     <= '0;
            orientation_out    <= '0;
            descriptor_valid   <= 1'b0;
        end else begin
            if (patch_latch) begin
                frozen_win <= window_buffer;
            end

            s1_valid <= start;
            if (start) begin
                active_orientation <= orientation;
                latched_score      <= keypoint_score_in;
                latched_x          <= keypoint_x_in;
                latched_y          <= keypoint_y_in;
            end

            descriptor_valid <= s1_valid;
            if (s1_valid) begin
                descriptor         <= desc_comb;
                keypoint_score_out <= latched_score;
                keypoint_x_out     <= latched_x;
                keypoint_y_out     <= latched_y;
                orientation_out    <= active_orientation;
            end
        end
    end

    assign busy = start || s1_valid || descriptor_valid;

endmodule
