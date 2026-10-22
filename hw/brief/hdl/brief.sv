// rBRIEF descriptor (256 bits).
//   - Captures the 37x37 blurred patch into a single frozen_win buffer on
//     patch_latch (sharing the single 37x37 window_buffer with orientation).
//   - Because 3x3 NMS guarantees >= 2 clock cycles between keypoints and
//     orientation completes in 2 cycles after patch_latch, frozen_win is
//     sampled on start using the registered 6-bit orientation (with max_fanout
//     replication in orientation.sv), allowing Vivado to constant-fold each
//     rotated 6-bit coordinate into a single LUT6 with zero multipliers/adders.

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

    // 64-sector Cosine and Sine LUTs (Q1.7 format: round(127 * cos/sin(2*pi*i/64)))
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

    // Single 37x37 patch buffer (latched on patch_latch, sampled 2 cycles later on start)
    logic [PIXEL_WIDTH-1:0] frozen_win [WINDOW_SIZE-1:0][WINDOW_SIZE-1:0];
    logic                   patch_pending;

    // Active sin/cos from 6-bit orientation (allows Vivado to collapse each
    // rotated coordinate bit into a single 6-input LUT6 of orientation[5:0])
    logic signed [7:0] cos_val, sin_val;
    assign cos_val = COS_LUT[orientation];
    assign sin_val = SIN_LUT[orientation];

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

            // Match OpenCV ORBextractor.cc (t0 < t1) polarity
            desc_comb[i] = (p1 < p2);
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            patch_pending      <= 1'b0;
            descriptor         <= '0;
            keypoint_score_out <= '0;
            keypoint_x_out     <= '0;
            keypoint_y_out     <= '0;
            orientation_out    <= '0;
            descriptor_valid   <= 1'b0;
        end else begin
            if (patch_latch) begin
                frozen_win    <= window_buffer;
                patch_pending <= 1'b1;
            end else if (start) begin
                patch_pending <= 1'b0;
            end

            descriptor_valid <= start;
            if (start) begin
                descriptor         <= desc_comb;
                keypoint_score_out <= keypoint_score_in;
                keypoint_x_out     <= keypoint_x_in;
                keypoint_y_out     <= keypoint_y_in;
                orientation_out    <= orientation;
            end
        end
    end

    assign busy = patch_pending || descriptor_valid;

endmodule
