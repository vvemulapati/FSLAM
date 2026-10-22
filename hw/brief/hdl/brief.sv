// rBRIEF descriptor (256 bits).
//   - Captures the 37x37 blurred patch into a 4-slot queue on patch_latch (when
//     the keypoint is dispatched to orientation, sharing the single 37x37
//     window_buffer across orientation and brief).
//   - Pipelined across 2 stages once orientation completes (start):
//       Stage 1: Latch sin/cos (with max_fanout replication), rotate all 256
//                test pairs, and register patch coordinates.
//       Stage 2: Sample queued patch at registered coordinates and compare pairs.

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

    // 4-slot patch queue (holds 37x37 patches while orientation's 4-stage pipeline completes)
    logic [PIXEL_WIDTH-1:0] frozen_win [0:3][WINDOW_SIZE-1:0][WINDOW_SIZE-1:0];
    logic [1:0]             wr_slot;
    logic [1:0]             rd_slot;

    // Stage 1 registers: latched sin/cos (replicated to limit fanout), slot index, and metadata
    (* max_fanout = 32 *) logic signed [7:0] cos_reg;
    (* max_fanout = 32 *) logic signed [7:0] sin_reg;
    logic                  s1_valid;
    logic [1:0]            s1_slot;
    logic [15:0]           s1_score;
    logic [11:0]           s1_x, s1_y;
    logic [ANGLE_BITS-1:0] s1_ori;

    // Stage 1 combinational rotation & Stage 2 registered coordinates
    logic [5:0] wx1_comb [0:DESCRIPTOR_BITS-1];
    logic [5:0] wy1_comb [0:DESCRIPTOR_BITS-1];
    logic [5:0] wx2_comb [0:DESCRIPTOR_BITS-1];
    logic [5:0] wy2_comb [0:DESCRIPTOR_BITS-1];
    logic [DESCRIPTOR_BITS-1:0] inb1_comb;
    logic [DESCRIPTOR_BITS-1:0] inb2_comb;

    logic                  s2_valid;
    logic [1:0]            s2_slot;
    logic [15:0]           s2_score;
    logic [11:0]           s2_x, s2_y;
    logic [ANGLE_BITS-1:0] s2_ori;
    logic [5:0]            wx1_q [0:DESCRIPTOR_BITS-1];
    logic [5:0]            wy1_q [0:DESCRIPTOR_BITS-1];
    logic [5:0]            wx2_q [0:DESCRIPTOR_BITS-1];
    logic [5:0]            wy2_q [0:DESCRIPTOR_BITS-1];
    logic [DESCRIPTOR_BITS-1:0] inb1_q;
    logic [DESCRIPTOR_BITS-1:0] inb2_q;

    always_comb begin
        for (int i = 0; i < DESCRIPTOR_BITS; i++) begin
            automatic logic signed [15:0] px1_cos, py1_sin, px1_sin, py1_cos;
            automatic logic signed [15:0] px2_cos, py2_sin, px2_sin, py2_cos;
            automatic logic signed [7:0]  rx1, ry1, rx2, ry2;
            automatic logic signed [7:0]  wx1, wy1, wx2, wy2;

            px1_cos = 16'($signed(pattern_coords[i][0])) * 16'($signed(cos_reg));
            py1_sin = 16'($signed(pattern_coords[i][1])) * 16'($signed(sin_reg));
            px1_sin = 16'($signed(pattern_coords[i][0])) * 16'($signed(sin_reg));
            py1_cos = 16'($signed(pattern_coords[i][1])) * 16'($signed(cos_reg));

            px2_cos = 16'($signed(pattern_coords[i][2])) * 16'($signed(cos_reg));
            py2_sin = 16'($signed(pattern_coords[i][3])) * 16'($signed(sin_reg));
            px2_sin = 16'($signed(pattern_coords[i][2])) * 16'($signed(sin_reg));
            py2_cos = 16'($signed(pattern_coords[i][3])) * 16'($signed(cos_reg));

            rx1 = 8'($signed(px1_cos - py1_sin) >>> 7);
            ry1 = 8'($signed(px1_sin + py1_cos) >>> 7);
            rx2 = 8'($signed(px2_cos - py2_sin) >>> 7);
            ry2 = 8'($signed(px2_sin + py2_cos) >>> 7);

            wx1 = 8'(WINDOW_CENTER) - rx1;
            wy1 = 8'(WINDOW_CENTER) - ry1;
            wx2 = 8'(WINDOW_CENTER) - rx2;
            wy2 = 8'(WINDOW_CENTER) - ry2;

            wx1_comb[i]  = wx1[5:0];
            wy1_comb[i]  = wy1[5:0];
            wx2_comb[i]  = wx2[5:0];
            wy2_comb[i]  = wy2[5:0];
            inb1_comb[i] = (wx1 >= 0 && wx1 < 8'(WINDOW_SIZE) && wy1 >= 0 && wy1 < 8'(WINDOW_SIZE));
            inb2_comb[i] = (wx2 >= 0 && wx2 < 8'(WINDOW_SIZE) && wy2 >= 0 && wy2 < 8'(WINDOW_SIZE));
        end
    end

    // Generate per-pair coordinate pipeline registers (avoids Verilator BLKLOOPINIT)
    genvar g;
    generate
        for (g = 0; g < DESCRIPTOR_BITS; g++) begin : gen_coord_regs
            always_ff @(posedge clk) begin
                if (!rst_n) begin
                    wx1_q[g] <= '0;
                    wy1_q[g] <= '0;
                    wx2_q[g] <= '0;
                    wy2_q[g] <= '0;
                end else if (s1_valid) begin
                    wx1_q[g] <= wx1_comb[g];
                    wy1_q[g] <= wy1_comb[g];
                    wx2_q[g] <= wx2_comb[g];
                    wy2_q[g] <= wy2_comb[g];
                end
            end
        end
    endgenerate

    // Stage 2 combinational patch sampling and comparison
    logic [DESCRIPTOR_BITS-1:0] desc_comb;

    always_comb begin
        desc_comb = '0;
        for (int i = 0; i < DESCRIPTOR_BITS; i++) begin
            automatic logic [PIXEL_WIDTH-1:0] p1, p2;
            p1 = inb1_q[i] ? frozen_win[s2_slot][wy1_q[i]][wx1_q[i]] : '0;
            p2 = inb2_q[i] ? frozen_win[s2_slot][wy2_q[i]][wx2_q[i]] : '0;
            // Match OpenCV ORBextractor.cc (t0 < t1) polarity
            desc_comb[i] = (p1 < p2);
        end
    end

    // Patch capture and 2-stage pipeline progression
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_slot            <= '0;
            rd_slot            <= '0;
            s1_valid           <= 1'b0;
            s1_slot            <= '0;
            cos_reg            <= '0;
            sin_reg            <= '0;
            s1_score           <= '0;
            s1_x               <= '0;
            s1_y               <= '0;
            s1_ori             <= '0;
            s2_valid           <= 1'b0;
            s2_slot            <= '0;
            s2_score           <= '0;
            s2_x               <= '0;
            s2_y               <= '0;
            s2_ori             <= '0;
            inb1_q             <= '0;
            inb2_q             <= '0;
            descriptor         <= '0;
            keypoint_score_out <= '0;
            keypoint_x_out     <= '0;
            keypoint_y_out     <= '0;
            orientation_out    <= '0;
            descriptor_valid   <= 1'b0;
        end else begin
            // Enqueue 37x37 patch when keypoint enters orientation pipeline
            if (patch_latch) begin
                frozen_win[wr_slot] <= window_buffer;
                wr_slot             <= wr_slot + 2'd1;
            end

            // Stage 1: capture completed orientation and pop corresponding patch slot
            if (start) begin
                s1_valid <= 1'b1;
                s1_slot  <= rd_slot;
                rd_slot  <= rd_slot + 2'd1;
                cos_reg  <= COS_LUT[orientation];
                sin_reg  <= SIN_LUT[orientation];
                s1_score <= keypoint_score_in;
                s1_x     <= keypoint_x_in;
                s1_y     <= keypoint_y_in;
                s1_ori   <= orientation;
            end else begin
                s1_valid <= 1'b0;
            end

            // Stage 2: register metadata and in-bounds flags
            s2_valid <= s1_valid;
            if (s1_valid) begin
                s2_slot  <= s1_slot;
                s2_score <= s1_score;
                s2_x     <= s1_x;
                s2_y     <= s1_y;
                s2_ori   <= s1_ori;
                inb1_q   <= inb1_comb;
                inb2_q   <= inb2_comb;
            end

            // Output stage: register descriptor from sampled patch
            descriptor_valid <= s2_valid;
            if (s2_valid) begin
                descriptor         <= desc_comb;
                keypoint_score_out <= s2_score;
                keypoint_x_out     <= s2_x;
                keypoint_y_out     <= s2_y;
                orientation_out    <= s2_ori;
            end
        end
    end

    assign busy = (wr_slot != rd_slot) || s1_valid || s2_valid || descriptor_valid;

endmodule
