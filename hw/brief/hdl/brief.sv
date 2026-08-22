//------------------------------------------------------------------------------
// File: brief.sv
// Description: Rotated BRIEF (rBRIEF) 256-bit descriptor generator.
//              Freezes the 37x37 smoothed window upon keypoint trigger, rotates
//              256 test pair coordinates using keypoint orientation with full 16-bit
//              intermediate precision, samples pixels from the frozen patch,
//              and generates the 256-bit binary descriptor.
//              Latches keypoint metadata (x, y, score) and outputs them aligned
//              with descriptor_valid.
//------------------------------------------------------------------------------

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
    input  logic                                  start,

    input  logic [15:0]                           keypoint_score_in,
    input  logic [11:0]                           keypoint_x_in,
    input  logic [11:0]                           keypoint_y_in,
    input  logic [ANGLE_BITS-1:0]                 orientation,
    input  logic [PIXEL_WIDTH-1:0]                window_buffer [WINDOW_SIZE-1:0][WINDOW_SIZE-1:0],
    input  logic                                  window_valid,

    // 256 pairs of (x1, y1, x2, y2) in 27x27 patch range (-13 to +13)
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

    // State machine
    typedef enum logic [1:0] {
        IDLE,
        GENERATE,
        DONE
    } state_t;

    state_t state;
    logic [7:0] pair_idx;
    logic [ANGLE_BITS-1:0] active_orientation;
    logic [DESCRIPTOR_BITS-1:0] desc_accum;

    // Latched patch buffer
    logic [PIXEL_WIDTH-1:0] frozen_win [WINDOW_SIZE-1:0][WINDOW_SIZE-1:0];

    // Latched metadata
    logic [15:0] latched_score;
    logic [11:0] latched_x, latched_y;

    // Active sin/cos
    logic signed [7:0] cos_val, sin_val;
    assign cos_val = COS_LUT[active_orientation];
    assign sin_val = SIN_LUT[active_orientation];

    // Current pair coordinates
    logic signed [COORD_BITS-1:0] x1, y1, x2, y2;
    assign x1 = pattern_coords[pair_idx][0];
    assign y1 = pattern_coords[pair_idx][1];
    assign x2 = pattern_coords[pair_idx][2];
    assign y2 = pattern_coords[pair_idx][3];

    // Rotate coordinates with 16-bit intermediate multiplication
    logic signed [15:0] prod_x1_cos, prod_y1_sin, prod_x1_sin, prod_y1_cos;
    logic signed [15:0] prod_x2_cos, prod_y2_sin, prod_x2_sin, prod_y2_cos;
    logic signed [7:0]  rot_x1, rot_y1, rot_x2, rot_y2;
    logic signed [7:0]  win_x1, win_y1, win_x2, win_y2;

    always_comb begin
        prod_x1_cos = 16'($signed(x1)) * 16'($signed(cos_val));
        prod_y1_sin = 16'($signed(y1)) * 16'($signed(sin_val));
        prod_x1_sin = 16'($signed(x1)) * 16'($signed(sin_val));
        prod_y1_cos = 16'($signed(y1)) * 16'($signed(cos_val));

        prod_x2_cos = 16'($signed(x2)) * 16'($signed(cos_val));
        prod_y2_sin = 16'($signed(y2)) * 16'($signed(sin_val));
        prod_x2_sin = 16'($signed(x2)) * 16'($signed(sin_val));
        prod_y2_cos = 16'($signed(y2)) * 16'($signed(cos_val));

        rot_x1 = 8'((prod_x1_cos - prod_y1_sin) >>> 7);
        rot_y1 = 8'((prod_x1_sin + prod_y1_cos) >>> 7);
        rot_x2 = 8'((prod_x2_cos - prod_y2_sin) >>> 7);
        rot_y2 = 8'((prod_x2_sin + prod_y2_cos) >>> 7);

        win_x1 = rot_x1 + 8'(WINDOW_CENTER);
        win_y1 = rot_y1 + 8'(WINDOW_CENTER);
        win_x2 = rot_x2 + 8'(WINDOW_CENTER);
        win_y2 = rot_y2 + 8'(WINDOW_CENTER);
    end

    // Pixel sampling from frozen window and comparison
    logic [PIXEL_WIDTH-1:0] p1, p2;
    logic bit_val;

    always_comb begin
        p1 = (win_x1 >= 0 && win_x1 < WINDOW_SIZE && win_y1 >= 0 && win_y1 < WINDOW_SIZE) ?
             frozen_win[win_y1[5:0]][win_x1[5:0]] : '0;
        p2 = (win_x2 >= 0 && win_x2 < WINDOW_SIZE && win_y2 >= 0 && win_y2 < WINDOW_SIZE) ?
             frozen_win[win_y2[5:0]][win_x2[5:0]] : '0;

        bit_val = (p1 > p2);
    end

    // Control FSM
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state              <= IDLE;
            pair_idx           <= '0;
            active_orientation <= '0;
            desc_accum         <= '0;
            descriptor         <= '0;
            keypoint_score_out <= '0;
            keypoint_x_out     <= '0;
            keypoint_y_out     <= '0;
            orientation_out    <= '0;
            latched_score      <= '0;
            latched_x          <= '0;
            latched_y          <= '0;
            descriptor_valid   <= 1'b0;
            busy               <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    descriptor_valid <= 1'b0;
                    if (start && window_valid) begin
                        state              <= GENERATE;
                        active_orientation <= orientation;
                        latched_score      <= keypoint_score_in;
                        latched_x          <= keypoint_x_in;
                        latched_y          <= keypoint_y_in;
                        frozen_win         <= window_buffer; // Freeze 37x37 patch
                        pair_idx           <= '0;
                        desc_accum         <= '0;
                        busy               <= 1'b1;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                GENERATE: begin
                    desc_accum[pair_idx] <= bit_val;
                    if (pair_idx == 8'd255) begin
                        state <= DONE;
                    end else begin
                        pair_idx <= pair_idx + 1;
                    end
                end

                DONE: begin
                    descriptor         <= desc_accum;
                    keypoint_score_out <= latched_score;
                    keypoint_x_out     <= latched_x;
                    keypoint_y_out     <= latched_y;
                    orientation_out    <= active_orientation;
                    descriptor_valid   <= 1'b1;
                    busy               <= 1'b0;
                    state              <= IDLE;
                end
            endcase
        end
    end

endmodule
