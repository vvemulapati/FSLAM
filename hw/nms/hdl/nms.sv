//------------------------------------------------------------------------------
// File: nms.sv
// Description: 3x3 Non-Maximum Suppression (NMS) module.
//              Continuous raster-scan streaming over a 3x3 sliding window.
//              Outputs keypoint if center score > 0 and greater than all neighbors
//              (with >= for right/down neighbors to resolve ties).
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module nms #(
    parameter int SCORE_WIDTH = 16,
    parameter int COORD_WIDTH = 12,
    parameter int IMG_WIDTH   = 640
) (
    input  logic                   clk,
    input  logic                   rst_n,

    input  logic                   corner_valid_in,
    input  logic [SCORE_WIDTH-1:0] corner_score_in,
    input  logic [COORD_WIDTH-1:0] corner_x_in,
    input  logic [COORD_WIDTH-1:0] corner_y_in,

    output logic                   keypoint_valid,
    output logic [SCORE_WIDTH-1:0] keypoint_score,
    output logic [COORD_WIDTH-1:0] keypoint_x,
    output logic [COORD_WIDTH-1:0] keypoint_y
);

    // 3x3 sliding window on corner score stream
    logic [SCORE_WIDTH-1:0] win [2:0][2:0];
    logic                   win_valid;

    window_buffer #(
        .DATA_WIDTH(SCORE_WIDTH),
        .LINE_WIDTH(IMG_WIDTH),
        .WINDOW_SIZE(3)
    ) u_win_buf (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(corner_valid_in),
        .in_data(corner_score_in),
        .out_valid(win_valid),
        .win(win)
    );

    // Delay coordinates to align with 3x3 window center (1-row + 1-col delay)
    logic [COORD_WIDTH-1:0] x_pipe [0:2];
    logic [COORD_WIDTH-1:0] y_pipe [0:2];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) begin
                x_pipe[i] <= '0;
                y_pipe[i] <= '0;
            end
        end else if (corner_valid_in) begin
            x_pipe[0] <= corner_x_in;
            y_pipe[0] <= corner_y_in;
            for (int i = 1; i < 3; i++) begin
                x_pipe[i] <= x_pipe[i-1];
                y_pipe[i] <= y_pipe[i-1];
            end
        end
    end

    // 3x3 local maximum check
    logic [SCORE_WIDTH-1:0] center_score;
    logic is_max;

    assign center_score = win[1][1];

    always_comb begin
        is_max = (center_score > 0);

        // Strict > for top, top-left, left, bottom-left
        if (center_score <= win[0][0] ||
            center_score <= win[0][1] ||
            center_score <= win[0][2] ||
            center_score <= win[1][0]) begin
            is_max = 1'b0;
        end

        // >= for right, bottom, bottom-right (tie-breaking)
        if (center_score < win[1][2] ||
            center_score < win[2][0] ||
            center_score < win[2][1] ||
            center_score < win[2][2]) begin
            is_max = 1'b0;
        end
    end

    // Output stage
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            keypoint_valid <= 1'b0;
            keypoint_score <= '0;
            keypoint_x     <= '0;
            keypoint_y     <= '0;
        end else begin
            keypoint_valid <= win_valid && is_max;
            if (win_valid && is_max) begin
                keypoint_score <= center_score;
                keypoint_x     <= (x_pipe[1] >= 1) ? (x_pipe[1] - 1) : '0;
                keypoint_y     <= (y_pipe[1] >= 1) ? (y_pipe[1] - 1) : '0;
            end else begin
                keypoint_score <= '0;
                keypoint_x     <= '0;
                keypoint_y     <= '0;
            end
        end
    end

endmodule
