// 3x3 non-maximum suppression on the FAST score stream. Outputs a keypoint if
// the center score is > 0 and greater than its neighbors (>= against the
// right/down neighbors, to break ties).

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

    // Coordinate of bottom-right window entry win[0][0]
    logic [COORD_WIDTH-1:0] win00_x;
    logic [COORD_WIDTH-1:0] win00_y;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            win00_x <= '0;
            win00_y <= '0;
        end else if (corner_valid_in) begin
            win00_x <= corner_x_in;
            win00_y <= corner_y_in;
        end
    end

    // 3x3 local maximum check (win[2][2] is top-left, win[0][0] is bottom-right)
    logic [SCORE_WIDTH-1:0] center_score;
    logic is_max;

    assign center_score = win[1][1];

    always_comb begin
        is_max = (center_score > 0) && (win00_x >= COORD_WIDTH'(2)) && (win00_y >= COORD_WIDTH'(2));

        // Strict > for top-left, top, top-right, left
        if (center_score <= win[2][2] ||
            center_score <= win[2][1] ||
            center_score <= win[2][0] ||
            center_score <= win[1][2]) begin
            is_max = 1'b0;
        end

        // >= for right, bottom-left, bottom, bottom-right (tie-breaking)
        if (center_score < win[1][0] ||
            center_score < win[0][2] ||
            center_score < win[0][1] ||
            center_score < win[0][0]) begin
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
                keypoint_x     <= win00_x - COORD_WIDTH'(1);
                keypoint_y     <= win00_y - COORD_WIDTH'(1);
            end else begin
                keypoint_score <= '0;
                keypoint_x     <= '0;
                keypoint_y     <= '0;
            end
        end
    end

endmodule
