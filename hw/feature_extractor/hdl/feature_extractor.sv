//------------------------------------------------------------------------------
// File: feature_extractor.sv
// Description: Complete single-scale ORB feature extraction pipeline.
//              Encapsulates:
//                1. FAST-9 Corner Detector
//                2. 3x3 Non-Maximum Suppression (NMS)
//                3. Keypoint FIFO (buffering x, y, score)
//                4. 7x7 Binomial Gaussian Blur
//                5. 37x37 Window Buffer on blurred stream
//                6. Streaming Intensity Centroid Orientation (64 sectors)
//                7. 256-bit Rotated BRIEF (rBRIEF) Descriptor Generator
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module feature_extractor #(
    parameter int PIXEL_WIDTH     = 6,
    parameter int IMG_WIDTH       = 640,
    parameter int IMG_HEIGHT      = 480,
    parameter int DESCRIPTOR_BITS = 256,
    parameter int FAST_THRESHOLD  = 20,
    parameter int FIFO_DEPTH      = 64,
    parameter int ANGLE_BITS      = 6,
    parameter int COORD_BITS      = 6
) (
    input  logic                                  clk,
    input  logic                                  rst_n,

    // Streaming pixel input from image pyramid
    input  logic                                  pixel_valid,
    input  logic [PIXEL_WIDTH-1:0]                pixel_in,

    // Pre-computed 256 BRIEF test coordinate pairs
    input  logic signed [COORD_BITS-1:0]          brief_pattern [0:DESCRIPTOR_BITS-1][0:3],

    // Output extracted feature stream
    output logic                                  feature_valid,
    output logic [DESCRIPTOR_BITS-1:0]            feature_descriptor,
    output logic [15:0]                           feature_score,
    output logic [11:0]                           feature_x,
    output logic [11:0]                           feature_y,
    output logic [ANGLE_BITS-1:0]                 feature_orientation,

    // Pipeline status
    output logic                                  busy,
    output logic                                  fifo_empty
);

    // 1. FAST Detector
    logic        fast_valid;
    logic [15:0] fast_score;
    logic [11:0] fast_x, fast_y;

    fast #(
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .THRESHOLD(FAST_THRESHOLD),
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT)
    ) u_fast (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_valid(pixel_valid),
        .pixel_in(pixel_in),
        .corner_valid(fast_valid),
        .corner_score(fast_score),
        .corner_x(fast_x),
        .corner_y(fast_y)
    );

    // 2. Non-Maximum Suppression (3x3)
    logic        nms_valid;
    logic [15:0] nms_score;
    logic [11:0] nms_x, nms_y;

    nms #(
        .SCORE_WIDTH(16),
        .COORD_WIDTH(12),
        .IMG_WIDTH(IMG_WIDTH)
    ) u_nms (
        .clk(clk),
        .rst_n(rst_n),
        .corner_valid_in(fast_valid),
        .corner_score_in(fast_score),
        .corner_x_in(fast_x),
        .corner_y_in(fast_y),
        .keypoint_valid(nms_valid),
        .keypoint_score(nms_score),
        .keypoint_x(nms_x),
        .keypoint_y(nms_y)
    );

    // 3. Gaussian Blur (7x7 Binomial Filter)
    logic                   blur_valid;
    logic [PIXEL_WIDTH-1:0] blur_pixel;

    gaussian_blur #(
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .KERNEL_SIZE(7),
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT)
    ) u_blur (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_valid(pixel_valid),
        .pixel_in(pixel_in),
        .blur_valid(blur_valid),
        .pixel_out(blur_pixel)
    );

    // 4. 37x37 Window Buffer for BRIEF Sampling
    logic [PIXEL_WIDTH-1:0] blur_win [36:0][36:0];
    logic                   blur_win_valid;

    window_buffer #(
        .DATA_WIDTH(PIXEL_WIDTH),
        .LINE_WIDTH(IMG_WIDTH),
        .WINDOW_SIZE(37)
    ) u_brief_win_buf (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(blur_valid),
        .in_data(blur_pixel),
        .out_valid(blur_win_valid),
        .win(blur_win)
    );

    // 5. Keypoint FIFO: Buffers (score, x, y) from NMS
    logic        kp_fifo_full, kp_fifo_empty;
    logic [39:0] kp_fifo_din, kp_fifo_dout;
    logic        kp_fifo_rd_en;
    logic [$clog2(FIFO_DEPTH):0] kp_fifo_count;

    assign kp_fifo_din = {nms_score, nms_y, nms_x};
    assign fifo_empty  = kp_fifo_empty;

    keypoint_fifo #(
        .DATA_WIDTH(40),
        .DEPTH(FIFO_DEPTH)
    ) u_keypoint_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .wr_en(nms_valid),
        .din(kp_fifo_din),
        .full(kp_fifo_full),
        .almost_full(),
        .overflow(),
        .rd_en(kp_fifo_rd_en),
        .dout(kp_fifo_dout),
        .empty(kp_fifo_empty),
        .almost_empty(),
        .underflow(),
        .count(kp_fifo_count)
    );

    // 6. Keypoint Dispatcher Interlock
    logic        ori_busy;
    logic        brief_busy;
    logic        ori_valid;
    logic [ANGLE_BITS-1:0] ori_angle;
    logic [15:0]           ori_score;
    logic [11:0]           ori_x, ori_y;

    logic        disp_valid;
    logic [15:0] disp_score;
    logic [11:0] disp_x, disp_y;
    logic        disp_pending;

    always_comb begin
        kp_fifo_rd_en = !kp_fifo_empty && !brief_busy && !ori_busy && !disp_pending && !disp_valid && !ori_valid;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            disp_valid   <= 1'b0;
            disp_score   <= '0;
            disp_x       <= '0;
            disp_y       <= '0;
            disp_pending <= 1'b0;
        end else begin
            disp_valid <= kp_fifo_rd_en;
            if (kp_fifo_rd_en) begin
                disp_score   <= kp_fifo_dout[39:24];
                disp_y       <= kp_fifo_dout[23:12];
                disp_x       <= kp_fifo_dout[11:0];
                disp_pending <= 1'b1;
            end else if (feature_valid || (!brief_busy && !disp_valid && !ori_valid)) begin
                disp_pending <= 1'b0;
            end
        end
    end

    // 7. Orientation Calculator
    orientation #(
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .WINDOW_SIZE(37),
        .IMG_WIDTH(IMG_WIDTH),
        .ANGLE_BITS(ANGLE_BITS)
    ) u_orientation (
        .clk(clk),
        .rst_n(rst_n),
        .keypoint_valid(disp_valid),
        .keypoint_score(disp_score),
        .keypoint_x(disp_x),
        .keypoint_y(disp_y),
        .pixel_valid(blur_valid),
        .pixel_in(blur_pixel),
        .orientation_out(ori_angle),
        .keypoint_score_out(ori_score),
        .keypoint_x_out(ori_x),
        .keypoint_y_out(ori_y),
        .orientation_valid(ori_valid),
        .busy(ori_busy)
    );

    // 8. rBRIEF Descriptor Generator
    brief #(
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .WINDOW_SIZE(37),
        .DESCRIPTOR_BITS(DESCRIPTOR_BITS),
        .ANGLE_BITS(ANGLE_BITS),
        .COORD_BITS(COORD_BITS)
    ) u_brief (
        .clk(clk),
        .rst_n(rst_n),
        .start(ori_valid),
        .keypoint_score_in(ori_score),
        .keypoint_x_in(ori_x),
        .keypoint_y_in(ori_y),
        .orientation(ori_angle),
        .window_buffer(blur_win),
        .window_valid(blur_win_valid),
        .pattern_coords(brief_pattern),
        .descriptor(feature_descriptor),
        .keypoint_score_out(feature_score),
        .keypoint_x_out(feature_x),
        .keypoint_y_out(feature_y),
        .orientation_out(feature_orientation),
        .descriptor_valid(feature_valid),
        .busy(brief_busy)
    );

    assign busy = brief_busy || ori_busy || !kp_fifo_empty;

endmodule
