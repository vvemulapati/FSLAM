// 7x7 binomial blur: outer product of [1 6 15 20 15 6 1], so sigma ~= 1.22
// (ORB-SLAM2 uses a 7x7 Gaussian with sigma = 2). The coefficients sum to
// 4096, so the result is rounded and shifted right by 12.

`timescale 1ns / 1ps

module gaussian_blur #(
    parameter int PIXEL_WIDTH = 6,
    parameter int KERNEL_SIZE = 7,
    parameter int IMG_WIDTH   = 640,
    parameter int IMG_HEIGHT  = 480
) (
    input  logic                   clk,
    input  logic                   rst_n,

    input  logic                   pixel_valid,
    input  logic [PIXEL_WIDTH-1:0] pixel_in,

    output logic                   blur_valid,
    output logic [PIXEL_WIDTH-1:0] pixel_out
);

    logic [PIXEL_WIDTH-1:0] win [6:0][6:0];
    logic                   win_valid;

    window_buffer #(
        .DATA_WIDTH(PIXEL_WIDTH),
        .LINE_WIDTH(IMG_WIDTH),
        .WINDOW_SIZE(KERNEL_SIZE)
    ) u_win_buf (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(pixel_valid),
        .in_data(pixel_in),
        .out_valid(win_valid),
        .win(win)
    );

    // Raster coordinate tracking aligned with the 7x7 window output
    logic [11:0] cur_x, cur_y;
    logic [11:0] win_x, win_y;
    logic        pixel_valid_d1;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cur_x          <= '0;
            cur_y          <= '0;
            win_x          <= '0;
            win_y          <= '0;
            pixel_valid_d1 <= 1'b0;
        end else begin
            pixel_valid_d1 <= pixel_valid;
            if (pixel_valid) begin
                win_x <= cur_x;
                win_y <= cur_y;
                if (cur_x == 12'(IMG_WIDTH - 1)) begin
                    cur_x <= '0;
                    cur_y <= (cur_y == 12'(IMG_HEIGHT - 1)) ? '0 : cur_y + 12'd1;
                end else begin
                    cur_x <= cur_x + 12'd1;
                end
            end
        end
    end

    logic center_valid;
    logic in_interior;

    assign center_valid = pixel_valid_d1 && ((win_y > 12'd3) || (win_y == 12'd3 && win_x >= 12'd3));
    assign in_interior  = win_valid && (win_x >= 12'd6) && (win_y >= 12'd6);

    // Horizontal pass ([1 6 15 20 15 6 1])
    localparam int ROW_SUM_W = PIXEL_WIDTH + 6;
    logic [ROW_SUM_W-1:0] row_sum_comb [0:6];

    always_comb begin
        for (int r = 0; r < 7; r++) begin
            automatic logic [ROW_SUM_W-1:0] s06, s15, s24, s3;
            s06 = ROW_SUM_W'(win[r][0]) + ROW_SUM_W'(win[r][6]);
            s15 = ROW_SUM_W'(win[r][1]) + ROW_SUM_W'(win[r][5]);
            s24 = ROW_SUM_W'(win[r][2]) + ROW_SUM_W'(win[r][4]);
            s3  = ROW_SUM_W'(win[r][3]);

            // 1*s06 + 6*s15 + 15*s24 + 20*s3
            row_sum_comb[r] = s06
                            + (s15 << 2) + (s15 << 1)
                            + (s24 << 4) - s24
                            + (s3  << 4) + (s3  << 2);
        end
    end

    logic                 s1_valid;
    logic                 s1_in_interior;
    logic [ROW_SUM_W-1:0] s1_row_sum [0:6];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s1_valid       <= 1'b0;
            s1_in_interior <= 1'b0;
            for (int r = 0; r < 7; r++) begin
                s1_row_sum[r] <= '0;
            end
        end else begin
            s1_valid       <= center_valid;
            s1_in_interior <= in_interior;
            if (center_valid) begin
                for (int r = 0; r < 7; r++) begin
                    s1_row_sum[r] <= row_sum_comb[r];
                end
            end
        end
    end

    // Vertical pass
    localparam int CONV_W = PIXEL_WIDTH + 13;
    logic [CONV_W-1:0] conv_sum;

    always_comb begin
        automatic logic [CONV_W-1:0] v06, v15, v24, v3;
        v06 = CONV_W'(s1_row_sum[0]) + CONV_W'(s1_row_sum[6]);
        v15 = CONV_W'(s1_row_sum[1]) + CONV_W'(s1_row_sum[5]);
        v24 = CONV_W'(s1_row_sum[2]) + CONV_W'(s1_row_sum[4]);
        v3  = CONV_W'(s1_row_sum[3]);

        conv_sum = v06
                 + (v15 << 2) + (v15 << 1)
                 + (v24 << 4) - v24
                 + (v3  << 4) + (v3  << 2);
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            blur_valid <= 1'b0;
            pixel_out  <= '0;
        end else begin
            blur_valid <= s1_valid;
            if (s1_valid) begin
                // Divide by 4096 (with rounding: + 2048); zero the 3-pixel border
                pixel_out <= s1_in_interior ? PIXEL_WIDTH'((conv_sum + CONV_W'(2048)) >> 12) : '0;
            end
        end
    end

endmodule
