// FAST-9/16 corner detector on a 7x7 window. A pixel is a corner if 9
// contiguous pixels on the radius-3 circle are all brighter than p + THRESHOLD
// or all darker than p - THRESHOLD. The score is the sum of absolute differences.

`timescale 1ns / 1ps

module fast #(
    parameter int PIXEL_WIDTH = 6,
    parameter int THRESHOLD   = 5,
    parameter int IMG_WIDTH   = 640,
    parameter int IMG_HEIGHT  = 480
) (
    input  logic                   clk,
    input  logic                   rst_n,

    input  logic                   pixel_valid,
    input  logic [PIXEL_WIDTH-1:0] pixel_in,

    output logic                   corner_valid,
    output logic [15:0]            corner_score,
    output logic [11:0]            corner_x,
    output logic [11:0]            corner_y
);

    localparam int N_CONTIG = 9;
    localparam int N_CIRCLE = 16;

    // 7x7 sliding window buffer
    logic [PIXEL_WIDTH-1:0] win [6:0][6:0];
    logic                   win_valid;

    window_buffer #(
        .DATA_WIDTH(PIXEL_WIDTH),
        .LINE_WIDTH(IMG_WIDTH),
        .WINDOW_SIZE(7)
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

    // Bresenham circle extraction (Radius 3 around center [3][3])
    logic [PIXEL_WIDTH-1:0] center_pixel;
    logic [PIXEL_WIDTH-1:0] circle [0:15];

    assign center_pixel = win[3][3];

    always_comb begin
        circle[0]  = win[0][3]; // ( 0, -3)
        circle[1]  = win[0][4]; // ( 1, -3)
        circle[2]  = win[1][5]; // ( 2, -2)
        circle[3]  = win[2][6]; // ( 3, -1)
        circle[4]  = win[3][6]; // ( 3,  0)
        circle[5]  = win[4][6]; // ( 3,  1)
        circle[6]  = win[5][5]; // ( 2,  2)
        circle[7]  = win[6][4]; // ( 1,  3)
        circle[8]  = win[6][3]; // ( 0,  3)
        circle[9]  = win[6][2]; // (-1,  3)
        circle[10] = win[5][1]; // (-2,  2)
        circle[11] = win[4][0]; // (-3,  1)
        circle[12] = win[3][0]; // (-3,  0)
        circle[13] = win[2][0]; // (-3, -1)
        circle[14] = win[1][1]; // (-2, -2)
        circle[15] = win[0][2]; // (-1, -3)
    end

    // Brighter / Darker comparison
    logic [15:0]          brighter, darker;
    logic [31:0]          brighter_2x, darker_2x;
    logic [PIXEL_WIDTH:0] diff [0:15];
    logic [PIXEL_WIDTH+1:0] pair_sum_comb [0:7];

    always_comb begin
        for (int i = 0; i < N_CIRCLE; i++) begin
            brighter[i] = ({2'b0, circle[i]} > ({2'b0, center_pixel} + 8'(THRESHOLD)));
            darker[i]   = ({2'b0, center_pixel} > ({2'b0, circle[i]} + 8'(THRESHOLD)));
            diff[i]     = (circle[i] > center_pixel) ? {1'b0, circle[i] - center_pixel}
                                                     : {1'b0, center_pixel - circle[i]};
        end
        for (int k = 0; k < 8; k++) begin
            pair_sum_comb[k] = {1'b0, diff[2*k]} + {1'b0, diff[2*k + 1]};
        end
    end

    assign brighter_2x = {brighter, brighter};
    assign darker_2x   = {darker, darker};

    // Contiguous 9-pixel test
    logic is_corner;
    always_comb begin
        is_corner = 1'b0;
        for (int i = 0; i < 16; i++) begin
            if ((&brighter_2x[i +: N_CONTIG]) || (&darker_2x[i +: N_CONTIG])) begin
                is_corner = 1'b1;
            end
        end
    end

    // Emit one score per pixel once the window center reaches (0, 0), zeroing
    // the 3-pixel border so the downstream NMS line buffers stay aligned.
    logic center_valid;
    logic in_interior;

    assign center_valid = pixel_valid_d1 && ((win_y > 12'd3) || (win_y == 12'd3 && win_x >= 12'd3));
    assign in_interior  = win_valid && (win_x >= 12'd6) && (win_y >= 12'd6);

    // Pipeline stage 1
    logic                   s1_valid;
    logic                   s1_use_score;
    logic [11:0]            s1_x, s1_y;
    logic [PIXEL_WIDTH+1:0] s1_pair_sum [0:7];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s1_valid     <= 1'b0;
            s1_use_score <= 1'b0;
            s1_x         <= '0;
            s1_y         <= '0;
            for (int k = 0; k < 8; k++) begin
                s1_pair_sum[k] <= '0;
            end
        end else begin
            s1_valid     <= center_valid;
            s1_use_score <= center_valid && in_interior && is_corner;
            if (center_valid) begin
                s1_x <= (win_x >= 12'd3) ? (win_x - 12'd3) : (win_x + 12'(IMG_WIDTH - 3));
                s1_y <= (win_x >= 12'd3) ? (win_y - 12'd3) : (win_y - 12'd4);
                for (int k = 0; k < 8; k++) begin
                    s1_pair_sum[k] <= pair_sum_comb[k];
                end
            end
        end
    end

    // Score sum
    logic [15:0] score_accum_s2;
    always_comb begin
        score_accum_s2 = '0;
        for (int k = 0; k < 8; k++) begin
            score_accum_s2 += 16'(s1_pair_sum[k]);
        end
    end

    // Register outputs
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            corner_valid <= 1'b0;
            corner_score <= '0;
            corner_x     <= '0;
            corner_y     <= '0;
        end else begin
            corner_valid <= s1_valid;
            corner_score <= s1_use_score ? score_accum_s2 : 16'd0;
            if (s1_valid) begin
                corner_x <= s1_x;
                corner_y <= s1_y;
            end
        end
    end

endmodule
