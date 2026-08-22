//------------------------------------------------------------------------------
// File: fast.sv
// Description: FAST-9/16 corner detector.
//              Examines 16 pixels on a Bresenham circle of radius 3 around pixel p.
//              Declares corner if 9 contiguous circle pixels are all brighter
//              than p + THRESHOLD or all darker than p - THRESHOLD.
//              Calculates FAST score as sum of absolute differences.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module fast #(
    parameter int PIXEL_WIDTH = 6,
    parameter int THRESHOLD   = 20,
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

    // Raster coordinate tracking (accounting for 3-pixel delay to window center)
    logic [11:0] cur_x, cur_y;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cur_x <= '0;
            cur_y <= '0;
        end else if (pixel_valid) begin
            if (cur_x == IMG_WIDTH - 1) begin
                cur_x <= '0;
                cur_y <= (cur_y == IMG_HEIGHT - 1) ? '0 : cur_y + 1;
            end else begin
                cur_x <= cur_x + 1;
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
    logic [15:0] brighter, darker;
    logic [31:0] brighter_2x, darker_2x;
    logic [15:0] diff [0:15];
    logic [15:0] score_accum;

    always_comb begin
        score_accum = '0;
        for (int i = 0; i < N_CIRCLE; i++) begin
            brighter[i] = (circle[i] > (center_pixel + THRESHOLD));
            darker[i]   = (center_pixel > (circle[i] + THRESHOLD));

            diff[i]     = (circle[i] > center_pixel) ? (circle[i] - center_pixel) : (center_pixel - circle[i]);
            score_accum += diff[i];
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

    // Valid interior region (margin of 3 pixels)
    logic in_interior;
    assign in_interior = (cur_x >= 3) && (cur_x < IMG_WIDTH - 3) &&
                         (cur_y >= 3) && (cur_y < IMG_HEIGHT - 3);

    // Register outputs
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            corner_valid <= 1'b0;
            corner_score <= '0;
            corner_x     <= '0;
            corner_y     <= '0;
        end else begin
            corner_valid <= win_valid && in_interior;
            corner_score <= (win_valid && in_interior && is_corner) ? score_accum : 16'd0;
            corner_x     <= (cur_x >= 3) ? (cur_x - 3) : '0;
            corner_y     <= (cur_y >= 3) ? (cur_y - 3) : '0;
        end
    end

endmodule
