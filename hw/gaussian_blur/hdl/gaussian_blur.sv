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

    // 7x7 Binomial coefficients (max value 400 requires at least 9 bits)
    localparam logic [8:0] KERNEL [0:6][0:6] = '{
        '{9'd1,  9'd6,  9'd15,  9'd20,  9'd15,  9'd6,  9'd1},
        '{9'd6,  9'd36, 9'd90,  9'd120, 9'd90,  9'd36, 9'd6},
        '{9'd15, 9'd90, 9'd225, 9'd300, 9'd225, 9'd90, 9'd15},
        '{9'd20, 9'd120,9'd300, 9'd400, 9'd300, 9'd120,9'd20},
        '{9'd15, 9'd90, 9'd225, 9'd300, 9'd225, 9'd90, 9'd15},
        '{9'd6,  9'd36, 9'd90,  9'd120, 9'd90,  9'd36, 9'd6},
        '{9'd1,  9'd6,  9'd15,  9'd20,  9'd15,  9'd6,  9'd1}
    };

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

    // Convolution accumulator
    logic [PIXEL_WIDTH+12:0] conv_sum;

    always_comb begin
        conv_sum = '0;
        for (int r = 0; r < 7; r++) begin
            for (int c = 0; c < 7; c++) begin
                conv_sum += (PIXEL_WIDTH+13)'(win[r][c] * KERNEL[r][c]);
            end
        end
    end

    logic center_valid;
    logic in_interior;

    assign center_valid = pixel_valid_d1 && ((win_y > 12'd3) || (win_y == 12'd3 && win_x >= 12'd3));
    assign in_interior  = win_valid && (win_x >= 12'd6) && (win_y >= 12'd6);

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            blur_valid <= 1'b0;
            pixel_out  <= '0;
        end else begin
            blur_valid <= center_valid;
            if (center_valid) begin
                // Divide by 4096 (with rounding: + 2048); zero the 3-pixel border
                pixel_out <= in_interior ? PIXEL_WIDTH'((conv_sum + 19'd2048) >> 12) : '0;
            end
        end
    end

endmodule
