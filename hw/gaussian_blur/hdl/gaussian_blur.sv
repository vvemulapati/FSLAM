//------------------------------------------------------------------------------
// File: gaussian_blur.sv
// Description: 7x7 Binomial Gaussian filter for image smoothing.
//              Kernel approximates Gaussian σ=2 using binomial expansion.
//              Sum of coefficients = 4096 = 2^12 (normalized by right shift >> 12).
//------------------------------------------------------------------------------

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

    // Convolution accumulator
    logic [PIXEL_WIDTH+12:0] conv_sum;

    always_comb begin
        conv_sum = '0;
        for (int r = 0; r < 7; r++) begin
            for (int c = 0; c < 7; c++) begin
                conv_sum += win[r][c] * KERNEL[r][c];
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            blur_valid <= 1'b0;
            pixel_out  <= '0;
        end else begin
            blur_valid <= win_valid;
            if (win_valid) begin
                // Divide by 4096 (with rounding: + 2048)
                pixel_out <= (conv_sum + 2048) >> 12;
            end
        end
    end

endmodule
