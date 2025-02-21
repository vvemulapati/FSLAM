/*
 * Gaussian Blur Module
 * 
 * Implements a 7x7 binomial Gaussian filter for image smoothing.
 * Used before orientation calculation to reduce noise.
 * Uses hardware-friendly binomial coefficients to avoid DSP usage.
 */

module gaussian_blur #(
    parameter PIXEL_WIDTH = 6,
    parameter KERNEL_SIZE = 7,
    parameter IMG_WIDTH = 640
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    pixel_valid,
    input  logic [PIXEL_WIDTH-1:0] pixel_in,
    
    // Output
    output logic                    blur_valid,
    output logic [PIXEL_WIDTH-1:0] pixel_out
);

    // Line buffers for 7 rows
    logic [PIXEL_WIDTH-1:0] line_buffers [KERNEL_SIZE-1:0][IMG_WIDTH-1:0];
    logic [PIXEL_WIDTH-1:0] window [KERNEL_SIZE-1:0][KERNEL_SIZE-1:0];
    
    // Write pointers for line buffers
    logic [11:0] write_ptr;
    logic [11:0] pixel_x, pixel_y;
    
    // Binomial kernel coefficients (7x7)
    // Approximates Gaussian with σ=2 using binomial expansion
    localparam logic [3:0] KERNEL [KERNEL_SIZE-1:0][KERNEL_SIZE-1:0] = '{
        '{1, 6, 15, 20, 15, 6, 1},
        '{6, 36, 90, 120, 90, 36, 6},
        '{15, 90, 225, 300, 225, 90, 15},
        '{20, 120, 300, 400, 300, 120, 20},
        '{15, 90, 225, 300, 225, 90, 15},
        '{6, 36, 90, 120, 90, 36, 6},
        '{1, 6, 15, 20, 15, 6, 1}
    };
    
    // Sum of all kernel coefficients for normalization
    localparam KERNEL_SUM = 4096; // 64^2
    
    // Convolution result
    logic [PIXEL_WIDTH+15:0] conv_sum; // Wide enough for accumulated result
    logic output_valid;
    
    // Pixel coordinate tracking
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_x <= '0;
            pixel_y <= '0;
            write_ptr <= '0;
        end else if (pixel_valid) begin
            if (pixel_x == IMG_WIDTH - 1) begin
                pixel_x <= '0;
                pixel_y <= pixel_y + 1;
                write_ptr <= '0;
            end else begin
                pixel_x <= pixel_x + 1;
                write_ptr <= write_ptr + 1;
            end
        end
    end
    
    // Line buffer management
    always_ff @(posedge clk) begin
        if (pixel_valid) begin
            // Shift line buffers
            for (int i = KERNEL_SIZE-1; i > 0; i--) begin
                line_buffers[i][write_ptr] <= line_buffers[i-1][write_ptr];
            end
            line_buffers[0][write_ptr] <= pixel_in;
        end
    end
    
    // Extract 7x7 window from line buffers
    always_comb begin
        for (int i = 0; i < KERNEL_SIZE; i++) begin
            for (int j = 0; j < KERNEL_SIZE; j++) begin
                logic [11:0] read_addr = (write_ptr >= j) ? write_ptr - j : IMG_WIDTH + write_ptr - j;
                window[i][j] = line_buffers[i][read_addr];
            end
        end
    end
    
    // Convolution computation
    always_comb begin
        conv_sum = '0;
        for (int i = 0; i < KERNEL_SIZE; i++) begin
            for (int j = 0; j < KERNEL_SIZE; j++) begin
                conv_sum += window[i][j] * KERNEL[i][j];
            end
        end
    end
    
    // Valid output region (exclude borders)
    always_comb begin
        output_valid = pixel_valid && 
                      (pixel_x >= KERNEL_SIZE/2) && 
                      (pixel_x < IMG_WIDTH - KERNEL_SIZE/2) &&
                      (pixel_y >= KERNEL_SIZE/2);
    end
    
    // Output stage
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blur_valid <= 1'b0;
            pixel_out <= '0;
        end else begin
            blur_valid <= output_valid;
            if (output_valid) begin
                // Normalize by dividing by kernel sum
                pixel_out <= conv_sum / KERNEL_SUM;
            end
        end
    end

endmodule