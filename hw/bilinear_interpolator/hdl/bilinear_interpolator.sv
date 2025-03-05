/*
 * Bilinear Interpolator Module
 * 
 * Performs bilinear interpolation for image scaling operations.
 * Used in image pyramid generation with scale factor 5/6 (1.2x downsampling).
 * 
 * The interpolation uses 4 neighboring pixels with weights based on the
 * resampled pixel location. There are 25 possible weight combinations
 * based on (x mod 6, y mod 6).
 */

module bilinear_interpolator #(
    parameter PIXEL_WIDTH = 8,
    parameter COORD_WIDTH = 12
) (
    input  logic                     clk,
    input  logic                     rst_n,
    input  logic                     valid_in,
    
    // Input pixels (2x2 neighborhood)
    input  logic [PIXEL_WIDTH-1:0]  pixel_00,  // Top-left
    input  logic [PIXEL_WIDTH-1:0]  pixel_01,  // Top-right  
    input  logic [PIXEL_WIDTH-1:0]  pixel_10,  // Bottom-left
    input  logic [PIXEL_WIDTH-1:0]  pixel_11,  // Bottom-right
    
    // Fractional coordinates (0-5 for mod 6 operation)
    input  logic [2:0]               frac_x,    // x mod 6
    input  logic [2:0]               frac_y,    // y mod 6
    
    // Output
    output logic [PIXEL_WIDTH-1:0]  pixel_out,
    output logic                     valid_out
);

    // Internal signals
    logic [PIXEL_WIDTH+5:0] weighted_sum;
    logic [5:0] weight_sum;
    logic valid_reg;
    
    // Weight calculation based on fractional coordinates
    logic [2:0] weight_x, weight_y;
    logic [2:0] weight_x_inv, weight_y_inv;
    
    // Individual pixel weights
    logic [5:0] w00, w01, w10, w11;
    
    // Weighted pixel values
    logic [PIXEL_WIDTH+5:0] wp00, wp01, wp10, wp11;
    
    always_comb begin
        // Calculate weights
        weight_x = frac_x;
        weight_y = frac_y;
        weight_x_inv = 3'd5 - frac_x;
        weight_y_inv = 3'd5 - frac_y;
        
        // Bilinear weights
        w00 = weight_x_inv * weight_y_inv;  // (5-x) * (5-y)
        w01 = weight_x * weight_y_inv;      // x * (5-y)
        w10 = weight_x_inv * weight_y;      // (5-x) * y
        w11 = weight_x * weight_y;          // x * y
        
        // Weighted pixel values
        wp00 = pixel_00 * w00;
        wp01 = pixel_01 * w01;
        wp10 = pixel_10 * w10;
        wp11 = pixel_11 * w11;
        
        // Sum of weights (should always be 25 for normalization)
        weight_sum = w00 + w01 + w10 + w11;
        
        // Total weighted sum
        weighted_sum = wp00 + wp01 + wp10 + wp11;
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_out <= '0;
            valid_out <= 1'b0;
        end else begin
            valid_out <= valid_in;
            if (valid_in) begin
                // Divide by 25 (weight_sum) to normalize
                // Using shift for efficiency: divide by 32 and multiply by 32/25
                pixel_out <= (weighted_sum * 32) / (weight_sum * 32) * 25;
                // Simplified: just divide by 25
                pixel_out <= weighted_sum / 25;
            end
        end
    end

endmodule