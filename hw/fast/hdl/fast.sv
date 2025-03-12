/*
 * FAST (Features from Accelerated Segment Test) Corner Detection Module
 * 
 * Detects corners by examining a Bresenham circle of 16 pixels around each pixel.
 * A pixel is a corner if 9 contiguous pixels are all brighter or darker than
 * the center pixel by a threshold.
 */

module fast #(
    parameter PIXEL_WIDTH = 6,
    parameter THRESHOLD = 20,
    parameter IMG_WIDTH = 640,
    parameter IMG_HEIGHT = 480
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    pixel_valid,
    input  logic [PIXEL_WIDTH-1:0] pixel_in,
    
    // Line buffer outputs (we need 7 rows for the Bresenham circle)
    input  logic [PIXEL_WIDTH-1:0] line_buffer [6:0],
    
    // Output
    output logic                    corner_valid,
    output logic [15:0]             corner_score,
    output logic [11:0]             corner_x,
    output logic [11:0]             corner_y
);

    // Internal pixel coordinates
    logic [11:0] pixel_x, pixel_y;
    
    // Bresenham circle pixels (16 pixels at radius 3)
    logic [PIXEL_WIDTH-1:0] circle_pixels [15:0];
    logic [PIXEL_WIDTH-1:0] center_pixel;
    
    // Threshold comparisons
    logic [15:0] brighter_mask, darker_mask;
    logic [15:0] contiguous_masks [15:0];
    
    // Corner detection results
    logic is_corner;
    logic [15:0] pixel_score;
    
    // Pixel coordinate tracking
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_x <= '0;
            pixel_y <= '0;
        end else if (pixel_valid) begin
            if (pixel_x == IMG_WIDTH - 1) begin
                pixel_x <= '0;
                pixel_y <= pixel_y + 1;
            end else begin
                pixel_x <= pixel_x + 1;
            end
        end
    end
    
    // Extract Bresenham circle pixels from line buffers
    // Circle pattern (relative to center at [3,3]):
    // Pixel indices: 0-15 around the circle
    always_comb begin
        // Center pixel from current line buffer position
        center_pixel = line_buffer[3]; // Middle of 7-line buffer
        
        // Bresenham circle pixels (clock-wise from top)
        circle_pixels[0]  = line_buffer[0];  // (0, -3)
        circle_pixels[1]  = line_buffer[0];  // (1, -3) - approximated
        circle_pixels[2]  = line_buffer[1];  // (2, -2)
        circle_pixels[3]  = line_buffer[1];  // (3, -1)
        circle_pixels[4]  = line_buffer[2];  // (3,  0)
        circle_pixels[5]  = line_buffer[3];  // (3,  1)
        circle_pixels[6]  = line_buffer[4];  // (2,  2)
        circle_pixels[7]  = line_buffer[4];  // (1,  3)
        circle_pixels[8]  = line_buffer[5];  // (0,  3)
        circle_pixels[9]  = line_buffer[5];  // (-1, 3)
        circle_pixels[10] = line_buffer[4];  // (-2, 2)
        circle_pixels[11] = line_buffer[3];  // (-3, 1)
        circle_pixels[12] = line_buffer[2];  // (-3, 0)
        circle_pixels[13] = line_buffer[1];  // (-3,-1)
        circle_pixels[14] = line_buffer[1];  // (-2,-2)
        circle_pixels[15] = line_buffer[0];  // (-1,-3)
    end
    
    // Threshold comparisons for each circle pixel
    always_comb begin
        for (int i = 0; i < 16; i++) begin
            brighter_mask[i] = (circle_pixels[i] > center_pixel + THRESHOLD);
            darker_mask[i]   = (circle_pixels[i] < center_pixel - THRESHOLD);
        end
    end
    
    // Generate contiguous masks for 9 consecutive pixels
    always_comb begin
        for (int i = 0; i < 16; i++) begin
            // Create mask for 9 contiguous pixels starting at position i
            contiguous_masks[i] = 16'h1FF << i; // 9 ones shifted by i
            // Handle wrap-around for circular buffer
            if (i > 7) begin
                contiguous_masks[i] = (16'h1FF << i) | (16'h1FF >> (16 - i));
            end
        end
    end
    
    // Corner detection logic
    always_comb begin
        is_corner = 1'b0;
        
        // Check if any 9 contiguous pixels are all brighter or all darker
        for (int i = 0; i < 16; i++) begin
            if ((brighter_mask & contiguous_masks[i]) == contiguous_masks[i] ||
                (darker_mask & contiguous_masks[i]) == contiguous_masks[i]) begin
                is_corner = 1'b1;
                break;
            end
        end
    end
    
    // Calculate corner score (sum of absolute differences)
    always_comb begin
        pixel_score = '0;
        if (is_corner) begin
            for (int i = 0; i < 16; i++) begin
                if (circle_pixels[i] > center_pixel) begin
                    pixel_score += (circle_pixels[i] - center_pixel);
                end else begin
                    pixel_score += (center_pixel - circle_pixels[i]);
                end
            end
        end
    end
    
    // Output valid region (avoid borders where circle extends outside image)
    logic in_valid_region;
    always_comb begin
        in_valid_region = (pixel_x >= 3) && (pixel_x < IMG_WIDTH - 3) &&
                         (pixel_y >= 3) && (pixel_y < IMG_HEIGHT - 3);
    end
    
    // Register outputs
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            corner_valid <= 1'b0;
            corner_score <= '0;
            corner_x <= '0;
            corner_y <= '0;
        end else begin
            corner_valid <= pixel_valid && is_corner && in_valid_region;
            if (pixel_valid && is_corner && in_valid_region) begin
                corner_score <= pixel_score;
                corner_x <= pixel_x;
                corner_y <= pixel_y;
            end
        end
    end

endmodule