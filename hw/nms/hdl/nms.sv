/*
 * Non-Maximum Suppression (NMS) Module
 * 
 * Suppresses weak corners in the neighborhood of strong corners.
 * Uses a 3x3 window to compare corner scores and only outputs
 * the pixel if it has the maximum score in its neighborhood.
 */

module nms #(
    parameter SCORE_WIDTH = 16,
    parameter COORD_WIDTH = 12,
    parameter IMG_WIDTH = 640
) (
    input  logic                        clk,
    input  logic                        rst_n,
    input  logic                        corner_valid_in,
    input  logic [SCORE_WIDTH-1:0]     corner_score_in,
    input  logic [COORD_WIDTH-1:0]     corner_x_in,
    input  logic [COORD_WIDTH-1:0]     corner_y_in,
    
    // Output
    output logic                        keypoint_valid,
    output logic [SCORE_WIDTH-1:0]     keypoint_score,
    output logic [COORD_WIDTH-1:0]     keypoint_x,
    output logic [COORD_WIDTH-1:0]     keypoint_y
);

    // Line buffers for corner scores (3 rows)
    logic [SCORE_WIDTH-1:0] score_line_buffer [2:0][IMG_WIDTH-1:0];
    logic [COORD_WIDTH-1:0] x_line_buffer [2:0][IMG_WIDTH-1:0];
    logic [COORD_WIDTH-1:0] y_line_buffer [2:0][IMG_WIDTH-1:0];
    logic                   valid_line_buffer [2:0][IMG_WIDTH-1:0];
    
    // Write pointer for line buffers
    logic [COORD_WIDTH-1:0] write_ptr;
    logic [COORD_WIDTH-1:0] pixel_x, pixel_y;
    
    // 3x3 window extraction
    logic [SCORE_WIDTH-1:0] score_window [2:0][2:0];
    logic [COORD_WIDTH-1:0] x_window [2:0][2:0];
    logic [COORD_WIDTH-1:0] y_window [2:0][2:0];
    logic                   valid_window [2:0][2:0];
    
    // Center pixel (the one being evaluated)
    logic [SCORE_WIDTH-1:0] center_score;
    logic [COORD_WIDTH-1:0] center_x, center_y;
    logic                   center_valid;
    
    // NMS decision
    logic is_local_maximum;
    logic output_valid;
    
    // Pixel coordinate tracking
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_x <= '0;
            pixel_y <= '0;
            write_ptr <= '0;
        end else if (corner_valid_in) begin
            pixel_x <= corner_x_in;
            pixel_y <= corner_y_in;
            write_ptr <= corner_x_in;
        end
    end
    
    // Line buffer management
    always_ff @(posedge clk) begin
        if (corner_valid_in) begin
            // Shift line buffers down
            score_line_buffer[2][write_ptr] <= score_line_buffer[1][write_ptr];
            score_line_buffer[1][write_ptr] <= score_line_buffer[0][write_ptr];
            score_line_buffer[0][write_ptr] <= corner_score_in;
            
            x_line_buffer[2][write_ptr] <= x_line_buffer[1][write_ptr];
            x_line_buffer[1][write_ptr] <= x_line_buffer[0][write_ptr];
            x_line_buffer[0][write_ptr] <= corner_x_in;
            
            y_line_buffer[2][write_ptr] <= y_line_buffer[1][write_ptr];
            y_line_buffer[1][write_ptr] <= y_line_buffer[0][write_ptr];
            y_line_buffer[0][write_ptr] <= corner_y_in;
            
            valid_line_buffer[2][write_ptr] <= valid_line_buffer[1][write_ptr];
            valid_line_buffer[1][write_ptr] <= valid_line_buffer[0][write_ptr];
            valid_line_buffer[0][write_ptr] <= corner_valid_in;
        end
    end
    
    // Extract 3x3 window from line buffers
    always_comb begin
        for (int i = 0; i < 3; i++) begin
            for (int j = 0; j < 3; j++) begin
                logic [COORD_WIDTH-1:0] read_addr;
                
                // Calculate read address with wrap-around
                if (write_ptr >= j) begin
                    read_addr = write_ptr - j;
                end else begin
                    read_addr = IMG_WIDTH + write_ptr - j;
                end
                
                score_window[i][j] = score_line_buffer[i][read_addr];
                x_window[i][j] = x_line_buffer[i][read_addr];
                y_window[i][j] = y_line_buffer[i][read_addr];
                valid_window[i][j] = valid_line_buffer[i][read_addr];
            end
        end
    end
    
    // Center pixel extraction
    always_comb begin
        center_score = score_window[1][1];  // Middle of 3x3 window
        center_x = x_window[1][1];
        center_y = y_window[1][1];
        center_valid = valid_window[1][1];
    end
    
    // Non-maximum suppression logic
    always_comb begin
        is_local_maximum = 1'b1;
        
        if (center_valid && center_score > 0) begin
            // Compare center with all 8 neighbors
            for (int i = 0; i < 3; i++) begin
                for (int j = 0; j < 3; j++) begin
                    // Skip center pixel
                    if (i == 1 && j == 1) continue;
                    
                    if (valid_window[i][j]) begin
                        // For pixels to the right and below, use >= to avoid ties
                        if ((i > 1) || (i == 1 && j > 1)) begin
                            if (center_score < score_window[i][j]) begin
                                is_local_maximum = 1'b0;
                                break;
                            end
                        end else begin
                            // For other neighbors, use strict >
                            if (center_score <= score_window[i][j]) begin
                                is_local_maximum = 1'b0;
                                break;
                            end
                        end
                    end
                end
                if (!is_local_maximum) break;
            end
        end else begin
            is_local_maximum = 1'b0;
        end
    end
    
    // Valid output region (exclude borders)
    always_comb begin
        output_valid = center_valid && 
                      is_local_maximum &&
                      (center_x >= 1) && 
                      (center_x < IMG_WIDTH - 1) &&
                      (center_y >= 1);
    end
    
    // Output stage
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            keypoint_valid <= 1'b0;
            keypoint_score <= '0;
            keypoint_x <= '0;
            keypoint_y <= '0;
        end else begin
            keypoint_valid <= output_valid;
            if (output_valid) begin
                keypoint_score <= center_score;
                keypoint_x <= center_x;
                keypoint_y <= center_y;
            end
        end
    end

endmodule