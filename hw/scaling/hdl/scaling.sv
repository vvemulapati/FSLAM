/*
 * Image Scaling Module
 * 
 * Implements image downsampling with scale factor 5/6 (1.2x reduction).
 * Uses bilinear interpolation for high-quality resampling.
 * Outputs resampled pixels every 5 out of 6 cycles in each dimension.
 */

module scaling #(
    parameter PIXEL_WIDTH = 6,
    parameter IMG_WIDTH = 640,
    parameter IMG_HEIGHT = 480
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    pixel_valid,
    input  logic [PIXEL_WIDTH-1:0] pixel_in,
    
    // Output scaled image
    output logic                    scaled_valid,
    output logic [PIXEL_WIDTH-1:0] pixel_out
);

    // Input pixel coordinates
    logic [11:0] in_x, in_y;
    
    // Output pixel coordinates 
    logic [11:0] out_x, out_y;
    
    // Line buffer for 2 rows (needed for bilinear interpolation)
    logic [PIXEL_WIDTH-1:0] line_buffer [1:0][IMG_WIDTH-1:0];
    logic [11:0] line_write_ptr;
    
    // 2x2 pixel window for interpolation
    logic [PIXEL_WIDTH-1:0] pixel_window [1:0][1:0];
    
    // Fractional coordinates for interpolation
    logic [2:0] frac_x, frac_y;
    
    // Scale factor tracking (5/6 downsampling)
    logic [2:0] scale_counter_x, scale_counter_y;
    logic output_pixel;
    
    // Bilinear interpolator instance
    logic interp_valid_in, interp_valid_out;
    logic [PIXEL_WIDTH-1:0] interp_out;
    
    bilinear_interpolator #(
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .COORD_WIDTH(12)
    ) u_interpolator (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(interp_valid_in),
        .pixel_00(pixel_window[0][0]),
        .pixel_01(pixel_window[0][1]),
        .pixel_10(pixel_window[1][0]),
        .pixel_11(pixel_window[1][1]),
        .frac_x(frac_x),
        .frac_y(frac_y),
        .pixel_out(interp_out),
        .valid_out(interp_valid_out)
    );
    
    // Input pixel coordinate tracking
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_x <= '0;
            in_y <= '0;
            line_write_ptr <= '0;
        end else if (pixel_valid) begin
            if (in_x == IMG_WIDTH - 1) begin
                in_x <= '0;
                in_y <= in_y + 1;
                line_write_ptr <= '0;
            end else begin
                in_x <= in_x + 1;
                line_write_ptr <= line_write_ptr + 1;
            end
        end
    end
    
    // Line buffer management
    always_ff @(posedge clk) begin
        if (pixel_valid) begin
            // Shift line buffers when starting new row
            if (in_x == 0 && in_y > 0) begin
                line_buffer[1] <= line_buffer[0];
            end
            line_buffer[0][line_write_ptr] <= pixel_in;
        end
    end
    
    // Scale factor logic (5/6 downsampling)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scale_counter_x <= '0;
            scale_counter_y <= '0;
            out_x <= '0;
            out_y <= '0;
        end else if (pixel_valid) begin
            // X counter
            if (scale_counter_x == 5) begin
                scale_counter_x <= '0;
            end else begin
                scale_counter_x <= scale_counter_x + 1;
            end
            
            // Y counter (increment at end of each row)
            if (in_x == IMG_WIDTH - 1) begin
                if (scale_counter_y == 5) begin
                    scale_counter_y <= '0;
                end else begin
                    scale_counter_y <= scale_counter_y + 1;
                end
            end
            
            // Output coordinates (only when outputting pixels)
            if (output_pixel) begin
                if (out_x == (IMG_WIDTH * 5 / 6) - 1) begin
                    out_x <= '0;
                    out_y <= out_y + 1;
                end else begin
                    out_x <= out_x + 1;
                end
            end
        end
    end
    
    // Determine when to output pixels (every 5 out of 6 in both dimensions)
    always_comb begin
        output_pixel = pixel_valid && 
                      (scale_counter_x != 5) && 
                      (scale_counter_y != 5) &&
                      (in_y > 0); // Need at least 2 rows for interpolation
    end
    
    // Extract 2x2 pixel window for bilinear interpolation
    always_comb begin
        logic [11:0] read_x = (line_write_ptr > 0) ? line_write_ptr - 1 : IMG_WIDTH - 1;
        
        // Current and previous pixels from current line
        pixel_window[0][0] = line_buffer[1][read_x];      // Previous line, previous pixel
        pixel_window[0][1] = line_buffer[1][line_write_ptr]; // Previous line, current pixel
        pixel_window[1][0] = line_buffer[0][read_x];      // Current line, previous pixel  
        pixel_window[1][1] = line_buffer[0][line_write_ptr]; // Current line, current pixel
        
        // Fractional coordinates based on scale counters
        frac_x = scale_counter_x;
        frac_y = scale_counter_y;
    end
    
    // Interpolation control
    always_comb begin
        interp_valid_in = output_pixel;
    end
    
    // Output stage
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scaled_valid <= 1'b0;
            pixel_out <= '0;
        end else begin
            scaled_valid <= interp_valid_out;
            pixel_out <= interp_out;
        end
    end

endmodule