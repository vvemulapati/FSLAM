/*
 * Orientation Calculation Module
 * 
 * Calculates keypoint orientation using intensity centroid method.
 * Operates on a 37x37 window around each keypoint and computes
 * moments m01 and m10 to determine the orientation angle.
 * Uses discretized angles (64 sectors) for hardware efficiency.
 */

module orientation #(
    parameter PIXEL_WIDTH = 6,
    parameter WINDOW_SIZE = 37,
    parameter COORD_WIDTH = 12,
    parameter ANGLE_SECTORS = 64,
    parameter ANGLE_BITS = 6
) (
    input  logic                        clk,
    input  logic                        rst_n,
    input  logic                        start,
    
    // Keypoint coordinates
    input  logic [COORD_WIDTH-1:0]     keypoint_x,
    input  logic [COORD_WIDTH-1:0]     keypoint_y,
    
    // Gaussian blurred image data (streaming)
    input  logic                        pixel_valid,
    input  logic [PIXEL_WIDTH-1:0]     pixel_in,
    input  logic [COORD_WIDTH-1:0]     pixel_x,
    input  logic [COORD_WIDTH-1:0]     pixel_y,
    
    // Output
    output logic [ANGLE_BITS-1:0]      orientation_out,
    output logic                        orientation_valid,
    output logic                        busy
);

    // Window buffer for 37x37 region
    logic [PIXEL_WIDTH-1:0] window_buffer [WINDOW_SIZE-1:0][WINDOW_SIZE-1:0];
    logic window_ready;
    
    // Moment calculation
    logic signed [31:0] m00, m01, m10;  // Image moments
    logic signed [31:0] m00_next, m01_next, m10_next;
    
    // Column sums for recursive calculation
    logic signed [15:0] col_sum_in, col_sum_out;
    logic signed [15:0] col_sums [WINDOW_SIZE-1:0];
    
    // Window center
    localparam WINDOW_CENTER = WINDOW_SIZE / 2;
    
    // State machine
    typedef enum logic [2:0] {
        IDLE,
        CAPTURE_WINDOW,
        CALC_MOMENTS,
        CALC_ANGLE,
        DONE
    } state_t;
    
    state_t current_state, next_state;
    
    // Angle calculation using LUT
    logic signed [31:0] abs_m01, abs_m10;
    logic [1:0] quadrant;
    logic [ANGLE_BITS-3:0] sector_in_quad;
    
    // Tan approximation LUT for one quadrant (16 sectors)
    localparam logic [7:0] TAN_LUT [15:0] = '{
        8'd51,   // tan(5.625°)  ≈ 0.199 * 256
        8'd171,  // tan(16.875°) ≈ 0.668 * 256  
        8'd383,  // tan(28.125°) ≈ 1.496 * 256
        8'd1287, // tan(39.375°) ≈ 5.027 * 256
        // ... (simplified for 4 sectors per quadrant)
        // In practice, would have all 16 values
        8'd51, 8'd171, 8'd383, 8'd1287,
        8'd51, 8'd171, 8'd383, 8'd1287,
        8'd51, 8'd171, 8'd383, 8'd1287,
        8'd51, 8'd171, 8'd383, 8'd1287
    };
    
    // Processing counters
    logic [5:0] row_cnt, col_cnt;
    
    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end
    
    always_comb begin
        next_state = current_state;
        case (current_state)
            IDLE: begin
                if (start) begin
                    next_state = CAPTURE_WINDOW;
                end
            end
            CAPTURE_WINDOW: begin
                if (window_ready) begin
                    next_state = CALC_MOMENTS;
                end
            end
            CALC_MOMENTS: begin
                if (row_cnt == WINDOW_SIZE - 1 && col_cnt == WINDOW_SIZE - 1) begin
                    next_state = CALC_ANGLE;
                end
            end
            CALC_ANGLE: begin
                next_state = DONE;
            end
            DONE: begin
                next_state = IDLE;
            end
        endcase
    end
    
    // Window capture logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_ready <= 1'b0;
        end else if (current_state == CAPTURE_WINDOW && pixel_valid) begin
            // Check if pixel is within the 37x37 window around keypoint
            if (pixel_x >= keypoint_x - WINDOW_CENTER && 
                pixel_x <= keypoint_x + WINDOW_CENTER &&
                pixel_y >= keypoint_y - WINDOW_CENTER && 
                pixel_y <= keypoint_y + WINDOW_CENTER) begin
                
                logic [5:0] win_x = pixel_x - keypoint_x + WINDOW_CENTER;
                logic [5:0] win_y = pixel_y - keypoint_y + WINDOW_CENTER;
                window_buffer[win_y][win_x] <= pixel_in;
                
                // Check if window is complete
                if (win_x == WINDOW_SIZE-1 && win_y == WINDOW_SIZE-1) begin
                    window_ready <= 1'b1;
                end
            end
        end else if (current_state == IDLE) begin
            window_ready <= 1'b0;
        end
    end
    
    // Column sum calculation
    always_comb begin
        col_sum_in = '0;
        for (int i = 0; i < WINDOW_SIZE; i++) begin
            col_sum_in += window_buffer[i][col_cnt];
        end
    end
    
    // Moment calculation using recursive approach
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m00 <= '0;
            m01 <= '0;
            m10 <= '0;
            row_cnt <= '0;
            col_cnt <= '0;
        end else if (current_state == CALC_MOMENTS) begin
            if (col_cnt < WINDOW_SIZE) begin
                // Calculate column sum
                col_sums[col_cnt] <= col_sum_in;
                
                // Update moments
                m00 <= m00 + col_sum_in;
                
                // m01 calculation (Y moment)
                logic signed [31:0] weighted_col_sum = '0;
                for (int i = 0; i < WINDOW_SIZE; i++) begin
                    weighted_col_sum += window_buffer[i][col_cnt] * (i - WINDOW_CENTER);
                end
                m01 <= m01 + weighted_col_sum;
                
                // m10 calculation (X moment)  
                m10 <= m10 + col_sum_in * (col_cnt - WINDOW_CENTER);
                
                col_cnt <= col_cnt + 1;
            end else begin
                col_cnt <= '0;
                if (row_cnt < WINDOW_SIZE - 1) begin
                    row_cnt <= row_cnt + 1;
                end
            end
        end else if (current_state == IDLE) begin
            m00 <= '0;
            m01 <= '0;
            m10 <= '0;
            row_cnt <= '0;
            col_cnt <= '0;
        end
    end
    
    // Angle calculation
    always_comb begin
        // Determine quadrant and absolute values
        abs_m01 = (m01 >= 0) ? m01 : -m01;
        abs_m10 = (m10 >= 0) ? m10 : -m10;
        
        quadrant[1] = (m01 < 0);  // Y sign
        quadrant[0] = (m10 < 0);  // X sign
        
        // Find sector within quadrant using tan approximation
        sector_in_quad = '0;
        for (int i = 0; i < 16; i++) begin
            // Check if abs_m10 * tan(angle) >= abs_m01
            if ((abs_m10 * TAN_LUT[i]) >> 8 >= abs_m01) begin
                sector_in_quad = i[ANGLE_BITS-3:0];
                break;
            end
        end
    end
    
    // Output logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            orientation_out <= '0;
            orientation_valid <= 1'b0;
            busy <= 1'b0;
        end else begin
            case (current_state)
                IDLE: begin
                    orientation_valid <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                    end else begin
                        busy <= 1'b0;
                    end
                end
                CAPTURE_WINDOW,
                CALC_MOMENTS: begin
                    busy <= 1'b1;
                end
                CALC_ANGLE: begin
                    // Combine quadrant and sector to get final angle
                    orientation_out <= {quadrant, sector_in_quad};
                end
                DONE: begin
                    orientation_valid <= 1'b1;
                    busy <= 1'b0;
                end
            endcase
        end
    end

endmodule