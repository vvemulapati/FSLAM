/*
 * Coordinate Rotator Module
 * 
 * Rotates BRIEF pattern coordinates based on keypoint orientation.
 * Uses precomputed sin/cos lookup tables for the discretized angles.
 * Performs 2D rotation: [x' y'] = [cos -sin; sin cos] * [x y]
 */

module rotator #(
    parameter COORD_WIDTH = 6,
    parameter ANGLE_BITS = 6,
    parameter FRAC_BITS = 7  // Fractional bits for sin/cos values
) (
    input  logic                              clk,
    input  logic                              rst_n,
    input  logic                              start,
    
    // Input coordinates (relative to center)
    input  logic signed [COORD_WIDTH-1:0]    coord_x_in,
    input  logic signed [COORD_WIDTH-1:0]    coord_y_in,
    
    // Rotation angle (discretized to 64 sectors)
    input  logic [ANGLE_BITS-1:0]            angle,
    
    // Output rotated coordinates
    output logic signed [COORD_WIDTH-1:0]    coord_x_out,
    output logic signed [COORD_WIDTH-1:0]    coord_y_out,
    output logic                              valid_out
);

    // Sin/Cos lookup tables for 64 angles (0 to 63)
    // Values are in signed 8-bit format with 7 fractional bits (range: -1 to +1)
    logic signed [7:0] cos_lut [63:0];
    logic signed [7:0] sin_lut [63:0];
    
    // Rotation computation
    logic signed [COORD_WIDTH+7:0] x_cos_term, x_sin_term;
    logic signed [COORD_WIDTH+7:0] y_cos_term, y_sin_term;
    logic signed [COORD_WIDTH+7:0] rotated_x, rotated_y;
    
    // Pipeline registers
    logic signed [COORD_WIDTH-1:0] coord_x_reg, coord_y_reg;
    logic [ANGLE_BITS-1:0] angle_reg;
    logic valid_reg;
    
    // Initialize sin/cos lookup tables
    initial begin
        // Generate sin/cos values for 64 sectors (each sector = 5.625 degrees)
        for (int i = 0; i < 64; i++) begin
            real angle_rad = (i * 2.0 * 3.14159265) / 64.0;
            cos_lut[i] = $signed($rtoi(127.0 * $cos(angle_rad)));
            sin_lut[i] = $signed($rtoi(127.0 * $sin(angle_rad)));
        end
    end
    
    // Input stage
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            coord_x_reg <= '0;
            coord_y_reg <= '0;
            angle_reg <= '0;
            valid_reg <= 1'b0;
        end else begin
            coord_x_reg <= coord_x_in;
            coord_y_reg <= coord_y_in;
            angle_reg <= angle;
            valid_reg <= start;
        end
    end
    
    // Rotation computation stage
    always_comb begin
        // Get sin/cos values for the angle
        logic signed [7:0] cos_val = cos_lut[angle_reg];
        logic signed [7:0] sin_val = sin_lut[angle_reg];
        
        // Compute rotation matrix multiplication
        // x' = x*cos - y*sin
        // y' = x*sin + y*cos
        
        x_cos_term = coord_x_reg * cos_val;
        x_sin_term = coord_y_reg * sin_val;
        y_cos_term = coord_y_reg * cos_val;
        y_sin_term = coord_x_reg * sin_val;
        
        rotated_x = x_cos_term - x_sin_term;
        rotated_y = y_sin_term + y_cos_term;
    end
    
    // Output stage
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            coord_x_out <= '0;
            coord_y_out <= '0;
            valid_out <= 1'b0;
        end else begin
            valid_out <= valid_reg;
            if (valid_reg) begin
                // Scale down by 2^7 to account for fractional bits
                coord_x_out <= rotated_x >>> FRAC_BITS;
                coord_y_out <= rotated_y >>> FRAC_BITS;
            end
        end
    end

endmodule