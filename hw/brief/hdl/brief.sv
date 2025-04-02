/*
 * BRIEF (Binary Robust Independent Elementary Features) Module
 * 
 * Generates rotation-invariant 256-bit binary descriptors for keypoints.
 * Uses rotated BRIEF patterns based on keypoint orientation.
 * Operates on a 37x37 window around each keypoint.
 */

module brief #(
    parameter PIXEL_WIDTH = 6,
    parameter WINDOW_SIZE = 37,
    parameter DESCRIPTOR_BITS = 256,
    parameter ANGLE_BITS = 6,      // 64 sectors
    parameter COORD_BITS = 6       // For 37x37 window
) (
    input  logic                          clk,
    input  logic                          rst_n,
    input  logic                          start,
    
    // Keypoint information
    input  logic [COORD_BITS-1:0]        keypoint_x,
    input  logic [COORD_BITS-1:0]        keypoint_y,
    input  logic [ANGLE_BITS-1:0]        orientation,
    
    // Window buffer interface (37x37)
    input  logic [PIXEL_WIDTH-1:0]       window_buffer [WINDOW_SIZE-1:0][WINDOW_SIZE-1:0],
    input  logic                          window_valid,
    
    // BRIEF pattern coordinates (256 pairs)
    input  logic signed [COORD_BITS-1:0] pattern_coords [DESCRIPTOR_BITS-1:0][3:0], // [pair][x1,y1,x2,y2]
    
    // Output
    output logic [DESCRIPTOR_BITS-1:0]   descriptor,
    output logic                          descriptor_valid,
    output logic                          busy
);

    // State machine
    typedef enum logic [2:0] {
        IDLE,
        ROTATE_PATTERN,
        GENERATE_DESCRIPTOR,
        DONE
    } state_t;
    
    state_t current_state, next_state;
    
    // Pattern processing
    logic [7:0] pattern_index;
    logic signed [COORD_BITS-1:0] rotated_coords [3:0]; // x1, y1, x2, y2
    logic signed [COORD_BITS-1:0] final_coords [3:0];
    
    // Rotation LUTs (cos and sin values for 64 sectors)
    logic signed [7:0] cos_lut [63:0];
    logic signed [7:0] sin_lut [63:0];
    
    // Pixel values for comparison
    logic [PIXEL_WIDTH-1:0] pixel_1, pixel_2;
    logic descriptor_bit;
    
    // Window center
    localparam WINDOW_CENTER = WINDOW_SIZE / 2;
    
    // Initialize rotation LUTs
    initial begin
        // Populate cos/sin LUTs for 64 sectors (0 to 63)
        // Each sector represents 360/64 = 5.625 degrees
        for (int i = 0; i < 64; i++) begin
            real angle_rad = (i * 2.0 * 3.14159265) / 64.0;
            cos_lut[i] = $signed($rtoi(127.0 * $cos(angle_rad)));
            sin_lut[i] = $signed($rtoi(127.0 * $sin(angle_rad)));
        end
    end
    
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
                if (start && window_valid) begin
                    next_state = ROTATE_PATTERN;
                end
            end
            ROTATE_PATTERN: begin
                next_state = GENERATE_DESCRIPTOR;
            end
            GENERATE_DESCRIPTOR: begin
                if (pattern_index == DESCRIPTOR_BITS - 1) begin
                    next_state = DONE;
                end
            end
            DONE: begin
                next_state = IDLE;
            end
        endcase
    end
    
    // Pattern rotation using orientation
    always_comb begin
        // Get rotation coefficients
        logic signed [7:0] cos_val = cos_lut[orientation];
        logic signed [7:0] sin_val = sin_lut[orientation];
        
        // Original pattern coordinates (relative to center)
        logic signed [COORD_BITS-1:0] x1 = pattern_coords[pattern_index][0];
        logic signed [COORD_BITS-1:0] y1 = pattern_coords[pattern_index][1];
        logic signed [COORD_BITS-1:0] x2 = pattern_coords[pattern_index][2];
        logic signed [COORD_BITS-1:0] y2 = pattern_coords[pattern_index][3];
        
        // Rotate coordinates: [x' y'] = [cos -sin; sin cos] * [x y]
        rotated_coords[0] = (cos_val * x1 - sin_val * y1) >>> 7; // x1'
        rotated_coords[1] = (sin_val * x1 + cos_val * y1) >>> 7; // y1'
        rotated_coords[2] = (cos_val * x2 - sin_val * y2) >>> 7; // x2'
        rotated_coords[3] = (sin_val * x2 + cos_val * y2) >>> 7; // y2'
        
        // Translate to window coordinates (add center offset)
        final_coords[0] = rotated_coords[0] + WINDOW_CENTER; // x1 final
        final_coords[1] = rotated_coords[1] + WINDOW_CENTER; // y1 final
        final_coords[2] = rotated_coords[2] + WINDOW_CENTER; // x2 final
        final_coords[3] = rotated_coords[3] + WINDOW_CENTER; // y2 final
    end
    
    // Pixel sampling and comparison
    always_comb begin
        // Bounds checking and pixel sampling
        if (final_coords[0] >= 0 && final_coords[0] < WINDOW_SIZE &&
            final_coords[1] >= 0 && final_coords[1] < WINDOW_SIZE) begin
            pixel_1 = window_buffer[final_coords[1]][final_coords[0]];
        end else begin
            pixel_1 = '0;
        end
        
        if (final_coords[2] >= 0 && final_coords[2] < WINDOW_SIZE &&
            final_coords[3] >= 0 && final_coords[3] < WINDOW_SIZE) begin
            pixel_2 = window_buffer[final_coords[3]][final_coords[2]];
        end else begin
            pixel_2 = '0;
        end
        
        // BRIEF comparison: descriptor_bit = (pixel_1 > pixel_2)
        descriptor_bit = (pixel_1 > pixel_2);
    end
    
    // Descriptor generation
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pattern_index <= '0;
            descriptor <= '0;
            descriptor_valid <= 1'b0;
            busy <= 1'b0;
        end else begin
            case (current_state)
                IDLE: begin
                    pattern_index <= '0;
                    descriptor_valid <= 1'b0;
                    if (start && window_valid) begin
                        busy <= 1'b1;
                        descriptor <= '0;
                    end else begin
                        busy <= 1'b0;
                    end
                end
                ROTATE_PATTERN: begin
                    // Rotation is combinational, move to next state
                end
                GENERATE_DESCRIPTOR: begin
                    // Set the bit in the descriptor
                    descriptor[pattern_index] <= descriptor_bit;
                    pattern_index <= pattern_index + 1;
                end
                DONE: begin
                    descriptor_valid <= 1'b1;
                    busy <= 1'b0;
                end
            endcase
        end
    end

endmodule