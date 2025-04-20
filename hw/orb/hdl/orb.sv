/*
 * ORB (Oriented FAST and Rotated BRIEF) Feature Extraction Module
 * 
 * Top-level module that integrates the complete ORB pipeline:
 * 1. Image pyramid generation
 * 2. FAST corner detection  
 * 3. Non-maximum suppression
 * 4. Gaussian blur for orientation
 * 5. Orientation calculation
 * 6. BRIEF descriptor generation
 * 7. Feature matching
 */

module orb #(
    parameter PIXEL_WIDTH = 6,
    parameter IMG_WIDTH = 640,
    parameter IMG_HEIGHT = 480,
    parameter PYRAMID_LEVELS = 4,
    parameter DESCRIPTOR_BITS = 256,
    parameter MAX_FEATURES_PER_LEVEL = 250
) (
    input  logic                        clk,
    input  logic                        rst_n,
    input  logic                        enable,
    
    // Input image stream
    input  logic                        pixel_valid,
    input  logic [PIXEL_WIDTH-1:0]     pixel_in,
    
    // BRIEF pattern coordinates (loaded externally)
    input  logic signed [5:0]           brief_pattern [DESCRIPTOR_BITS-1:0][3:0],
    
    // Output features 
    output logic                        feature_valid,
    output logic [DESCRIPTOR_BITS-1:0] feature_descriptor,
    output logic [15:0]                 feature_score,
    output logic [11:0]                 feature_x,
    output logic [11:0]                 feature_y,
    output logic [5:0]                  feature_orientation,
    output logic [1:0]                  feature_level,
    
    // Status
    output logic                        processing,
    output logic [10:0]                 total_features
);

    // Image pyramid outputs
    logic [PYRAMID_LEVELS-1:0] pyramid_valid;
    logic [PIXEL_WIDTH-1:0] pyramid_pixels [PYRAMID_LEVELS-1:0];
    logic [11:0] pyramid_widths [PYRAMID_LEVELS-1:0];
    logic [11:0] pyramid_heights [PYRAMID_LEVELS-1:0];
    
    // FAST corner detection outputs (per level)
    logic [PYRAMID_LEVELS-1:0] corner_valid;
    logic [15:0] corner_scores [PYRAMID_LEVELS-1:0];
    logic [11:0] corner_x_coords [PYRAMID_LEVELS-1:0];
    logic [11:0] corner_y_coords [PYRAMID_LEVELS-1:0];
    
    // NMS outputs (per level) 
    logic [PYRAMID_LEVELS-1:0] keypoint_valid;
    logic [15:0] keypoint_scores [PYRAMID_LEVELS-1:0];
    logic [11:0] keypoint_x_coords [PYRAMID_LEVELS-1:0];
    logic [11:0] keypoint_y_coords [PYRAMID_LEVELS-1:0];
    
    // Gaussian blur outputs (per level)
    logic [PYRAMID_LEVELS-1:0] blur_valid;
    logic [PIXEL_WIDTH-1:0] blur_pixels [PYRAMID_LEVELS-1:0];
    
    // Orientation outputs (per level)
    logic [PYRAMID_LEVELS-1:0] orientation_valid;
    logic [5:0] orientations [PYRAMID_LEVELS-1:0];
    logic [PYRAMID_LEVELS-1:0] orientation_busy;
    
    // BRIEF outputs (per level)
    logic [PYRAMID_LEVELS-1:0] brief_valid;
    logic [DESCRIPTOR_BITS-1:0] descriptors [PYRAMID_LEVELS-1:0];
    logic [PYRAMID_LEVELS-1:0] brief_busy;
    
    // Feature count per level
    logic [10:0] level_feature_counts [PYRAMID_LEVELS-1:0];
    
    // Image pyramid generation
    image_pyramid #(
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .PYRAMID_LEVELS(PYRAMID_LEVELS)
    ) u_pyramid (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_valid(pixel_valid),
        .pixel_in(pixel_in),
        .level_valid(pyramid_valid),
        .level_pixels(pyramid_pixels),
        .level_widths(pyramid_widths),
        .level_heights(pyramid_heights)
    );
    
    // Generate processing pipeline for each pyramid level
    genvar level;
    generate
        for (level = 0; level < PYRAMID_LEVELS; level++) begin : gen_level_processing
            
            // FAST corner detection
            fast #(
                .PIXEL_WIDTH(PIXEL_WIDTH),
                .THRESHOLD(20),
                .IMG_WIDTH(pyramid_widths[level]),
                .IMG_HEIGHT(pyramid_heights[level])
            ) u_fast (
                .clk(clk),
                .rst_n(rst_n),
                .pixel_valid(pyramid_valid[level]),
                .pixel_in(pyramid_pixels[level]),
                .line_buffer(/* Connect line buffer outputs */),
                .corner_valid(corner_valid[level]),
                .corner_score(corner_scores[level]),
                .corner_x(corner_x_coords[level]),
                .corner_y(corner_y_coords[level])
            );
            
            // Non-maximum suppression
            nms #(
                .SCORE_WIDTH(16),
                .COORD_WIDTH(12),
                .IMG_WIDTH(pyramid_widths[level])
            ) u_nms (
                .clk(clk),
                .rst_n(rst_n),
                .corner_valid_in(corner_valid[level]),
                .corner_score_in(corner_scores[level]),
                .corner_x_in(corner_x_coords[level]),
                .corner_y_in(corner_y_coords[level]),
                .keypoint_valid(keypoint_valid[level]),
                .keypoint_score(keypoint_scores[level]),
                .keypoint_x(keypoint_x_coords[level]),
                .keypoint_y(keypoint_y_coords[level])
            );
            
            // Gaussian blur for orientation calculation
            gaussian_blur #(
                .PIXEL_WIDTH(PIXEL_WIDTH),
                .KERNEL_SIZE(7),
                .IMG_WIDTH(pyramid_widths[level])
            ) u_blur (
                .clk(clk),
                .rst_n(rst_n),
                .pixel_valid(pyramid_valid[level]),
                .pixel_in(pyramid_pixels[level]),
                .blur_valid(blur_valid[level]),
                .pixel_out(blur_pixels[level])
            );
            
            // Orientation calculation
            orientation #(
                .PIXEL_WIDTH(PIXEL_WIDTH),
                .WINDOW_SIZE(37),
                .COORD_WIDTH(12),
                .ANGLE_SECTORS(64),
                .ANGLE_BITS(6)
            ) u_orientation (
                .clk(clk),
                .rst_n(rst_n),
                .start(keypoint_valid[level]),
                .keypoint_x(keypoint_x_coords[level]),
                .keypoint_y(keypoint_y_coords[level]),
                .pixel_valid(blur_valid[level]),
                .pixel_in(blur_pixels[level]),
                .pixel_x(/* pixel coordinate tracking needed */),
                .pixel_y(/* pixel coordinate tracking needed */),
                .orientation_out(orientations[level]),
                .orientation_valid(orientation_valid[level]),
                .busy(orientation_busy[level])
            );
            
            // BRIEF descriptor generation  
            brief #(
                .PIXEL_WIDTH(PIXEL_WIDTH),
                .WINDOW_SIZE(37),
                .DESCRIPTOR_BITS(DESCRIPTOR_BITS),
                .ANGLE_BITS(6),
                .COORD_BITS(6)
            ) u_brief (
                .clk(clk),
                .rst_n(rst_n),
                .start(orientation_valid[level]),
                .keypoint_x(keypoint_x_coords[level][5:0]),
                .keypoint_y(keypoint_y_coords[level][5:0]),
                .orientation(orientations[level]),
                .window_buffer(/* 37x37 window from blur output */),
                .window_valid(1'b1),
                .pattern_coords(brief_pattern),
                .descriptor(descriptors[level]),
                .descriptor_valid(brief_valid[level]),
                .busy(brief_busy[level])
            );
            
            // Feature counter for this level
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    level_feature_counts[level] <= '0;
                end else if (brief_valid[level]) begin
                    if (level_feature_counts[level] < MAX_FEATURES_PER_LEVEL) begin
                        level_feature_counts[level] <= level_feature_counts[level] + 1;
                    end
                end
            end
        end
    endgenerate
    
    // Output arbitration - round-robin between levels
    logic [1:0] output_level_select;
    logic [1:0] next_output_level;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_level_select <= '0;
        end else if (feature_valid) begin
            output_level_select <= next_output_level;
        end
    end
    
    // Round-robin arbitration
    always_comb begin
        next_output_level = output_level_select;
        for (int i = 1; i <= PYRAMID_LEVELS; i++) begin
            logic [1:0] check_level = (output_level_select + i) % PYRAMID_LEVELS;
            if (brief_valid[check_level]) begin
                next_output_level = check_level;
                break;
            end
        end
    end
    
    // Output muxing
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            feature_valid <= 1'b0;
            feature_descriptor <= '0;
            feature_score <= '0;
            feature_x <= '0;
            feature_y <= '0;
            feature_orientation <= '0;
            feature_level <= '0;
        end else begin
            feature_valid <= brief_valid[output_level_select];
            if (brief_valid[output_level_select]) begin
                feature_descriptor <= descriptors[output_level_select];
                feature_score <= keypoint_scores[output_level_select];
                feature_x <= keypoint_x_coords[output_level_select];
                feature_y <= keypoint_y_coords[output_level_select];
                feature_orientation <= orientations[output_level_select];
                feature_level <= output_level_select;
            end
        end
    end
    
    // Status outputs
    always_comb begin
        processing = |orientation_busy || |brief_busy;
        
        total_features = '0;
        for (int i = 0; i < PYRAMID_LEVELS; i++) begin
            total_features += level_feature_counts[i];
        end
    end

endmodule