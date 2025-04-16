/*
 * Image Pyramid Module
 * 
 * Generates a 4-level image pyramid with scale factor 1.2 between levels.
 * Level 0: Original image (640x480)
 * Level 1: 533x400 (640/1.2 x 480/1.2)
 * Level 2: 444x333 (533/1.2 x 400/1.2)  
 * Level 3: 370x278 (444/1.2 x 333/1.2)
 */

module image_pyramid #(
    parameter PIXEL_WIDTH = 6,
    parameter IMG_WIDTH = 640,
    parameter IMG_HEIGHT = 480,
    parameter PYRAMID_LEVELS = 4
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    pixel_valid,
    input  logic [PIXEL_WIDTH-1:0] pixel_in,
    
    // Output for each pyramid level
    output logic [PYRAMID_LEVELS-1:0] level_valid,
    output logic [PIXEL_WIDTH-1:0]    level_pixels [PYRAMID_LEVELS-1:0],
    output logic [11:0]               level_widths [PYRAMID_LEVELS-1:0],
    output logic [11:0]               level_heights [PYRAMID_LEVELS-1:0]
);

    // Level 0 (original image) - direct passthrough
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            level_valid[0] <= 1'b0;
            level_pixels[0] <= '0;
        end else begin
            level_valid[0] <= pixel_valid;
            level_pixels[0] <= pixel_in;
        end
    end
    
    // Scaling modules for each level
    logic [PYRAMID_LEVELS-2:0] scale_valid_in, scale_valid_out;
    logic [PIXEL_WIDTH-1:0] scale_pixel_in [PYRAMID_LEVELS-2:0];
    logic [PIXEL_WIDTH-1:0] scale_pixel_out [PYRAMID_LEVELS-2:0];
    
    // Generate scaling modules for levels 1, 2, 3
    genvar i;
    generate
        for (i = 0; i < PYRAMID_LEVELS-1; i++) begin : gen_scaling
            
            // Calculate dimensions for this level
            localparam LEVEL_WIDTH = (i == 0) ? (IMG_WIDTH * 5 / 6) :
                                     (i == 1) ? (IMG_WIDTH * 25 / 36) :
                                     (i == 2) ? (IMG_WIDTH * 125 / 216) : 
                                                (IMG_WIDTH * 625 / 1296);
                                                
            localparam LEVEL_HEIGHT = (i == 0) ? (IMG_HEIGHT * 5 / 6) :
                                      (i == 1) ? (IMG_HEIGHT * 25 / 36) :
                                      (i == 2) ? (IMG_HEIGHT * 125 / 216) :
                                                 (IMG_HEIGHT * 625 / 1296);
            
            scaling #(
                .PIXEL_WIDTH(PIXEL_WIDTH),
                .IMG_WIDTH((i == 0) ? IMG_WIDTH : LEVEL_WIDTH),
                .IMG_HEIGHT((i == 0) ? IMG_HEIGHT : LEVEL_HEIGHT)
            ) u_scaling (
                .clk(clk),
                .rst_n(rst_n),
                .pixel_valid(scale_valid_in[i]),
                .pixel_in(scale_pixel_in[i]),
                .scaled_valid(scale_valid_out[i]),
                .pixel_out(scale_pixel_out[i])
            );
            
            // Set the level dimensions
            always_comb begin
                level_widths[i+1] = LEVEL_WIDTH;
                level_heights[i+1] = LEVEL_HEIGHT;
            end
            
            // Connect outputs
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    level_valid[i+1] <= 1'b0;
                    level_pixels[i+1] <= '0;
                end else begin
                    level_valid[i+1] <= scale_valid_out[i];
                    level_pixels[i+1] <= scale_pixel_out[i];
                end
            end
        end
    endgenerate
    
    // Connect pyramid levels in series
    always_comb begin
        // Level 0 dimensions
        level_widths[0] = IMG_WIDTH;
        level_heights[0] = IMG_HEIGHT;
        
        // First scaling module gets original image
        scale_valid_in[0] = pixel_valid;
        scale_pixel_in[0] = pixel_in;
        
        // Connect subsequent levels
        for (int j = 1; j < PYRAMID_LEVELS-1; j++) begin
            scale_valid_in[j] = scale_valid_out[j-1];
            scale_pixel_in[j] = scale_pixel_out[j-1];
        end
    end

endmodule