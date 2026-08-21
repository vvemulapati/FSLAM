//------------------------------------------------------------------------------
// File: image_pyramid.sv
// Description: Multi-level image pyramid generator with 1.2x downscaling (5/6 factor).
//              Level 0: 640x480 (Original)
//              Level 1: 533x400
//              Level 2: 444x333
//              Level 3: 370x278
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module image_pyramid #(
    parameter int PIXEL_WIDTH    = 6,
    parameter int IMG_WIDTH      = 640,
    parameter int IMG_HEIGHT     = 480,
    parameter int PYRAMID_LEVELS = 4
) (
    input  logic                                clk,
    input  logic                                rst_n,

    input  logic                                pixel_valid,
    input  logic [PIXEL_WIDTH-1:0]              pixel_in,

    output logic [PYRAMID_LEVELS-1:0]           level_valid,
    output logic [PIXEL_WIDTH-1:0]              level_pixels  [PYRAMID_LEVELS-1:0],
    output logic [11:0]                         level_widths  [PYRAMID_LEVELS-1:0],
    output logic [11:0]                         level_heights [PYRAMID_LEVELS-1:0]
);

    // Dimensions for each level
    localparam int WIDTHS  [0:3] = '{640, 533, 444, 370};
    localparam int HEIGHTS [0:3] = '{480, 400, 333, 278};

    // Level 0: direct passthrough
    assign level_valid[0]   = pixel_valid;
    assign level_pixels[0]  = pixel_in;
    assign level_widths[0]  = 12'(WIDTHS[0]);
    assign level_heights[0] = 12'(HEIGHTS[0]);

    // Cascaded scaling for levels 1, 2, 3
    genvar i;
    generate
        for (i = 0; i < PYRAMID_LEVELS-1; i++) begin : gen_scaling
            assign level_widths[i+1]  = 12'(WIDTHS[i+1]);
            assign level_heights[i+1] = 12'(HEIGHTS[i+1]);

            scaling #(
                .PIXEL_WIDTH(PIXEL_WIDTH),
                .IMG_WIDTH(WIDTHS[i]),
                .IMG_HEIGHT(HEIGHTS[i])
            ) u_scaling (
                .clk(clk),
                .rst_n(rst_n),
                .pixel_valid(level_valid[i]),
                .pixel_in(level_pixels[i]),
                .scaled_valid(level_valid[i+1]),
                .pixel_out(level_pixels[i+1])
            );
        end
    endgenerate

endmodule
