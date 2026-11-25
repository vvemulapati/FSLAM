// Multi-level image pyramid, 5/6 scale per level (640x480 -> 533x400 -> 444x333 -> 370x277
// for default 640x480 input).

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

    function automatic int calc_level_dim(input int base_dim, input int lvl_idx);
        int d;
        d = base_dim;
        for (int s = 0; s < lvl_idx; s++) begin
            d = (d * 5) / 6;
        end
        return d;
    endfunction

    // Level 0: direct passthrough
    assign level_valid[0]   = pixel_valid;
    assign level_pixels[0]  = pixel_in;
    assign level_widths[0]  = 12'(IMG_WIDTH);
    assign level_heights[0] = 12'(IMG_HEIGHT);

    // Cascaded 5/6 scaling for levels 1..PYRAMID_LEVELS-1
    genvar i;
    generate
        for (i = 0; i < PYRAMID_LEVELS - 1; i++) begin : gen_scaling
            localparam int IN_W  = calc_level_dim(IMG_WIDTH, i);
            localparam int IN_H  = calc_level_dim(IMG_HEIGHT, i);
            localparam int OUT_W = calc_level_dim(IMG_WIDTH, i + 1);
            localparam int OUT_H = calc_level_dim(IMG_HEIGHT, i + 1);

            assign level_widths[i+1]  = 12'(OUT_W);
            assign level_heights[i+1] = 12'(OUT_H);

            scaling #(
                .PIXEL_WIDTH(PIXEL_WIDTH),
                .IMG_WIDTH(IN_W),
                .IMG_HEIGHT(IN_H)
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
