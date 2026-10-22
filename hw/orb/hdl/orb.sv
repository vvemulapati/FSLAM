// Multi-scale ORB: a 4-level pyramid feeds one feature_extractor per level, and
// a fixed-priority arbiter (level 0 first) merges their per-level output FIFOs.

`timescale 1ns / 1ps

module orb #(
    parameter int PIXEL_WIDTH            = 6,
    parameter int IMG_WIDTH              = 640,
    parameter int IMG_HEIGHT             = 480,
    parameter int PYRAMID_LEVELS         = 4,
    parameter int DESCRIPTOR_BITS        = 256,
    parameter int FAST_THRESHOLD         = 5,
    parameter int MAX_FEATURES_PER_LEVEL = 250
) (
    input  logic                                  clk,
    input  logic                                  rst_n,
    input  logic                                  enable,

    // Streaming input pixels
    input  logic                                  pixel_valid,
    input  logic [PIXEL_WIDTH-1:0]                pixel_in,

    // 256 BRIEF test pairs (bit_pattern_31_)
    input  logic signed [5:0]                     brief_pattern [0:DESCRIPTOR_BITS-1][0:3],

    // Downstream FIFO backpressure
    input  logic                                  feature_ready,

    // Output extracted feature stream
    output logic                                  feature_valid,
    output logic [DESCRIPTOR_BITS-1:0]            feature_descriptor,
    output logic [15:0]                           feature_score,
    output logic [11:0]                           feature_x,
    output logic [11:0]                           feature_y,
    output logic [5:0]                            feature_orientation,
    output logic [1:0]                            feature_level,

    output logic                                  processing,
    output logic [10:0]                           total_features
);

    // Image pyramid outputs
    logic [PYRAMID_LEVELS-1:0]          pyr_valid;
    logic [PIXEL_WIDTH-1:0]             pyr_pixels  [PYRAMID_LEVELS-1:0];
    logic [11:0]                        pyr_widths  [PYRAMID_LEVELS-1:0];
    logic [11:0]                        pyr_heights [PYRAMID_LEVELS-1:0];

    image_pyramid #(
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .PYRAMID_LEVELS(PYRAMID_LEVELS)
    ) u_pyramid (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_valid(pixel_valid && enable),
        .pixel_in(pixel_in),
        .level_valid(pyr_valid),
        .level_pixels(pyr_pixels),
        .level_widths(pyr_widths),
        .level_heights(pyr_heights)
    );

    // Per-level pipeline signals
    logic [PYRAMID_LEVELS-1:0]          lvl_feat_valid;
    logic [DESCRIPTOR_BITS-1:0]         lvl_feat_desc   [PYRAMID_LEVELS-1:0];
    logic [15:0]                        lvl_feat_score  [PYRAMID_LEVELS-1:0];
    logic [11:0]                        lvl_feat_x      [PYRAMID_LEVELS-1:0];
    logic [11:0]                        lvl_feat_y      [PYRAMID_LEVELS-1:0];
    logic [5:0]                         lvl_feat_ori    [PYRAMID_LEVELS-1:0];
    logic [PYRAMID_LEVELS-1:0]          lvl_busy;
    logic [PYRAMID_LEVELS-1:0]          lvl_fifo_empty;

    // Per-level output FIFO (prevents in-flight pipeline bursts from overwriting
    // features when multiple levels fire simultaneously or downstream stalls).
    localparam int LVL_FIFO_W = DESCRIPTOR_BITS + 16 + 12 + 12 + 6; // 302 bits
    logic [PYRAMID_LEVELS-1:0]          lvl_out_empty;
    logic [PYRAMID_LEVELS-1:0]          lvl_out_rd_en;
    logic [LVL_FIFO_W-1:0]              lvl_out_dout [PYRAMID_LEVELS-1:0];

    localparam int LEVEL_WIDTHS  [0:3] = '{640, 533, 444, 370};
    localparam int LEVEL_HEIGHTS [0:3] = '{480, 400, 333, 277};

    // One feature_extractor + output buffer FIFO per pyramid level
    genvar lvl;
    generate
        for (lvl = 0; lvl < PYRAMID_LEVELS; lvl++) begin : gen_feature_extractor
            feature_extractor #(
                .PIXEL_WIDTH(PIXEL_WIDTH),
                .IMG_WIDTH(LEVEL_WIDTHS[lvl]),
                .IMG_HEIGHT(LEVEL_HEIGHTS[lvl]),
                .DESCRIPTOR_BITS(DESCRIPTOR_BITS),
                .FAST_THRESHOLD(FAST_THRESHOLD),
                .FIFO_DEPTH(32),
                .ANGLE_BITS(6),
                .COORD_BITS(6)
            ) u_feature_extractor (
                .clk(clk),
                .rst_n(rst_n),
                .pixel_valid(pyr_valid[lvl]),
                .pixel_in(pyr_pixels[lvl]),
                .brief_pattern(brief_pattern),
                .feature_valid(lvl_feat_valid[lvl]),
                .feature_descriptor(lvl_feat_desc[lvl]),
                .feature_score(lvl_feat_score[lvl]),
                .feature_x(lvl_feat_x[lvl]),
                .feature_y(lvl_feat_y[lvl]),
                .feature_orientation(lvl_feat_ori[lvl]),
                .busy(lvl_busy[lvl]),
                .fifo_empty(lvl_fifo_empty[lvl])
            );

            descriptor_fifo #(
                .DATA_WIDTH(LVL_FIFO_W),
                .DEPTH(32),
                .ALMOST_FULL_THRESH(24)
            ) u_lvl_out_fifo (
                .clk(clk),
                .rst_n(rst_n),
                .wr_en(lvl_feat_valid[lvl]),
                .din({lvl_feat_desc[lvl], lvl_feat_score[lvl], lvl_feat_y[lvl], lvl_feat_x[lvl], lvl_feat_ori[lvl]}),
                .full(),
                .almost_full(),
                .overflow(),
                .rd_en(lvl_out_rd_en[lvl]),
                .dout(lvl_out_dout[lvl]),
                .empty(lvl_out_empty[lvl]),
                .almost_empty(),
                .underflow(),
                .count()
            );
        end
    endgenerate

    // Fixed-priority arbiter: lowest level with non-empty FIFO wins
    logic [1:0] arb_grant_idx;
    logic       arb_grant_valid;

    always_comb begin
        arb_grant_valid = 1'b0;
        arb_grant_idx   = 2'd0;
        for (int k = 0; k < PYRAMID_LEVELS; k++) begin
            if (!lvl_out_empty[k]) begin
                arb_grant_valid = 1'b1;
                arb_grant_idx   = 2'(k);
                break;
            end
        end
    end

    always_comb begin
        for (int k = 0; k < PYRAMID_LEVELS; k++) begin
            lvl_out_rd_en[k] = arb_grant_valid && feature_ready && (arb_grant_idx == 2'(k));
        end
    end

    // Register arbitrated feature output
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            feature_valid       <= 1'b0;
            feature_descriptor  <= '0;
            feature_score       <= '0;
            feature_x           <= '0;
            feature_y           <= '0;
            feature_orientation <= '0;
            feature_level       <= '0;
        end else if (arb_grant_valid && feature_ready) begin
            feature_valid       <= 1'b1;
            feature_level       <= arb_grant_idx;
            feature_descriptor  <= lvl_out_dout[arb_grant_idx][301:46];
            feature_score       <= lvl_out_dout[arb_grant_idx][45:30];
            feature_y           <= lvl_out_dout[arb_grant_idx][29:18];
            feature_x           <= lvl_out_dout[arb_grant_idx][17:6];
            feature_orientation <= lvl_out_dout[arb_grant_idx][5:0];
        end else begin
            feature_valid       <= 1'b0;
        end
    end

    // Feature count
    logic [10:0] count_reg;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            count_reg <= '0;
        end else if (feature_valid) begin
            count_reg <= count_reg + 1'b1;
        end
    end

    assign total_features = count_reg;
    assign processing     = (|lvl_busy) || (!(&lvl_fifo_empty)) || (!(&lvl_out_empty)) || (|lvl_feat_valid) || feature_valid;

endmodule
