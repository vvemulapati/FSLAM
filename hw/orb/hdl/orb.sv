//------------------------------------------------------------------------------
// File: orb.sv
// Description: Multi-scale ORB feature extraction top module.
//              Integrates:
//                1. 4-level image pyramid (1.2x scale factor)
//                2. Parallel per-level extraction pipelines (level_pipeline)
//                3. Multi-level round-robin / priority output arbiter
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module orb #(
    parameter int PIXEL_WIDTH            = 6,
    parameter int IMG_WIDTH              = 640,
    parameter int IMG_HEIGHT             = 480,
    parameter int PYRAMID_LEVELS         = 4,
    parameter int DESCRIPTOR_BITS        = 256,
    parameter int MAX_FEATURES_PER_LEVEL = 250
) (
    input  logic                                  clk,
    input  logic                                  rst_n,
    input  logic                                  enable,

    // Streaming input pixels
    input  logic                                  pixel_valid,
    input  logic [PIXEL_WIDTH-1:0]                pixel_in,

    // Pre-computed 256 BRIEF test coordinate pairs in 27x27 patch
    input  logic signed [5:0]                     brief_pattern [0:DESCRIPTOR_BITS-1][0:3],

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

    localparam int LEVEL_WIDTHS  [0:3] = '{640, 533, 444, 370};
    localparam int LEVEL_HEIGHTS [0:3] = '{480, 400, 333, 278};

    // Parallel Feature Extractor Instantiation (one per pyramid level)
    genvar lvl;
    generate
        for (lvl = 0; lvl < PYRAMID_LEVELS; lvl++) begin : gen_feature_extractor
            feature_extractor #(
                .PIXEL_WIDTH(PIXEL_WIDTH),
                .IMG_WIDTH(LEVEL_WIDTHS[lvl]),
                .IMG_HEIGHT(LEVEL_HEIGHTS[lvl]),
                .DESCRIPTOR_BITS(DESCRIPTOR_BITS),
                .FAST_THRESHOLD(20),
                .FIFO_DEPTH(64),
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
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Per-Level Feature Holding Skid Registers:
    // Captures 1-cycle pulses from parallel extractors to prevent collision loss
    // -------------------------------------------------------------------------
    logic [PYRAMID_LEVELS-1:0]          hold_valid;
    logic [DESCRIPTOR_BITS-1:0]         hold_desc   [PYRAMID_LEVELS-1:0];
    logic [15:0]                        hold_score  [PYRAMID_LEVELS-1:0];
    logic [11:0]                        hold_x      [PYRAMID_LEVELS-1:0];
    logic [11:0]                        hold_y      [PYRAMID_LEVELS-1:0];
    logic [5:0]                         hold_ori    [PYRAMID_LEVELS-1:0];

    // Priority Grant Arbiter across active held features
    logic [1:0] arb_grant_idx;
    logic       arb_grant_valid;

    always_comb begin
        arb_grant_valid = 1'b0;
        arb_grant_idx   = 2'd0;
        for (int k = 0; k < PYRAMID_LEVELS; k++) begin
            if (hold_valid[k] || lvl_feat_valid[k]) begin
                arb_grant_valid = 1'b1;
                arb_grant_idx   = 2'(k);
                break;
            end
        end
    end

    // Capture incoming features & Drain granted features
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hold_valid          <= '0;
            feature_valid       <= 1'b0;
            feature_descriptor  <= '0;
            feature_score       <= '0;
            feature_x           <= '0;
            feature_y           <= '0;
            feature_orientation <= '0;
            feature_level       <= '0;
            for (int k = 0; k < PYRAMID_LEVELS; k++) begin
                hold_desc[k]  <= '0;
                hold_score[k] <= '0;
                hold_x[k]     <= '0;
                hold_y[k]     <= '0;
                hold_ori[k]   <= '0;
            end
        end else begin
            // 1. Capture incoming features from parallel extractors into holding registers
            for (int k = 0; k < PYRAMID_LEVELS; k++) begin
                if (lvl_feat_valid[k]) begin
                    hold_valid[k] <= 1'b1;
                    hold_desc[k]  <= lvl_feat_desc[k];
                    hold_score[k] <= lvl_feat_score[k];
                    hold_x[k]     <= lvl_feat_x[k];
                    hold_y[k]     <= lvl_feat_y[k];
                    hold_ori[k]   <= lvl_feat_ori[k];
                end
            end

            // 2. Drain 1 granted feature per cycle onto the output stream
            if (arb_grant_valid) begin
                feature_valid <= 1'b1;
                feature_level <= arb_grant_idx;

                if (hold_valid[arb_grant_idx]) begin
                    feature_descriptor  <= hold_desc[arb_grant_idx];
                    feature_score       <= hold_score[arb_grant_idx];
                    feature_x           <= hold_x[arb_grant_idx];
                    feature_y           <= hold_y[arb_grant_idx];
                    feature_orientation <= hold_ori[arb_grant_idx];
                end else begin
                    // Direct pass-through if arriving on current cycle
                    feature_descriptor  <= lvl_feat_desc[arb_grant_idx];
                    feature_score       <= lvl_feat_score[arb_grant_idx];
                    feature_x           <= lvl_feat_x[arb_grant_idx];
                    feature_y           <= lvl_feat_y[arb_grant_idx];
                    feature_orientation <= lvl_feat_ori[arb_grant_idx];
                end

                // Clear the granted holding register (unless reloaded on same cycle)
                if (!lvl_feat_valid[arb_grant_idx]) begin
                    hold_valid[arb_grant_idx] <= 1'b0;
                end
            end else begin
                feature_valid <= 1'b0;
            end
        end
    end

    // Total feature counting
    logic [10:0] count_reg;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count_reg <= '0;
        end else if (feature_valid) begin
            count_reg <= count_reg + 1'b1;
        end
    end

    assign total_features = count_reg;
    assign processing     = (|lvl_busy) || (!(&lvl_fifo_empty)) || (|hold_valid);

endmodule
