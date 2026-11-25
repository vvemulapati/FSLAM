// Multi-scale ORB: an N-level pyramid feeds one feature_extractor per level, and
// a fixed-priority arbiter (level 0 first) merges their outputs.

`timescale 1ns / 1ps

module orb #(
    parameter int PIXEL_WIDTH     = 6,
    parameter int IMG_WIDTH       = 640,
    parameter int IMG_HEIGHT      = 480,
    parameter int PYRAMID_LEVELS  = 4,
    parameter int DESCRIPTOR_BITS = 256,
    parameter int FAST_THRESHOLD  = 5,
    parameter int FIFO_DEPTH      = 512
) (
    input  logic                                  clk,
    input  logic                                  rst_n,
    input  logic                                  enable,

    input  logic                                  pixel_valid,
    input  logic [PIXEL_WIDTH-1:0]                pixel_in,

    input  logic signed [5:0]                     brief_pattern [0:DESCRIPTOR_BITS-1][0:3],

    input  logic                                  feature_ready,

    output logic                                  feature_valid,
    output logic [DESCRIPTOR_BITS-1:0]            feature_descriptor,
    output logic [15:0]                           feature_score,
    output logic [11:0]                           feature_x,
    output logic [11:0]                           feature_y,
    output logic [5:0]                            feature_orientation,
    output logic [1:0]                            feature_level,

    output logic                                  stall_req,
    output logic                                  processing,
    output logic [15:0]                           total_features
);

    function automatic int calc_level_dim(input int base_dim, input int lvl_idx);
        int d;
        d = base_dim;
        for (int s = 0; s < lvl_idx; s++) begin
            d = (d * 5) / 6;
        end
        return d;
    endfunction

    localparam int LEVEL_BITS = (PYRAMID_LEVELS > 1) ? $clog2(PYRAMID_LEVELS) : 1;

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

    localparam int LVL_FIFO_W = DESCRIPTOR_BITS + 16 + 12 + 12 + 6; // 302 bits

    logic [PYRAMID_LEVELS-1:0]          lvl_feat_valid;
    logic [DESCRIPTOR_BITS-1:0]         lvl_feat_desc  [PYRAMID_LEVELS-1:0];
    logic [15:0]                        lvl_feat_score [PYRAMID_LEVELS-1:0];
    logic [11:0]                        lvl_feat_x     [PYRAMID_LEVELS-1:0];
    logic [11:0]                        lvl_feat_y     [PYRAMID_LEVELS-1:0];
    logic [5:0]                         lvl_feat_ori   [PYRAMID_LEVELS-1:0];
    logic [PYRAMID_LEVELS-1:0]          lvl_stall;
    logic [PYRAMID_LEVELS-1:0]          lvl_busy;
    logic [PYRAMID_LEVELS-1:0]          lvl_fifo_empty;

    logic [PYRAMID_LEVELS-1:0]          lvl_out_empty;
    logic [PYRAMID_LEVELS-1:0]          lvl_out_afull;
    logic [PYRAMID_LEVELS-1:0]          lvl_out_has_data;
    logic [PYRAMID_LEVELS-1:0]          lvl_out_rd_en;
    logic [LVL_FIFO_W-1:0]              lvl_out_dout [PYRAMID_LEVELS-1:0];
    logic [5:0]                         lvl_out_cnt  [PYRAMID_LEVELS-1:0];

    genvar lvl;
    generate
        for (lvl = 0; lvl < PYRAMID_LEVELS; lvl++) begin : gen_feature_extractor
            localparam int LVL_W = calc_level_dim(IMG_WIDTH, lvl);
            localparam int LVL_H = calc_level_dim(IMG_HEIGHT, lvl);

            feature_extractor #(
                .PIXEL_WIDTH(PIXEL_WIDTH),
                .IMG_WIDTH(LVL_W),
                .IMG_HEIGHT(LVL_H),
                .DESCRIPTOR_BITS(DESCRIPTOR_BITS),
                .FAST_THRESHOLD(FAST_THRESHOLD),
                .FIFO_DEPTH(FIFO_DEPTH),
                .ANGLE_BITS(6),
                .COORD_BITS(6)
            ) u_extractor (
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
                .stall_req(lvl_stall[lvl]),
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
                .almost_full(lvl_out_afull[lvl]),
                .overflow(),
                .rd_en(lvl_out_rd_en[lvl]),
                .dout(lvl_out_dout[lvl]),
                .empty(lvl_out_empty[lvl]),
                .almost_empty(),
                .underflow(),
                .count(lvl_out_cnt[lvl])
            );

            assign lvl_out_has_data[lvl] = !lvl_out_empty[lvl] || (lvl_out_cnt[lvl] != '0);
        end
    endgenerate

    // Fixed-priority arbiter: lowest level wins
    logic [LEVEL_BITS-1:0] arb_grant_idx;
    logic                  arb_grant_valid;
    wire                   out_stage_ready = !feature_valid || feature_ready;

    always_comb begin
        arb_grant_valid = 1'b0;
        arb_grant_idx   = '0;
        for (int k = 0; k < PYRAMID_LEVELS; k++) begin
            if (!lvl_out_empty[k]) begin
                arb_grant_valid = 1'b1;
                arb_grant_idx   = LEVEL_BITS'(k);
                break;
            end
        end
    end

    always_comb begin
        for (int k = 0; k < PYRAMID_LEVELS; k++) begin
            lvl_out_rd_en[k] = arb_grant_valid && out_stage_ready && (arb_grant_idx == LEVEL_BITS'(k));
        end
    end

    // Hold output valid while downstream FIFO is full
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            feature_valid       <= 1'b0;
            feature_descriptor  <= '0;
            feature_score       <= '0;
            feature_x           <= '0;
            feature_y           <= '0;
            feature_orientation <= '0;
            feature_level       <= '0;
        end else if (out_stage_ready) begin
            if (arb_grant_valid) begin
                feature_valid       <= 1'b1;
                feature_level       <= 2'(arb_grant_idx);
                feature_descriptor  <= lvl_out_dout[arb_grant_idx][301:46];
                feature_score       <= lvl_out_dout[arb_grant_idx][45:30];
                feature_y           <= lvl_out_dout[arb_grant_idx][29:18];
                feature_x           <= lvl_out_dout[arb_grant_idx][17:6];
                feature_orientation <= lvl_out_dout[arb_grant_idx][5:0];
            end else begin
                feature_valid       <= 1'b0;
            end
        end
    end

    logic [15:0] count_reg;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            count_reg <= '0;
        end else if (feature_valid && feature_ready) begin
            count_reg <= count_reg + 16'd1;
        end
    end

    assign stall_req      = (|lvl_stall) || (|lvl_out_afull);
    assign total_features = count_reg;
    assign processing     = (|lvl_busy) || (!(&lvl_fifo_empty)) || (|lvl_out_has_data) ||
                            (|lvl_feat_valid) || feature_valid;

endmodule
