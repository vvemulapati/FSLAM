// Hamming-distance matcher for 256-bit descriptors. A query matches a stored
// descriptor if the distance is <= HAMMING_THRESHOLD.

`timescale 1ns / 1ps

module feature_matcher #(
    parameter int DESCRIPTOR_BITS   = 256,
    parameter int SCORE_WIDTH       = 16,
    parameter int COORD_WIDTH       = 12,
    parameter int MAX_FEATURES      = 256,
    parameter int HAMMING_THRESHOLD = 50
) (
    input  logic                          clk,
    input  logic                          rst_n,
    input  logic                          enable,

    // Store new map feature into memory
    input  logic                          store_valid,
    input  logic [DESCRIPTOR_BITS-1:0]    store_descriptor,
    input  logic [SCORE_WIDTH-1:0]        store_score,
    input  logic [COORD_WIDTH-1:0]        store_x,
    input  logic [COORD_WIDTH-1:0]        store_y,

    // Query feature
    input  logic                          query_valid,
    input  logic [DESCRIPTOR_BITS-1:0]    query_descriptor,
    output logic                          query_ready,

    // Match output
    output logic                          match_valid,
    output logic [DESCRIPTOR_BITS-1:0]    matched_descriptor,
    output logic [SCORE_WIDTH-1:0]        matched_score,
    output logic [COORD_WIDTH-1:0]        matched_x,
    output logic [COORD_WIDTH-1:0]        matched_y,
    output logic [7:0]                    hamming_distance,
    output logic [8:0]                    num_features
);

    localparam int ADDR_WIDTH = (MAX_FEATURES > 1) ? $clog2(MAX_FEATURES) : 1;

    typedef struct packed {
        logic [DESCRIPTOR_BITS-1:0] desc;
        logic [SCORE_WIDTH-1:0]     score;
        logic [COORD_WIDTH-1:0]     x;
        logic [COORD_WIDTH-1:0]     y;
    } feature_entry_t;

    // Feature storage
    (* ram_style = "block" *)
    feature_entry_t feature_mem [0:MAX_FEATURES-1];
    feature_entry_t mem_rd_data;
    logic [ADDR_WIDTH-1:0] mem_rd_addr;
    logic [8:0] feat_count;

    always_ff @(posedge clk) begin
        if (enable && store_valid && (feat_count < 9'(MAX_FEATURES))) begin
            feature_mem[feat_count[ADDR_WIDTH-1:0]].desc  <= store_descriptor;
            feature_mem[feat_count[ADDR_WIDTH-1:0]].score <= store_score;
            feature_mem[feat_count[ADDR_WIDTH-1:0]].x     <= store_x;
            feature_mem[feat_count[ADDR_WIDTH-1:0]].y     <= store_y;
        end
        mem_rd_data <= feature_mem[mem_rd_addr];
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            feat_count <= '0;
        end else if (enable && store_valid && (feat_count < 9'(MAX_FEATURES))) begin
            feat_count <= feat_count + 9'd1;
        end
    end

    assign num_features = feat_count;

    // Population count
    function automatic logic [4:0] popcount16(input logic [15:0] val);
        logic [4:0] cnt;
        cnt = '0;
        for (int b = 0; b < 16; b++) begin
            cnt += {4'b0, val[b]};
        end
        return cnt;
    endfunction

    typedef enum logic [1:0] {
        M_IDLE,
        M_SEARCH,
        M_FETCH_BEST,
        M_OUTPUT
    } match_state_t;

    match_state_t m_state;
    logic [8:0]  search_idx;
    logic [7:0]  best_dist;
    logic [8:0]  best_idx;
    logic        best_valid;
    logic [DESCRIPTOR_BITS-1:0] query_desc_reg;

    // Matching pipeline
    logic        s1_valid, s2_valid;
    logic [8:0]  s1_idx,   s2_idx;
    logic [4:0]  s2_part_cnt [0:15];

    assign mem_rd_addr = (m_state == M_FETCH_BEST) ? best_idx[ADDR_WIDTH-1:0]
                                                   : search_idx[ADDR_WIDTH-1:0];

    wire [DESCRIPTOR_BITS-1:0] xor_bits = query_desc_reg ^ mem_rd_data.desc;

    logic [7:0] s3_dist;
    always_comb begin
        s3_dist = '0;
        for (int g = 0; g < 16; g++) begin
            s3_dist += {3'b0, s2_part_cnt[g]};
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_state            <= M_IDLE;
            search_idx         <= '0;
            best_dist          <= 8'hFF;
            best_idx           <= '0;
            best_valid         <= 1'b0;
            query_desc_reg     <= '0;
            s1_valid           <= 1'b0;
            s1_idx             <= '0;
            s2_valid           <= 1'b0;
            s2_idx             <= '0;
            for (int g = 0; g < 16; g++) begin
                s2_part_cnt[g] <= '0;
            end
            match_valid        <= 1'b0;
            matched_descriptor <= '0;
            matched_score      <= '0;
            matched_x          <= '0;
            matched_y          <= '0;
            hamming_distance   <= '0;
        end else begin
            s1_valid <= (m_state == M_SEARCH) && (search_idx < feat_count);
            s1_idx   <= search_idx;

            s2_valid <= s1_valid;
            s2_idx   <= s1_idx;
            if (s1_valid) begin
                for (int g = 0; g < 16; g++) begin
                    s2_part_cnt[g] <= popcount16(xor_bits[g*16 +: 16]);
                end
            end

            if (s2_valid) begin
                if ((s3_dist < best_dist) && (s3_dist <= 8'(HAMMING_THRESHOLD))) begin
                    best_dist  <= s3_dist;
                    best_idx   <= s2_idx;
                    best_valid <= 1'b1;
                end
            end

            case (m_state)
                M_IDLE: begin
                    match_valid <= 1'b0;
                    if (enable && query_valid && (feat_count > 0)) begin
                        m_state        <= M_SEARCH;
                        query_desc_reg <= query_descriptor;
                        search_idx     <= '0;
                        best_dist      <= 8'hFF;
                        best_valid     <= 1'b0;
                    end
                end

                M_SEARCH: begin
                    if (search_idx < feat_count) begin
                        search_idx <= search_idx + 9'd1;
                    end else if (!s1_valid && !s2_valid) begin
                        m_state <= M_FETCH_BEST;
                    end
                end

                M_FETCH_BEST: begin
                    m_state <= M_OUTPUT;
                end

                M_OUTPUT: begin
                    if (best_valid) begin
                        match_valid        <= 1'b1;
                        matched_descriptor <= mem_rd_data.desc;
                        matched_score      <= mem_rd_data.score;
                        matched_x          <= mem_rd_data.x;
                        matched_y          <= mem_rd_data.y;
                        hamming_distance   <= best_dist;
                    end else begin
                        match_valid <= 1'b0;
                    end
                    m_state <= M_IDLE;
                end

                default: m_state <= M_IDLE;
            endcase
        end
    end

    assign query_ready = (m_state == M_IDLE);

endmodule
