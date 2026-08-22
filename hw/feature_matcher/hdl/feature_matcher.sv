//------------------------------------------------------------------------------
// File: feature_matcher.sv
// Description: Hardware feature matching unit.
//              Computes Hamming distance between 256-bit binary descriptors.
//              Matches descriptors if Hamming distance <= HAMMING_THRESHOLD.
//------------------------------------------------------------------------------

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

    typedef struct packed {
        logic [DESCRIPTOR_BITS-1:0] desc;
        logic [SCORE_WIDTH-1:0]     score;
        logic [COORD_WIDTH-1:0]     x;
        logic [COORD_WIDTH-1:0]     y;
        logic                       valid;
    } feature_entry_t;

    feature_entry_t feature_mem [0:MAX_FEATURES-1];
    logic [8:0] feat_count;

    // Feature storage logic
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            feat_count <= '0;
            for (int i = 0; i < MAX_FEATURES; i++) begin
                feature_mem[i] <= '0;
            end
        end else if (enable && store_valid) begin
            if (feat_count < MAX_FEATURES) begin
                feature_mem[feat_count].desc  <= store_descriptor;
                feature_mem[feat_count].score <= store_score;
                feature_mem[feat_count].x     <= store_x;
                feature_mem[feat_count].y     <= store_y;
                feature_mem[feat_count].valid <= 1'b1;
                feat_count <= feat_count + 1;
            end
        end
    end

    assign num_features = feat_count;

    // Function to calculate population count of XOR bits
    function automatic logic [7:0] popcount256(input logic [255:0] val);
        logic [7:0] cnt;
        cnt = '0;
        for (int b = 0; b < 256; b++) begin
            if (val[b]) cnt++;
        end
        return cnt;
    endfunction

    // Matching FSM
    typedef enum logic [1:0] {
        M_IDLE,
        M_SEARCH,
        M_OUTPUT
    } match_state_t;

    match_state_t m_state;
    logic [8:0]  search_idx;
    logic [7:0]  best_dist;
    logic [8:0]  best_idx;
    logic        best_valid;
    logic [DESCRIPTOR_BITS-1:0] query_desc_reg;
    logic [7:0]  cur_dist;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_state        <= M_IDLE;
            search_idx     <= '0;
            best_dist      <= 8'hFF;
            best_idx       <= '0;
            best_valid     <= 1'b0;
            query_desc_reg <= '0;
            cur_dist       <= '0;
            match_valid    <= 1'b0;
            matched_descriptor <= '0;
            matched_score  <= '0;
            matched_x      <= '0;
            matched_y      <= '0;
            hamming_distance <= '0;
        end else begin
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
                        if (feature_mem[search_idx].valid) begin
                            cur_dist = popcount256(query_desc_reg ^ feature_mem[search_idx].desc);
                            if (cur_dist < best_dist && cur_dist <= HAMMING_THRESHOLD) begin
                                best_dist  <= cur_dist;
                                best_idx   <= search_idx;
                                best_valid <= 1'b1;
                            end
                        end
                        search_idx <= search_idx + 1;
                    end else begin
                        m_state <= M_OUTPUT;
                    end
                end

                M_OUTPUT: begin
                    if (best_valid) begin
                        match_valid        <= 1'b1;
                        matched_descriptor <= feature_mem[best_idx].desc;
                        matched_score      <= feature_mem[best_idx].score;
                        matched_x          <= feature_mem[best_idx].x;
                        matched_y          <= feature_mem[best_idx].y;
                        hamming_distance   <= best_dist;
                    end else begin
                        match_valid <= 1'b0;
                    end
                    m_state <= M_IDLE;
                end
            endcase
        end
    end

    assign query_ready = (m_state == M_IDLE);

endmodule
