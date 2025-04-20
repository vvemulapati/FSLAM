/*
 * Feature Matcher Module
 * 
 * Matches ORB descriptors using Hamming distance.
 * Maintains a heap of keypoints sorted by Harris scores.
 * Implements nearest neighbor matching with distance threshold.
 */

module feature_matcher #(
    parameter DESCRIPTOR_BITS = 256,
    parameter SCORE_WIDTH = 16,
    parameter COORD_WIDTH = 12,
    parameter MAX_FEATURES = 1000,
    parameter HAMMING_THRESHOLD = 50
) (
    input  logic                            clk,
    input  logic                            rst_n,
    input  logic                            enable,
    
    // New feature input
    input  logic                            new_feature_valid,
    input  logic [DESCRIPTOR_BITS-1:0]     new_descriptor,
    input  logic [SCORE_WIDTH-1:0]         new_score,
    input  logic [COORD_WIDTH-1:0]         new_x,
    input  logic [COORD_WIDTH-1:0]         new_y,
    
    // Query feature for matching
    input  logic                            query_valid,
    input  logic [DESCRIPTOR_BITS-1:0]     query_descriptor,
    
    // Match results
    output logic                            match_valid,
    output logic [DESCRIPTOR_BITS-1:0]     matched_descriptor,
    output logic [SCORE_WIDTH-1:0]         matched_score,
    output logic [COORD_WIDTH-1:0]         matched_x,
    output logic [COORD_WIDTH-1:0]         matched_y,
    output logic [7:0]                      hamming_distance,
    
    // Status
    output logic                            heap_full,
    output logic [10:0]                     num_features
);

    // Feature storage (heap sorted by score)
    typedef struct packed {
        logic [DESCRIPTOR_BITS-1:0] descriptor;
        logic [SCORE_WIDTH-1:0]     score;
        logic [COORD_WIDTH-1:0]     x;
        logic [COORD_WIDTH-1:0]     y;
        logic                       valid;
    } feature_t;
    
    feature_t feature_heap [MAX_FEATURES-1:0];
    logic [10:0] heap_size;
    
    // Hamming distance calculation
    logic [DESCRIPTOR_BITS-1:0] xor_result;
    logic [7:0] hamming_dist;
    logic [10:0] match_index;
    logic match_found;
    
    // Best match tracking
    logic [7:0] best_distance;
    logic [10:0] best_index;
    logic best_valid;
    
    // State machine for matching
    typedef enum logic [2:0] {
        IDLE,
        SEARCH,
        OUTPUT_MATCH,
        DONE
    } match_state_t;
    
    match_state_t match_state;
    logic [10:0] search_index;
    
    // Heap management
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            heap_size <= '0;
            for (int i = 0; i < MAX_FEATURES; i++) begin
                feature_heap[i] <= '0;
            end
        end else if (enable && new_feature_valid) begin
            // Add new feature to heap
            if (heap_size < MAX_FEATURES) begin
                // Simple insertion - insert at end and bubble up
                feature_heap[heap_size].descriptor <= new_descriptor;
                feature_heap[heap_size].score <= new_score;
                feature_heap[heap_size].x <= new_x;
                feature_heap[heap_size].y <= new_y;
                feature_heap[heap_size].valid <= 1'b1;
                heap_size <= heap_size + 1;
            end else begin
                // Heap is full - replace lowest score if new score is higher
                if (new_score > feature_heap[MAX_FEATURES-1].score) begin
                    feature_heap[MAX_FEATURES-1].descriptor <= new_descriptor;
                    feature_heap[MAX_FEATURES-1].score <= new_score;
                    feature_heap[MAX_FEATURES-1].x <= new_x;
                    feature_heap[MAX_FEATURES-1].y <= new_y;
                    feature_heap[MAX_FEATURES-1].valid <= 1'b1;
                end
            end
        end
    end
    
    // Hamming distance calculation
    function automatic logic [7:0] calc_hamming_distance(
        input logic [DESCRIPTOR_BITS-1:0] desc1,
        input logic [DESCRIPTOR_BITS-1:0] desc2
    );
        logic [DESCRIPTOR_BITS-1:0] xor_bits;
        logic [7:0] count;
        
        xor_bits = desc1 ^ desc2;
        count = 0;
        
        for (int i = 0; i < DESCRIPTOR_BITS; i++) begin
            if (xor_bits[i]) count++;
        end
        
        return count;
    endfunction
    
    // Match state machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            match_state <= IDLE;
            search_index <= '0;
            best_distance <= 8'hFF;
            best_index <= '0;
            best_valid <= 1'b0;
        end else begin
            case (match_state)
                IDLE: begin
                    if (enable && query_valid) begin
                        match_state <= SEARCH;
                        search_index <= '0;
                        best_distance <= 8'hFF;
                        best_valid <= 1'b0;
                    end
                end
                
                SEARCH: begin
                    if (search_index < heap_size) begin
                        // Calculate Hamming distance for current feature
                        if (feature_heap[search_index].valid) begin
                            hamming_dist = calc_hamming_distance(
                                query_descriptor, 
                                feature_heap[search_index].descriptor
                            );
                            
                            // Update best match if this is better
                            if (hamming_dist < best_distance && 
                                hamming_dist <= HAMMING_THRESHOLD) begin
                                best_distance <= hamming_dist;
                                best_index <= search_index;
                                best_valid <= 1'b1;
                            end
                        end
                        
                        search_index <= search_index + 1;
                    end else begin
                        match_state <= OUTPUT_MATCH;
                    end
                end
                
                OUTPUT_MATCH: begin
                    match_state <= DONE;
                end
                
                DONE: begin
                    match_state <= IDLE;
                end
            endcase
        end
    end
    
    // Output logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            match_valid <= 1'b0;
            matched_descriptor <= '0;
            matched_score <= '0;
            matched_x <= '0;
            matched_y <= '0;
            hamming_distance <= '0;
        end else if (match_state == OUTPUT_MATCH && best_valid) begin
            match_valid <= 1'b1;
            matched_descriptor <= feature_heap[best_index].descriptor;
            matched_score <= feature_heap[best_index].score;
            matched_x <= feature_heap[best_index].x;
            matched_y <= feature_heap[best_index].y;
            hamming_distance <= best_distance;
        end else begin
            match_valid <= 1'b0;
        end
    end
    
    // Status outputs
    always_comb begin
        heap_full = (heap_size >= MAX_FEATURES);
        num_features = heap_size;
    end

endmodule