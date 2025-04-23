/*
 * Hardware Top-Level Module
 * 
 * Top-level integration of the ORB-SLAM accelerator system.
 * Interfaces with CPU via AXI-Stream and DMA.
 * Integrates ORB feature extraction and matching pipeline.
 */

module hw_top #(
    parameter PIXEL_WIDTH = 6,
    parameter IMG_WIDTH = 640,
    parameter IMG_HEIGHT = 480,
    parameter DESCRIPTOR_BITS = 256,
    parameter AXI_DATA_WIDTH = 32,
    parameter AXI_ADDR_WIDTH = 32
) (
    // Clock and Reset
    input  logic                        aclk,
    input  logic                        aresetn,
    
    // AXI4-Stream Slave (Input image data from DMA)
    input  logic [AXI_DATA_WIDTH-1:0]  s_axis_tdata,
    input  logic                        s_axis_tvalid,
    input  logic                        s_axis_tlast,
    output logic                        s_axis_tready,
    
    // AXI4-Stream Master (Output feature data to DMA)
    output logic [AXI_DATA_WIDTH-1:0]  m_axis_tdata,
    output logic                        m_axis_tvalid,
    output logic                        m_axis_tlast,
    input  logic                        m_axis_tready,
    
    // Control and Status Registers (AXI4-Lite interface)
    input  logic [AXI_ADDR_WIDTH-1:0]  s_axi_awaddr,
    input  logic                        s_axi_awvalid,
    output logic                        s_axi_awready,
    input  logic [AXI_DATA_WIDTH-1:0]  s_axi_wdata,
    input  logic [3:0]                  s_axi_wstrb,
    input  logic                        s_axi_wvalid,
    output logic                        s_axi_wready,
    output logic [1:0]                  s_axi_bresp,
    output logic                        s_axi_bvalid,
    input  logic                        s_axi_bready,
    input  logic [AXI_ADDR_WIDTH-1:0]  s_axi_araddr,
    input  logic                        s_axi_arvalid,
    output logic                        s_axi_arready,
    output logic [AXI_DATA_WIDTH-1:0]  s_axi_rdata,
    output logic [1:0]                  s_axi_rresp,
    output logic                        s_axi_rvalid,
    input  logic                        s_axi_rready,
    
    // Interrupt
    output logic                        interrupt
);

    // Internal signals
    logic enable;
    logic [31:0] control_reg, status_reg;
    logic frame_start, frame_end;
    
    // Pixel stream interface
    logic pixel_valid;
    logic [PIXEL_WIDTH-1:0] pixel_data;
    logic [11:0] pixel_x, pixel_y;
    
    // ORB feature extraction outputs
    logic feature_valid;
    logic [DESCRIPTOR_BITS-1:0] feature_descriptor;
    logic [15:0] feature_score;
    logic [11:0] feature_x, feature_y;
    logic [5:0] feature_orientation;
    logic [1:0] feature_level;
    
    // Feature matching interface
    logic match_valid;
    logic [DESCRIPTOR_BITS-1:0] matched_descriptor;
    logic [15:0] matched_score;
    logic [11:0] matched_x, matched_y;
    logic [7:0] hamming_distance;
    
    // Status signals
    logic orb_processing;
    logic [10:0] total_features;
    logic matcher_heap_full;
    logic [10:0] num_matched_features;
    
    // FIFO interfaces
    logic input_fifo_full, input_fifo_empty;
    logic output_fifo_full, output_fifo_empty;
    logic input_fifo_rd_en, output_fifo_wr_en;
    
    // BRIEF pattern storage (loaded via AXI)
    logic signed [5:0] brief_pattern [DESCRIPTOR_BITS-1:0][3:0];
    
    // Input pixel stream processing
    logic [AXI_DATA_WIDTH-1:0] input_fifo_dout;
    logic input_fifo_valid;
    
    // Pixel unpacking (assuming 4 pixels per AXI word for 6-bit pixels)
    logic [1:0] pixel_select;
    
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            pixel_x <= '0;
            pixel_y <= '0;
            pixel_select <= '0;
        end else if (pixel_valid) begin
            if (pixel_select == 3) begin
                pixel_select <= '0;
                if (pixel_x == IMG_WIDTH - 1) begin
                    pixel_x <= '0;
                    pixel_y <= pixel_y + 1;
                end else begin
                    pixel_x <= pixel_x + 1;
                end
            end else begin
                pixel_select <= pixel_select + 1;
            end
        end
    end
    
    // Pixel extraction from AXI data
    always_comb begin
        case (pixel_select)
            2'b00: pixel_data = input_fifo_dout[5:0];
            2'b01: pixel_data = input_fifo_dout[11:6];
            2'b10: pixel_data = input_fifo_dout[17:12];
            2'b11: pixel_data = input_fifo_dout[23:18];
        endcase
        
        pixel_valid = input_fifo_valid && enable;
        input_fifo_rd_en = pixel_valid && (pixel_select == 3 || 
                          (pixel_x == IMG_WIDTH - 1 && pixel_y == IMG_HEIGHT - 1));
    end
    
    // Frame boundary detection
    always_comb begin
        frame_start = (pixel_x == 0) && (pixel_y == 0) && pixel_valid;
        frame_end = (pixel_x == IMG_WIDTH - 1) && (pixel_y == IMG_HEIGHT - 1) && pixel_valid;
    end
    
    // Input FIFO for image data
    fifo_32x512 u_input_fifo (
        .clk(aclk),
        .rst(~aresetn),
        .din(s_axis_tdata),
        .wr_en(s_axis_tvalid && s_axis_tready),
        .rd_en(input_fifo_rd_en),
        .dout(input_fifo_dout),
        .full(input_fifo_full),
        .empty(input_fifo_empty),
        .valid(input_fifo_valid)
    );
    
    assign s_axis_tready = ~input_fifo_full;
    
    // ORB feature extraction pipeline
    orb #(
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .PYRAMID_LEVELS(4),
        .DESCRIPTOR_BITS(DESCRIPTOR_BITS),
        .MAX_FEATURES_PER_LEVEL(250)
    ) u_orb (
        .clk(aclk),
        .rst_n(aresetn),
        .enable(enable),
        .pixel_valid(pixel_valid),
        .pixel_in(pixel_data),
        .brief_pattern(brief_pattern),
        .feature_valid(feature_valid),
        .feature_descriptor(feature_descriptor),
        .feature_score(feature_score),
        .feature_x(feature_x),
        .feature_y(feature_y),
        .feature_orientation(feature_orientation),
        .feature_level(feature_level),
        .processing(orb_processing),
        .total_features(total_features)
    );
    
    // Feature matcher
    feature_matcher #(
        .DESCRIPTOR_BITS(DESCRIPTOR_BITS),
        .SCORE_WIDTH(16),
        .COORD_WIDTH(12),
        .MAX_FEATURES(1000),
        .HAMMING_THRESHOLD(50)
    ) u_matcher (
        .clk(aclk),
        .rst_n(aresetn),
        .enable(enable),
        .new_feature_valid(feature_valid),
        .new_descriptor(feature_descriptor),
        .new_score(feature_score),
        .new_x(feature_x),
        .new_y(feature_y),
        .query_valid(feature_valid), // For now, query with new features
        .query_descriptor(feature_descriptor),
        .match_valid(match_valid),
        .matched_descriptor(matched_descriptor),
        .matched_score(matched_score),
        .matched_x(matched_x),
        .matched_y(matched_y),
        .hamming_distance(hamming_distance),
        .heap_full(matcher_heap_full),
        .num_features(num_matched_features)
    );
    
    // Output feature packing
    logic [AXI_DATA_WIDTH-1:0] output_data;
    logic output_data_valid;
    
    // Pack feature data for output (simplified - multiple words per feature)
    always_comb begin
        // Pack descriptor, coordinates, score, etc. into output words
        // This is a simplified version - actual implementation would need
        // multiple cycles per feature to send all data
        output_data = {feature_level, feature_orientation, 
                      feature_y[7:0], feature_x[7:0], feature_score[7:0]};
        output_data_valid = feature_valid;
        output_fifo_wr_en = output_data_valid && ~output_fifo_full;
    end
    
    // Output FIFO for feature data
    fifo_32x512 u_output_fifo (
        .clk(aclk),
        .rst(~aresetn),
        .din(output_data),
        .wr_en(output_fifo_wr_en),
        .rd_en(m_axis_tready && m_axis_tvalid),
        .dout(m_axis_tdata),
        .full(output_fifo_full),
        .empty(output_fifo_empty),
        .valid(m_axis_tvalid)
    );
    
    assign m_axis_tlast = output_fifo_empty; // Simplified
    
    // Control and Status Registers
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            control_reg <= '0;
            enable <= 1'b0;
            // Initialize BRIEF patterns to default values
            for (int i = 0; i < DESCRIPTOR_BITS; i++) begin
                for (int j = 0; j < 4; j++) begin
                    brief_pattern[i][j] <= $random; // Placeholder
                end
            end
        end else begin
            // AXI register interface implementation
            // Simplified - actual implementation would handle full AXI protocol
            if (s_axi_awvalid && s_axi_wvalid) begin
                case (s_axi_awaddr[7:0])
                    8'h00: control_reg <= s_axi_wdata;
                    // Add more register addresses for BRIEF patterns
                endcase
            end
            
            enable <= control_reg[0];
        end
    end
    
    // Status register
    always_comb begin
        status_reg[0] = orb_processing;
        status_reg[1] = matcher_heap_full;
        status_reg[2] = input_fifo_empty;
        status_reg[3] = output_fifo_full;
        status_reg[15:4] = total_features;
        status_reg[26:16] = num_matched_features;
        status_reg[31:27] = '0;
    end
    
    // AXI read interface (simplified)
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            s_axi_rdata <= '0;
            s_axi_rvalid <= 1'b0;
        end else if (s_axi_arvalid) begin
            case (s_axi_araddr[7:0])
                8'h00: s_axi_rdata <= control_reg;
                8'h04: s_axi_rdata <= status_reg;
                default: s_axi_rdata <= '0;
            endcase
            s_axi_rvalid <= 1'b1;
        end else if (s_axi_rready) begin
            s_axi_rvalid <= 1'b0;
        end
    end
    
    // AXI handshaking (simplified)
    assign s_axi_awready = 1'b1;
    assign s_axi_wready = 1'b1;
    assign s_axi_bvalid = s_axi_wvalid;
    assign s_axi_bresp = 2'b00;
    assign s_axi_arready = 1'b1;
    assign s_axi_rresp = 2'b00;
    
    // Interrupt generation
    assign interrupt = frame_end || matcher_heap_full;

endmodule

// Simple FIFO module (placeholder - use vendor IP in real implementation)
module fifo_32x512 (
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] din,
    input  logic        wr_en,
    input  logic        rd_en,
    output logic [31:0] dout,
    output logic        full,
    output logic        empty,
    output logic        valid
);
    
    logic [31:0] memory [511:0];
    logic [8:0] wr_ptr, rd_ptr;
    logic [9:0] count;
    
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
            count <= '0;
            valid <= 1'b0;
        end else begin
            if (wr_en && !full) begin
                memory[wr_ptr] <= din;
                wr_ptr <= wr_ptr + 1;
                count <= count + 1;
            end
            
            if (rd_en && !empty) begin
                dout <= memory[rd_ptr];
                rd_ptr <= rd_ptr + 1;
                count <= count - 1;
                valid <= 1'b1;
            end else begin
                valid <= 1'b0;
            end
        end
    end
    
    assign full = (count == 512);
    assign empty = (count == 0);
    
endmodule