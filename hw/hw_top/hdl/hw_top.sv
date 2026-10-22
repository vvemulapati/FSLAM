// Top level: AXI4-Stream pixel input (four 6-bit pixels per 32-bit word) and
// AXI4-Stream feature output (one 10-word packet per feature).

`timescale 1ns / 1ps

module hw_top #(
    parameter int PIXEL_WIDTH     = 6,
    parameter int IMG_WIDTH       = 640,
    parameter int IMG_HEIGHT      = 480,
    parameter int DESCRIPTOR_BITS = 256,
    parameter int AXI_DATA_WIDTH  = 32
) (
    input  logic                      aclk,
    input  logic                      aresetn,

    // AXI4-Stream Slave: Input Pixel Stream (DMA from DRAM)
    input  logic [AXI_DATA_WIDTH-1:0] s_axis_tdata,
    input  logic                      s_axis_tvalid,
    input  logic                      s_axis_tlast,
    output logic                      s_axis_tready,

    // AXI4-Stream Master: Output Extracted Features (DMA to DRAM)
    output logic [AXI_DATA_WIDTH-1:0] m_axis_tdata,
    output logic                      m_axis_tvalid,
    output logic                      m_axis_tlast,
    input  logic                      m_axis_tready,

    // Interrupt output to ARM CPU upon frame completion
    output logic                      interrupt
);

    // ORB feature signals
    logic feature_valid;
    logic [DESCRIPTOR_BITS-1:0] feature_descriptor;
    logic [15:0] feature_score;
    logic [11:0] feature_x, feature_y;
    logic [5:0]  feature_orientation;
    logic [1:0]  feature_level;
    logic        orb_processing;
    logic [10:0] total_features;

    // Descriptor FIFO (304-bit entries)
    logic         desc_fifo_wr_en;
    logic [303:0] desc_fifo_din;
    logic         desc_fifo_full;
    logic         desc_fifo_almost_full;
    logic         desc_fifo_overflow;
    logic         desc_fifo_rd_en;
    logic [303:0] desc_fifo_dout;
    logic         desc_fifo_empty;
    logic         desc_fifo_almost_empty;
    logic         desc_fifo_underflow;
    logic [6:0]   desc_fifo_count;

    // Backpressure pixel unpacking when the descriptor FIFO is filling up
    logic fifo_backpressure;
    assign fifo_backpressure = desc_fifo_almost_full || desc_fifo_full;

    // Input holding register: isolates top-level AXI-Stream input pins from
    // the internal pixel unpacker and pyramid fanout.
    logic [AXI_DATA_WIDTH-1:0] axis_buf_data;
    logic                      axis_buf_last;
    logic                      axis_buf_valid;
    logic [1:0]                byte_sel;
    logic [PIXEL_WIDTH-1:0]    pixel_in;
    logic                      pixel_valid;

    wire word_consumed = pixel_valid && (byte_sel == 2'b11);
    assign s_axis_tready = !axis_buf_valid || word_consumed;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            axis_buf_data  <= '0;
            axis_buf_last  <= 1'b0;
            axis_buf_valid <= 1'b0;
            byte_sel       <= '0;
        end else begin
            if (s_axis_tready) begin
                axis_buf_valid <= s_axis_tvalid;
                if (s_axis_tvalid) begin
                    axis_buf_data <= s_axis_tdata;
                    axis_buf_last <= s_axis_tlast;
                end
            end
            if (pixel_valid) begin
                byte_sel <= byte_sel + 2'd1;
            end
        end
    end

    always_comb begin
        case (byte_sel)
            2'b00: pixel_in = axis_buf_data[5:0];
            2'b01: pixel_in = axis_buf_data[13:8];
            2'b10: pixel_in = axis_buf_data[21:16];
            2'b11: pixel_in = axis_buf_data[29:24];
        endcase
        pixel_valid = axis_buf_valid && !fifo_backpressure;
    end

    // ORB test pattern from OpenCV (bit_pattern_31_)
    logic signed [5:0] brief_pattern [0:255][0:3];
    `include "orb_pattern_table.svh"

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
        .enable(1'b1),
        .pixel_valid(pixel_valid),
        .pixel_in(pixel_in),
        .brief_pattern(brief_pattern),
        .feature_ready(!desc_fifo_full),
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

    // Pack: [303:48]=descriptor, [47:32]=score, [31:20]=y, [19:8]=x, [7:2]=orientation, [1:0]=level
    assign desc_fifo_wr_en = feature_valid && !desc_fifo_full;
    assign desc_fifo_din   = {feature_descriptor, feature_score, feature_y, feature_x, feature_orientation, feature_level};

    descriptor_fifo #(
        .DATA_WIDTH(304),
        .DEPTH(64),
        .ALMOST_FULL_THRESH(32)
    ) u_descriptor_fifo (
        .clk(aclk),
        .rst_n(aresetn),
        .wr_en(desc_fifo_wr_en),
        .din(desc_fifo_din),
        .full(desc_fifo_full),
        .almost_full(desc_fifo_almost_full),
        .overflow(desc_fifo_overflow),
        .rd_en(desc_fifo_rd_en),
        .dout(desc_fifo_dout),
        .empty(desc_fifo_empty),
        .almost_empty(desc_fifo_almost_empty),
        .underflow(desc_fifo_underflow),
        .count(desc_fifo_count)
    );

    // Output serializer: each FIFO entry goes out as ten 32-bit words
    logic [31:0] out_words [0:9];
    logic [3:0]  out_word_idx;
    logic        out_busy;

    // Pop next feature from FIFO when serializer is idle
    assign desc_fifo_rd_en = !desc_fifo_empty && !out_busy;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            out_busy      <= 1'b0;
            out_word_idx  <= '0;
            m_axis_tvalid <= 1'b0;
            m_axis_tdata  <= '0;
            m_axis_tlast  <= 1'b0;
            for (int w = 0; w < 10; w++) begin
                out_words[w] <= '0;
            end
        end else begin
            if (desc_fifo_rd_en) begin
                out_busy <= 1'b1;
                out_word_idx <= '0;
                out_words[0] <= {8'b0, desc_fifo_dout[1:0], desc_fifo_dout[7:2], desc_fifo_dout[47:32]};
                out_words[1] <= {4'h0, desc_fifo_dout[31:20], 4'h0, desc_fifo_dout[19:8]};
                for (int w = 0; w < 8; w++) begin
                    out_words[2 + w] <= desc_fifo_dout[48 + w*32 +: 32];
                end
                m_axis_tvalid <= 1'b1;
                m_axis_tdata  <= {8'b0, desc_fifo_dout[1:0], desc_fifo_dout[7:2], desc_fifo_dout[47:32]};
                m_axis_tlast  <= 1'b0;
            end else if (out_busy) begin
                if (m_axis_tready) begin
                    if (out_word_idx == 4'd9) begin
                        out_busy      <= 1'b0;
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;
                    end else begin
                        out_word_idx  <= out_word_idx + 4'd1;
                        m_axis_tdata  <= out_words[out_word_idx + 4'd1];
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= (out_word_idx == 4'd8);
                    end
                end
            end else begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end
        end
    end

    // Frame completion interrupt: latch frame end on the last pixel of axis_buf_last,
    // then assert interrupt once the ORB pipeline, FIFO, and serializer stay idle.
    logic       frame_end_pending;
    logic [5:0] idle_cnt;
    wire        pipeline_idle = !orb_processing && desc_fifo_empty && !out_busy && !feature_valid;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            frame_end_pending <= 1'b0;
            idle_cnt          <= '0;
            interrupt         <= 1'b0;
        end else begin
            if (pixel_valid && axis_buf_last && (byte_sel == 2'b11)) begin
                frame_end_pending <= 1'b1;
                idle_cnt          <= '0;
                interrupt         <= 1'b0;
            end else if (frame_end_pending) begin
                if (pipeline_idle) begin
                    if (idle_cnt == 6'd63) begin
                        interrupt         <= 1'b1;
                        frame_end_pending <= 1'b0;
                    end else begin
                        idle_cnt <= idle_cnt + 6'd1;
                    end
                end else begin
                    idle_cnt <= '0;
                end
            end else if (pixel_valid) begin
                interrupt <= 1'b0;
            end
        end
    end

endmodule
