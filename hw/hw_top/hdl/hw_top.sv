//------------------------------------------------------------------------------
// File: hw_top.sv
// Description: Top-level SoC wrapper for the ORB feature extractor.
//              Interfaces:
//                - AXI4-Stream Slave : Incoming DMA pixel stream
//                - AXI4-Stream Master: Outgoing extracted feature packets
//                - AXI4-Lite Slave   : Control/Status registers & feature thresholding
//------------------------------------------------------------------------------

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

    // AXI4-Lite Slave: Control & Status Register Interface
    input  logic [31:0]               s_axi_awaddr,
    input  logic                      s_axi_awvalid,
    output logic                      s_axi_awready,
    input  logic [31:0]               s_axi_wdata,
    input  logic [3:0]                s_axi_wstrb,
    input  logic                      s_axi_wvalid,
    output logic                      s_axi_wready,
    output logic [1:0]                s_axi_bresp,
    output logic                      s_axi_bvalid,
    input  logic                      s_axi_bready,

    input  logic [31:0]               s_axi_araddr,
    input  logic                      s_axi_arvalid,
    output logic                      s_axi_arready,
    output logic [31:0]               s_axi_rdata,
    output logic [1:0]                s_axi_rresp,
    output logic                      s_axi_rvalid,
    input  logic                      s_axi_rready,

    // Interrupt output to ARM CPU upon frame completion
    output logic                      interrupt
);

    // Control & Status Registers
    logic [31:0] control_reg;
    logic        enable;
    assign enable = control_reg[0];

    // AXI4-Lite write transactions
    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            control_reg   <= '0;
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
        end else begin
            if (s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                if (s_axi_awaddr[7:0] == 8'h00) begin
                    control_reg <= s_axi_wdata;
                end
                s_axi_bvalid <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
                if (s_axi_bready) begin
                    s_axi_bvalid <= 1'b0;
                end
            end
        end
    end

    // ORB feature signals
    logic feature_valid;
    logic [DESCRIPTOR_BITS-1:0] feature_descriptor;
    logic [15:0] feature_score;
    logic [11:0] feature_x, feature_y;
    logic [5:0]  feature_orientation;
    logic [1:0]  feature_level;
    logic        orb_processing;
    logic [10:0] total_features;

    // Status register reading
    logic [31:0] status_reg;
    assign status_reg = {16'h0, total_features[10:0], 4'h0, orb_processing};

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= '0;
            s_axi_rresp   <= 2'b00;
        end else begin
            if (s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                case (s_axi_araddr[7:0])
                    8'h00: s_axi_rdata <= control_reg;
                    8'h04: s_axi_rdata <= status_reg;
                    default: s_axi_rdata <= '0;
                endcase
            end else begin
                s_axi_arready <= 1'b0;
                if (s_axi_rready) begin
                    s_axi_rvalid <= 1'b0;
                end
            end
        end
    end

    // Pixel Stream Unpacking (4 pixels per 32-bit AXI word: 6-bit per pixel)
    logic [1:0] byte_sel;
    logic [PIXEL_WIDTH-1:0] pixel_in;
    logic pixel_valid;

    always_comb begin
        case (byte_sel)
            2'b00: pixel_in = s_axis_tdata[5:0];
            2'b01: pixel_in = s_axis_tdata[13:8];
            2'b10: pixel_in = s_axis_tdata[21:16];
            2'b11: pixel_in = s_axis_tdata[29:24];
        endcase
        pixel_valid = s_axis_tvalid && enable;
    end

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            byte_sel <= '0;
        end else if (pixel_valid) begin
            byte_sel <= byte_sel + 1;
        end
    end

    assign s_axis_tready = enable && (byte_sel == 2'b11 || !s_axis_tvalid);

    // Official standard OpenCV 256-pair ORB test coordinate pattern (bit_pattern_31_)
    logic signed [5:0] brief_pattern [0:255][0:3];
    `include "orb_pattern_table.svh"

    // ORB feature extraction pipeline instance
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
        .pixel_in(pixel_in),
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

    // -------------------------------------------------------------------------
    // Descriptor FIFO: Buffers 304-bit extracted feature packets from ORB core
    // -------------------------------------------------------------------------
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
    logic [8:0]   desc_fifo_count;

    // Pack: [303:48]=descriptor, [47:32]=score, [31:20]=y, [19:8]=x, [7:2]=orientation, [1:0]=level
    assign desc_fifo_wr_en = feature_valid;
    assign desc_fifo_din   = {feature_descriptor, feature_score, feature_y, feature_x, feature_orientation, feature_level};

    descriptor_fifo #(
        .DATA_WIDTH(304),
        .DEPTH(256)
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

    // -------------------------------------------------------------------------
    // AXI4-Stream Master Output Serialization:
    // Reads feature packets from Descriptor FIFO and serializes to 10 32-bit words
    // -------------------------------------------------------------------------
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
        end else begin
            if (desc_fifo_rd_en) begin
                out_busy <= 1'b1;
                out_word_idx <= '0;
                out_words[0] <= {6'b0, desc_fifo_dout[1:0], desc_fifo_dout[7:2], desc_fifo_dout[47:32]};
                out_words[1] <= {4'h0, desc_fifo_dout[31:20], 4'h0, desc_fifo_dout[19:8]};
                for (int w = 0; w < 8; w++) begin
                    out_words[2 + w] <= desc_fifo_dout[48 + w*32 +: 32];
                end
                m_axis_tvalid <= 1'b1;
                m_axis_tdata  <= {6'b0, desc_fifo_dout[1:0], desc_fifo_dout[7:2], desc_fifo_dout[47:32]};
                m_axis_tlast  <= 1'b0;
            end else if (out_busy) begin
                if (m_axis_tready) begin
                    if (out_word_idx == 4'd9) begin
                        out_busy      <= 1'b0;
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;
                    end else begin
                        out_word_idx  <= out_word_idx + 1;
                        m_axis_tdata  <= out_words[out_word_idx + 1];
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

    // Frame interrupt upon completion (ORB processing finished and Descriptor FIFO empty)
    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            interrupt <= 1'b0;
        end else begin
            interrupt <= (s_axis_tlast && !orb_processing && desc_fifo_empty && !out_busy);
        end
    end

endmodule
