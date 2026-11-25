// Top level: AXI4-Stream pixel input (four pixels per 32-bit word) and
// AXI4-Stream feature output (one 10-word packet per feature, plus an optional
// end-of-frame trailer packet when TLAST_ON_EOF = 1).

`timescale 1ns / 1ps

module hw_top #(
    parameter int PIXEL_WIDTH      = 6,
    parameter int IMG_WIDTH        = 640,
    parameter int IMG_HEIGHT       = 480,
    parameter int DESCRIPTOR_BITS  = 256,
    parameter int AXI_DATA_WIDTH   = 32,
    parameter int PYRAMID_LEVELS   = 4,
    parameter bit TLAST_ON_EOF     = 1'b0,
    parameter int IRQ_PULSE_CYCLES = 16
) (
    input  logic                      aclk,
    input  logic                      aresetn,

    // Pixel input stream (4 pixels in byte lanes [5:0], [13:8], [21:16], [29:24])
    input  logic [AXI_DATA_WIDTH-1:0] s_axis_tdata,
    input  logic                      s_axis_tvalid,
    input  logic                      s_axis_tlast,
    output logic                      s_axis_tready,

    // Feature output stream (10 x 32-bit words per feature)
    output logic [AXI_DATA_WIDTH-1:0] m_axis_tdata,
    output logic                      m_axis_tvalid,
    output logic                      m_axis_tlast,
    input  logic                      m_axis_tready,

    output logic                      interrupt
);

    // 2-FF asynchronous reset synchronizer + synchronous fanout-controlled reset tree
    (* ASYNC_REG = "TRUE" *) logic [1:0] rst_sync;
    (* max_fanout = 64 *)    logic       core_rst_n;
    (* max_fanout = 64 *)    logic       orb_rst_n;
    logic                                frame_clear_req;

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rst_sync <= 2'b00;
        end else begin
            rst_sync <= {rst_sync[0], 1'b1};
        end
    end

    always_ff @(posedge aclk) begin
        if (!rst_sync[1]) begin
            core_rst_n <= 1'b0;
            orb_rst_n  <= 1'b0;
        end else begin
            core_rst_n <= 1'b1;
            orb_rst_n  <= !frame_clear_req;
        end
    end

    logic                       feature_valid;
    logic [DESCRIPTOR_BITS-1:0] feature_descriptor;
    logic [15:0]                feature_score;
    logic [11:0]                feature_x, feature_y;
    logic [5:0]                 feature_orientation;
    logic [1:0]                 feature_level;
    logic                       orb_stall;
    logic                       orb_processing;
    logic [15:0]                total_features;

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

    typedef enum logic [1:0] {
        EOF_IDLE    = 2'd0,
        EOF_DRAIN   = 2'd1,
        EOF_TRAILER = 2'd2,
        EOF_CLEAR   = 2'd3
    } eof_state_t;

    eof_state_t  eof_state;
    logic [5:0]  idle_cnt;
    logic [1:0]  clear_cnt;
    logic [7:0]  irq_cnt;
    logic [15:0] last_frame_features;
    wire         frame_end_pending = (eof_state != EOF_IDLE);

    // Backpressure pixel unpacking when descriptor FIFO fills or rBRIEF is busy
    logic fifo_backpressure;
    assign fifo_backpressure = desc_fifo_almost_full || desc_fifo_full || orb_stall;

    // Unpack 4 pixels per 32-bit word
    logic [AXI_DATA_WIDTH-1:0] axis_buf_data;
    logic                      axis_buf_last;
    logic                      axis_buf_valid;
    logic [1:0]                byte_sel;
    logic [PIXEL_WIDTH-1:0]    pixel_in;
    logic                      pixel_valid;

    wire word_consumed = pixel_valid && (byte_sel == 2'b11);
    assign s_axis_tready = (!axis_buf_valid || word_consumed) && !frame_end_pending && orb_rst_n;

    always_ff @(posedge aclk) begin
        if (!core_rst_n) begin
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
            end else if (word_consumed) begin
                axis_buf_valid <= 1'b0;
            end
            if (pixel_valid) begin
                byte_sel <= byte_sel + 2'd1;
            end
        end
    end

    always_comb begin
        pixel_in    = axis_buf_data[int'(byte_sel) * 8 +: PIXEL_WIDTH];
        pixel_valid = axis_buf_valid && !fifo_backpressure && !frame_end_pending && orb_rst_n;
    end

    logic signed [5:0] brief_pattern [0:255][0:3];
    `include "orb_pattern_table.svh"

    orb #(
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .PYRAMID_LEVELS(PYRAMID_LEVELS),
        .DESCRIPTOR_BITS(DESCRIPTOR_BITS),
        .FIFO_DEPTH(512)
    ) u_orb (
        .clk(aclk),
        .rst_n(orb_rst_n),
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
        .stall_req(orb_stall),
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
        .rst_n(core_rst_n),
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

    // Output serializer: each FIFO entry (or EOF trailer) goes out as ten 32-bit words
    logic [31:0] out_words [0:9];
    logic [3:0]  out_word_idx;
    logic        out_busy;
    logic        out_is_trailer;

    wire trailer_launch  = TLAST_ON_EOF && (eof_state == EOF_TRAILER) && !out_busy;
    wire trailer_done    = out_busy && out_is_trailer && m_axis_tready && (out_word_idx == 4'd9);

    assign desc_fifo_rd_en = !desc_fifo_empty && !out_busy && (eof_state != EOF_TRAILER);

    always_ff @(posedge aclk) begin
        if (!core_rst_n) begin
            out_busy       <= 1'b0;
            out_is_trailer <= 1'b0;
            out_word_idx   <= '0;
            m_axis_tvalid  <= 1'b0;
            m_axis_tdata   <= '0;
            m_axis_tlast   <= 1'b0;
            for (int w = 0; w < 10; w++) begin
                out_words[w] <= '0;
            end
        end else begin
            if (desc_fifo_rd_en) begin
                out_busy       <= 1'b1;
                out_is_trailer <= 1'b0;
                out_word_idx   <= '0;
                out_words[0]   <= {8'b0, desc_fifo_dout[1:0], desc_fifo_dout[7:2], desc_fifo_dout[47:32]};
                out_words[1]   <= {4'h0, desc_fifo_dout[31:20], 4'h0, desc_fifo_dout[19:8]};
                for (int w = 0; w < 8; w++) begin
                    out_words[2 + w] <= desc_fifo_dout[48 + w*32 +: 32];
                end
                m_axis_tvalid <= 1'b1;
                m_axis_tdata  <= {8'b0, desc_fifo_dout[1:0], desc_fifo_dout[7:2], desc_fifo_dout[47:32]};
                m_axis_tlast  <= 1'b0;
            end else if (trailer_launch) begin
                out_busy       <= 1'b1;
                out_is_trailer <= 1'b1;
                out_word_idx   <= '0;
                out_words[0]   <= {8'hFF, 2'b11, 6'h3F, last_frame_features};
                out_words[1]   <= 32'hFFFF_FFFF;
                for (int w = 0; w < 8; w++) begin
                    out_words[2 + w] <= 32'h0000_0000;
                end
                m_axis_tvalid <= 1'b1;
                m_axis_tdata  <= {8'hFF, 2'b11, 6'h3F, last_frame_features};
                m_axis_tlast  <= 1'b0;
            end else if (out_busy) begin
                if (m_axis_tready) begin
                    if (out_word_idx == 4'd9) begin
                        out_busy       <= 1'b0;
                        out_is_trailer <= 1'b0;
                        m_axis_tvalid  <= 1'b0;
                        m_axis_tlast   <= 1'b0;
                    end else begin
                        out_word_idx  <= out_word_idx + 4'd1;
                        m_axis_tdata  <= out_words[out_word_idx + 4'd1];
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= (out_word_idx == 4'd8) && (!TLAST_ON_EOF || out_is_trailer);
                    end
                end
            end else begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end
        end
    end

    // Frame drain, optional EOF trailer, inter-frame ORB reset, and bounded IRQ pulse
    wire pipeline_idle = !orb_processing && desc_fifo_empty && (desc_fifo_count == '0) &&
                         !out_busy && !feature_valid;
    assign frame_clear_req = (eof_state == EOF_CLEAR);

    always_ff @(posedge aclk) begin
        if (!core_rst_n) begin
            eof_state           <= EOF_IDLE;
            idle_cnt            <= '0;
            clear_cnt           <= '0;
            irq_cnt             <= '0;
            last_frame_features <= '0;
            interrupt           <= 1'b0;
        end else begin
            if (interrupt) begin
                if (IRQ_PULSE_CYCLES > 0) begin
                    if (irq_cnt == 8'd0) begin
                        interrupt <= 1'b0;
                    end else begin
                        irq_cnt <= irq_cnt - 8'd1;
                    end
                end else if (pixel_valid) begin
                    interrupt <= 1'b0;
                end
            end

            case (eof_state)
                EOF_IDLE: begin
                    if (pixel_valid && axis_buf_last && (byte_sel == 2'b11)) begin
                        eof_state <= EOF_DRAIN;
                        idle_cnt  <= '0;
                        interrupt <= 1'b0;
                    end
                end

                EOF_DRAIN: begin
                    if (pipeline_idle) begin
                        if (idle_cnt == 6'd63) begin
                            last_frame_features <= total_features;
                            if (TLAST_ON_EOF) begin
                                eof_state <= EOF_TRAILER;
                            end else begin
                                eof_state <= EOF_CLEAR;
                                clear_cnt <= 2'd0;
                            end
                        end else begin
                            idle_cnt <= idle_cnt + 6'd1;
                        end
                    end else begin
                        idle_cnt <= '0;
                    end
                end

                EOF_TRAILER: begin
                    if (trailer_done) begin
                        eof_state <= EOF_CLEAR;
                        clear_cnt <= 2'd0;
                    end
                end

                EOF_CLEAR: begin
                    if (clear_cnt == 2'd1) begin
                        eof_state <= EOF_IDLE;
                        interrupt <= 1'b1;
                        irq_cnt   <= (IRQ_PULSE_CYCLES > 0) ? 8'(IRQ_PULSE_CYCLES - 1) : 8'd0;
                    end else begin
                        clear_cnt <= clear_cnt + 2'd1;
                    end
                end

                default: eof_state <= EOF_IDLE;
            endcase
        end
    end

endmodule
