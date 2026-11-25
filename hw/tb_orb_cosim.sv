// VCS testbench: streams input_pixels.hex into hw_top and writes output
// feature packets to rtl_features.txt.

`timescale 1ns / 1ps

module tb_orb_cosim;

    localparam int PIXEL_WIDTH     = 6;
    localparam int IMG_WIDTH       = 640;
    localparam int IMG_HEIGHT      = 480;
    localparam int DESCRIPTOR_BITS = 256;
    localparam int AXI_DATA_WIDTH  = 32;

    logic clk;
    logic rst_n;

    logic [AXI_DATA_WIDTH-1:0] s_axis_tdata;
    logic                      s_axis_tvalid;
    logic                      s_axis_tlast;
    wire                       s_axis_tready;

    wire [AXI_DATA_WIDTH-1:0]  m_axis_tdata;
    wire                       m_axis_tvalid;
    wire                       m_axis_tlast;
    logic                      m_axis_tready;

    wire interrupt;

    // 150 MHz clock (6.666 ns period)
    always #3.333 clk = ~clk;

    hw_top #(
        .PIXEL_WIDTH(PIXEL_WIDTH),
        .IMG_WIDTH(IMG_WIDTH),
        .IMG_HEIGHT(IMG_HEIGHT),
        .DESCRIPTOR_BITS(DESCRIPTOR_BITS),
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) dut (
        .aclk(clk),
        .aresetn(rst_n),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tlast(s_axis_tlast),
        .s_axis_tready(s_axis_tready),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tlast(m_axis_tlast),
        .m_axis_tready(m_axis_tready),
        .interrupt(interrupt)
    );

    int rtl_feature_count;
    int word_idx;
    logic [31:0] captured_packet [0:9];
    int out_file;

    // Reassemble 10-word AXI4-Stream feature packets
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rtl_feature_count <= 0;
            word_idx          <= 0;
            for (int k = 0; k < 10; k++) begin
                captured_packet[k] <= '0;
            end
        end else if (m_axis_tvalid && m_axis_tready) begin
            captured_packet[word_idx] <= m_axis_tdata;
            if (word_idx == 9 || m_axis_tlast) begin
                rtl_feature_count <= rtl_feature_count + 1;
                word_idx          <= 0;

                // Use live m_axis_tdata on word 9 before captured_packet[9] updates
                $fwrite(out_file, "FEAT: level=%0d ori=%0d score=%0d x=%0d y=%0d desc=%08x%08x%08x%08x%08x%08x%08x%08x\n",
                    captured_packet[0][23:22],
                    captured_packet[0][21:16],
                    captured_packet[0][15:0],
                    captured_packet[1][11:0],
                    captured_packet[1][27:16],
                    (word_idx == 9 ? m_axis_tdata : captured_packet[9]),
                    captured_packet[8], captured_packet[7], captured_packet[6],
                    captured_packet[5], captured_packet[4], captured_packet[3], captured_packet[2]
                );
            end else begin
                word_idx <= word_idx + 1;
            end
        end
    end

    logic [31:0] pixel_mem [0:(IMG_WIDTH*IMG_HEIGHT/4)-1];
    int num_words;

    initial begin
        clk           = 0;
        rst_n         = 0;
        s_axis_tdata  = 0;
        s_axis_tvalid = 0;
        s_axis_tlast  = 0;
        m_axis_tready = 1;

        out_file = $fopen("rtl_features.txt", "w");

`ifdef DUMP_FSDB
        $fsdbDumpfile("dump.fsdb");
        $fsdbDumpvars(0, tb_orb_cosim);
`endif

`ifdef DUMP_VCD
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_orb_cosim);
`endif

        $readmemh("input_pixels.hex", pixel_mem);
        num_words = (IMG_WIDTH * IMG_HEIGHT) / 4;

        repeat (4) @(posedge clk);
        rst_n <= 1'b1;
        repeat (4) @(posedge clk);

        for (int i = 0; i < num_words; i++) begin
            s_axis_tdata  <= pixel_mem[i];
            s_axis_tvalid <= 1'b1;
            s_axis_tlast  <= (i == num_words - 1);
            do begin
                @(posedge clk);
            end while (!(s_axis_tvalid && s_axis_tready));
        end
        s_axis_tvalid <= 1'b0;
        s_axis_tlast  <= 1'b0;
        s_axis_tdata  <= '0;

        fork
            begin
                while (!interrupt) @(posedge clk);
                repeat (10) @(posedge clk);
            end
            begin
                #25000000; // 25 ms watchdog
                $display("ERROR: Timeout waiting for interrupt");
                $fclose(out_file);
                $fatal(1);
            end
        join_any

        $fclose(out_file);
        $display("Extracted %0d features", rtl_feature_count);
        $finish;
    end

endmodule
