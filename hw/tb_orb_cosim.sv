// VCS testbench: streams input_pixels.hex into hw_top and writes the output
// features to rtl_features.txt for comparison with the Python reference model.

`timescale 1ns / 1ps

module tb_orb_cosim;

    localparam int PIXEL_WIDTH     = 6;
    localparam int IMG_WIDTH       = 640;
    localparam int IMG_HEIGHT      = 480;
    localparam int DESCRIPTOR_BITS = 256;
    localparam int AXI_DATA_WIDTH  = 32;

    logic clk;
    logic rst_n;

    // AXI4-Stream Slave (DMA Pixel input)
    logic [AXI_DATA_WIDTH-1:0] s_axis_tdata;
    logic                      s_axis_tvalid;
    logic                      s_axis_tlast;
    wire                       s_axis_tready;

    // AXI4-Stream Master (Feature output)
    wire [AXI_DATA_WIDTH-1:0]  m_axis_tdata;
    wire                       m_axis_tvalid;
    wire                       m_axis_tlast;
    logic                      m_axis_tready;

    wire         interrupt;

    // Clock generation (150 MHz = 6.666 ns period)
    always #3.333 clk = ~clk;

    // DUT Instantiation
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

    // Feature capture structures
    int rtl_feature_count;
    int word_idx;
    logic [31:0] captured_packet [0:9];
    int out_file;

    // Feature packet receiver
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

                // Log feature (using m_axis_tdata for the current final word)
                $fwrite(out_file, "FEAT: level=%0d ori=%0d score=%0d x=%0d y=%0d desc=%08x%08x%08x%08x%08x%08x%08x%08x\n",
                    captured_packet[0][23:22], // level: bits [23:22]
                    captured_packet[0][21:16], // orientation: bits [21:16]
                    captured_packet[0][15:0],  // score: bits [15:0]
                    captured_packet[1][11:0],  // x: bits [11:0]
                    captured_packet[1][27:16], // y: bits [27:16]
                    (word_idx == 9 ? m_axis_tdata : captured_packet[9]),
                    captured_packet[8], captured_packet[7], captured_packet[6],
                    captured_packet[5], captured_packet[4], captured_packet[3], captured_packet[2]
                );
            end else begin
                word_idx <= word_idx + 1;
            end
        end
    end

    // Test stimulus
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

        $display("[TB] Start");

`ifdef DUMP_FSDB
        $display("[TB] Dumping FSDB waveforms to dump.fsdb...");
        $fsdbDumpfile("dump.fsdb");
        $fsdbDumpvars(0, tb_orb_cosim);
`endif

`ifdef DUMP_VCD
        $display("[TB] Dumping VCD waveforms to dump.vcd...");
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_orb_cosim);
`endif

        // Load input pixel hex data
        $readmemh("input_pixels.hex", pixel_mem);
        num_words = (IMG_WIDTH * IMG_HEIGHT) / 4;
        $display("[TB] Loaded %0d input words (%0d pixels)", num_words, num_words * 4);

        // Synchronous reset
        repeat (4) @(posedge clk);
        rst_n <= 1'b1;
        repeat (4) @(posedge clk);

        // Stream pixel words using standard AXI4-Stream handshake
        $display("[TB] Streaming image pixels into AXI4-Stream slave...");
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

        // Wait for frame completion interrupt
        $display("[TB] All pixels streamed. Waiting for interrupt...");
        fork
            begin
                while (!interrupt) @(posedge clk);
                repeat (10) @(posedge clk);
            end
            begin
                #500000; // Timeout watchdog
                $display("[TB] ERROR: Timeout waiting for interrupt");
            end
        join_any

        $fclose(out_file);
        $display("[TB] Done: %0d features", rtl_feature_count);
        $finish;
    end

endmodule
