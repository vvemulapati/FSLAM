//------------------------------------------------------------------------------
// File: tb_orb_cosim.sv
// Description: Co-simulation testbench for validating RTL feature extraction
//              against the golden Python reference model.
//------------------------------------------------------------------------------

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

    // AXI4-Lite Control
    logic [31:0] s_axi_awaddr, s_axi_wdata, s_axi_araddr;
    logic        s_axi_awvalid, s_axi_wvalid, s_axi_arvalid;
    wire         s_axi_awready, s_axi_wready, s_axi_bvalid, s_axi_arready, s_axi_rvalid;
    wire [1:0]   s_axi_bresp, s_axi_rresp;
    wire [31:0]  s_axi_rdata;
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
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wstrb(4'hF),
        .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(1'b1),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(1'b1),
        .interrupt(interrupt)
    );

    // Feature capture structures
    int rtl_feature_count;
    int word_idx;
    logic [31:0] captured_packet [0:9];
    int out_file;

    // Feature packet receiver
    always_ff @(posedge clk or negedge rst_n) begin
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
        s_axi_awaddr  = 0;
        s_axi_wdata   = 0;
        s_axi_awvalid = 0;
        s_axi_wvalid  = 0;
        s_axi_araddr  = 0;
        s_axi_arvalid = 0;

        out_file = $fopen("rtl_features.txt", "w");

        $display("================================================================");
        $display(" Starting ORB Hardware Co-Simulation Testbench (VCS)");
        $display("================================================================");

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

        // Reset
        #20;
        rst_n = 1;
        #20;

        // Enable accelerator via AXI4-Lite
        $display("[TB] Writing Control Register = 0x1 (Enable Accelerator)");
        s_axi_awaddr  = 32'h00;
        s_axi_wdata   = 32'h01;
        s_axi_awvalid = 1;
        s_axi_wvalid  = 1;
        @(posedge clk);
        while (!s_axi_awready) @(posedge clk);
        s_axi_awvalid = 0;
        s_axi_wvalid  = 0;
        #20;

        // Stream pixel words
        $display("[TB] Streaming image pixels into AXI4-Stream slave...");
        for (int i = 0; i < num_words; i++) begin
            @(posedge clk);
            while (!s_axis_tready) @(posedge clk);
            s_axis_tdata  = pixel_mem[i];
            s_axis_tvalid = 1'b1;
            s_axis_tlast  = (i == num_words - 1);
        end
        @(posedge clk);
        s_axis_tvalid = 1'b0;
        s_axis_tlast  = 1'b0;

        // Wait for pipeline and descriptor FIFO drain
        $display("[TB] All pixels streamed. Draining pipeline and descriptor FIFO...");
        fork
            begin
                #1000;
                while (dut.orb_processing || !dut.u_descriptor_fifo.empty || dut.out_busy) @(posedge clk);
                #200;
            end
            begin
                #500000; // Timeout watchdog
            end
        join_any

        $fclose(out_file);
        $display("================================================================");
        $display("[TB] Simulation Complete! Total RTL Features Extracted: %0d", rtl_feature_count);
        $display("================================================================");
        $finish;
    end

endmodule
