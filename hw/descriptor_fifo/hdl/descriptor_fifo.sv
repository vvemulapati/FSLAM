//------------------------------------------------------------------------------
// File: descriptor_fifo.sv
// Description: Synchronous FIFO for buffering extracted ORB feature descriptors
//              and metadata (304 bits total):
//                - [303:48] : 256-bit binary rBRIEF descriptor
//                - [47:32]  : 16-bit FAST corner score
//                - [31:20]  : 12-bit keypoint Y coordinate
//                - [19:8]   : 12-bit keypoint X coordinate
//                - [7:2]    : 6-bit orientation angle sector (0..63)
//                - [1:0]    : 2-bit pyramid octave level (0..3)
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module descriptor_fifo #(
    parameter int DATA_WIDTH = 304,
    parameter int DEPTH      = 256,
    parameter int ADDR_WIDTH = $clog2(DEPTH)
) (
    input  logic                   clk,
    input  logic                   rst_n,

    // Write Interface (from ORB extractor output)
    input  logic                   wr_en,
    input  logic [DATA_WIDTH-1:0]  din,
    output logic                   full,
    output logic                   almost_full,
    output logic                   overflow,

    // Read Interface (to AXI4-Stream serializer / DMA / Matcher)
    input  logic                   rd_en,
    output logic [DATA_WIDTH-1:0]  dout,
    output logic                   empty,
    output logic                   almost_empty,
    output logic                   underflow,

    // Status
    output logic [ADDR_WIDTH:0]    count
);

    // Memory array
    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // Read and write pointers
    logic [ADDR_WIDTH-1:0] wr_ptr;
    logic [ADDR_WIDTH-1:0] rd_ptr;
    logic [ADDR_WIDTH:0]   fifo_count;

    // Status flags
    assign empty        = (fifo_count == 0);
    assign full         = (fifo_count == DEPTH);
    assign almost_empty = (fifo_count <= 1);
    assign almost_full  = (fifo_count >= DEPTH - 1);
    assign count        = fifo_count;

    // First-Word Fall-Through (FWFT) data out
    assign dout         = mem[rd_ptr];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr     <= '0;
            rd_ptr     <= '0;
            fifo_count <= '0;
            overflow   <= 1'b0;
            underflow  <= 1'b0;
        end else begin
            overflow  <= 1'b0;
            underflow <= 1'b0;

            case ({wr_en && !full, rd_en && !empty})
                2'b10: begin // Write only
                    mem[wr_ptr] <= din;
                    wr_ptr      <= (wr_ptr == DEPTH - 1) ? '0 : wr_ptr + 1'b1;
                    fifo_count  <= fifo_count + 1'b1;
                end
                2'b01: begin // Read only
                    rd_ptr      <= (rd_ptr == DEPTH - 1) ? '0 : rd_ptr + 1'b1;
                    fifo_count  <= fifo_count - 1'b1;
                end
                2'b11: begin // Simultaneous Write and Read
                    mem[wr_ptr] <= din;
                    wr_ptr      <= (wr_ptr == DEPTH - 1) ? '0 : wr_ptr + 1'b1;
                    rd_ptr      <= (rd_ptr == DEPTH - 1) ? '0 : rd_ptr + 1'b1;
                    // Count remains unchanged
                end
                default: ;
            endcase

            // Error monitoring
            if (wr_en && full && !(rd_en && !empty)) begin
                overflow <= 1'b1;
            end
            if (rd_en && empty) begin
                underflow <= 1'b1;
            end
        end
    end

endmodule
