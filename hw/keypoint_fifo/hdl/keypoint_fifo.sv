//------------------------------------------------------------------------------
// File: keypoint_fifo.sv
// Description: Parameterized synchronous FIFO for buffering detected keypoints
//              and their associated metadata (score, coordinates, orientation).
//              Decouples front-end corner detection (FAST/NMS) and orientation
//              from multi-cycle descriptor generation (rBRIEF).
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module keypoint_fifo #(
    parameter int DATA_WIDTH = 46, // [45:44]=level, [43:38]=orientation, [37:22]=score, [21:10]=y, [9:0]=x
    parameter int DEPTH      = 64,
    parameter int ALMOST_FULL_THRESH = DEPTH - 4
) (
    input  logic                  clk,
    input  logic                  rst_n,

    // Write Interface (Push from NMS / Orientation)
    input  logic                  wr_en,
    input  logic [DATA_WIDTH-1:0] din,
    output logic                  full,
    output logic                  almost_full,
    output logic                  overflow,

    // Read Interface (Pop to Descriptor Generator / Matcher)
    input  logic                  rd_en,
    output logic [DATA_WIDTH-1:0] dout,
    output logic                  empty,
    output logic                  almost_empty,
    output logic                  underflow,

    // Status
    output logic [$clog2(DEPTH):0] count
);

    localparam int ADDR_WIDTH = $clog2(DEPTH);

    // Memory storage
    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // Read and write pointers
    logic [ADDR_WIDTH-1:0] wr_ptr;
    logic [ADDR_WIDTH-1:0] rd_ptr;
    logic [ADDR_WIDTH:0]   item_count;

    // Status flags
    assign count        = item_count;
    assign empty        = (item_count == 0);
    assign full         = (item_count == DEPTH);
    assign almost_empty = (item_count <= 1);
    assign almost_full  = (item_count >= ALMOST_FULL_THRESH);

    // First-word fall-through / synchronous read
    assign dout = mem[rd_ptr];

    // FIFO State update
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr     <= '0;
            rd_ptr     <= '0;
            item_count <= '0;
            overflow   <= 1'b0;
            underflow  <= 1'b0;
        end else begin
            overflow  <= 1'b0;
            underflow <= 1'b0;

            case ({wr_en && !full, rd_en && !empty})
                2'b10: begin // Write only
                    mem[wr_ptr] <= din;
                    wr_ptr      <= (wr_ptr == DEPTH - 1) ? '0 : wr_ptr + 1'b1;
                    item_count  <= item_count + 1'b1;
                end

                2'b01: begin // Read only
                    rd_ptr     <= (rd_ptr == DEPTH - 1) ? '0 : rd_ptr + 1'b1;
                    item_count <= item_count - 1'b1;
                end

                2'b11: begin // Simultaneous Read & Write
                    mem[wr_ptr] <= din;
                    wr_ptr      <= (wr_ptr == DEPTH - 1) ? '0 : wr_ptr + 1'b1;
                    rd_ptr      <= (rd_ptr == DEPTH - 1) ? '0 : rd_ptr + 1'b1;
                    // item_count unchanged
                end

                2'b00: begin
                    if (wr_en && full)  overflow  <= 1'b1;
                    if (rd_en && empty) underflow <= 1'b1;
                end
            endcase
        end
    end

endmodule
