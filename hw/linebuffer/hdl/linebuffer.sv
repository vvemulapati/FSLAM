// Line buffer: delays the input stream by LINE_WIDTH samples (one image row),
// using a simple dual-port RAM.

`timescale 1ns / 1ps

// Simple dual-port RAM, registered read (1-cycle latency).
module linebuffer_ram #(
    parameter int DATA_WIDTH = 6,
    parameter int DEPTH      = 640,
    parameter int ADDR_WIDTH = (DEPTH > 1) ? $clog2(DEPTH) : 1,
    parameter     RAM_STYLE  = "block" // "block", "distributed", or "ultra"
) (
    input  logic                  clk,

    // Write port
    input  logic                  wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [DATA_WIDTH-1:0] wr_data,

    // Read port
    input  logic                  rd_en,
    input  logic [ADDR_WIDTH-1:0] rd_addr,
    output logic [DATA_WIDTH-1:0] rd_data
);

    (* ram_style = RAM_STYLE *)
    logic [DATA_WIDTH-1:0] mem_array [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem_array[wr_addr] <= wr_data;
        end
    end

    always_ff @(posedge clk) begin
        if (rd_en) begin
            rd_data <= mem_array[rd_addr];
        end
    end

endmodule


module linebuffer #(
    parameter int DATA_WIDTH = 6,
    parameter int LINE_WIDTH = 640,
    parameter     RAM_STYLE  = "block" // "block", "distributed", or "ultra"
) (
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic                  in_valid,
    input  logic [DATA_WIDTH-1:0] in_data,

    output logic                  out_valid,
    output logic [DATA_WIDTH-1:0] out_data
);

    localparam int ADDR_WIDTH = (LINE_WIDTH > 1) ? $clog2(LINE_WIDTH) : 1;

    // Pointer and control registers
    logic [ADDR_WIDTH-1:0] wr_ptr;
    logic [ADDR_WIDTH-1:0] rd_ptr;
    logic [ADDR_WIDTH:0]   fill_count;
    logic                  line_filled;

    logic                  ram_wr_en;
    logic                  ram_rd_en;
    logic [DATA_WIDTH-1:0] ram_rdata;

    assign ram_wr_en = in_valid;
    assign ram_rd_en = in_valid;

    linebuffer_ram #(
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH(LINE_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .RAM_STYLE(RAM_STYLE)
    ) u_linebuffer_ram (
        .clk(clk),
        .wr_en(ram_wr_en),
        .wr_addr(wr_ptr),
        .wr_data(in_data),
        .rd_en(ram_rd_en),
        .rd_addr(rd_ptr),
        .rd_data(ram_rdata)
    );

    // Read/write pointers (rd_ptr stays one slot ahead of wr_ptr so the
    // synchronous RAM read is already registered when the next sample arrives).
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr      <= '0;
            rd_ptr      <= ADDR_WIDTH'(1);
            fill_count  <= '0;
            line_filled <= 1'b0;
        end else if (in_valid) begin
            if (wr_ptr == ADDR_WIDTH'(LINE_WIDTH - 1)) begin
                wr_ptr <= '0;
            end else begin
                wr_ptr <= wr_ptr + 1'b1;
            end

            if (rd_ptr == ADDR_WIDTH'(LINE_WIDTH - 1)) begin
                rd_ptr <= '0;
            end else begin
                rd_ptr <= rd_ptr + 1'b1;
            end

            if (!line_filled) begin
                if (fill_count == (ADDR_WIDTH+1)'(LINE_WIDTH - 1)) begin
                    line_filled <= 1'b1;
                end else begin
                    fill_count  <= fill_count + 1'b1;
                end
            end
        end
    end

    assign out_valid = in_valid && line_filled;
    assign out_data  = ram_rdata;

endmodule
