//------------------------------------------------------------------------------
// File: linebuffer.sv
// Description: Streaming line buffer with explicit 2-Port (Simple Dual-Port)
//              RAM inference submodule across Xilinx Vivado, Synopsys VCS,
//              and Intel Quartus.
//              Delays incoming pixel stream by exactly LINE_WIDTH clock cycles.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

// =============================================================================
// Explicit Simple Dual-Port (2-Port) RAM Module for Synthesis Inference
// =============================================================================
module linebuffer_ram #(
    parameter int DATA_WIDTH = 6,
    parameter int DEPTH      = 640,
    parameter int ADDR_WIDTH = (DEPTH > 1) ? $clog2(DEPTH) : 1,
    parameter     RAM_STYLE  = "block" // "block", "distributed", or "ultra"
) (
    input  logic                  clk,

    // Port A: Synchronous Write Port (No reset on memory elements)
    input  logic                  wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [DATA_WIDTH-1:0] wr_data,

    // Port B: Synchronous Read Port (Infers internal BRAM output register)
    input  logic                  rd_en,
    input  logic [ADDR_WIDTH-1:0] rd_addr,
    output logic [DATA_WIDTH-1:0] rd_data
);

    // -------------------------------------------------------------------------
    // Dedicated 2-Port Memory Array
    // -------------------------------------------------------------------------
    (* ram_style = RAM_STYLE *)
    logic [DATA_WIDTH-1:0] mem_array [0:DEPTH-1];

    // Port A: Dedicated Write Port
    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem_array[wr_addr] <= wr_data;
        end
    end

    // Port B: Dedicated Read Port
    always_ff @(posedge clk) begin
        if (rd_en) begin
            rd_data <= mem_array[rd_addr];
        end
    end

endmodule


// =============================================================================
// Top-Level Streaming Line Buffer Controller
// =============================================================================
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

    // -------------------------------------------------------------------------
    // Dedicated 2-Port RAM Instance with mem_array
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Address & Control Logic (Synchronous reset)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr      <= '0;
            rd_ptr      <= '0;
            fill_count  <= '0;
            line_filled <= 1'b0;
            out_valid   <= 1'b0;
        end else begin
            out_valid <= in_valid && line_filled;

            if (in_valid) begin
                // Advance write pointer
                if (wr_ptr == LINE_WIDTH - 1) begin
                    wr_ptr <= '0;
                end else begin
                    wr_ptr <= wr_ptr + 1'b1;
                end

                // Track when the line buffer is fully primed (LINE_WIDTH cycles)
                if (line_filled) begin
                    if (rd_ptr == LINE_WIDTH - 1) begin
                        rd_ptr <= '0;
                    end else begin
                        rd_ptr <= rd_ptr + 1'b1;
                    end
                end else begin
                    if (fill_count == LINE_WIDTH - 1) begin
                        line_filled <= 1'b1;
                        rd_ptr      <= '0;
                    end else begin
                        fill_count  <= fill_count + 1'b1;
                    end
                end
            end
        end
    end

    assign out_data = ram_rdata;

endmodule
