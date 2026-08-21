//------------------------------------------------------------------------------
// File: linebuffer.sv
// Description: Streaming line buffer designed for clean 2-Port Block RAM (SDP)
//              inference across Xilinx Vivado, Synopsys VCS/DC, and Intel Quartus.
//              Delays incoming pixel stream by exactly LINE_WIDTH clock cycles.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

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

    // -------------------------------------------------------------------------
    // 2-Port RAM Memory Array (Simple Dual-Port RAM Template)
    // -------------------------------------------------------------------------
    (* ram_style = RAM_STYLE *)
    logic [DATA_WIDTH-1:0] mem [0:LINE_WIDTH-1];

    // Pointer and control registers
    logic [ADDR_WIDTH-1:0] wr_ptr;
    logic [ADDR_WIDTH-1:0] rd_ptr;
    logic [ADDR_WIDTH:0]   fill_count;
    logic                  line_filled;

    // Synchronous memory read output
    logic [DATA_WIDTH-1:0] ram_rdata;
    logic                  ram_rd_en;
    logic                  ram_wr_en;

    assign ram_wr_en = in_valid;
    assign ram_rd_en = in_valid && line_filled;

    // -------------------------------------------------------------------------
    // Port A: Synchronous Dedicated Write Port (NO async reset for BRAM template)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (ram_wr_en) begin
            mem[wr_ptr] <= in_data;
        end
    end

    // -------------------------------------------------------------------------
    // Port B: Synchronous Dedicated Read Port (Infers BRAM output register)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (in_valid) begin
            ram_rdata <= mem[rd_ptr];
        end
    end

    // -------------------------------------------------------------------------
    // Address & Control Logic (Synchronous with async reset rst_n)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr      <= '0;
            rd_ptr      <= '0;
            fill_count  <= '0;
            line_filled <= 1'b0;
            out_valid   <= 1'b0;
        end else begin
            out_valid <= ram_rd_en;

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
