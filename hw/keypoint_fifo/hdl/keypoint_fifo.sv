//------------------------------------------------------------------------------
// File: keypoint_fifo.sv
// Description: FPGA BRAM-friendly synchronous FIFO for buffering detected keypoints
//              and their associated metadata (score, coordinates, orientation).
//              Decouples front-end corner detection (FAST/NMS) and orientation
//              from multi-cycle descriptor generation (rBRIEF).
//              Features:
//                - Explicit Simple Dual-Port Block RAM (SDP) submodule
//                - Synchronous write and read ports (no asynchronous reads)
//                - Dedicated (* ram_style = "block" *) memory array mem_array
//                - First-Word Fall-Through (FWFT) skid-buffer output stage
//                - 100% synchronous reset
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

// =============================================================================
// Explicit Simple Dual-Port (2-Port) RAM Submodule for BRAM Inference
// =============================================================================
module keypoint_fifo_ram #(
    parameter int DATA_WIDTH = 46,
    parameter int DEPTH      = 64,
    parameter int ADDR_WIDTH = (DEPTH > 1) ? $clog2(DEPTH) : 1,
    parameter     RAM_STYLE  = "block" // "block", "distributed", or "ultra"
) (
    input  logic                  clk,

    // Port A: Synchronous Dedicated Write Port
    input  logic                  wr_en,
    input  logic [ADDR_WIDTH-1:0] wr_addr,
    input  logic [DATA_WIDTH-1:0] wr_data,

    // Port B: Synchronous Dedicated Read Port
    input  logic                  rd_en,
    input  logic [ADDR_WIDTH-1:0] rd_addr,
    output logic [DATA_WIDTH-1:0] rd_data
);

    // Dedicated 2-Port Memory Array
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


// =============================================================================
// Top-Level BRAM-Friendly FWFT FIFO Controller
// =============================================================================
module keypoint_fifo #(
    parameter int DATA_WIDTH         = 46, // [45:44]=level, [43:38]=orientation, [37:22]=score, [21:10]=y, [9:0]=x
    parameter int DEPTH              = 64,
    parameter int ALMOST_FULL_THRESH = DEPTH - 4,
    parameter int ADDR_WIDTH         = (DEPTH > 1) ? $clog2(DEPTH) : 1,
    parameter     RAM_STYLE          = "block"
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
    output logic [ADDR_WIDTH:0]   count
);

    // RAM address pointers and unread tracker
    logic [ADDR_WIDTH-1:0] wr_ptr;
    logic [ADDR_WIDTH-1:0] rd_ptr;
    logic [ADDR_WIDTH:0]   bram_count;
    logic [ADDR_WIDTH:0]   fifo_count;

    // RAM interface signals
    logic                  bram_wr_en;
    logic                  bram_rd_en;
    logic [DATA_WIDTH-1:0] bram_rd_data;
    logic                  bram_rd_valid;

    // Output skid-buffer registers for FWFT operation
    logic [DATA_WIDTH-1:0] dout_reg;
    logic                  dout_valid;
    logic [DATA_WIDTH-1:0] skid_reg;
    logic                  skid_valid;

    // -------------------------------------------------------------------------
    // BRAM Submodule Instance
    // -------------------------------------------------------------------------
    assign bram_wr_en = wr_en && !full;

    keypoint_fifo_ram #(
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH(DEPTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .RAM_STYLE(RAM_STYLE)
    ) u_keypoint_ram (
        .clk(clk),
        .wr_en(bram_wr_en),
        .wr_addr(wr_ptr),
        .wr_data(din),
        .rd_en(bram_rd_en),
        .rd_addr(rd_ptr),
        .rd_data(bram_rd_data)
    );

    // -------------------------------------------------------------------------
    // Write Pointer Management
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= '0;
        end else if (bram_wr_en) begin
            wr_ptr <= (wr_ptr == DEPTH - 1) ? '0 : wr_ptr + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // BRAM Read Control & Skid Buffer Flow
    // The skid buffer has 2 slots: dout_reg (head) and skid_reg (backup).
    // A read from BRAM is issued whenever BRAM contains data and there will be
    // room in the skid buffer next cycle to receive the data.
    // -------------------------------------------------------------------------
    logic skid_ready;
    assign skid_ready = (!skid_valid && !bram_rd_valid) ||
                        (rd_en && (!skid_valid || !bram_rd_valid));

    assign bram_rd_en = (bram_count > 0) && skid_ready;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr        <= '0;
            bram_rd_valid <= 1'b0;
            bram_count    <= '0;
        end else begin
            bram_rd_valid <= bram_rd_en;

            if (bram_rd_en) begin
                rd_ptr <= (rd_ptr == DEPTH - 1) ? '0 : rd_ptr + 1'b1;
            end

            case ({bram_wr_en, bram_rd_en})
                2'b10: bram_count <= bram_count + 1'b1;
                2'b01: bram_count <= bram_count - 1'b1;
                default: ; // 2'b00 or 2'b11: count unchanged
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // FWFT Output Stage (Registers)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            dout_reg   <= '0;
            dout_valid <= 1'b0;
            skid_reg   <= '0;
            skid_valid <= 1'b0;
        end else begin
            if (bram_rd_valid) begin
                if (rd_en && dout_valid) begin
                    // Consumer popping current word: advance skid to dout, new BRAM data to skid
                    if (skid_valid) begin
                        dout_reg   <= skid_reg;
                        skid_reg   <= bram_rd_data;
                        skid_valid <= 1'b1;
                    end else begin
                        dout_reg   <= bram_rd_data;
                        skid_valid <= 1'b0;
                    end
                end else if (!dout_valid) begin
                    // Head was empty: prime dout directly
                    dout_reg   <= bram_rd_data;
                    dout_valid <= 1'b1;
                    skid_valid <= 1'b0;
                end else begin
                    // Head is full and consumer not reading: capture in skid register
                    skid_reg   <= bram_rd_data;
                    skid_valid <= 1'b1;
                end
            end else if (rd_en && dout_valid) begin
                // No incoming BRAM data, but consumer popped
                if (skid_valid) begin
                    dout_reg   <= skid_reg;
                    skid_valid <= 1'b0;
                end else begin
                    dout_valid <= 1'b0;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Total FIFO Item Counting and Status Flags
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            fifo_count <= '0;
            overflow   <= 1'b0;
            underflow  <= 1'b0;
        end else begin
            overflow  <= 1'b0;
            underflow <= 1'b0;

            case ({wr_en && !full, rd_en && !empty})
                2'b10: fifo_count <= fifo_count + 1'b1;
                2'b01: fifo_count <= fifo_count - 1'b1;
                default: ; // 2'b00 or 2'b11: count unchanged
            endcase

            if (wr_en && full)  overflow  <= 1'b1;
            if (rd_en && empty) underflow <= 1'b1;
        end
    end

    assign dout         = dout_reg;
    assign empty        = !dout_valid;
    assign full         = (fifo_count == DEPTH);
    assign almost_empty = (fifo_count <= 1);
    assign almost_full  = (fifo_count >= ALMOST_FULL_THRESH);
    assign count        = fifo_count;

endmodule
