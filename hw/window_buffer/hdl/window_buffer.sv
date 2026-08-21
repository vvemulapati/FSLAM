//------------------------------------------------------------------------------
// File: window_buffer.sv
// Description: Generic N×N sliding 2D window buffer. Instantiates (WINDOW_SIZE-1)
//              line buffers and N shift registers to expose a 2D pixel neighborhood.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module window_buffer #(
    parameter int DATA_WIDTH  = 6,
    parameter int LINE_WIDTH  = 640,
    parameter int WINDOW_SIZE = 7
) (
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic                  in_valid,
    input  logic [DATA_WIDTH-1:0] in_data,

    output logic                  out_valid,
    output logic [DATA_WIDTH-1:0] win [WINDOW_SIZE-1:0][WINDOW_SIZE-1:0]
);

    localparam int N_LB = WINDOW_SIZE - 1;

    logic [DATA_WIDTH-1:0] lb_out_data  [0:N_LB-1];
    logic                  lb_out_valid [0:N_LB-1];
    logic                  sr_out_valid [0:WINDOW_SIZE-1];

    // Instantiate line buffers in series
    genvar i;
    generate
        for (i = 0; i < N_LB; i++) begin : gen_linebufs
            if (i == 0) begin : gen_first_lb
                linebuffer #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .LINE_WIDTH(LINE_WIDTH)
                ) u_linebuf (
                    .clk(clk),
                    .rst_n(rst_n),
                    .in_valid(in_valid),
                    .in_data(in_data),
                    .out_valid(lb_out_valid[0]),
                    .out_data(lb_out_data[0])
                );
            end else begin : gen_next_lb
                linebuffer #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .LINE_WIDTH(LINE_WIDTH)
                ) u_linebuf (
                    .clk(clk),
                    .rst_n(rst_n),
                    .in_valid(lb_out_valid[i-1]),
                    .in_data(lb_out_data[i-1]),
                    .out_valid(lb_out_valid[i]),
                    .out_data(lb_out_data[i])
                );
            end
        end
    endgenerate

    // Instantiate 1D shift register for each row (row 0 is newest, row N_LB is oldest)
    generate
        for (i = 0; i < WINDOW_SIZE; i++) begin : gen_shift_registers
            if (i == 0) begin : gen_first_sr
                shift_register #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .REGISTER_WIDTH(WINDOW_SIZE)
                ) u_sr (
                    .clk(clk),
                    .rst_n(rst_n),
                    .in_valid(in_valid),
                    .in_data(in_data),
                    .out_valid(sr_out_valid[0]),
                    .out_data(win[0])
                );
            end else begin : gen_next_sr
                shift_register #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .REGISTER_WIDTH(WINDOW_SIZE)
                ) u_sr (
                    .clk(clk),
                    .rst_n(rst_n),
                    .in_valid(lb_out_valid[i-1]),
                    .in_data(lb_out_data[i-1]),
                    .out_valid(sr_out_valid[i]),
                    .out_data(win[i])
                );
            end
        end
    endgenerate

    assign out_valid = sr_out_valid[WINDOW_SIZE-1];

endmodule
