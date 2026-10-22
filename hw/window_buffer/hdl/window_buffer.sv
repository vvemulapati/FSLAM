// WINDOW_SIZE x WINDOW_SIZE sliding window: WINDOW_SIZE-1 line buffers plus one
// shift register per row. Uses registered per-row warmup flags (row_filled) and
// window_filled so each row enable is a single 2-input AND gate (1 LUT level)
// instead of an N_LB-stage combinational ripple chain.

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

    localparam int N_LB          = WINDOW_SIZE - 1;
    localparam int WARMUP_PIXELS = (WINDOW_SIZE - 1) * LINE_WIDTH + (WINDOW_SIZE - 1);
    localparam int CNT_WIDTH     = $clog2(WARMUP_PIXELS + 1);

    logic [DATA_WIDTH-1:0] lb_out_data  [0:N_LB-1];
    logic                  sr_out_valid [0:WINDOW_SIZE-1];

    // Registered warmup flags: row_filled[r] asserts once (r+1)*LINE_WIDTH
    // valid pixels have arrived, flattening the N_LB-stage ripple-AND chain
    // into a single 2-input AND per row.
    logic [CNT_WIDTH-1:0] fill_cnt;
    logic                 row_filled [0:N_LB-1];
    logic                 window_filled;
    logic                 in_valid_d1;
    logic                 row_valid  [0:N_LB-1];

    always_comb begin
        for (int r = 0; r < N_LB; r++) begin
            row_valid[r] = in_valid && row_filled[r];
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            fill_cnt      <= '0;
            window_filled <= 1'b0;
            in_valid_d1   <= 1'b0;
            for (int r = 0; r < N_LB; r++) begin
                row_filled[r] <= 1'b0;
            end
        end else begin
            in_valid_d1 <= in_valid && (window_filled || (fill_cnt == CNT_WIDTH'(WARMUP_PIXELS)));
            if (in_valid && !window_filled) begin
                for (int r = 0; r < N_LB; r++) begin
                    if (fill_cnt == CNT_WIDTH'((r + 1) * LINE_WIDTH - 1)) begin
                        row_filled[r] <= 1'b1;
                    end
                end
                if (fill_cnt == CNT_WIDTH'(WARMUP_PIXELS)) begin
                    window_filled <= 1'b1;
                end else begin
                    fill_cnt <= fill_cnt + 1'b1;
                end
            end
        end
    end

    // Instantiate line buffers in series, each enabled by single-LUT row_valid[i-1]
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
                    .out_valid(),
                    .out_data(lb_out_data[0])
                );
            end else begin : gen_next_lb
                linebuffer #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .LINE_WIDTH(LINE_WIDTH)
                ) u_linebuf (
                    .clk(clk),
                    .rst_n(rst_n),
                    .in_valid(row_valid[i-1]),
                    .in_data(lb_out_data[i-1]),
                    .out_valid(),
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
                    .in_valid(row_valid[i-1]),
                    .in_data(lb_out_data[i-1]),
                    .out_valid(sr_out_valid[i]),
                    .out_data(win[i])
                );
            end
        end
    endgenerate

    assign out_valid = in_valid_d1;

endmodule
