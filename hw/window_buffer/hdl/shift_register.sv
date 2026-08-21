// Shift register holding the last REGISTER_WIDTH samples of one window row.

`timescale 1ns / 1ps

module shift_register #(
    parameter int DATA_WIDTH = 6,
    parameter int REGISTER_WIDTH = 7
) (
    input  logic                   clk,
    input  logic                   rst_n,

    input  logic                   in_valid,
    input  logic [DATA_WIDTH-1:0]  in_data,

    output logic                   out_valid,
    output logic [DATA_WIDTH-1:0]  out_data [REGISTER_WIDTH-1:0]
);

    logic [REGISTER_WIDTH-1:0] valid_pipe;
    logic                      last_in_valid;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_pipe    <= '0;
            last_in_valid <= 1'b0;
            for (int i = 0; i < REGISTER_WIDTH; i++) begin
                out_data[i] <= '0;
            end
        end else begin
            last_in_valid <= in_valid;
            if (in_valid) begin
                out_data[0] <= in_data;
                for (int i = 1; i < REGISTER_WIDTH; i++) begin
                    out_data[i] <= out_data[i-1];
                end
                valid_pipe <= {valid_pipe[REGISTER_WIDTH-2:0], 1'b1};
            end
        end
    end

    assign out_valid = valid_pipe[REGISTER_WIDTH-1] && last_in_valid;

endmodule
