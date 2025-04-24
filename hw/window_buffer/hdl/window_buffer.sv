//------------------------------------------------------------------------------
// File: line_window_buffer.sv
// Desc: Generic N×N sliding-window buffer: instantiates (WINDOW_SIZE-1) linebuffers and
//       builds a WIN_SIZE×WIN_SIZE neighborhood. Outputs `win[row][col]` and `out_valid`
//       once the window is “full.”
//------------------------------------------------------------------------------

module line_window_buffer #(
  parameter int DATA_WIDTH = 8,
  parameter int LINE_WIDTH = 6,
  parameter int WINDOW_SIZE   = 3
)(
  input  logic                     clk,
  input  logic                     rst,       // sync, active-high reset

  // incoming pixel stream
  input  logic                     in_valid,
  input  logic [DATA_WIDTH-1:0]    in_data,

  output logic                     out_valid,
  output logic [DATA_WIDTH-1:0]    win [WINDOW_SIZE][WINDOW_SIZE]
);

  // Number of line buffers needed
  localparam int N_LB   = WINDOW_SIZE - 1;

  logic [DATA_WIDTH-1:0] lb_data [N_LB];
  logic                 lb_valid[N_LB];

  logic out_valid_window[WINDOW_SIZE];
  assign out_valid = out_valid_window[0];

  for (genvar i = 0; i < N_LB; i++) begin : gen_linebufs
    wire in_v;
    wire [DATA_WIDTH-1:0] in_d;
    if(i == (N_LB - 1)) begin
        assign in_v = in_valid;
        assign in_d = in_data;
    end else begin
        assign in_v = lb_valid[i+1];
        assign in_d = lb_data[i+1];
    end
    linebuffer #(
      .DATA_WIDTH(DATA_WIDTH),
      .LINE_WIDTH(LINE_WIDTH-1)
    ) lb_inst (
      .clk      (clk),
      .rst      (rst),
      .in_valid(in_v),
      .in_data (in_d),
      .out_valid(lb_valid[i]),
      .out_data (lb_data[i])
    );
  end

  for (genvar i = 0; i < WINDOW_SIZE; i++) begin : gen_window
    wire in_v;
    wire [DATA_WIDTH-1:0] in_d;
    if(i == N_LB) begin
        assign in_v = in_valid;
        assign in_d = in_data;
    end else begin
        assign in_v = lb_valid[i];
        assign in_d = lb_data[i];
    end
    shift_register #(
      .DATA_WIDTH(DATA_WIDTH),
      .LINE_WIDTH(LINE_WIDTH),
      .REGISTER_WIDTH(WINDOW_SIZE)
    ) sr (
      .clk      (clk),
      .rst      (rst),
      .in_valid(in_v),
      .in_data (in_d),
      .out_valid(out_valid_window[i]),
      .out_data (win[i])
    );
  end


endmodule

