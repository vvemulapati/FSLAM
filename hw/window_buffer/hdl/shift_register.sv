
module shift_register #(
  // typed parameters
  parameter int DATA_WIDTH = 8,           
  parameter int LINE_WIDTH = 640,
  parameter int REGISTER_WIDTH = 3
) (
  input  logic                   clk,
  input  logic                   rst,       // sync, active-high reset

  // incoming pixel stream
  input  logic                     in_valid,
  input  logic [DATA_WIDTH-1:0]    in_data,

  // window valid + WIN_SIZE×WIN_SIZE outputs as 2D array
  output logic                     out_valid,
  output logic [DATA_WIDTH-1:0]    out_data [REGISTER_WIDTH]
);

  bit [REGISTER_WIDTH-1:0] out_valid_window;
  always_ff @(posedge clk) begin
    if (rst) begin
      out_valid_window  <= '0;
    end else begin
      if(in_valid) begin
        for(int i = 0; i < REGISTER_WIDTH; i++) begin
          //out_valid_window[i] <= (i == (REGISTER_WIDTH - 1)) ? in_valid : out_valid_window[i + 1];
          //out_data[i] <= (i == 0) ? in_data : out_data[i - 1];
          out_data[i] <= (i == (REGISTER_WIDTH - 1)) ? in_data : out_data[i+1];
          out_valid_window[i] <= (i == 0) ? in_valid : out_valid_window[i - 1];
        end
      end
    end
  end

  //assign out_valid = out_valid_window[0];
  assign out_valid = out_valid_window[REGISTER_WIDTH - 1];

endmodule
