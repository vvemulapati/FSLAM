//------------------------------------------------------------------------------
// File: linebuffer.sv
// Desc: Simple single-line buffer: delays an input stream by exactly LINE_WIDTH samples.
//------------------------------------------------------------------------------

module linebuffer #(
  // typed parameters
  parameter int DATA_WIDTH = 8,           
  parameter int LINE_WIDTH = 640          
) (
  input  logic                   clk,
  input  logic                   rst,        // synchronous active‐high reset

  // streaming pixel in
  input  logic                   in_valid,
  input  logic [DATA_WIDTH-1:0]  in_data,

  // delayed pixel out
  output logic                   out_valid,
  output logic [DATA_WIDTH-1:0]  out_data
);

  // derive address width automatically
  localparam int ADDR_WIDTH = $clog2(LINE_WIDTH);

  // Storage rounded to nearest power of 2
  logic [DATA_WIDTH-1:0] mem[LINE_WIDTH];

  // pointers and counter
  logic [ADDR_WIDTH-1:0]                write_ptr, read_ptr;
  int   sample_cnt;

  always_ff @(posedge clk) begin
    if (rst) begin
      write_ptr  <= '0;
      read_ptr   <= '0;
      sample_cnt <= '0;
      out_valid  <= 1'b0;
      out_data   <= '0;
    end else begin
      // default: assume no valid output unless we drive it below
      out_valid <= 1'b0;

      /* verilator lint_off WIDTHEXPAND */
      if (in_valid) begin
        // write the new pixel into RAM
        mem[write_ptr] <= in_data;
        write_ptr      <= (write_ptr >= (LINE_WIDTH - 1)) ? '0 : write_ptr + 1;

        // count up until we've filled one line
        if (sample_cnt < LINE_WIDTH) begin
          sample_cnt <= sample_cnt + 1;
		end else begin
        // once full, start reading back delayed pixels
          out_data  <= mem[read_ptr];
          read_ptr  <= (read_ptr >= (LINE_WIDTH - 1)) ? '0 : read_ptr + 1;
          out_valid <= 1'b1;
        end
      end
      /* verilator lint_on WIDTHEXPAND */
    end
  end

endmodule


