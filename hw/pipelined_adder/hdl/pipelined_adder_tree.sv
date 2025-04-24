//------------------------------------------------------------------------------
// File: pipelined_adder_tree.sv
// Desc: Parameterized N-input pipelined adder tree.
//       Builds a balanced binary tree of adders and inserts pipeline registers
//       at each tree level to achieve a fixed latency of LEVELS + 1 cycles.
//------------------------------------------------------------------------------

module pipelined_adder_tree #(
  parameter int DATA_WIDTH      = 8,      // bit-width of each input
  parameter int N_INPUTS   = 8,       // number of inputs to sum
  parameter int NUM_STAGES = $clog2(DATA_WIDTH)
)(
  input  logic                   clk,
  input  logic                   rst,       // synchronous active-high reset

  // input interface
  input  logic                   valid_in,
  input  logic [DATA_WIDTH-1:0]       in_data [N_INPUTS],

  // output interface
  output logic                   valid_out,
  output logic [DATA_WIDTH-1 + $clog2(N_INPUTS):0]       sum
);

  // number of tree levels = ceil(log2(N_INPUTS))
  localparam int LEVELS = $clog2(N_INPUTS);

  // create array to hold data at each level
  // max fan-in halves each level (ceil division)
  logic [DATA_WIDTH-1+$clog2(N_INPUTS):0] stage_data [0:LEVELS][0:N_INPUTS-1];
  logic             stage_valid[0:LEVELS];

  // stage 0: latch inputs
  always_ff @(posedge clk) begin
    if (rst) begin
      for (int i = 0; i < N_INPUTS; i++) begin
        stage_data[0][i]  <= '0;
      end
      stage_valid[0] <= 1'b0;
    end else begin
      for (int i = 0; i < N_INPUTS; i++) begin
        stage_data[0][i] <= in_data[i];
      end
      stage_valid[0] <= valid_in;
    end
  end

  // generate tree levels
    for (genvar lvl = 0; lvl < LEVELS; lvl++) begin : gen_levels
      // number of nodes at this level
      localparam int N_IN = (N_INPUTS + (1<<lvl) - 1) >> lvl;
      // number of adders = floor(N_IN/2)
      localparam int N_ADDS = N_IN >> 1;

      always_ff @(posedge clk) begin
        if (rst) begin
          stage_valid[lvl+1] <= 1'b0;
          // clear next stage data
          for (int j = 0; j < ((N_IN+1)>>1); j++) begin
            stage_data[lvl+1][j] <= '0;
          end
        end else begin
          // pairwise add
          for (int j = 0; j < N_ADDS; j++) begin
            stage_data[lvl+1][j] <= stage_data[lvl][2*j] + stage_data[lvl][2*j+1];
          end
          // if odd, pass through last element
          if (N_IN & 1) begin
            stage_data[lvl+1][N_ADDS] <= stage_data[lvl][2*N_ADDS];
          end
          // propagate valid
          stage_valid[lvl+1] <= stage_valid[lvl];
        end
      end
    end

  // final output
  // after LEVELS+1 cycles, stage_data[LEVELS][0] holds the sum of all inputs
  always_ff @(posedge clk) begin
    if (rst) begin
      valid_out <= 1'b0;
      sum   <= '0;
    end else begin
      valid_out <= stage_valid[LEVELS];
      sum   <= stage_data[LEVELS][0];
    end
  end

endmodule

