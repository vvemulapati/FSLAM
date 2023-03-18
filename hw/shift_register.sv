`ifndef __SHIFT_REGISTER_SV__
`define __SHIFT_REGISTER_SV__

module shift_register #(
   parameter DATA_WIDTH = 8,
   parameter SIZE = 8
) (
   input bit clk,
   input bit rst_n,

   input logic shift_en,

   input logic [DATA_WIDTH-1:0] in_data,
   // Raw data
   output logic [SIZE-1: 0][DATA_WIDTH-1:0] data,
   output logic [DATA_WIDTH-1:0] out_data
);

always_ff @(posedge clk) begin
   if(~rst_n) begin
      data <= '0;
      out_data <= '0;
   end else begin
      // Shift data
      if(shift_en) begin
         data <= {data[SIZE-2:0], in_data};
         out_data <= data[SIZE-1];
      end
   end
end

endmodule

`endif


