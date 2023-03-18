`ifndef __LINEBUFFER_SV__
`define __LINEBUFFER_SV__

module linebuffer #(
   parameter PIXEL_DEPTH = 8,
   parameter WIDTH = 8,
   parameter NUM_LINES = 1
) (
   input bit clk,
   input bit rst_n,

   input logic [PIXEL_DEPTH-1:0] in_pixel,
   output logic [PIXEL_DEPTH-1:0] out_pixel
);

logic [PIXEL_DEPTH-1:0] out_data[NUM_LINES];
logic [PIXEL_DEPTH-1:0] in_data[NUM_LINES];

assign in_data[0] = in_pixel;

assign out_pixel = out_data[NUM_LINES-1];

always_comb begin
   for(int i = 1; i < NUM_LINES; i++) begin
      in_data[i] = out_data[i-1];
   end
end

generate 
for(genvar i = 0; i < NUM_LINES; i++) begin
   shift_register #(.DATA_WIDTH(PIXEL_DEPTH), .SIZE(WIDTH)) line_i (
      .clk,
      .rst_n,
      .shift_en(1'b1),

      .in_data(in_data[i]),
      .out_data(out_data[i])
   );

end
endgenerate

endmodule

`endif

