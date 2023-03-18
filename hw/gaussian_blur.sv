`ifndef __GAUSSIAN_BLUR_SV__
`define __GAUSSIAN_BLUR_SV__

module gaussian_blur #(
   parameter PIXEL_DEPTH = 8,
   // Pixels Per Cycle
   parameter KERNEL_SIZE = 7,
   parameter PPC = 1
) (
   input bit clk,
   input bit rst_n,

   input logic [KERNEL_SIZE-1:0][KERNEL_SIZE-1 + (PPC-1): 0][PIXEL_DEPTH-1:0] in_data,
   output logic [PIXEL_DEPTH-1:0] out_pixel
);

int kernel[KERNEL_SIZE][KERNEL_SIZE] = '{
   '{1, 6, 15, 20, 15, 6, 1},
   '{6, 36, 90, 120, 90, 36, 6},
   '{15, 90, 225, 300, 225, 90, 15},
   '{20, 120, 300, 400, 300, 120, 20},
   '{15, 90, 225, 300, 225, 90, 15},
   '{6, 36, 90, 120, 90, 36, 6},
   '{1, 6, 15, 20, 15, 6, 1}
};

logic [PIXEL_DEPTH + 10: 0] temp_data;

always_comb begin
   temp_data = 0;
   for(int i = 0; i < KERNEL_SIZE; i++)
      for(int j = 0; j < KERNEL_SIZE; j++)
         temp_data += in_data[i][j] * kernel[i][j];
end

always_ff @(posedge clk) begin
   if(~rst_n) begin
      out_pixel <= '0;
   end else begin
      // Divide by 4096
      out_pixel <= temp_data >> 12;
   end
end

endmodule

`endif

