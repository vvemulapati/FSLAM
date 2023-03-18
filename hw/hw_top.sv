`ifndef __FAST_SV__
`define __FAST_SV__

module fast #(
   parameter PIXEL_DEPTH = 8,
   // Only works with KERNEL_SIZE=7
   parameter KERNEL_SIZE = 7,
   // Pixels Per Cycle
   parameter PPC = 1
) (
   input bit clk,
   input bit rst_n,

   input logic [KERNEL_SIZE-1:0][KERNEL_SIZE-1 + (PPC-1): 0][PIXEL_DEPTH-1:0] in_data,
   output logic [PIXEL_DEPTH-1+5:0] out_pixel
);

localparam LENGTH = 16;
localparam N = 9; // Number of contiguous pixels to be considered a corner

// Flattening of the pixels in the Bresnaham circle
logic [PIXEL_DEPTH - 1: 0] pixel_circle[LENGTH];
// Difference between the center pixel and the Bresnaham circle
logic [PIXEL_DEPTH - 1: 0] pixel_diff[LENGTH];

// Pixel score is the sum of the absolute differences between the center pixels
// and the circle pixels
logic [PIXEL_DEPTH - 1 + 5: 0] pixel_score;

logic [LENGTH - 1: 0] brighter;
logic [2* LENGTH - 1: 0] brighter_brighter; // Duplication of signal for easy checking for contiguous values
logic [LENGTH - 1: 0] bright_bitmask;

logic [LENGTH - 1: 0] darker;
logic [2* LENGTH - 1: 0] darker_darker; // Duplication of signal for easy checking for contiguous values
logic [LENGTH - 1: 0] dark_bitmask;


// Boolean whether the pixel is a corner
logic is_corner;

wire [PIXEL_DEPTH-1:0] center_pixel = in_data[3][3];

always_comb begin
   pixel_circle[0] = in_data[0][3];
   pixel_circle[1] = in_data[0][4];
   pixel_circle[2] = in_data[1][5];
   pixel_circle[3] = in_data[2][6];
   pixel_circle[4] = in_data[3][6];
   pixel_circle[5] = in_data[4][6];
   pixel_circle[6] = in_data[5][5];
   pixel_circle[7] = in_data[6][4];
   pixel_circle[8] = in_data[6][3];
   pixel_circle[9] = in_data[6][2];

   pixel_circle[10] = in_data[5][1];
   pixel_circle[11] = in_data[4][0];
   pixel_circle[12] = in_data[3][0];
   pixel_circle[13] = in_data[2][0];
   pixel_circle[14] = in_data[1][1];
   pixel_circle[15] = in_data[0][2];
end

always_comb begin
   pixel_score = 0;
   for(int i = 0; i < LENGTH; i++) begin
      brighter[i] = center_pixel > pixel_circle[i];
      darker[i] = center_pixel <= pixel_circle[i];
      // Get the absolute difference
      pixel_diff[i] = (center_pixel > pixel_circle[i]) ? 
         (center_pixel - pixel_circle[i]) : (pixel_circle[i] - center_pixel);

      pixel_score += pixel_diff[i];
   end
end

assign brighter_brighter = {brighter, brighter};
assign darker_darker = {darker, darker};

always_comb begin

   // If any of the bitmasks are contiguous, it is a corner
   for(int i = 0; i < LENGTH; i++) begin
      bright_bitmask[i] = &brighter_brighter[i +: N];
      dark_bitmask[i] = &darker_darker[i +: N];
   end

   is_corner = (|bright_bitmask) | (|dark_bitmask);
end

always_ff @(posedge clk) begin
   if(~rst_n) begin
      out_pixel <= '0;
   end else begin
      out_pixel <= is_corner ? pixel_score : '0;
   end
end

endmodule

`endif

