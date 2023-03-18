`ifndef __NMS_SV__
`define __NMS_SV__

module nms #(
   parameter PIXEL_SCORE_DEPTH = 13,
   parameter NMS_DEPTH = 3
) (
   input bit clk,
   input bit rst_n,

   input logic [PIXEL_SCORE_DEPTH - 1: 0] in_score,
   output logic [PIXEL_SCORE_DEPTH - 1:0] out_score
);


// Just store the last DEPTH scores and only choose the biggest one

logic [NMS_DEPTH - 1: 0][PIXEL_SCORE_DEPTH - 1: 0] scores;

logic suppress;

// Find the greatest score amongs the scores. 
always_comb begin
   suppress = 1;
   for(int i = 0; i < NMS_DEPTH; i ++) begin
      if(scores[i] > in_score)
         suppress = 0;
   end
end

always_ff @(posedge clk) begin
   if(~rst_n) begin
      out_score <= '0;
   end else begin
      out_score <= suppress ? '0 : in_score;
   end
end

always_ff @(posedge clk) begin
   if(~rst_n) begin
      scores <= '0;
   end else begin
      scores <= {scores[NMS_DEPTH - 2: 0], in_score};
   end
end


endmodule

`endif


