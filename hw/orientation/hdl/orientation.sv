// Keypoint orientation by intensity centroid over a 37x37 window, using running
// moment updates:
//   m00[n+1] = m00[n] + S(Cin) - S(Cout)
//   m01[n+1] = m01[n] + m01(Cin) - m01(Cout)
//   m10[n+1] = m10[n] + 18*S(Cin) + 19*S(Cout) - m00[n]
// theta = atan2(m01, m10) is quantized to 64 sectors across a 5-stage pipeline
// using quadrant signs and 15 tangent thresholds.

`timescale 1ns / 1ps

module orientation #(
    parameter int PIXEL_WIDTH = 6,
    parameter int WINDOW_SIZE = 37,
    parameter int IMG_WIDTH   = 640,
    parameter int ANGLE_BITS  = 6   // 64 sectors total (16 per quadrant)
) (
    input  logic                   clk,
    input  logic                   rst_n,

    input  logic                   keypoint_valid,
    input  logic [15:0]            keypoint_score,
    input  logic [11:0]            keypoint_x,
    input  logic [11:0]            keypoint_y,

    // Incoming column Cin from 37x37 window
    input  logic                   col_valid,
    input  logic                   win_valid,
    input  logic [PIXEL_WIDTH-1:0] col_in [WINDOW_SIZE-1:0],

    output logic [ANGLE_BITS-1:0]  orientation_out,
    output logic [15:0]            keypoint_score_out,
    output logic [11:0]            keypoint_x_out,
    output logic [11:0]            keypoint_y_out,
    output logic                   orientation_valid,
    output logic                   busy
);

    // Incoming column sum S(Cin) and vertical moment m01(Cin) (s1).
    // col_in[0] is dy = +18 (bottom), col_in[36] is dy = -18 (top).
    logic signed [15:0] s_cin_comb,   s_cin_q;
    logic signed [20:0] m01_cin_comb, m01_cin_q;
    logic signed [27:0] s_cin_x18_q;
    logic               s1_col_valid;
    logic               s1_kp_valid;
    logic [15:0]        s1_kp_score;
    logic [11:0]        s1_kp_x, s1_kp_y;

    always_comb begin
        s_cin_comb   = '0;
        m01_cin_comb = '0;
        for (int y = 0; y < WINDOW_SIZE; y++) begin
            s_cin_comb   += 16'($signed({1'b0, col_in[y]}));
            m01_cin_comb += 21'($signed({1'b0, col_in[y]}) * $signed(7'((WINDOW_SIZE / 2) - y)));
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s1_col_valid <= 1'b0;
            s_cin_q      <= '0;
            m01_cin_q    <= '0;
            s_cin_x18_q  <= '0;
            s1_kp_valid  <= 1'b0;
            s1_kp_score  <= '0;
            s1_kp_x      <= '0;
            s1_kp_y      <= '0;
        end else begin
            s1_col_valid <= col_valid;
            if (col_valid) begin
                s_cin_q     <= s_cin_comb;
                m01_cin_q   <= m01_cin_comb;
                // 18 * S(Cin) = (S(Cin) << 4) + (S(Cin) << 1)
                s_cin_x18_q <= (28'(s_cin_comb) <<< 4) + (28'(s_cin_comb) <<< 1);
            end

            s1_kp_valid <= keypoint_valid && win_valid;
            if (keypoint_valid && win_valid) begin
                s1_kp_score <= keypoint_score;
                s1_kp_x     <= keypoint_x;
                s1_kp_y     <= keypoint_y;
            end
        end
    end

    // Running 37-column moment accumulators and keypoint snapshot (s2)
    logic signed [15:0] s_cout_pipe   [0:WINDOW_SIZE-1];
    logic signed [20:0] m01_cout_pipe [0:WINDOW_SIZE-1];

    wire signed [15:0] s_cout       = s_cout_pipe[WINDOW_SIZE-1];
    wire signed [20:0] m01_cout     = m01_cout_pipe[WINDOW_SIZE-1];
    // 19 * S(Cout) = (S(Cout) << 4) + (S(Cout) << 1) + S(Cout)
    wire signed [27:0] s_cout_x19   = (28'(s_cout) <<< 4) + ((28'(s_cout) <<< 1) + 28'(s_cout));
    wire signed [27:0] m10_col_term = s_cin_x18_q + s_cout_x19;

    logic signed [23:0] m00_reg, m00_next;
    logic signed [27:0] m01_reg, m01_next;
    logic signed [27:0] m10_reg, m10_next;

    always_comb begin
        m00_next = m00_reg + (24'(s_cin_q) - 24'(s_cout));
        m01_next = m01_reg + (28'(m01_cin_q) - 28'(m01_cout));
        m10_next = (m10_reg - 28'(m00_reg)) + m10_col_term;
    end

    logic               s2_kp_valid;
    logic signed [27:0] s2_m01, s2_m10;
    logic [15:0]        s2_kp_score;
    logic [11:0]        s2_kp_x, s2_kp_y;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < WINDOW_SIZE; i++) begin
                s_cout_pipe[i]   <= '0;
                m01_cout_pipe[i] <= '0;
            end
            m00_reg     <= '0;
            m01_reg     <= '0;
            m10_reg     <= '0;
            s2_kp_valid <= 1'b0;
            s2_m01      <= '0;
            s2_m10      <= '0;
            s2_kp_score <= '0;
            s2_kp_x     <= '0;
            s2_kp_y     <= '0;
        end else begin
            if (s1_col_valid) begin
                s_cout_pipe[0]   <= s_cin_q;
                m01_cout_pipe[0] <= m01_cin_q;
                for (int i = 1; i < WINDOW_SIZE; i++) begin
                    s_cout_pipe[i]   <= s_cout_pipe[i-1];
                    m01_cout_pipe[i] <= m01_cout_pipe[i-1];
                end
                m00_reg <= m00_next;
                m01_reg <= m01_next;
                m10_reg <= m10_next;
            end

            s2_kp_valid <= s1_kp_valid;
            if (s1_kp_valid) begin
                s2_m01      <= m01_next;
                s2_m10      <= m10_next;
                s2_kp_score <= s1_kp_score;
                s2_kp_x     <= s1_kp_x;
                s2_kp_y     <= s1_kp_y;
            end
        end
    end

    // Quadrant signs and |m01|, |m10| magnitudes (s3)
    logic        s3_kp_valid;
    logic [1:0]  s3_quadrant;
    logic [27:0] s3_abs_m01, s3_abs_m10;
    logic [15:0] s3_kp_score;
    logic [11:0] s3_kp_x, s3_kp_y;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s3_kp_valid <= 1'b0;
            s3_quadrant <= '0;
            s3_abs_m01  <= '0;
            s3_abs_m10  <= '0;
            s3_kp_score <= '0;
            s3_kp_x     <= '0;
            s3_kp_y     <= '0;
        end else begin
            s3_kp_valid <= s2_kp_valid;
            if (s2_kp_valid) begin
                s3_abs_m01 <= (s2_m01 >= 0) ? 28'(s2_m01) : 28'(-s2_m01);
                s3_abs_m10 <= (s2_m10 >= 0) ? 28'(s2_m10) : 28'(-s2_m10);

                // Quadrant encoding: 0 = [0, 90), 1 = [90, 180), 2 = [180, 270), 3 = [270, 360)
                if (s2_m10 >= 0 && s2_m01 >= 0)      s3_quadrant <= 2'b00;
                else if (s2_m10 < 0 && s2_m01 >= 0)  s3_quadrant <= 2'b01;
                else if (s2_m10 < 0 && s2_m01 < 0)   s3_quadrant <= 2'b10;
                else                                 s3_quadrant <= 2'b11;

                s3_kp_score <= s2_kp_score;
                s3_kp_x     <= s2_kp_x;
                s3_kp_y     <= s2_kp_y;
            end
        end
    end

    // Parallel |m10| * round(256 * tan(theta)) products (s4)
    localparam logic [12:0] TAN_THRESH [0:14] = '{
        13'd25,   //  5.625 deg
        13'd51,   // 11.250 deg
        13'd78,   // 16.875 deg
        13'd106,  // 22.500 deg
        13'd137,  // 28.125 deg
        13'd171,  // 33.750 deg
        13'd210,  // 39.375 deg
        13'd256,  // 45.000 deg
        13'd312,  // 50.625 deg
        13'd383,  // 56.250 deg
        13'd479,  // 61.875 deg
        13'd618,  // 67.500 deg
        13'd844,  // 73.125 deg
        13'd1287, // 78.750 deg
        13'd2599  // 84.375 deg
    };

    logic        s4_kp_valid;
    logic [1:0]  s4_quadrant;
    logic [39:0] s4_prod [0:14];
    logic [39:0] s4_rhs;
    logic [15:0] s4_kp_score;
    logic [11:0] s4_kp_x, s4_kp_y;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s4_kp_valid <= 1'b0;
            s4_quadrant <= '0;
            s4_rhs      <= '0;
            for (int s = 0; s < 15; s++) begin
                s4_prod[s] <= '0;
            end
            s4_kp_score <= '0;
            s4_kp_x     <= '0;
            s4_kp_y     <= '0;
        end else begin
            s4_kp_valid <= s3_kp_valid;
            if (s3_kp_valid) begin
                s4_quadrant <= s3_quadrant;
                s4_rhs      <= (40'(s3_abs_m01) << 8);
                for (int s = 0; s < 15; s++) begin
                    s4_prod[s] <= 40'(s3_abs_m10) * 40'(TAN_THRESH[s]);
                end
                s4_kp_score <= s3_kp_score;
                s4_kp_x     <= s3_kp_x;
                s4_kp_y     <= s3_kp_y;
            end
        end
    end

    // Priority-encode sector within quadrant and register output
    logic [3:0] sector_in_quad;
    logic [3:0] quad_sector;

    always_comb begin
        sector_in_quad = 4'd15;
        for (int s = 0; s < 15; s++) begin
            if (s4_prod[s] >= s4_rhs) begin
                sector_in_quad = 4'(s);
                break;
            end
        end
        quad_sector = s4_quadrant[0] ? (4'd15 - sector_in_quad) : sector_in_quad;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            orientation_valid  <= 1'b0;
            orientation_out    <= '0;
            keypoint_score_out <= '0;
            keypoint_x_out     <= '0;
            keypoint_y_out     <= '0;
        end else begin
            orientation_valid <= s4_kp_valid;
            if (s4_kp_valid) begin
                orientation_out    <= {s4_quadrant, quad_sector};
                keypoint_score_out <= s4_kp_score;
                keypoint_x_out     <= s4_kp_x;
                keypoint_y_out     <= s4_kp_y;
            end
        end
    end

    assign busy = keypoint_valid || s1_kp_valid || s2_kp_valid || s3_kp_valid || s4_kp_valid || orientation_valid;

endmodule
