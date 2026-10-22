// Keypoint orientation by intensity centroid over a 37x37 window, using running
// moment updates and a 3-stage pipeline:
//   Stage 1: 37-input column sums S(Cin) and m01(Cin) from shared 37x37 window
//   Stage 2: Recursive moment accumulators m00, m01, m10 + |m01|, |m10|, quadrant
//   Stage 3: 15-threshold tangent sector search and angle output (with max_fanout)

`timescale 1ns / 1ps

module orientation #(
    parameter int PIXEL_WIDTH = 6,
    parameter int WINDOW_SIZE = 37,
    parameter int IMG_WIDTH   = 640,
    parameter int ANGLE_BITS  = 6   // 64 sectors total (16 per quadrant)
) (
    input  logic                   clk,
    input  logic                   rst_n,

    // Keypoint trigger from NMS / FIFO dispatch
    input  logic                   keypoint_valid,
    input  logic [15:0]            keypoint_score,
    input  logic [11:0]            keypoint_x,
    input  logic [11:0]            keypoint_y,

    // Shared 37x1 incoming column Cin from feature_extractor's 37x37 window_buffer
    input  logic                   col_valid,
    input  logic                   win_valid,
    input  logic [PIXEL_WIDTH-1:0] col_in [WINDOW_SIZE-1:0],

    (* max_fanout = 16 *) output logic [ANGLE_BITS-1:0] orientation_out,
    output logic [15:0]            keypoint_score_out,
    output logic [11:0]            keypoint_x_out,
    output logic [11:0]            keypoint_y_out,
    output logic                   orientation_valid,
    output logic                   busy
);

    // -------------------------------------------------------------------------
    // Stage 1: Column sum S(Cin) and column moment m01(Cin)
    // col_in[0] is bottom-right (dy = +18, dx = +18), col_in[36] is top-right (dy = -18, dx = +18).
    // -------------------------------------------------------------------------
    logic signed [15:0] s_cin_comb,   s_cin_q;
    logic signed [20:0] m01_cin_comb, m01_cin_q;
    logic signed [27:0] s_cin_x18_q;
    logic               s1_col_valid;

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
        end else begin
            s1_col_valid <= col_valid;
            if (col_valid) begin
                s_cin_q     <= s_cin_comb;
                m01_cin_q   <= m01_cin_comb;
                // 18 * S(Cin) = (S(Cin) << 4) + (S(Cin) << 1)
                s_cin_x18_q <= (28'(s_cin_comb) <<< 4) + (28'(s_cin_comb) <<< 1);
            end
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: 37-step delay line for S(Cout), m01(Cout), recursive moments,
    //          and absolute value / quadrant decoding on keypoint_valid
    // -------------------------------------------------------------------------
    logic signed [15:0] s_cout_pipe   [0:WINDOW_SIZE-1];
    logic signed [20:0] m01_cout_pipe [0:WINDOW_SIZE-1];

    wire signed [15:0] s_cout     = s_cout_pipe[WINDOW_SIZE-1];
    wire signed [20:0] m01_cout   = m01_cout_pipe[WINDOW_SIZE-1];
    // 19 * S(Cout) = (S(Cout) << 4) + (S(Cout) << 1) + S(Cout)
    wire signed [27:0] s_cout_x19 = (28'(s_cout) <<< 4) + (28'(s_cout) <<< 1) + 28'(s_cout);

    logic signed [23:0] m00_reg, m00_next;
    logic signed [27:0] m01_reg, m01_next;
    logic signed [27:0] m10_reg, m10_next;

    logic [1:0]  quadrant_comb, quadrant_s2;
    logic [27:0] abs_m01_comb,  abs_m01_s2;
    logic [27:0] abs_m10_comb,  abs_m10_s2;
    logic        s2_kp_valid;
    logic [15:0] s2_kp_score;
    logic [11:0] s2_kp_x, s2_kp_y;

    always_comb begin
        m00_next = m00_reg + 24'(s_cin_q) - 24'(s_cout);
        m01_next = m01_reg + 28'(m01_cin_q) - 28'(m01_cout);
        m10_next = m10_reg + s_cin_x18_q + s_cout_x19 - 28'(m00_reg);

        abs_m01_comb = (m01_next >= 0) ? 28'(m01_next) : 28'(-m01_next);
        abs_m10_comb = (m10_next >= 0) ? 28'(m10_next) : 28'(-m10_next);

        // Quadrant encoding: 0 = [0, 90), 1 = [90, 180), 2 = [180, 270), 3 = [270, 360)
        if (m10_next >= 0 && m01_next >= 0)      quadrant_comb = 2'b00;
        else if (m10_next < 0 && m01_next >= 0)  quadrant_comb = 2'b01;
        else if (m10_next < 0 && m01_next < 0)   quadrant_comb = 2'b10;
        else                                     quadrant_comb = 2'b11;
    end

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
            s2_kp_score <= '0;
            s2_kp_x     <= '0;
            s2_kp_y     <= '0;
            abs_m01_s2  <= '0;
            abs_m10_s2  <= '0;
            quadrant_s2 <= '0;
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

            s2_kp_valid <= keypoint_valid && win_valid;
            if (keypoint_valid && win_valid) begin
                s2_kp_score <= keypoint_score;
                s2_kp_x     <= keypoint_x;
                s2_kp_y     <= keypoint_y;
                abs_m01_s2  <= abs_m01_comb;
                abs_m10_s2  <= abs_m10_comb;
                quadrant_s2 <= quadrant_comb;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Stage 3: 15-threshold tangent sector search and angle output
    // -------------------------------------------------------------------------
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

    logic [3:0] sector_in_quad;
    always_comb begin
        sector_in_quad = 4'd15; // default to last sector
        for (int s = 0; s < 15; s++) begin
            if ((40'(abs_m10_s2) * 40'(TAN_THRESH[s])) >= (40'(abs_m01_s2) << 8)) begin
                sector_in_quad = 4'(s);
                break;
            end
        end
    end

    // Invert sector_in_quad in Quadrant 1 ([90, 180)) and Quadrant 3 ([270, 360))
    // so the 64-sector index progresses monotonically counter-clockwise from 0..63.
    logic [3:0] quad_sector;
    logic [5:0] computed_angle;
    assign quad_sector    = quadrant_s2[0] ? (4'd15 - sector_in_quad) : sector_in_quad;
    assign computed_angle = {quadrant_s2, quad_sector};

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            orientation_out    <= '0;
            keypoint_score_out <= '0;
            keypoint_x_out     <= '0;
            keypoint_y_out     <= '0;
            orientation_valid  <= 1'b0;
        end else begin
            orientation_valid <= s2_kp_valid;
            if (s2_kp_valid) begin
                orientation_out    <= computed_angle;
                keypoint_score_out <= s2_kp_score;
                keypoint_x_out     <= s2_kp_x;
                keypoint_y_out     <= s2_kp_y;
            end
        end
    end

    assign busy = keypoint_valid || s2_kp_valid || orientation_valid;

endmodule
