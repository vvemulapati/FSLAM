// Keypoint orientation by intensity centroid over a 37x37 window, using running
// moment updates:
//   m00[n+1] = m00[n] + S(Cin) - S(Cout)
//   m01[n+1] = m01[n] + m01(Cin) - m01(Cout)
//   m10[n+1] = m10[n] + 18*S(Cin) + 19*S(Cout) - m00[n]
// theta = atan2(m01, m10) is quantized to 64 sectors using the quadrant signs
// and 15 tangent thresholds. Keypoint x, y and score are passed through with
// orientation_valid.

`timescale 1ns / 1ps

module orientation #(
    parameter int PIXEL_WIDTH   = 6,
    parameter int WINDOW_SIZE   = 37,
    parameter int IMG_WIDTH     = 640,
    parameter int ANGLE_BITS    = 6   // 64 sectors total (16 per quadrant)
) (
    input  logic                   clk,
    input  logic                   rst_n,

    // Keypoint trigger from NMS
    input  logic                   keypoint_valid,
    input  logic [15:0]            keypoint_score,
    input  logic [11:0]            keypoint_x,
    input  logic [11:0]            keypoint_y,

    // Smoothed pixel stream from Gaussian blur
    input  logic                   pixel_valid,
    input  logic [PIXEL_WIDTH-1:0] pixel_in,

    output logic [ANGLE_BITS-1:0]  orientation_out,
    output logic [15:0]            keypoint_score_out,
    output logic [11:0]            keypoint_x_out,
    output logic [11:0]            keypoint_y_out,
    output logic                   orientation_valid,
    output logic                   busy
);

    // 37-row window buffer to access 37x1 incoming column Cin
    logic [PIXEL_WIDTH-1:0] win [WINDOW_SIZE-1:0][WINDOW_SIZE-1:0];
    logic                   win_valid;

    window_buffer #(
        .DATA_WIDTH(PIXEL_WIDTH),
        .LINE_WIDTH(IMG_WIDTH),
        .WINDOW_SIZE(WINDOW_SIZE)
    ) u_win_buf (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(pixel_valid),
        .in_data(pixel_in),
        .out_valid(win_valid),
        .win(win)
    );

    logic pixel_valid_d1;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            pixel_valid_d1 <= 1'b0;
        end else begin
            pixel_valid_d1 <= pixel_valid;
        end
    end

    // Column sum S(Cin) and column moment m01(Cin).
    // win[0][0] is bottom-right (dy = +18, dx = +18), win[36][0] is top-right (dy = -18, dx = +18).
    logic signed [15:0] s_cin;
    logic signed [20:0] m01_cin;

    always_comb begin
        s_cin   = '0;
        m01_cin = '0;
        for (int y = 0; y < WINDOW_SIZE; y++) begin
            s_cin   += 16'($signed({1'b0, win[y][0]}));
            m01_cin += 21'($signed({1'b0, win[y][0]}) * $signed(7'((WINDOW_SIZE / 2) - y)));
        end
    end

    // Delay line of 37 valid steps to produce S(Cout) and m01(Cout)
    logic signed [15:0] s_cout_pipe [0:WINDOW_SIZE-1];
    logic signed [20:0] m01_cout_pipe [0:WINDOW_SIZE-1];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < WINDOW_SIZE; i++) begin
                s_cout_pipe[i]   <= '0;
                m01_cout_pipe[i] <= '0;
            end
        end else if (pixel_valid_d1) begin
            s_cout_pipe[0]   <= s_cin;
            m01_cout_pipe[0] <= m01_cin;
            for (int i = 1; i < WINDOW_SIZE; i++) begin
                s_cout_pipe[i]   <= s_cout_pipe[i-1];
                m01_cout_pipe[i] <= m01_cout_pipe[i-1];
            end
        end
    end

    wire signed [15:0] s_cout   = s_cout_pipe[WINDOW_SIZE-1];
    wire signed [20:0] m01_cout = m01_cout_pipe[WINDOW_SIZE-1];

    // Recursive moment tracking registers
    logic signed [23:0] m00_reg, m00_next;
    logic signed [27:0] m01_reg, m01_next;
    logic signed [27:0] m10_reg, m10_next;

    always_comb begin
        m00_next = m00_reg + 24'(s_cin) - 24'(s_cout);
        m01_next = m01_reg + 28'(m01_cin) - 28'(m01_cout);
        m10_next = m10_reg + (28'(s_cin) * 28'sd18) + (28'(s_cout) * 28'sd19) - 28'(m00_reg);
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m00_reg <= '0;
            m01_reg <= '0;
            m10_reg <= '0;
        end else if (pixel_valid_d1) begin
            m00_reg <= m00_next;
            m01_reg <= m01_next;
            m10_reg <= m10_next;
        end
    end

    // Quadrant and Angle sector calculation
    logic [1:0]  quadrant;
    logic [27:0] abs_m01, abs_m10;

    always_comb begin
        abs_m01 = (m01_next >= 0) ? 28'(m01_next) : 28'(-m01_next);
        abs_m10 = (m10_next >= 0) ? 28'(m10_next) : 28'(-m10_next);

        // Quadrant encoding: 0 = [0, 90), 1 = [90, 180), 2 = [180, 270), 3 = [270, 360)
        if (m10_next >= 0 && m01_next >= 0)      quadrant = 2'b00;
        else if (m10_next < 0 && m01_next >= 0)  quadrant = 2'b01;
        else if (m10_next < 0 && m01_next < 0)   quadrant = 2'b10;
        else                                     quadrant = 2'b11;
    end

    // 15 tangent threshold multipliers (tan(theta) * 256) for 16 sectors per quadrant
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
            if ((40'(abs_m10) * 40'(TAN_THRESH[s])) >= (40'(abs_m01) << 8)) begin
                sector_in_quad = 4'(s);
                break;
            end
        end
    end

    logic [5:0] computed_angle;
    assign computed_angle = {quadrant, sector_in_quad};

    // Output stage gated by keypoint trigger
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            orientation_out    <= '0;
            keypoint_score_out <= '0;
            keypoint_x_out     <= '0;
            keypoint_y_out     <= '0;
            orientation_valid  <= 1'b0;
            busy               <= 1'b0;
        end else begin
            orientation_valid  <= keypoint_valid && win_valid;
            if (keypoint_valid && win_valid) begin
                orientation_out    <= computed_angle;
                keypoint_score_out <= keypoint_score;
                keypoint_x_out     <= keypoint_x;
                keypoint_y_out     <= keypoint_y;
            end
            busy <= 1'b0;
        end
    end

endmodule
