// Pipelined rBRIEF descriptor generator (256 bits).
// Buffers the 37-row blurred stream in a 64-column circular distributed RAM
// and evaluates PAIRS_PER_CYCLE test pairs per clock cycle across a 4-stage
// pipeline (pattern fetch -> coordinate rotation -> LUTRAM read -> row mux &
// compare).

`timescale 1ns / 1ps

module brief_col_sdp_lutram #(
    parameter int DATA_WIDTH = 6
) (
    input  logic                  clk,
    input  logic                  wr_en,
    input  logic [5:0]            wr_addr,
    input  logic [DATA_WIDTH-1:0] wr_data,
    input  logic [5:0]            rd_addr,
    output logic [DATA_WIDTH-1:0] rd_data
);

    (* ram_style = "distributed" *)
    logic [DATA_WIDTH-1:0] mem [0:63];

    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
        rd_data <= mem[rd_addr];
    end

endmodule


module brief #(
    parameter int PIXEL_WIDTH     = 6,
    parameter int WINDOW_SIZE     = 37,
    parameter int DESCRIPTOR_BITS = 256,
    parameter int ANGLE_BITS      = 6,
    parameter int COORD_BITS      = 6,
    parameter int PAIRS_PER_CYCLE = 2,
    parameter int ORI_LATENCY     = 5
) (
    input  logic                                  clk,
    input  logic                                  rst_n,

    // Incoming 37-pixel blurred column
    input  logic                                  col_valid,
    input  logic [PIXEL_WIDTH-1:0]                col_in [WINDOW_SIZE-1:0],

    // Keypoint trigger (patch_latch ORI_LATENCY cycles ahead of start)
    input  logic                                  patch_latch,
    input  logic                                  start,
    input  logic [15:0]                           keypoint_score_in,
    input  logic [11:0]                           keypoint_x_in,
    input  logic [11:0]                           keypoint_y_in,
    input  logic [ANGLE_BITS-1:0]                 orientation,

    // 256 test pairs (x1, y1, x2, y2), coordinates in [-13, 12]
    input  logic signed [COORD_BITS-1:0]          pattern_coords [0:DESCRIPTOR_BITS-1][0:3],

    output logic [DESCRIPTOR_BITS-1:0]            descriptor,
    output logic [15:0]                           keypoint_score_out,
    output logic [11:0]                           keypoint_x_out,
    output logic [11:0]                           keypoint_y_out,
    output logic [ANGLE_BITS-1:0]                 orientation_out,
    output logic                                  descriptor_valid,
    output logic                                  stall_req,
    output logic                                  busy
);

    localparam int WINDOW_CENTER  = WINDOW_SIZE / 2; // 18
    localparam int N_STEPS        = DESCRIPTOR_BITS / PAIRS_PER_CYCLE;
    localparam int STEP_BITS      = (N_STEPS > 1) ? $clog2(N_STEPS) : 1;
    localparam int NUM_READ_PORTS = 2 * PAIRS_PER_CYCLE;
    localparam int CMD_DEPTH      = 16;
    localparam int CMD_WIDTH      = 6 + ANGLE_BITS + 16 + 12 + 12; // 52 bits

    // 64-sector Cosine and Sine LUTs (Q1.7 format)
    localparam logic signed [7:0] COS_LUT [0:63] = '{
         8'sd127,  8'sd126,  8'sd125,  8'sd122,  8'sd117,  8'sd112,  8'sd106,  8'sd98,
         8'sd90,   8'sd81,   8'sd71,   8'sd60,   8'sd49,   8'sd37,   8'sd25,   8'sd12,
         8'sd0,   -8'sd12,  -8'sd25,  -8'sd37,  -8'sd49,  -8'sd60,  -8'sd71,  -8'sd81,
        -8'sd90,  -8'sd98,  -8'sd106, -8'sd112, -8'sd117, -8'sd122, -8'sd125, -8'sd126,
        -8'sd127, -8'sd126, -8'sd125, -8'sd122, -8'sd117, -8'sd112, -8'sd106, -8'sd98,
        -8'sd90,  -8'sd81,  -8'sd71,  -8'sd60,  -8'sd49,  -8'sd37,  -8'sd25,  -8'sd12,
         8'sd0,    8'sd12,   8'sd25,   8'sd37,   8'sd49,   8'sd60,   8'sd71,   8'sd81,
         8'sd90,   8'sd98,   8'sd106,  8'sd112,  8'sd117,  8'sd122,  8'sd125,  8'sd126
    };

    localparam logic signed [7:0] SIN_LUT [0:63] = '{
         8'sd0,    8'sd12,   8'sd25,   8'sd37,   8'sd49,   8'sd60,   8'sd71,   8'sd81,
         8'sd90,   8'sd98,   8'sd106,  8'sd112,  8'sd117,  8'sd122,  8'sd125,  8'sd126,
         8'sd127,  8'sd126,  8'sd125,  8'sd122,  8'sd117,  8'sd112,  8'sd106,  8'sd98,
         8'sd90,   8'sd81,   8'sd71,   8'sd60,   8'sd49,   8'sd37,   8'sd25,   8'sd12,
         8'sd0,   -8'sd12,  -8'sd25,  -8'sd37,  -8'sd49,  -8'sd60,  -8'sd71,  -8'sd81,
        -8'sd90,  -8'sd98,  -8'sd106, -8'sd112, -8'sd117, -8'sd122, -8'sd125, -8'sd126,
        -8'sd127, -8'sd126, -8'sd125, -8'sd122, -8'sd117, -8'sd112, -8'sd106, -8'sd98,
        -8'sd90,  -8'sd81,  -8'sd71,  -8'sd60,  -8'sd49,  -8'sd37,  -8'sd25,  -8'sd12
    };

    // Circular column write pointer and patch right-edge delay pipe aligned with orientation
    logic [5:0]             col_wr_ptr;
    logic [ORI_LATENCY-1:0] patch_pipe_valid;
    logic [5:0]             patch_pipe_ptr [0:ORI_LATENCY-1];
    wire  [5:0]             pending_right_ptr = patch_pipe_ptr[ORI_LATENCY-1];
    wire                    patch_pending     = |patch_pipe_valid;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            col_wr_ptr       <= '0;
            patch_pipe_valid <= '0;
            for (int i = 0; i < ORI_LATENCY; i++) begin
                patch_pipe_ptr[i] <= '0;
            end
        end else begin
            if (col_valid) begin
                col_wr_ptr <= col_wr_ptr + 6'd1;
            end
            patch_pipe_valid[0] <= patch_latch;
            if (patch_latch) begin
                patch_pipe_ptr[0] <= col_wr_ptr;
            end
            for (int i = 1; i < ORI_LATENCY; i++) begin
                patch_pipe_valid[i] <= patch_pipe_valid[i-1];
                patch_pipe_ptr[i]   <= patch_pipe_ptr[i-1];
            end
        end
    end

    // 64-column distributed LUTRAM bank per row and read port
    logic [5:0]             ram_rd_addr [0:NUM_READ_PORTS-1];
    logic [PIXEL_WIDTH-1:0] ram_rd_data [0:NUM_READ_PORTS-1][0:WINDOW_SIZE-1];

    genvar gp, gr;
    generate
        for (gp = 0; gp < NUM_READ_PORTS; gp++) begin : gen_port
            for (gr = 0; gr < WINDOW_SIZE; gr++) begin : gen_row
                brief_col_sdp_lutram #(
                    .DATA_WIDTH(PIXEL_WIDTH)
                ) u_col_ram (
                    .clk(clk),
                    .wr_en(col_valid),
                    .wr_addr(col_wr_ptr),
                    .wr_data(col_in[gr]),
                    .rd_addr(ram_rd_addr[gp]),
                    .rd_data(ram_rd_data[gp][gr])
                );
            end
        end
    endgenerate

    // Small command FIFO to absorb keypoints arriving while pipeline drains
    (* ram_style = "distributed" *)
    logic [CMD_WIDTH-1:0] cmd_mem [0:CMD_DEPTH-1];
    logic [3:0]           cmd_wr_ptr, cmd_rd_ptr;
    logic [4:0]           cmd_count;
    wire                  cmd_empty = (cmd_count == 5'd0);
    wire                  cmd_full  = (cmd_count == 5'(CMD_DEPTH));

    logic                 engine_running;
    logic                 s0_valid, s1_valid, s2_valid;
    wire                  kp_busy   = engine_running || s0_valid || s1_valid || s2_valid;
    wire                  cmd_push  = start && !cmd_full;
    wire                  cmd_pop   = !kp_busy && !cmd_empty;
    wire  [CMD_WIDTH-1:0] cmd_head  = cmd_mem[cmd_rd_ptr];

    always_ff @(posedge clk) begin
        if (cmd_push) begin
            cmd_mem[cmd_wr_ptr] <= {pending_right_ptr, orientation, keypoint_score_in, keypoint_y_in, keypoint_x_in};
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cmd_wr_ptr <= '0;
            cmd_rd_ptr <= '0;
            cmd_count  <= '0;
        end else begin
            if (cmd_push) begin
                cmd_wr_ptr <= cmd_wr_ptr + 4'd1;
            end
            if (cmd_pop) begin
                cmd_rd_ptr <= cmd_rd_ptr + 4'd1;
            end
            case ({cmd_push, cmd_pop})
                2'b10:   cmd_count <= cmd_count + 5'd1;
                2'b01:   cmd_count <= cmd_count - 5'd1;
                default: ;
            endcase
        end
    end

    // Keypoint context and pattern pair fetch (s0)
    logic [5:0]               active_right_ptr;
    logic [ANGLE_BITS-1:0]    active_orientation;
    logic [15:0]              active_score;
    logic [11:0]              active_x, active_y;
    logic signed [7:0]        active_cos, active_sin;

    logic [STEP_BITS-1:0]     step_idx;
    logic                     s0_last;
    logic signed [COORD_BITS-1:0] s0_x1 [0:PAIRS_PER_CYCLE-1];
    logic signed [COORD_BITS-1:0] s0_y1 [0:PAIRS_PER_CYCLE-1];
    logic signed [COORD_BITS-1:0] s0_x2 [0:PAIRS_PER_CYCLE-1];
    logic signed [COORD_BITS-1:0] s0_y2 [0:PAIRS_PER_CYCLE-1];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            engine_running     <= 1'b0;
            step_idx           <= '0;
            active_right_ptr   <= '0;
            active_orientation <= '0;
            active_score       <= '0;
            active_x           <= '0;
            active_y           <= '0;
            active_cos         <= '0;
            active_sin         <= '0;
            s0_valid           <= 1'b0;
            s0_last            <= 1'b0;
            for (int p = 0; p < PAIRS_PER_CYCLE; p++) begin
                s0_x1[p] <= '0;
                s0_y1[p] <= '0;
                s0_x2[p] <= '0;
                s0_y2[p] <= '0;
            end
        end else begin
            if (cmd_pop) begin
                active_right_ptr   <= cmd_head[51:46];
                active_orientation <= cmd_head[45:40];
                active_score       <= cmd_head[39:24];
                active_y           <= cmd_head[23:12];
                active_x           <= cmd_head[11:0];
                active_cos         <= COS_LUT[cmd_head[45:40]];
                active_sin         <= SIN_LUT[cmd_head[45:40]];
                engine_running     <= 1'b1;
                step_idx           <= '0;
                s0_valid           <= 1'b0;
                s0_last            <= 1'b0;
            end else if (engine_running) begin
                s0_valid <= 1'b1;
                s0_last  <= (step_idx == STEP_BITS'(N_STEPS - 1));
                for (int p = 0; p < PAIRS_PER_CYCLE; p++) begin
                    automatic int idx = int'(step_idx) * PAIRS_PER_CYCLE + p;
                    s0_x1[p] <= pattern_coords[idx][0];
                    s0_y1[p] <= pattern_coords[idx][1];
                    s0_x2[p] <= pattern_coords[idx][2];
                    s0_y2[p] <= pattern_coords[idx][3];
                end
                if (step_idx == STEP_BITS'(N_STEPS - 1)) begin
                    engine_running <= 1'b0;
                end else begin
                    step_idx <= step_idx + 1'b1;
                end
            end else begin
                s0_valid <= 1'b0;
                s0_last  <= 1'b0;
            end
        end
    end

    // Rotate test pair coordinates and compute circular RAM column addresses (s1)
    logic       s1_last;
    logic [5:0] s1_rd_col [0:NUM_READ_PORTS-1];
    logic [5:0] s1_wy1    [0:PAIRS_PER_CYCLE-1];
    logic [5:0] s1_wy2    [0:PAIRS_PER_CYCLE-1];
    logic       s1_ib1    [0:PAIRS_PER_CYCLE-1];
    logic       s1_ib2    [0:PAIRS_PER_CYCLE-1];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s1_valid <= 1'b0;
            s1_last  <= 1'b0;
            for (int gp_i = 0; gp_i < NUM_READ_PORTS; gp_i++) begin
                s1_rd_col[gp_i] <= '0;
            end
            for (int p = 0; p < PAIRS_PER_CYCLE; p++) begin
                s1_wy1[p] <= '0;
                s1_wy2[p] <= '0;
                s1_ib1[p] <= 1'b0;
                s1_ib2[p] <= 1'b0;
            end
        end else begin
            s1_valid <= s0_valid;
            s1_last  <= s0_last;
            if (s0_valid) begin
                for (int p = 0; p < PAIRS_PER_CYCLE; p++) begin
                    automatic logic signed [15:0] px1_cos, py1_sin, px1_sin, py1_cos;
                    automatic logic signed [15:0] px2_cos, py2_sin, px2_sin, py2_cos;
                    automatic logic signed [7:0]  rx1, ry1, rx2, ry2;
                    automatic logic signed [7:0]  wx1, wy1, wx2, wy2;

                    px1_cos = 16'($signed(s0_x1[p])) * 16'($signed(active_cos));
                    py1_sin = 16'($signed(s0_y1[p])) * 16'($signed(active_sin));
                    px1_sin = 16'($signed(s0_x1[p])) * 16'($signed(active_sin));
                    py1_cos = 16'($signed(s0_y1[p])) * 16'($signed(active_cos));

                    px2_cos = 16'($signed(s0_x2[p])) * 16'($signed(active_cos));
                    py2_sin = 16'($signed(s0_y2[p])) * 16'($signed(active_sin));
                    px2_sin = 16'($signed(s0_x2[p])) * 16'($signed(active_sin));
                    py2_cos = 16'($signed(s0_y2[p])) * 16'($signed(active_cos));

                    rx1 = 8'($signed(px1_cos - py1_sin) >>> 7);
                    ry1 = 8'($signed(px1_sin + py1_cos) >>> 7);
                    rx2 = 8'($signed(px2_cos - py2_sin) >>> 7);
                    ry2 = 8'($signed(px2_sin + py2_cos) >>> 7);

                    wx1 = 8'(WINDOW_CENTER) - rx1;
                    wy1 = 8'(WINDOW_CENTER) - ry1;
                    wx2 = 8'(WINDOW_CENTER) - rx2;
                    wy2 = 8'(WINDOW_CENTER) - ry2;

                    s1_rd_col[2*p + 0] <= active_right_ptr - wx1[5:0];
                    s1_rd_col[2*p + 1] <= active_right_ptr - wx2[5:0];
                    s1_wy1[p]          <= wy1[5:0];
                    s1_wy2[p]          <= wy2[5:0];
                    s1_ib1[p]          <= (wx1 >= 0 && wx1 < 8'(WINDOW_SIZE) && wy1 >= 0 && wy1 < 8'(WINDOW_SIZE));
                    s1_ib2[p]          <= (wx2 >= 0 && wx2 < 8'(WINDOW_SIZE) && wy2 >= 0 && wy2 < 8'(WINDOW_SIZE));
                end
            end
        end
    end

    always_comb begin
        for (int gp_i = 0; gp_i < NUM_READ_PORTS; gp_i++) begin
            ram_rd_addr[gp_i] = s1_rd_col[gp_i];
        end
    end

    // Delay row indices and bounds flags to match 1-cycle LUTRAM read (s2)
    logic       s2_last;
    logic [5:0] s2_wy1 [0:PAIRS_PER_CYCLE-1];
    logic [5:0] s2_wy2 [0:PAIRS_PER_CYCLE-1];
    logic       s2_ib1 [0:PAIRS_PER_CYCLE-1];
    logic       s2_ib2 [0:PAIRS_PER_CYCLE-1];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            s2_valid <= 1'b0;
            s2_last  <= 1'b0;
            for (int p = 0; p < PAIRS_PER_CYCLE; p++) begin
                s2_wy1[p] <= '0;
                s2_wy2[p] <= '0;
                s2_ib1[p] <= 1'b0;
                s2_ib2[p] <= 1'b0;
            end
        end else begin
            s2_valid <= s1_valid;
            s2_last  <= s1_last;
            if (s1_valid) begin
                for (int p = 0; p < PAIRS_PER_CYCLE; p++) begin
                    s2_wy1[p] <= s1_wy1[p];
                    s2_wy2[p] <= s1_wy2[p];
                    s2_ib1[p] <= s1_ib1[p];
                    s2_ib2[p] <= s1_ib2[p];
                end
            end
        end
    end

    // Select target rows, compare pixel intensities, and shift into descriptor
    logic [PAIRS_PER_CYCLE-1:0] pair_bits;
    logic [DESCRIPTOR_BITS-1:0] desc_shift;
    logic [DESCRIPTOR_BITS-1:0] desc_next;

    always_comb begin
        for (int p = 0; p < PAIRS_PER_CYCLE; p++) begin
            automatic logic [PIXEL_WIDTH-1:0] p1, p2;
            p1 = (s2_ib1[p] && (s2_wy1[p] < 6'(WINDOW_SIZE))) ? ram_rd_data[2*p + 0][s2_wy1[p]] : '0;
            p2 = (s2_ib2[p] && (s2_wy2[p] < 6'(WINDOW_SIZE))) ? ram_rd_data[2*p + 1][s2_wy2[p]] : '0;
            pair_bits[p] = (p1 < p2);
        end
        if (PAIRS_PER_CYCLE >= DESCRIPTOR_BITS) begin
            desc_next = pair_bits;
        end else begin
            desc_next = {pair_bits, desc_shift[DESCRIPTOR_BITS-1 : PAIRS_PER_CYCLE]};
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            desc_shift         <= '0;
            descriptor         <= '0;
            keypoint_score_out <= '0;
            keypoint_x_out     <= '0;
            keypoint_y_out     <= '0;
            orientation_out    <= '0;
            descriptor_valid   <= 1'b0;
        end else begin
            if (s2_valid) begin
                desc_shift <= desc_next;
            end

            descriptor_valid <= s2_valid && s2_last;
            if (s2_valid && s2_last) begin
                descriptor         <= desc_next;
                keypoint_score_out <= active_score;
                keypoint_x_out     <= active_x;
                keypoint_y_out     <= active_y;
                orientation_out    <= active_orientation;
            end
        end
    end

    assign stall_req = patch_pending || !cmd_empty || kp_busy;
    assign busy      = patch_latch || patch_pending || start || !cmd_empty || kp_busy || descriptor_valid;

endmodule
