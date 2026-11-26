`timescale 1ns / 1ps

module cgh_2d_top #(
    parameter integer NUM_LINES   = 240,
    parameter [4:0]   SHIFT_PASS1 = 5'd9,
    parameter [4:0]   SHIFT_PASS2 = 5'd10,
    parameter [4:0]   SHIFT_PASS3 = 5'd2,
    parameter [4:0]   SHIFT_PASS4 = 5'd5
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire               ext_we,
    input  wire        [15:0] ext_addr,
    input  wire signed [11:0] ext_din_re,
    input  wire signed [11:0] ext_din_im,
    output wire signed [11:0] ext_dout_re,
    output wire signed [11:0] ext_dout_im,
    output wire               busy,
    output reg                done
);

    localparam S_IDLE      = 3'd0;
    localparam S_LINE_PREP = 3'd1;
    localparam S_FEED_WAIT = 3'd2;
    localparam S_FEED      = 3'd3;
    localparam S_WAIT_DONE = 3'd4;

    reg [2:0] state;
    reg [1:0] pass;
    reg [7:0] line_idx;
    reg [7:0] rd_idx;
    reg [7:0] feed_cnt;

    assign busy = (state != S_IDLE);

    function [15:0] calc_2d_addr;
        input [7:0] row;
        input [7:0] col;
        begin
            calc_2d_addr = ({8'd0, row} << 8) - ({8'd0, row} << 4) + {8'd0, col};
        end
    endfunction

    function signed [11:0] shift_sat12;
        input signed [31:0] val;
        input        [4:0]  sh;
        reg signed [31:0] rounded;
        begin
            if (sh == 5'd0)
                rounded = val;
            else
                rounded = (val + (32'sd1 <<< (sh - 5'd1))) >>> sh;

            if (rounded > 32'sd2047)
                shift_sat12 = 12'sd2047;
            else if (rounded < -32'sd2048)
                shift_sat12 = -12'sd2048;
            else
                shift_sat12 = rounded[11:0];
        end
    endfunction

    reg                int_we;
    reg         [7:0]  wr_idx;
    reg  signed [11:0] int_din_re, int_din_im;

    wire               is_col_pass = (pass == 2'd1) || (pass == 2'd3);
    wire        [7:0]  cur_elem    = (state == S_WAIT_DONE) ? wr_idx : rd_idx;
    wire        [15:0] int_addr    = is_col_pass ? calc_2d_addr(cur_elem, line_idx)
                                                 : calc_2d_addr(line_idx, cur_elem);

    wire               ram_we      = busy ? int_we     : ext_we;
    wire        [15:0] ram_addr    = busy ? int_addr   : ext_addr;
    wire signed [11:0] ram_din_re  = busy ? int_din_re : ext_din_re;
    wire signed [11:0] ram_din_im  = busy ? int_din_im : ext_din_im;
    wire signed [11:0] ram_dout_re, ram_dout_im;

    assign ext_dout_re = ram_dout_re;
    assign ext_dout_im = ram_dout_im;

    frame_ram_240_2d u_frame_ram (
        .clk     (clk),
        .we      (ram_we),
        .addr    (ram_addr),
        .din_re  (ram_din_re),
        .din_im  (ram_din_im),
        .dout_re (ram_dout_re),
        .dout_im (ram_dout_im)
    );

    reg                b_start;
    reg                b_inv;
    reg                b_in_valid;
    reg  signed [31:0] b_in_re, b_in_im;
    wire               b_out_valid;
    wire        [7:0]  b_out_idx;
    wire signed [31:0] b_out_re, b_out_im;
    wire               b_done;

    mixed_radix_240_1d u_bluestein (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (b_start),
        .inv_en    (b_inv),
        .in_valid  (b_in_valid),
        .in_re     (b_in_re),
        .in_im     (b_in_im),
        .out_valid (b_out_valid),
        .out_idx   (b_out_idx),
        .out_re    (b_out_re),
        .out_im    (b_out_im),
        .done      (b_done)
    );

    wire        [7:0]  h_addr = (state == S_WAIT_DONE) ? b_out_idx : line_idx;
    wire signed [15:0] h_re, h_im;

    rom_H_inv u_rom_h (
        .clk  (clk),
        .addr (h_addr),
        .re   (h_re),
        .im   (h_im)
    );

    // -------------------------------------------------------------------------
    // Pipelined Pass 2 H_inv[x] * H_inv[y] Multiplier (5 stages -> Write)
    // -------------------------------------------------------------------------
    reg  signed [15:0] hx_re, hx_im;

    reg                p1_valid;
    reg         [7:0]  p1_idx;
    reg  signed [31:0] p1_re, p1_im;

    reg                p2_valid;
    reg         [7:0]  p2_idx;
    reg  signed [49:0] mx_rr, mx_ii, mx_ri, mx_ir;
    reg  signed [15:0] p2_hy_re, p2_hy_im;

    wire signed [49:0] hx_sum_re = (mx_rr - mx_ii + 50'sd8192) >>> 14;
    wire signed [49:0] hx_sum_im = (mx_ri + mx_ir + 50'sd8192) >>> 14;

    reg                p3_valid;
    reg         [7:0]  p3_idx;
    reg  signed [31:0] p3_re, p3_im;
    reg  signed [15:0] p3_hy_re, p3_hy_im;

    reg                p4_valid;
    reg         [7:0]  p4_idx;
    reg  signed [49:0] my_rr, my_ii, my_ri, my_ir;

    wire signed [49:0] hy_sum_re = (my_rr - my_ii + 50'sd8192) >>> 14;
    wire signed [49:0] hy_sum_im = (my_ri + my_ir + 50'sd8192) >>> 14;

    reg                p5_valid;
    reg         [7:0]  p5_idx;
    reg  signed [31:0] p5_re, p5_im;

    wire [4:0] cur_shift = (pass == 2'd0) ? SHIFT_PASS1 :
                           (pass == 2'd1) ? SHIFT_PASS2 :
                           (pass == 2'd2) ? SHIFT_PASS3 : SHIFT_PASS4;

    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            pass       <= 2'd0;
            line_idx   <= 8'd0;
            rd_idx     <= 8'd0;
            feed_cnt   <= 8'd0;
            int_we     <= 1'b0;
            wr_idx     <= 8'd0;
            int_din_re <= 12'sd0;
            int_din_im <= 12'sd0;
            b_start    <= 1'b0;
            b_inv      <= 1'b0;
            b_in_valid <= 1'b0;
            b_in_re    <= 32'sd0;
            b_in_im    <= 32'sd0;
            hx_re      <= 16'sd0;
            hx_im      <= 16'sd0;
            p1_valid   <= 1'b0;
            p1_idx     <= 8'd0;
            p1_re      <= 32'sd0;
            p1_im      <= 32'sd0;
            p2_valid   <= 1'b0;
            p2_idx     <= 8'd0;
            mx_rr      <= 50'sd0;
            mx_ii      <= 50'sd0;
            mx_ri      <= 50'sd0;
            mx_ir      <= 50'sd0;
            p2_hy_re   <= 16'sd0;
            p2_hy_im   <= 16'sd0;
            p3_valid   <= 1'b0;
            p3_idx     <= 8'd0;
            p3_re      <= 32'sd0;
            p3_im      <= 32'sd0;
            p3_hy_re   <= 16'sd0;
            p3_hy_im   <= 16'sd0;
            p4_valid   <= 1'b0;
            p4_idx     <= 8'd0;
            my_rr      <= 50'sd0;
            my_ii      <= 50'sd0;
            my_ri      <= 50'sd0;
            my_ir      <= 50'sd0;
            p5_valid   <= 1'b0;
            p5_idx     <= 8'd0;
            p5_re      <= 32'sd0;
            p5_im      <= 32'sd0;
            done       <= 1'b0;
        end else begin
            int_we     <= 1'b0;
            b_start    <= 1'b0;
            b_in_valid <= 1'b0;
            p1_valid   <= 1'b0;
            p2_valid   <= 1'b0;
            p3_valid   <= 1'b0;
            p4_valid   <= 1'b0;
            p5_valid   <= 1'b0;
            done       <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        pass     <= 2'd0;
                        line_idx <= 8'd0;
                        state    <= S_LINE_PREP;
                    end
                end

                S_LINE_PREP: begin
                    b_start  <= 1'b1;
                    b_inv    <= (pass >= 2'd2);
                    rd_idx   <= 8'd0;
                    feed_cnt <= 8'd0;
                    state    <= S_FEED_WAIT;
                end

                S_FEED_WAIT: begin
                    rd_idx <= 8'd1;
                    state  <= S_FEED;
                end

                S_FEED: begin
                    if (feed_cnt == 8'd0) begin
                        hx_re <= h_re;
                        hx_im <= h_im;
                    end

                    // Apply GUARD_SHIFT = 6 (<< 6) to match C++ fixed-point model
                    b_in_valid <= 1'b1;
                    b_in_re    <= {{14{ram_dout_re[11]}}, ram_dout_re, 6'd0};
                    b_in_im    <= {{14{ram_dout_im[11]}}, ram_dout_im, 6'd0};

                    if (feed_cnt < 8'd238) begin
                        rd_idx <= feed_cnt + 8'd2;
                    end

                    if (feed_cnt == 8'd239) begin
                        state <= S_WAIT_DONE;
                    end else begin
                        feed_cnt <= feed_cnt + 8'd1;
                    end
                end

                S_WAIT_DONE: begin
                    if (pass == 2'd1) begin
                        if (b_out_valid) begin
                            p1_valid <= 1'b1;
                            p1_idx   <= b_out_idx;
                            p1_re    <= b_out_re;
                            p1_im    <= b_out_im;
                        end
                        if (p1_valid) begin
                            p2_valid <= 1'b1;
                            p2_idx   <= p1_idx;
                            mx_rr    <= $signed(p1_re) * $signed(hx_re);
                            mx_ii    <= $signed(p1_im) * $signed(hx_im);
                            mx_ri    <= $signed(p1_re) * $signed(hx_im);
                            mx_ir    <= $signed(p1_im) * $signed(hx_re);
                            p2_hy_re <= h_re;
                            p2_hy_im <= h_im;
                        end
                        if (p2_valid) begin
                            p3_valid <= 1'b1;
                            p3_idx   <= p2_idx;
                            p3_re    <= hx_sum_re[31:0];
                            p3_im    <= hx_sum_im[31:0];
                            p3_hy_re <= p2_hy_re;
                            p3_hy_im <= p2_hy_im;
                        end
                        if (p3_valid) begin
                            p4_valid <= 1'b1;
                            p4_idx   <= p3_idx;
                            my_rr    <= $signed(p3_re) * $signed(p3_hy_re);
                            my_ii    <= $signed(p3_im) * $signed(p3_hy_im);
                            my_ri    <= $signed(p3_re) * $signed(p3_hy_im);
                            my_ir    <= $signed(p3_im) * $signed(p3_hy_re);
                        end
                        if (p4_valid) begin
                            p5_valid <= 1'b1;
                            p5_idx   <= p4_idx;
                            p5_re    <= hy_sum_re[31:0];
                            p5_im    <= hy_sum_im[31:0];
                        end
                        if (p5_valid) begin
                            int_we     <= 1'b1;
                            wr_idx     <= p5_idx;
                            int_din_re <= shift_sat12(p5_re, SHIFT_PASS2);
                            int_din_im <= shift_sat12(p5_im, SHIFT_PASS2);
                        end
                    end else begin
                        if (b_out_valid) begin
                            int_we     <= 1'b1;
                            wr_idx     <= b_out_idx;
                            int_din_re <= shift_sat12(b_out_re, cur_shift);
                            int_din_im <= shift_sat12(b_out_im, cur_shift);
                        end
                    end

                    if (b_done) begin
                        if (line_idx == (NUM_LINES - 1)) begin
                            line_idx <= 8'd0;
                            if (pass == 2'd3) begin
                                done  <= 1'b1;
                                state <= S_IDLE;
                            end else begin
                                pass  <= pass + 2'd1;
                                state <= S_LINE_PREP;
                            end
                        end else begin
                            line_idx <= line_idx + 8'd1;
                            state    <= S_LINE_PREP;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule