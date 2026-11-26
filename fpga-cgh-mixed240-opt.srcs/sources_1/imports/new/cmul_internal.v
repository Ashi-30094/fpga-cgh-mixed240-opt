`timescale 1ns / 1ps
module cmul_unit #(
    parameter FRAC_BITS = 14
)(
    input wire signed [31:0] a_re,
    input wire signed [31:0] a_im,
    input wire signed [15:0] b_re,
    input wire signed [15:0] b_im,
    output wire signed [31:0] y_re,
    output wire signed [31:0] y_im
);

    wire signed [47:0] p_rr = a_re * b_re;
    wire signed [47:0] p_ii = a_im * b_im;
    wire signed [47:0] p_ri = a_re * b_im;
    wire signed [47:0] p_ir = a_im * b_re;
    
    wire signed [49:0] re_full = p_rr - p_ii;
    wire signed [49:0] im_full = p_ri + p_ir;
    
    wire signed [49:0] half = 50'sd1 <<< (FRAC_BITS - 1);
    
    wire signed [49:0] re_shifted = (re_full + half) >>> FRAC_BITS;
    wire signed [49:0] im_shifted = (im_full + half) >>> FRAC_BITS;
    
    assign y_re = re_shifted[31:0];
    assign y_im = im_shifted[31:0];
    
 endmodule