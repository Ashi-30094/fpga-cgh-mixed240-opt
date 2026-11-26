`timescale 1ns / 1ps

module butterfly_unit(
    input  wire               inv_en,
    input  wire               shift_en,
    input  wire signed [31:0] even_re,
    input  wire signed [31:0] even_im,
    input  wire signed [31:0] odd_re,
    input  wire signed [31:0] odd_im,
    input  wire signed [15:0] w_re,
    input  wire signed [15:0] w_im,
    output wire signed [31:0] out_even_re,
    output wire signed [31:0] out_even_im,
    output wire signed [31:0] out_odd_re,
    output wire signed [31:0] out_odd_im
);

    wire signed [15:0] w_im_adj = inv_en ? -w_im : w_im;
    
    wire signed [31:0] w_odd_re;
    wire signed [31:0] w_odd_im;
    
    cmul_unit #(
        .FRAC_BITS(14)
    ) u_cmul (
        .a_re (odd_re),
        .a_im (odd_im),
        .b_re (w_re),
        .b_im (w_im_adj),
        .y_re (w_odd_re),
        .y_im (w_odd_im)
    );
    
    
    // 3. Butterfly addition and subtraction (33bit signed to prevent overflow)
    wire signed [32:0] sum_re  = $signed(even_re) + $signed(w_odd_re);
    wire signed [32:0] sum_im  = $signed(even_im) + $signed(w_odd_im);
    wire signed [32:0] diff_re = $signed(even_re) - $signed(w_odd_re);
    wire signed [32:0] diff_im = $signed(even_im) - $signed(w_odd_im);

    // 4. Divide by 2 with rounding when shift_en == 1: (val + 1) >>> 1
    wire signed [32:0] sum_re_sh  = (sum_re  + 33'sd1) >>> 1;
    wire signed [32:0] sum_im_sh  = (sum_im  + 33'sd1) >>> 1;
    wire signed [32:0] diff_re_sh = (diff_re + 33'sd1) >>> 1;
    wire signed [32:0] diff_im_sh = (diff_im + 33'sd1) >>> 1;

    // 5. Select output based on shift_en
    assign out_even_re = shift_en ? sum_re_sh[31:0]  : sum_re[31:0];
    assign out_even_im = shift_en ? sum_im_sh[31:0]  : sum_im[31:0];
    assign out_odd_re  = shift_en ? diff_re_sh[31:0] : diff_re[31:0];
    assign out_odd_im  = shift_en ? diff_im_sh[31:0] : diff_im[31:0];

endmodule 
