`timescale 1ns / 1ps

module work_ram_240 (
    input  wire               clk,
    input  wire               we,
    input  wire        [7:0]  addr,
    input  wire signed [23:0] din_re,
    input  wire signed [23:0] din_im,
    output reg  signed [23:0] dout_re,
    output reg  signed [23:0] dout_im
);
    (* ram_style = "block" *) reg [47:0] mem [0:239];

    always @(posedge clk) begin
        if (we) begin
            mem[addr] <= {din_re, din_im};
        end
        dout_re <= mem[addr][47:24];
        dout_im <= mem[addr][23:0];
    end
endmodule