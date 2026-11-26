`timescale 1ns / 1ps

module rom_holo_1bit (
    input  wire        clk,
    input  wire [15:0] addr,
    output wire        dout
);
    reg [15:0] mem [0:3599];

    initial begin
        $readmemh("holo_16x3600.mem", mem);
    end

    reg [15:0] word_q;
    reg [3:0]  bit_sel_q;

    always @(posedge clk) begin
        word_q    <= mem[addr[15:4]];
        bit_sel_q <= addr[3:0];
    end

    assign dout = word_q[bit_sel_q];
endmodule