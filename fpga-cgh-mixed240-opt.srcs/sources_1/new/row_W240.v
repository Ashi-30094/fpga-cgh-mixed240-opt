`timescale 1ns / 1ps

module rom_W240 (
    input wire               clk,
    input wire        [7:0]  addr,
    output reg signed [15:0] re,
    output reg signed [15:0] im
);

    (* rom_style = "block" *) reg [31:0] mem [0:239];
    
    initial begin
        $readmemh("rom_W240.mem", mem);
    end
    
    always @(posedge clk) begin
        re <= mem[addr][31:16];
        im <= mem[addr][15:0];
    end
endmodule