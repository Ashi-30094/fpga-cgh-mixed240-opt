`timescale 1ns / 1ps

module frame_ram_240_2d (
    input  wire               clk,
    input  wire               we,
    input  wire        [15:0] addr,    // 0 to 57599 (y * 240 + x)
    input  wire signed [11:0] din_re,
    input  wire signed [11:0] din_im,
    output wire signed [11:0] dout_re,
    output wire signed [11:0] dout_im
);

    wire [23:0] din_word = {din_re, din_im};

    // Bank select by upper address bits (no subtractor needed)
    // Bank 0: 0     .. 32767 (addr[15]    == 1'b0)
    // Bank 1: 32768 .. 49151 (addr[15:14] == 2'b10)
    // Bank 2: 49152 .. 57343 (addr[15:13] == 3'b110)
    // Bank 3: 57344 .. 57599 (addr[15:13] == 3'b111)
    wire sel_b0 = (addr[15]    == 1'b0);
    wire sel_b1 = (addr[15:14] == 2'b10);
    wire sel_b2 = (addr[15:13] == 3'b110);
    wire sel_b3 = (addr[15:13] == 3'b111);

    wire [1:0] bank_sel = sel_b0 ? 2'd0 :
                          sel_b1 ? 2'd1 :
                          sel_b2 ? 2'd2 : 2'd3;

    // 4 partitioned memory banks
    (* ram_style = "block" *)       reg [23:0] ram_b0 [0:32767]; // 24 BRAMs
    (* ram_style = "block" *)       reg [23:0] ram_b1 [0:16383]; // 12 BRAMs
    (* ram_style = "block" *)       reg [23:0] ram_b2 [0:8191];  //  6 BRAMs
    (* ram_style = "distributed" *) reg [23:0] ram_b3 [0:255];   //  0 BRAMs (LUTs)

    reg [23:0] dout_b0, dout_b1, dout_b2, dout_b3;
    reg [1:0]  bank_sel_d;

    always @(posedge clk) begin
        // Bank 0 (32K)
        if (we && sel_b0) ram_b0[addr[14:0]] <= din_word;
        dout_b0 <= ram_b0[addr[14:0]];

        // Bank 1 (16K)
        if (we && sel_b1) ram_b1[addr[13:0]] <= din_word;
        dout_b1 <= ram_b1[addr[13:0]];

        // Bank 2 (8K)
        if (we && sel_b2) ram_b2[addr[12:0]] <= din_word;
        dout_b2 <= ram_b2[addr[12:0]];

        // Bank 3 (256 words)
        if (we && sel_b3) ram_b3[addr[7:0]]  <= din_word;
        dout_b3 <= ram_b3[addr[7:0]];

        // Delay bank selector by 1 clock to match synchronous RAM read
        bank_sel_d <= bank_sel;
    end

    reg [23:0] dout_mux;
    always @(*) begin
        case (bank_sel_d)
            2'd0:    dout_mux = dout_b0;
            2'd1:    dout_mux = dout_b1;
            2'd2:    dout_mux = dout_b2;
            default: dout_mux = dout_b3;
        endcase
    end

    assign dout_re = dout_mux[23:12];
    assign dout_im = dout_mux[11:0];

endmodule