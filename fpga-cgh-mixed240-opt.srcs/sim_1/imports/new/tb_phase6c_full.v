`timescale 1ns / 1ps

module tb_phase6c_full;

    // C++固定小数点モデル(Mixed-Radix 12bit)で確定した各パスの右シフト量
    localparam [4:0] P_SHIFT1 = 5'd9;
    localparam [4:0] P_SHIFT2 = 5'd10;
    localparam [4:0] P_SHIFT3 = 5'd2;
    localparam [4:0] P_SHIFT4 = 5'd5;

    reg                clk;
    reg                rst_n;
    reg                start;
    reg                ext_we;
    reg         [15:0] ext_addr;
    reg  signed [11:0] ext_din_re, ext_din_im;
    wire signed [11:0] ext_dout_re, ext_dout_im;
    wire               busy;
    wire               done;

    cgh_2d_top #(
        .NUM_LINES   (240),
        .SHIFT_PASS1 (P_SHIFT1),
        .SHIFT_PASS2 (P_SHIFT2),
        .SHIFT_PASS3 (P_SHIFT3),
        .SHIFT_PASS4 (P_SHIFT4)
    ) uut (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start),
        .ext_we      (ext_we),
        .ext_addr    (ext_addr),
        .ext_din_re  (ext_din_re),
        .ext_din_im  (ext_din_im),
        .ext_dout_re (ext_dout_re),
        .ext_dout_im (ext_dout_im),
        .busy        (busy),
        .done        (done)
    );

    // 100 MHz clock (Period = 10 ns)
    always #5 clk = ~clk;

    reg [23:0] in_mem [0:57599];
    integer i, fd;
    time t_start, t_end;

    initial begin
        $readmemh("hologram_in_240x240.mem", in_mem);

        clk = 0; rst_n = 0; start = 0;
        ext_we = 0; ext_addr = 0; ext_din_re = 0; ext_din_im = 0;
        #25;
        rst_n = 1;
        #20;

        $display("=================================================================");
        $display("[1/3] Loading 240x240 Hologram (57,600 pixels) into Frame RAM...");
        $display("=================================================================");
        ext_we = 1;
        for (i = 0; i < 57600; i = i + 1) begin
            ext_addr   = i[15:0];
            ext_din_re = in_mem[i][23:12];
            ext_din_im = in_mem[i][11:0];
            @(posedge clk); #1;
        end
        ext_we = 0;

        $display("[2/3] Starting Full 240x240 2D CGH Reconstruction (960 lines)...");
        @(posedge clk);
        t_start = $time;
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        // Wait for all 4 passes (240x4 = 960 lines) to complete
        wait (done == 1'b1);
        t_end = $time;
        @(posedge clk); #1;

        $display("=================================================================");
        $display(" Full 240x240 Reconstruction Completed!");
        $display("   Hardware Calc Time (@100MHz) = %0d ns (approx %0d ms)",
                 (t_end - t_start), (t_end - t_start) / 1000000);
        $display("=================================================================");

        $display("[3/3] Exporting 57,600 reconstructed pixels to hw_recon_240x240.txt...");
        fd = $fopen("C:/Vivado_Work/fpga-cgh-mixed240/cgh_mr_240/hw_recon_240x240.txt", "w");
        if (fd == 0) begin
            $display("ERROR: Could not open output file! Check the directory path.");
            $finish;
        end

        for (i = 0; i < 57600; i = i + 1) begin
            ext_addr = i[15:0];
            @(posedge clk); #1;
            $fwrite(fd, "%0d %0d\n", ext_dout_re, ext_dout_im);
        end
        $fclose(fd);

        $display("Export Finished! File: C:/Vivado_Work/fpga-cgh-mixed240/cgh_mr_240/hw_recon_240x240.txt");
        $finish;
    end

endmodule