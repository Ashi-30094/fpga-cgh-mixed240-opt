`timescale 1ns / 1ps

module tb_mr_1d;
    reg               clk = 1'b0;
    reg               rst_n = 1'b0;
    reg               start = 1'b0;
    reg               inv_en = 1'b0;
    reg               in_valid = 1'b0;
    reg signed [31:0] in_re = 32'sd0;
    reg signed [31:0] in_im = 32'sd0;

    wire               out_valid;
    wire        [7:0]  out_idx;
    wire signed [31:0] out_re;
    wire signed [31:0] out_im;
    wire               done;

    always #5 clk = ~clk; // 100 MHz

    mixed_radix_240_1d dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start),
        .inv_en    (inv_en),
        .in_valid  (in_valid),
        .in_re     (in_re),
        .in_im     (in_im),
        .out_valid (out_valid),
        .out_idx   (out_idx),
        .out_re    (out_re),
        .out_im    (out_im),
        .done      (done)
    );

    integer fd, rc, i, idx_tmp, sh_re_tmp, sh_im_tmp;
    reg [8*256-1:0] header_line;

    integer ref_in_re   [0:239], ref_in_im   [0:239];
    integer ref_fwd_re  [0:239], ref_fwd_im  [0:239];
    integer ref_inv_re  [0:239], ref_inv_im  [0:239];
    integer hw_out_re   [0:239], hw_out_im   [0:239];

    integer err_cnt, cyc_cnt;

    always @(posedge clk) begin
        if (out_valid) begin
            hw_out_re[out_idx] <= out_re;
            hw_out_im[out_idx] <= out_im;
        end
    end

    task run_and_check;
        input is_inv;
        begin
            err_cnt = 0;
            cyc_cnt = 0;

            @(posedge clk);
            inv_en <= is_inv;
            start  <= 1'b1;
            @(posedge clk);
            start  <= 1'b0;
            @(posedge clk); // cgh_2d_topのS_FEED_WAIT相当(1サイクル待機)

            for (i = 0; i < 240; i = i + 1) begin
                in_valid <= 1'b1;
                in_re    <= ref_in_re[i] <<< 6; // GUARD_SHIFT = 6
                in_im    <= ref_in_im[i] <<< 6;
                @(posedge clk);
                cyc_cnt = cyc_cnt + 1;
            end
            in_valid <= 1'b0;
            in_re    <= 32'sd0;
            in_im    <= 32'sd0;

            while (!done) begin
                @(posedge clk);
                cyc_cnt = cyc_cnt + 1;
            end
            @(posedge clk);

            for (i = 0; i < 240; i = i + 1) begin
                if (!is_inv) begin
                    if (hw_out_re[i] !== ref_fwd_re[i] || hw_out_im[i] !== ref_fwd_im[i]) begin
                        if (err_cnt < 10)
                            $display("[FWD MISMATCH] k=%0d: HW=(%0d, %0d) REF=(%0d, %0d)",
                                     i, hw_out_re[i], hw_out_im[i], ref_fwd_re[i], ref_fwd_im[i]);
                        err_cnt = err_cnt + 1;
                    end
                end else begin
                    if (hw_out_re[i] !== ref_inv_re[i] || hw_out_im[i] !== ref_inv_im[i]) begin
                        if (err_cnt < 10)
                            $display("[INV MISMATCH] k=%0d: HW=(%0d, %0d) REF=(%0d, %0d)",
                                     i, hw_out_re[i], hw_out_im[i], ref_inv_re[i], ref_inv_im[i]);
                        err_cnt = err_cnt + 1;
                    end
                end
            end

            if (err_cnt == 0)
                $display("==> [%s] PASS! (240/240 Bit-Exact, cycles = %0d)",
                         is_inv ? "INV FFT" : "FWD FFT", cyc_cnt);
            else
                $display("==> [%s] FAIL! (errors = %0d / 240)",
                         is_inv ? "INV FFT" : "FWD FFT", err_cnt);
        end
    endtask

    initial begin
        fd = $fopen("C:/Vivado_Work/fpga-cgh-mixed240/cgh_mr_240/tb_mr_1d_ref.txt", "r");
        if (fd == 0) begin
            $display("ERROR: tb_mr_1d_ref.txt を開けません。パスを確認してください。");
            $finish;
        end
        rc = $fgets(header_line, fd); // 1行目のコメントをスキップ
        for (i = 0; i < 240; i = i + 1) begin
            rc = $fscanf(fd, "%d %d %d %d %d %d %d %d %d\n",
                         idx_tmp, ref_in_re[i], ref_in_im[i],
                         ref_fwd_re[i], ref_fwd_im[i],
                         sh_re_tmp, sh_im_tmp,
                         ref_inv_re[i], ref_inv_im[i]);
        end
        $fclose(fd);

        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        run_and_check(1'b0); // 順FFT検証
        repeat (5) @(posedge clk);
        run_and_check(1'b1); // 逆FFT検証

        $finish;
    end
endmodule