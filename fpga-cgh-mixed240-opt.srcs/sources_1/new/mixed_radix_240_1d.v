`timescale 1ns / 1ps

module mixed_radix_240_1d (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire               inv_en,
    input  wire               in_valid,
    input  wire signed [31:0] in_re,
    input  wire signed [31:0] in_im,
    output reg                out_valid,
    output reg         [7:0]  out_idx,
    output reg  signed [31:0] out_re,
    output reg  signed [31:0] out_im,
    output reg                done
);

    // FSM 状態定義
    localparam S_IDLE      = 3'd0;
    localparam S_LOAD      = 3'd1;
    localparam S_S1_LOAD   = 3'd2;
    localparam S_S1_BF     = 3'd3;
    localparam S_S1_TW     = 3'd4;
    localparam S_S2_DFT3   = 3'd5;
    localparam S_S2_TW15   = 3'd6;
    localparam S_S3_DFT5   = 3'd7;

    reg [2:0] state;
    reg       done_wait_mode;
    reg [3:0] wait_cnt;
    reg       inv_reg;

    // パイプライン発行制御 (II = 1)
    reg       issue_done;
    reg       p1_valid, p2_valid, p3_valid;

    wire issue_en = (!done_wait_mode) && (!issue_done) &&
                    ((state == S_S1_LOAD) || (state == S_S1_BF)   || (state == S_S1_TW) ||
                     (state == S_S2_DFT3) || (state == S_S2_TW15) || (state == S_S3_DFT5));

    // パイプライン運搬用メタデータ
    reg [3:0] p1_n1;
    reg [3:0] p1_k1;
    reg [3:0] p1_bf_e, p2_bf_e, p3_bf_e;
    reg [3:0] p1_bf_o;
    reg [1:0] p1_m1,   p2_m1,   p3_m1;
    reg [2:0] p1_m2,   p2_m2,   p3_m2;
    reg [7:0] p1_addr, p2_addr, p3_addr;

    // 各種ループカウンタ
    reg [7:0] in_cnt;
    reg [3:0] n1;        // 0..15
    reg [3:0] n2;        // 0..14
    reg [1:0] s_stage;   // 0..3 (16点FFTステージ)
    reg [2:0] bf_idx;    // 0..7 (バタフライ番号)
    reg [3:0] k1;        // 0..15
    reg [1:0] l1;        // 0..2
    reg [2:0] l2;        // 0..4
    reg [1:0] m1;        // 0..2
    reg [2:0] m2;        // 0..4

    // 16点小バッファ (レジスタ配列)
    reg signed [23:0] tmp16_re [0:15];
    reg signed [23:0] tmp16_im [0:15];

    // 累算レジスタ (3点DFT / 5点DFT用)
    reg signed [23:0] acc_re, acc_im;

    // -------------------------------------------------------------------------
    // インデックス・アドレス計算用ヘルパー
    // -------------------------------------------------------------------------
    function [3:0] rev4;
        input [3:0] v;
        begin
            rev4 = {v[0], v[1], v[2], v[3]};
        end
    endfunction

    // 16点FFTのバタフライ対象インデックスと回転因子インデックス
    reg [3:0] bf_idx_e, bf_idx_o;
    reg [7:0] bf_w_idx;
    always @(*) begin
        case (s_stage)
            2'd0: begin
                bf_idx_e = {bf_idx, 1'b0};
                bf_idx_o = {bf_idx, 1'b1};
                bf_w_idx = 8'd0;
            end
            2'd1: begin
                bf_idx_e = {bf_idx[2:1], 1'b0, bf_idx[0]};
                bf_idx_o = {bf_idx[2:1], 1'b1, bf_idx[0]};
                bf_w_idx = bf_idx[0] ? 8'd60 : 8'd0; // k*4*15
            end
            2'd2: begin
                bf_idx_e = {bf_idx[2], 1'b0, bf_idx[1:0]};
                bf_idx_o = {bf_idx[2], 1'b1, bf_idx[1:0]};
                bf_w_idx = {4'd0, bf_idx[1:0], 2'd0} * 8'd7 + {5'd0, bf_idx[1:0], 1'b0}; // k*30
            end
            default: begin // 2'd3
                bf_idx_e = {1'b0, bf_idx};
                bf_idx_o = {1'b1, bf_idx};
                bf_w_idx = ({4'd0, bf_idx} << 4) - {5'd0, bf_idx}; // k*15
            end
        endcase
    end

    // 3点DFT用 W3 インデックス: ((m1 * l1) % 3) * 80
    reg [7:0] w3_idx;
    always @(*) begin
        if (m1 == 2'd0 || l1 == 2'd0)
            w3_idx = 8'd0;
        else if (m1 == l1)
            w3_idx = 8'd80;
        else
            w3_idx = 8'd160;
    end

    // 5点DFT用 W5 インデックス: ((m2 * l2) % 5) * 48
    wire [4:0] m2_x_l2 = m2 * l2;
    wire [2:0] m2l2_mod5 = (m2_x_l2 >= 5'd15) ? (m2_x_l2 - 5'd15) :
                           (m2_x_l2 >= 5'd10) ? (m2_x_l2 - 5'd10) :
                           (m2_x_l2 >= 5'd5)  ? (m2_x_l2 - 5'd5)  : m2_x_l2[2:0];
    wire [7:0] w5_idx = ({5'd0, m2l2_mod5} << 5) + ({5'd0, m2l2_mod5} << 4); // * 48

    // RAM アドレス計算 (乗算器不使用)
    wire [7:0] addr_15n1_n2    = ({4'd0, n1} << 4) - {4'd0, n1} + {4'd0, n2};
    wire [7:0] addr_15k1_n2    = ({4'd0, k1} << 4) - {4'd0, k1} + {4'd0, n2};
    wire [7:0] addr_15k1_5m1m2 = ({4'd0, k1} << 4) - {4'd0, k1} + ({6'd0, m1} << 2) + {6'd0, m1} + {5'd0, m2};
    wire [7:0] addr_15k1_5l1m2 = ({4'd0, k1} << 4) - {4'd0, k1} + ({6'd0, l1} << 2) + {6'd0, l1} + {5'd0, m2};
    wire [7:0] addr_out_k      = {4'd0, k1} + ({6'd0, l1} << 4) + ({5'd0, l2} << 5) + ({5'd0, l2} << 4);

    // -------------------------------------------------------------------------
    // 単一ROM (W_240) & ピンポンRAM (ram_a / ram_b)
    // -------------------------------------------------------------------------
    reg [7:0] rom_addr;
    always @(*) begin
        case (state)
            S_S1_BF:   rom_addr = bf_w_idx;
            S_S1_TW:   rom_addr = n2 * k1;
            S_S2_DFT3: rom_addr = w3_idx;
            S_S2_TW15: rom_addr = ({4'd0, m2} * {6'd0, l1}) << 4; // m2 * l1 * 16
            S_S3_DFT5: rom_addr = w5_idx;
            default:   rom_addr = 8'd0;
        endcase
    end

    wire signed [15:0] rom_re_raw, rom_im_raw;
    rom_W240 u_rom_w240 (
        .clk  (clk),
        .addr (rom_addr),
        .re   (rom_re_raw),
        .im   (rom_im_raw)
    );
    wire signed [15:0] rom_re = rom_re_raw;
    wire signed [15:0] rom_im = inv_reg ? -rom_im_raw : rom_im_raw;

    reg                ram_a_we;
    reg         [7:0]  ram_a_addr;
    reg  signed [23:0] ram_a_din_re, ram_a_din_im;
    wire signed [23:0] ram_a_dout_re, ram_a_dout_im;

    work_ram_240 u_ram_a (
        .clk     (clk),
        .we      (ram_a_we),
        .addr    (ram_a_addr),
        .din_re  (ram_a_din_re),
        .din_im  (ram_a_din_im),
        .dout_re (ram_a_dout_re),
        .dout_im (ram_a_dout_im)
    );

    reg                ram_b_we;
    reg         [7:0]  ram_b_addr;
    reg  signed [23:0] ram_b_din_re, ram_b_din_im;
    wire signed [23:0] ram_b_dout_re, ram_b_dout_im;

    work_ram_240 u_ram_b (
        .clk     (clk),
        .we      (ram_b_we),
        .addr    (ram_b_addr),
        .din_re  (ram_b_din_re),
        .din_im  (ram_b_din_im),
        .dout_re (ram_b_dout_re),
        .dout_im (ram_b_dout_im)
    );

    // -------------------------------------------------------------------------
    // 共有 2段パイプライン複素乗算器 (T1入力 -> T2乗算 -> T3出力)
    // -------------------------------------------------------------------------
    reg  signed [23:0] cmul_a_re, cmul_a_im;
    reg  signed [15:0] cmul_b_re, cmul_b_im;
    reg  signed [39:0] m_rr, m_ii, m_ri, m_ir;
    reg  signed [23:0] cmul_re, cmul_im;

    wire signed [40:0] cmul_sum_re = (m_rr - m_ii + 41'sd8192) >>> 14;
    wire signed [40:0] cmul_sum_im = (m_ri + m_ir + 41'sd8192) >>> 14;

    // T1 ステージ: ROM/RAM 出力と同期したオペランド選択
    always @(*) begin
        case (state)
            S_S1_BF: begin
                cmul_a_re = tmp16_re[p1_bf_o];
                cmul_a_im = tmp16_im[p1_bf_o];
            end
            S_S1_TW: begin
                cmul_a_re = tmp16_re[p1_k1];
                cmul_a_im = tmp16_im[p1_k1];
            end
            S_S2_DFT3: begin
                cmul_a_re = ram_b_dout_re;
                cmul_a_im = ram_b_dout_im;
            end
            S_S2_TW15: begin
                cmul_a_re = ram_a_dout_re;
                cmul_a_im = ram_a_dout_im;
            end
            S_S3_DFT5: begin
                cmul_a_re = ram_b_dout_re;
                cmul_a_im = ram_b_dout_im;
            end
            default: begin
                cmul_a_re = 24'sd0;
                cmul_a_im = 24'sd0;
            end
        endcase
        cmul_b_re = rom_re;
        cmul_b_im = rom_im;
    end

    always @(posedge clk) begin
        // Stage T1 -> T2: 乗算
        m_rr <= cmul_a_re * cmul_b_re;
        m_ii <= cmul_a_im * cmul_b_im;
        m_ri <= cmul_a_re * cmul_b_im;
        m_ir <= cmul_a_im * cmul_b_re;
        // Stage T2 -> T3: 加減算 + 四捨五入右シフト(14bit)
        cmul_re <= cmul_sum_re[23:0];
        cmul_im <= cmul_sum_im[23:0];
    end

    // T3 ステージ: バタフライ加減算 & 1bit右シフト (四捨五入)
    wire signed [24:0] bf_sum_re  = ({tmp16_re[p3_bf_e][23], tmp16_re[p3_bf_e]} + {cmul_re[23], cmul_re} + 25'sd1) >>> 1;
    wire signed [24:0] bf_sum_im  = ({tmp16_im[p3_bf_e][23], tmp16_im[p3_bf_e]} + {cmul_im[23], cmul_im} + 25'sd1) >>> 1;
    wire signed [24:0] bf_diff_re = ({tmp16_re[p3_bf_e][23], tmp16_re[p3_bf_e]} - {cmul_re[23], cmul_re} + 25'sd1) >>> 1;
    wire signed [24:0] bf_diff_im = ({tmp16_im[p3_bf_e][23], tmp16_im[p3_bf_e]} - {cmul_im[23], cmul_im} + 25'sd1) >>> 1;

    // T3 ステージ: 3点DFT / 5点DFT 最終和
    wire signed [23:0] acc_plus_cmul_re = acc_re + cmul_re;
    wire signed [23:0] acc_plus_cmul_im = acc_im + cmul_im;

    // -------------------------------------------------------------------------
    // RAM 制御信号のマルチプレクサ (Read RAM と Write RAM を完全分離)
    // -------------------------------------------------------------------------
    always @(*) begin
        ram_a_we     = 1'b0;
        ram_a_addr   = 8'd0;
        ram_a_din_re = 24'sd0;
        ram_a_din_im = 24'sd0;

        ram_b_we     = 1'b0;
        ram_b_addr   = 8'd0;
        ram_b_din_re = 24'sd0;
        ram_b_din_im = 24'sd0;

        case (state)
            S_LOAD: begin
                ram_a_we     = in_valid;
                ram_a_addr   = in_cnt;
                ram_a_din_re = in_re[23:0];
                ram_a_din_im = in_im[23:0];
            end
            S_S1_LOAD: begin
                ram_a_addr   = addr_15n1_n2;
            end
            S_S1_TW: begin
                ram_b_we     = p3_valid;
                ram_b_addr   = p3_addr;
                ram_b_din_re = cmul_re;
                ram_b_din_im = cmul_im;
            end
            S_S2_DFT3: begin
                // Read: ram_b (T0), Write: ram_a (T3, m1==2)
                ram_b_addr   = addr_15k1_5m1m2;
                ram_a_we     = p3_valid && (p3_m1 == 2'd2);
                ram_a_addr   = p3_addr;
                ram_a_din_re = acc_plus_cmul_re;
                ram_a_din_im = acc_plus_cmul_im;
            end
            S_S2_TW15: begin
                // Read: ram_a (T0), Write: ram_b (T3)
                ram_a_addr   = addr_15k1_5l1m2;
                ram_b_we     = p3_valid;
                ram_b_addr   = p3_addr;
                ram_b_din_re = cmul_re;
                ram_b_din_im = cmul_im;
            end
            S_S3_DFT5: begin
                // Read: ram_b (T0)
                ram_b_addr   = addr_15k1_5l1m2;
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // メイン FSM + パイプラインシフトレジスタ
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            done_wait_mode <= 1'b0;
            wait_cnt       <= 4'd0;
            inv_reg        <= 1'b0;
            issue_done     <= 1'b0;
            p1_valid       <= 1'b0;
            p2_valid       <= 1'b0;
            p3_valid       <= 1'b0;
            p1_n1          <= 4'd0;
            p1_k1          <= 4'd0;
            p1_bf_e        <= 4'd0;
            p2_bf_e        <= 4'd0;
            p3_bf_e        <= 4'd0;
            p1_bf_o        <= 4'd0;
            p1_m1          <= 2'd0;
            p2_m1          <= 2'd0;
            p3_m1          <= 2'd0;
            p1_m2          <= 3'd0;
            p2_m2          <= 3'd0;
            p3_m2          <= 3'd0;
            p1_addr        <= 8'd0;
            p2_addr        <= 8'd0;
            p3_addr        <= 8'd0;
            in_cnt         <= 8'd0;
            n1             <= 4'd0;
            n2             <= 4'd0;
            s_stage        <= 2'd0;
            bf_idx         <= 3'd0;
            k1             <= 4'd0;
            l1             <= 2'd0;
            l2             <= 3'd0;
            m1             <= 2'd0;
            m2             <= 3'd0;
            acc_re         <= 24'sd0;
            acc_im         <= 24'sd0;
            out_valid      <= 1'b0;
            out_idx        <= 8'd0;
            out_re         <= 32'sd0;
            out_im         <= 32'sd0;
            done           <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            done      <= 1'b0;

            // パイプライン有効フラグとメタデータのシフト
            p1_valid <= issue_en;
            p2_valid <= (state == S_S1_LOAD) ? 1'b0 : p1_valid;
            p3_valid <= p2_valid;

            p1_n1    <= n1;
            p1_k1    <= k1;
            p1_bf_e  <= bf_idx_e;
            p2_bf_e  <= p1_bf_e;
            p3_bf_e  <= p2_bf_e;
            p1_bf_o  <= bf_idx_o;

            p1_m1    <= m1;
            p2_m1    <= p1_m1;
            p3_m1    <= p2_m1;

            p1_m2    <= m2;
            p2_m2    <= p1_m2;
            p3_m2    <= p2_m2;

            p2_addr  <= p1_addr;
            p3_addr  <= p2_addr;
            case (state)
                S_S1_TW:   p1_addr <= addr_15k1_n2;
                S_S2_DFT3: p1_addr <= addr_15k1_5l1m2;
                S_S2_TW15: p1_addr <= addr_15k1_5l1m2;
                S_S3_DFT5: p1_addr <= addr_out_k;
                default:   p1_addr <= 8'd0;
            endcase

            if (done_wait_mode) begin
                // Pass 2の5段パイプライン完了を待つため8サイクル遅延後にdoneを出力
                if (wait_cnt == 4'd7) begin
                    done           <= 1'b1;
                    done_wait_mode <= 1'b0;
                    state          <= S_IDLE;
                end else begin
                    wait_cnt <= wait_cnt + 4'd1;
                end
            end else begin
                case (state)
                    S_IDLE: begin
                        if (start) begin
                            inv_reg    <= inv_en;
                            in_cnt     <= 8'd0;
                            issue_done <= 1'b0;
                            state      <= S_LOAD;
                        end
                    end

                    S_LOAD: begin
                        if (in_valid) begin
                            if (in_cnt == 8'd239) begin
                                n1         <= 4'd0;
                                n2         <= 4'd0;
                                issue_done <= 1'b0;
                                state      <= S_S1_LOAD;
                            end else begin
                                in_cnt <= in_cnt + 8'd1;
                            end
                        end
                    end

                    // Stage 1a: ram_a から16点をビットリバーサル順で tmp16 へ連続ロード (17 cycles)
                    S_S1_LOAD: begin
                        if (p1_valid) begin
                            tmp16_re[rev4(p1_n1)] <= ram_a_dout_re;
                            tmp16_im[rev4(p1_n1)] <= ram_a_dout_im;
                        end

                        if (issue_en) begin
                            if (n1 == 4'd15)
                                issue_done <= 1'b1;
                            else
                                n1 <= n1 + 4'd1;
                        end

                        if (issue_done && p1_valid) begin
                            issue_done <= 1'b0;
                            s_stage    <= 2'd0;
                            bf_idx     <= 3'd0;
                            state      <= S_S1_BF;
                        end
                    end

                    // Stage 1b: 16点 Radix-2 FFT (各ステージ 8+3 = 11 cycles)
                    S_S1_BF: begin
                        if (p3_valid) begin
                            tmp16_re[p3_bf_e] <= bf_sum_re[23:0];
                            tmp16_im[p3_bf_e] <= bf_sum_im[23:0];
                            tmp16_re[p3_bf_o] <= bf_diff_re[23:0];
                            tmp16_im[p3_bf_o] <= bf_diff_im[23:0];
                        end

                        if (issue_en) begin
                            if (bf_idx == 3'd7)
                                issue_done <= 1'b1;
                            else
                                bf_idx <= bf_idx + 3'd1;
                        end

                        if (issue_done && !p1_valid && !p2_valid && p3_valid) begin
                            issue_done <= 1'b0;
                            bf_idx     <= 3'd0;
                            if (s_stage == 2'd3) begin
                                k1    <= 4'd0;
                                state <= S_S1_TW;
                            end else begin
                                s_stage <= s_stage + 2'd1;
                            end
                        end
                    end

                    // Stage 1c: Twiddle W_240 乗算 -> ram_b 連続書込 (16+3 = 19 cycles)
                    S_S1_TW: begin
                        if (issue_en) begin
                            if (k1 == 4'd15)
                                issue_done <= 1'b1;
                            else
                                k1 <= k1 + 4'd1;
                        end

                        if (issue_done && !p1_valid && !p2_valid && p3_valid) begin
                            issue_done <= 1'b0;
                            k1         <= 4'd0;
                            if (n2 == 4'd14) begin
                                k1    <= 4'd0;
                                m2    <= 3'd0;
                                l1    <= 2'd0;
                                m1    <= 2'd0;
                                state <= S_S2_DFT3;
                            end else begin
                                n2    <= n2 + 4'd1;
                                n1    <= 4'd0;
                                state <= S_S1_LOAD;
                            end
                        end
                    end

                    // Stage 2a: 3点 DFT (ram_b読出 -> 720連続積和 -> ram_a書込, 723 cycles)
                    S_S2_DFT3: begin
                        if (p3_valid) begin
                            if (p3_m1 == 2'd0) begin
                                acc_re <= cmul_re;
                                acc_im <= cmul_im;
                            end else if (p3_m1 == 2'd1) begin
                                acc_re <= acc_re + cmul_re;
                                acc_im <= acc_im + cmul_im;
                            end
                            // p3_m1 == 2'd2 は ram_a_we により acc + cmul を直接書込
                        end

                        if (issue_en) begin
                            if (m1 == 2'd2) begin
                                m1 <= 2'd0;
                                if (l1 == 2'd2) begin
                                    l1 <= 2'd0;
                                    if (m2 == 3'd4) begin
                                        m2 <= 3'd0;
                                        if (k1 == 4'd15)
                                            issue_done <= 1'b1;
                                        else
                                            k1 <= k1 + 4'd1;
                                    end else begin
                                        m2 <= m2 + 3'd1;
                                    end
                                end else begin
                                    l1 <= l1 + 2'd1;
                                end
                            end else begin
                                m1 <= m1 + 2'd1;
                            end
                        end

                        if (issue_done && !p1_valid && !p2_valid && p3_valid) begin
                            issue_done <= 1'b0;
                            k1         <= 4'd0;
                            m2         <= 3'd0;
                            l1         <= 2'd0;
                            state      <= S_S2_TW15;
                        end
                    end

                    // Stage 2b: Twiddle W_15 乗算 (ram_a読出 -> 240連続乗算 -> ram_b書込, 243 cycles)
                    S_S2_TW15: begin
                        if (issue_en) begin
                            if (l1 == 2'd2) begin
                                l1 <= 2'd0;
                                if (m2 == 3'd4) begin
                                    m2 <= 3'd0;
                                    if (k1 == 4'd15)
                                        issue_done <= 1'b1;
                                    else
                                        k1 <= k1 + 4'd1;
                                end else begin
                                    m2 <= m2 + 3'd1;
                                end
                            end else begin
                                l1 <= l1 + 2'd1;
                            end
                        end

                        if (issue_done && !p1_valid && !p2_valid && p3_valid) begin
                            issue_done <= 1'b0;
                            k1         <= 4'd0;
                            l1         <= 2'd0;
                            l2         <= 3'd0;
                            m2         <= 3'd0;
                            state      <= S_S3_DFT5;
                        end
                    end

                    // Stage 3: 5点 DFT (ram_b読出 -> 1200連続積和 -> 出力, 1203 cycles)
                    S_S3_DFT5: begin
                        if (p3_valid) begin
                            if (p3_m2 == 3'd0) begin
                                acc_re <= cmul_re;
                                acc_im <= cmul_im;
                            end else if (p3_m2 < 3'd4) begin
                                acc_re <= acc_re + cmul_re;
                                acc_im <= acc_im + cmul_im;
                            end else begin
                                out_valid <= 1'b1;
                                out_idx   <= p3_addr;
                                out_re    <= {{8{acc_plus_cmul_re[23]}}, acc_plus_cmul_re};
                                out_im    <= {{8{acc_plus_cmul_im[23]}}, acc_plus_cmul_im};
                            end
                        end

                        if (issue_en) begin
                            if (m2 == 3'd4) begin
                                m2 <= 3'd0;
                                if (l2 == 3'd4) begin
                                    l2 <= 3'd0;
                                    if (l1 == 2'd2) begin
                                        l1 <= 2'd0;
                                        if (k1 == 4'd15)
                                            issue_done <= 1'b1;
                                        else
                                            k1 <= k1 + 4'd1;
                                    end else begin
                                        l1 <= l1 + 2'd1;
                                    end
                                end else begin
                                    l2 <= l2 + 3'd1;
                                end
                            end else begin
                                m2 <= m2 + 3'd1;
                            end
                        end

                        if (issue_done && !p1_valid && !p2_valid && p3_valid) begin
                            issue_done     <= 1'b0;
                            done_wait_mode <= 1'b1;
                            wait_cnt       <= 4'd0;
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule