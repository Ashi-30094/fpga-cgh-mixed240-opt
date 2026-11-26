`timescale 1ns / 1ps

module basys3_cgh_vga_top (
    input  wire        clk,       // 100 MHz
    input  wire        btnC,      // Start / Re-run
    input  wire        btnU,      // Reset (active high)
    input  wire [2:0]  sw,        // sw[0]: invert, sw[2:1]: brightness gain
    output wire [1:0]  led,       // led[0]: busy, led[1]: done
    output reg  [6:0]  seg,       // 7-segment cathodes (active low)
    output reg         dp,        // decimal point (active low)
    output reg  [3:0]  an,        // 7-segment anodes (active low)
    output reg  [3:0]  vgaRed,
    output reg  [3:0]  vgaGreen,
    output reg  [3:0]  vgaBlue,
    output wire        Hsync,
    output wire        Vsync
);

    wire rst_n = ~btnU;

    // Button synchronizer & rising-edge detector for btnC
    reg [1:0] btn_sync;
    reg       btn_prev;
    wire      btn_start_pulse = btn_sync[1] & ~btn_prev;
    always @(posedge clk) begin
        if (!rst_n) begin
            btn_sync <= 2'b00;
            btn_prev <= 1'b0;
        end else begin
            btn_sync <= {btn_sync[0], btnC};
            btn_prev <= btn_sync[1];
        end
    end

    // -------------------------------------------------------------------------
    // 1. 1-bit Hologram ROM & Loader FSM
    // -------------------------------------------------------------------------
    localparam M_LOAD_PREP   = 3'd0;
    localparam M_LOAD_WAIT   = 3'd1;
    localparam M_LOAD        = 3'd2;
    localparam M_START_PULSE = 3'd3;
    localparam M_CALC        = 3'd4;
    localparam M_DISPLAY     = 3'd5;

    reg  [2:0]  m_state;
    reg  [15:0] rom_addr;
    reg  [15:0] wr_cnt;
    wire        rom_bit;

    rom_holo_1bit u_holo_rom (
        .clk  (clk),
        .addr (rom_addr),
        .dout (rom_bit)
    );

    reg                load_we;
    reg         [15:0] load_addr;
    reg  signed [11:0] load_din_re;
    reg                cgh_start;

    wire               cgh_busy;
    wire               cgh_done;
    wire signed [11:0] cgh_dout_re, cgh_dout_im;

    // VGA address muxed into cgh_2d_top during M_DISPLAY
    wire [15:0] vga_addr;
    wire [15:0] ext_addr_mux = (m_state == M_DISPLAY) ? vga_addr : load_addr;

    // Mixed-Radix FFT (16x3x5) 用のシフト量 (9, 10, 2, 5) に修正
    cgh_2d_top #(
        .NUM_LINES   (240),
        .SHIFT_PASS1 (5'd9),
        .SHIFT_PASS2 (5'd10),
        .SHIFT_PASS3 (5'd2),
        .SHIFT_PASS4 (5'd5)
    ) u_cgh_core (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (cgh_start),
        .ext_we      (load_we),
        .ext_addr    (ext_addr_mux),
        .ext_din_re  (load_din_re),
        .ext_din_im  (12'sd0),
        .ext_dout_re (cgh_dout_re),
        .ext_dout_im (cgh_dout_im),
        .busy        (cgh_busy),
        .done        (cgh_done)
    );

    assign led[0] = (m_state != M_DISPLAY);
    assign led[1] = (m_state == M_DISPLAY);

    // 0.1 ms BCD Timer (100 MHz / 10,000 = 10 kHz -> 0.1 ms per tick)
    reg [13:0] tick_cnt;
    reg [3:0]  bcd0, bcd1, bcd2, bcd3; // bcd3 bcd2 bcd1 . bcd0 [ms]

    always @(posedge clk) begin
        if (!rst_n) begin
            m_state     <= M_LOAD_PREP;
            rom_addr    <= 16'd0;
            wr_cnt      <= 16'd0;
            load_we     <= 1'b0;
            load_addr   <= 16'd0;
            load_din_re <= 12'sd0;
            cgh_start   <= 1'b0;
            tick_cnt    <= 14'd0;
            bcd0 <= 4'd0; bcd1 <= 4'd0; bcd2 <= 4'd0; bcd3 <= 4'd0;
        end else begin
            load_we   <= 1'b0;
            cgh_start <= 1'b0;

            case (m_state)
                M_LOAD_PREP: begin
                    rom_addr <= 16'd0;
                    wr_cnt   <= 16'd0;
                    tick_cnt <= 14'd0;
                    bcd0 <= 4'd0; bcd1 <= 4'd0; bcd2 <= 4'd0; bcd3 <= 4'd0;
                    m_state  <= M_LOAD_WAIT;
                end

                M_LOAD_WAIT: begin
                    rom_addr <= 16'd1;
                    m_state  <= M_LOAD;
                end

                M_LOAD: begin
                    load_we     <= 1'b1;
                    load_addr   <= wr_cnt;
                    load_din_re <= rom_bit ? 12'sd1024 : 12'sd0;

                    if (wr_cnt < 16'd57598) begin
                        rom_addr <= wr_cnt + 16'd2;
                    end

                    if (wr_cnt == 16'd57599) begin
                        m_state <= M_START_PULSE;
                    end else begin
                        wr_cnt <= wr_cnt + 16'd1;
                    end
                end

                M_START_PULSE: begin
                    cgh_start <= 1'b1;
                    m_state   <= M_CALC;
                end

                M_CALC: begin
                    if (cgh_done) begin
                        m_state <= M_DISPLAY;
                    end else begin
                        if (tick_cnt == 14'd9999) begin
                            tick_cnt <= 14'd0;
                            if (bcd0 == 4'd9) begin
                                bcd0 <= 4'd0;
                                if (bcd1 == 4'd9) begin
                                    bcd1 <= 4'd0;
                                    if (bcd2 == 4'd9) begin
                                        bcd2 <= 4'd0;
                                        if (bcd3 < 4'd9) bcd3 <= bcd3 + 4'd1;
                                    end else bcd2 <= bcd2 + 4'd1;
                                end else bcd1 <= bcd1 + 4'd1;
                            end else bcd0 <= bcd0 + 4'd1;
                        end else begin
                            tick_cnt <= tick_cnt + 14'd1;
                        end
                    end
                end

                M_DISPLAY: begin
                    if (btn_start_pulse) begin
                        m_state <= M_LOAD_PREP;
                    end
                end

                default: m_state <= M_LOAD_PREP;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 2. VGA 640x480 @ 60Hz Controller (25 MHz pixel tick in 100 MHz domain)
    //    Displays 240x240 image zoomed 2x (480x480) centered at x=80..559
    // -------------------------------------------------------------------------
    reg [1:0] pix_div;
    reg [9:0] h_cnt;
    reg [9:0] v_cnt;

    wire in_image = (m_state == M_DISPLAY) && (h_cnt >= 10'd80) && (h_cnt < 10'd560) && (v_cnt < 10'd480);
    wire [7:0] img_x = (h_cnt - 10'd80) >> 1;
    wire [7:0] img_y = 8'd239 - v_cnt[8:1]; // Flip Y to match BMP orientation

    // addr = img_x * 240 + img_y = (img_x << 8) - (img_x << 4) + img_y
    assign vga_addr = ({8'd0, img_x} << 8) - ({8'd0, img_x} << 4) + {8'd0, img_y};

    assign Hsync = ~((h_cnt >= 10'd656) && (h_cnt < 10'd752));
    assign Vsync = ~((v_cnt >= 10'd490) && (v_cnt < 10'd492));

    // Intensity pipeline across the 4 clocks of each 25 MHz pixel
    reg [23:0] sq_re, sq_im;
    reg [24:0] mag2;
    reg [3:0]  gray4;

    wire [24:0] mag_shifted = (sw[2:1] == 2'd0) ? (mag2 >> 15) :
                              (sw[2:1] == 2'd1) ? (mag2 >> 14) :
                              (sw[2:1] == 2'd2) ? (mag2 >> 13) : (mag2 >> 16);
    wire [3:0]  sat_val     = (|mag_shifted[24:4]) ? 4'hF : mag_shifted[3:0];
    wire [3:0]  disp_val    = sw[0] ? sat_val : (4'hF - sat_val);

    always @(posedge clk) begin
        if (!rst_n) begin
            pix_div  <= 2'd0;
            h_cnt    <= 10'd0;
            v_cnt    <= 10'd0;
            sq_re    <= 24'd0;
            sq_im    <= 24'd0;
            mag2     <= 25'd0;
            gray4    <= 4'd0;
            vgaRed   <= 4'd0;
            vgaGreen <= 4'd0;
            vgaBlue  <= 4'd0;
        end else begin
            pix_div <= pix_div + 2'd1;

            if (pix_div == 2'd1) begin
                sq_re <= $signed(cgh_dout_re) * $signed(cgh_dout_re);
                sq_im <= $signed(cgh_dout_im) * $signed(cgh_dout_im);
            end else if (pix_div == 2'd2) begin
                mag2 <= sq_re + sq_im;
            end else if (pix_div == 2'd3) begin
                if (in_image) begin
                    vgaRed   <= disp_val;
                    vgaGreen <= disp_val;
                    vgaBlue  <= disp_val;
                end else begin
                    vgaRed   <= 4'd0;
                    vgaGreen <= 4'd0;
                    vgaBlue  <= 4'd0;
                end

                if (h_cnt == 10'd799) begin
                    h_cnt <= 10'd0;
                    if (v_cnt == 10'd524) v_cnt <= 10'd0;
                    else                  v_cnt <= v_cnt + 10'd1;
                end else begin
                    h_cnt <= h_cnt + 10'd1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. 4-Digit 7-Segment Display Driver (Displays XXX.X ms)
    // -------------------------------------------------------------------------
    reg [18:0] ref_cnt;
    always @(posedge clk) begin
        if (!rst_n) ref_cnt <= 19'd0;
        else        ref_cnt <= ref_cnt + 19'd1;
    end

    reg [3:0] cur_digit;
    always @(*) begin
        case (ref_cnt[18:17])
            2'd0: begin an = 4'b1110; cur_digit = bcd0; dp = 1'b1; end
            2'd1: begin an = 4'b1101; cur_digit = bcd1; dp = 1'b0; end // Decimal point ON
            2'd2: begin an = 4'b1011; cur_digit = bcd2; dp = 1'b1; end
            2'd3: begin an = 4'b0111; cur_digit = bcd3; dp = 1'b1; end
        endcase
    end

    always @(*) begin
        case (cur_digit)
            4'd0: seg = 7'b1000000;
            4'd1: seg = 7'b1111001;
            4'd2: seg = 7'b0100100;
            4'd3: seg = 7'b0110000;
            4'd4: seg = 7'b0011001;
            4'd5: seg = 7'b0010010;
            4'd6: seg = 7'b0000010;
            4'd7: seg = 7'b1111000;
            4'd8: seg = 7'b0000000;
            4'd9: seg = 7'b0010000;
            default: seg = 7'b1111111;
        endcase
    end

endmodule