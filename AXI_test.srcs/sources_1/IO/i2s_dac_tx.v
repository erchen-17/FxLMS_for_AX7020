module i2s_dac_tx (
    input  wire        rst_n,

    input  wire        bclk_neg,     // BCLK 下降沿
    input  wire        lrclk,        // 0 = Left, 1 = Right

    input  wire [23:0] dac_data,     // 并行音频数据
    input  wire        dac_valid,    // 保留接口，但本测试不用

    output reg         dac_dat
);

    reg [5:0]  bit_cnt;
    reg [23:0] shift_reg;
    reg        lrclk_d;

    // 记录 LRCLK 边沿
    always @(negedge bclk_neg or negedge rst_n) begin
        if (!rst_n)
            lrclk_d <= 1'b0;
        else
            lrclk_d <= lrclk;
    end

    always @(negedge bclk_neg or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt   <= 6'd0;
            shift_reg <= 24'd0;
            dac_dat   <= 1'b0;
        end else begin

            // LRCLK 翻转 → 新声道开始
            if (lrclk_d != lrclk) begin
                bit_cnt   <= 6'd0;
                shift_reg <= dac_data;   // 装载新数据
                dac_dat   <= 1'b0;
            end else begin
                bit_cnt <= bit_cnt + 1'b1;

                // I2S：第 1bit 空位
                if (bit_cnt == 6'd0) begin
                    dac_dat <= 1'b0;
                end
                // 24bit 数据
                else if (bit_cnt <= 6'd24) begin
                    dac_dat   <= shift_reg[23];
                    shift_reg <= {shift_reg[22:0], 1'b0};
                end
                // 填 0
                else begin
                    dac_dat <= 1'b0;
                end
            end
        end
    end

endmodule