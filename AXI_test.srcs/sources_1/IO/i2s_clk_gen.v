module i2s_clk_gen (
    input  wire clock_50m,   // FPGA 系统时钟 50MHz
    input  wire rst_n,        // 低有效复位

    output wire bclk,        // I2S Bit Clock
    output wire lrclk,       // I2S Word Select (Left/Right Clock)

    output wire bclk_pos,    // BCLK 上升沿：内部控制用
    output wire bclk_neg     // BCLK 下降沿：ADC / DAC 采样用
);

    // =========================================================
    // 参数说明
    // Fs      = 48kHz
    // Slot    = 32bit × 2 = 64 BCLK / LRCLK
    // BCLK    = 48k × 64 = 3.072 MHz
    // =========================================================

    localparam integer ACC_WIDTH = 16;

    // 分数分频步进值：
    // BCLK ≈ 50MHz × BCLK_STEP / 2^16
    localparam integer BCLK_STEP = 4027;

    // =========================================================
    // 寄存器定义
    // =========================================================
    reg bclk_r;                         // 内部 BCLK 寄存器
    reg [ACC_WIDTH-1:0] acc;            // 分数分频累加器

    reg [6:0] bit_cnt;                  // 0~63，一个 LRCLK 周期
    reg lrclk_r;                        // LRCLK 寄存器

    // =========================================================
    // BCLK 生成（分数分频）
    // =========================================================
    // 原理：
    // - acc 每个 50MHz 时钟加一次 BCLK_STEP
    // - acc 溢出时翻转一次 BCLK
    // =========================================================
    always @(posedge clock_50m or negedge rst_n) begin
        if (!rst_n) begin
            acc    <= 0;
            bclk_r <= 1'b0;
        end else begin
            acc <= acc + BCLK_STEP;
            if (acc[ACC_WIDTH-1]) begin
                bclk_r <= ~bclk_r;      // 翻转 BCLK
                acc[ACC_WIDTH-1] <= 1'b0;
            end
        end
    end

    // =========================================================
    // LRCLK 生成（严格 64 个 BCLK 翻转一次）
    // =========================================================
    // I2S 标准：
    // - LRCLK = 0 → Left Channel
    // - LRCLK = 1 → Right Channel
    // =========================================================
always @(posedge bclk_r or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt <= 6'd0;
            lrclk_r <= 1'b0;
        end else begin
            if (bit_cnt == 6'd31) begin
                bit_cnt <= 6'd0;
                lrclk_r <= ~lrclk_r;   // 每 32 个 BCLK 翻转
            end else begin
                bit_cnt <= bit_cnt + 1'b1;
            end
        end
    end

    // =========================================================
    // 输出信号绑定
    // =========================================================
    assign bclk     = bclk_r;
    assign lrclk    = lrclk_r;

    // 明确区分两个沿的用途
    assign bclk_pos =  bclk_r;   // 上升沿：内部计数、状态机
    assign bclk_neg = ~bclk_r;   // 下降沿：ADC / DAC 数据采样

endmodule