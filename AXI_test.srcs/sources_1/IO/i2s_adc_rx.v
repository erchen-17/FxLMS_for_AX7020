module i2s_adc_rx (
    input  wire        rst_n,       // ����Ч��λ

    input  wire        bclk_neg,     // BCLK �½���
    input  wire        lrclk,        // 0 = Left, 1 = Right
    input  wire        adc_dat,      // WM8731 ADC ��������

    output reg [23:0]  adc_left,     // Left ������������
    output reg         adc_valid     // Left ������Чָʾ
);

    // =========================================================
    // �ڲ��Ĵ���
    // =========================================================
    reg [5:0]  bit_cnt;
    reg [23:0] shift_reg;
    reg        lrclk_d;

    // =========================================================
    // LRCLK �ӳ٣����ڼ�����
    // =========================================================
    always @(negedge bclk_neg or negedge rst_n) begin
        if (!rst_n)
            lrclk_d <= 1'b0;
        else
            lrclk_d <= lrclk;
    end

    // =========================================================
    // ADC ���ݽ��գ�Left ������
    // =========================================================
    always @(negedge bclk_neg or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt   <= 6'd0;
            shift_reg <= 24'd0;
            adc_left  <= 24'd0;
            adc_valid <= 1'b0;
        end else begin
            adc_valid <= 1'b0;  // Ĭ����Ч

            // ---------- LRCLK ���أ���������ʼ ----------
            if (lrclk_d != lrclk) begin
                bit_cnt <= 6'd0;
            end
            // ---------- Left Slot ----------
            else if (lrclk == 1'b0) begin
                bit_cnt <= bit_cnt + 1'b1;

                if (bit_cnt >= 6'd2 && bit_cnt <= 6'd25) begin
                    shift_reg <= {shift_reg[22:0], adc_dat};
                end

                if (bit_cnt == 6'd26) begin
                    adc_left  <= shift_reg;
                    adc_valid <= 1'b1;
                end
            end
        end
    end

endmodule