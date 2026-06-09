module i2s_adc_rx_1 (
    input  wire        rst_n,       // ����Ч��λ

    input  wire        bclk_neg,     // BCLK �½���
    input  wire        lrclk,        // 0 = Left, 1 = Right
    input  wire        adc_dat_1,      // WM8731 ADC ��������

    output reg [23:0]  adc_left_1,     // Left ������������
    output reg         adc_valid_1     // Left ������Чָʾ
);

    // =========================================================
    // �ڲ��Ĵ���
    // =========================================================
    reg [5:0]  bit_cnt_1;
    reg [23:0] shift_reg_1;
    reg        lrclk_d_1;

    // =========================================================
    // LRCLK �ӳ٣����ڼ�����
    // =========================================================
    always @(negedge bclk_neg or negedge rst_n) begin
        if (!rst_n)
            lrclk_d_1 <= 1'b0;
        else
            lrclk_d_1 <= lrclk;
    end

    // =========================================================
    // ADC ���ݽ��գ�Left ������
    // =========================================================
    always @(negedge bclk_neg or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt_1   <= 6'd0;
            shift_reg_1 <= 24'd0;
            adc_left_1  <= 24'd0;
            adc_valid_1 <= 1'b0;
        end else begin
            adc_valid_1 <= 1'b0;  // Ĭ����Ч

            // ---------- LRCLK ���أ���������ʼ ----------
            if (lrclk_d_1 != lrclk) begin
                bit_cnt_1 <= 6'd0;
            end
            // ---------- Left Slot ----------
            else if (lrclk == 1'b0) begin
                bit_cnt_1 <= bit_cnt_1 + 1'b1;

                if (bit_cnt_1 >= 6'd2 && bit_cnt_1 <= 6'd25) begin
                    shift_reg_1 <= {shift_reg_1[22:0], adc_dat_1};
                end

                if (bit_cnt_1 == 6'd26) begin
                    adc_left_1  <= shift_reg_1;
                    adc_valid_1 <= 1'b1;
                end
            end
        end
    end

endmodule