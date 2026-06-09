module reg_config_1(
    input  clock_50m,
    input  reset_n,

    output i2c_sclk_1,
    inout  i2c_sdat_1
);

    reg clock_20k;
    reg [15:0] clock_20k_cnt;
    reg [1:0] config_step;
    reg [3:0] reg_index;
    reg [23:0] i2c_data_1;
    reg [15:0] reg_data;
    reg start_1;

    wire ack_1;
    wire tr_end_1;

    i2c_com_1 u_i2c_com_1 (
        .clock_i2c_1 (clock_20k),
        .reset_n     (reset_n),
        .ack_1       (ack_1),
        .i2c_data_1  (i2c_data_1),
        .start_1     (start_1),
        .tr_end_1    (tr_end_1),
        .i2c_sclk_1  (i2c_sclk_1),
        .i2c_sdat_1  (i2c_sdat_1)
    );

    always @(posedge clock_50m or negedge reset_n)
        if(!reset_n)
            clock_20k_cnt <= 0;
        else if(clock_20k_cnt < 2499)
            clock_20k_cnt <= clock_20k_cnt + 1'b1;
        else
            clock_20k_cnt <= 0;

    always @(posedge clock_50m or negedge reset_n)
        if(!reset_n)
            clock_20k <= 0;
        else if(clock_20k_cnt >= 2499)
            clock_20k <= ~clock_20k;

    always @(posedge clock_20k or negedge reset_n)
        if(!reset_n) begin
            config_step <= 0;
            start_1 <= 0;
            reg_index <= 0;
        end else begin
            if(reg_index < 10) begin
                case(config_step)
                    0: begin
                        i2c_data_1 <= {8'h34, reg_data}; // 地址你后面自己改
                        start_1 <= 1;
                        config_step <= 1;
                    end
                    1: if(tr_end_1) begin
                        start_1 <= 0;
                        if(!ack_1)
                            config_step <= 2;
                        else
                            config_step <= 0;
                    end
                    2: begin
                        reg_index <= reg_index + 1'b1;
                        config_step <= 0;
                    end
                endcase
            end
        end

    always @(reg_index) begin
        case(reg_index)
            0: reg_data <= 16'h001f;
            1: reg_data <= 16'h021f;
            2: reg_data <= 16'h0461;
            3: reg_data <= 16'h0661;
            4: reg_data <= 16'h0814;
            5: reg_data <= 16'h0a01;
            6: reg_data <= 16'h0c00;
            7: reg_data <= 16'h0e0a;
            8: reg_data <= 16'h1000;
            9: reg_data <= 16'h1201;
            default: reg_data <= 16'h001a;
        endcase
    end

endmodule