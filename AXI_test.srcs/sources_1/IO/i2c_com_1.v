module i2c_com_1(
    input  clock_i2c_1,        // 20kHz I2C clock
    input  reset_n,

    output ack_1,
    input  [23:0] i2c_data_1,
    input  start_1,
    output reg tr_end_1,
    output reg [5:0] cyc_count_1,

    output i2c_sclk_1,
    inout  i2c_sdat_1
);

    reg reg_sdat;
    reg sclk;
    reg ack1, ack2, ack3;

    assign ack_1 = ack1 | ack2 | ack3;
    assign i2c_sclk_1 = sclk | (((cyc_count_1 >= 4) && (cyc_count_1 <= 30)) ? ~clock_i2c_1 : 1'b0);
    assign i2c_sdat_1 = reg_sdat ? 1'bz : 1'b0;

    always @(posedge clock_i2c_1 or negedge reset_n)
        if(!reset_n)
            cyc_count_1 <= 6'b111111;
        else begin
            if(start_1 == 0)
                cyc_count_1 <= 0;
            else if(cyc_count_1 < 6'b111111)
                cyc_count_1 <= cyc_count_1 + 1'b1;
        end

    always @(posedge clock_i2c_1 or negedge reset_n)
        if(!reset_n) begin
            tr_end_1 <= 0;
            ack1 <= 1;
            ack2 <= 1;
            ack3 <= 1;
            sclk <= 1;
            reg_sdat <= 1;
        end else begin
            case(cyc_count_1)
                0: begin ack1<=1;ack2<=1;ack3<=1;tr_end_1<=0;sclk<=1;reg_sdat<=1;end
                1: reg_sdat <= 0;
                2: sclk <= 0;
                3: reg_sdat <= i2c_data_1[23];
                4: reg_sdat <= i2c_data_1[22];
                5: reg_sdat <= i2c_data_1[21];
                6: reg_sdat <= i2c_data_1[20];
                7: reg_sdat <= i2c_data_1[19];
                8: reg_sdat <= i2c_data_1[18];
                9: reg_sdat <= i2c_data_1[17];
                10: reg_sdat <= i2c_data_1[16];
                11: reg_sdat <= 1;

                12: begin reg_sdat <= i2c_data_1[15]; ack1 <= i2c_sdat_1; end
                13: reg_sdat <= i2c_data_1[14];
                14: reg_sdat <= i2c_data_1[13];
                15: reg_sdat <= i2c_data_1[12];
                16: reg_sdat <= i2c_data_1[11];
                17: reg_sdat <= i2c_data_1[10];
                18: reg_sdat <= i2c_data_1[9];
                19: reg_sdat <= i2c_data_1[8];
                20: reg_sdat <= 1;

                21: begin reg_sdat <= i2c_data_1[7]; ack2 <= i2c_sdat_1; end
                22: reg_sdat <= i2c_data_1[6];
                23: reg_sdat <= i2c_data_1[5];
                24: reg_sdat <= i2c_data_1[4];
                25: reg_sdat <= i2c_data_1[3];
                26: reg_sdat <= i2c_data_1[2];
                27: reg_sdat <= i2c_data_1[1];
                28: reg_sdat <= i2c_data_1[0];
                29: reg_sdat <= 1;

                30: begin ack3 <= i2c_sdat_1; sclk <= 0; reg_sdat <= 0; end
                31: sclk <= 1;
                32: begin reg_sdat <= 1; tr_end_1 <= 1; end
            endcase
        end

endmodule