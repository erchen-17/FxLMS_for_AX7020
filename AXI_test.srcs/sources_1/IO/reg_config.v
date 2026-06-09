module reg_config(
	clock_50m,  
	i2c_sclk,  
	i2c_sdat,  
	reset_n
);  

	input clock_50m;  
	input reset_n;  
	output i2c_sclk;  
	inout i2c_sdat;  
 
	reg clock_20k;              //这个20k时钟是给i2c准备的，因为iic是低速总线
	reg [15:0]clock_20k_cnt;  
	reg [1:0]config_step;       //配置的步骤，有点类似状态机
	reg [3:0]reg_index;         //
	reg [23:0]i2c_data;  
	reg [15:0]reg_data;  
	reg start;  
 
	wire ack;
	wire tr_end;
 
	i2c_com i2c_com(
	   .clock_i2c(clock_20k),  
		.reset_n(reset_n),  
		.ack(ack),  
		.i2c_data(i2c_data),  
		.start(start),  
		.tr_end(tr_end),  
		.i2c_sclk(i2c_sclk),  
		.i2c_sdat(i2c_sdat)
	);  
  /**************************************************************************/
	always@(posedge clock_50m or negedge reset_n) //产生i2c控制时钟-20khz  
	if(!reset_n)  
		clock_20k_cnt <= 0;  
	else if(clock_20k_cnt < 2499)  
		clock_20k_cnt <= clock_20k_cnt + 1'b1;  
	else  
		clock_20k_cnt <= 0;  

	always@(posedge clock_50m or negedge reset_n) //产生i2c控制时钟-20khz  
	if(!reset_n)  
		clock_20k <= 0;  
	else if(clock_20k_cnt >= 2499)  
		clock_20k <= ~clock_20k;  

	always@(posedge clock_20k or negedge reset_n) //配置过程控制  
	if(!reset_n) begin  
		config_step<=0;  
		start<=0;  
		reg_index<=0;  
	end  
	else begin  
		if(reg_index<10) begin  
			case(config_step)  
				0:
					begin
						i2c_data<={8'h34,reg_data};  //器件地址0011010+一位写入0，reg_data是器件的寄存器写入值
						start<=1;                    //start=1的时候开启iic，两根线开始群魔乱舞
						config_step<=1;  
					end

				1:
					if(tr_end) begin                //if（tredn）意味着24位数据发送完全，传输结束
						start<=0; 
						if(!ack)  						  //如果任何一字节的数据没有收到ack，则重新发送
							config_step<=2;  
						else  
							config_step<=0;
					end

				2: begin reg_index <= reg_index + 1'b1; config_step <= 0; end
																//切换到下一个寄存器的值，重新发送一次（一共发送十次）
				default:config_step <= 0;
			endcase
		end	
	end  

	always@(reg_index)    								//需要搞明白这些寄存器的数值分别是什么意思
	begin  
		case(reg_index)  
			0:reg_data<=16'h001f;  
			1:reg_data<=16'h021f;  
			2:reg_data<=16'h0461; //0100 0011 0000  46f is the most adequate num
			3:reg_data<=16'h0661;  
			4:reg_data<=16'h0814;//000010100，禁用麦克风20dB增益  
			5:reg_data<=16'h0a01;  
			6:reg_data<=16'h0c00;  
			7:reg_data<=16'h0e0a;  //000001010
			8:reg_data<=16'h1000;  //12.288Mhz工作时钟，48kHz采样率，256fs
			9:reg_data<=16'h1201;  
			default:reg_data<=16'h001a;  
		endcase  
	end  

endmodule