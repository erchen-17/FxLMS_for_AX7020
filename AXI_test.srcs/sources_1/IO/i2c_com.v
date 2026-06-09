module i2c_com(
	input  clock_i2c, 	      //wm8731控制接口传输所需时钟，0-400khz，此处为20khz  
	input  reset_n,   
	output ack, 			      //应答信号  
	input  [23:0]i2c_data,   	//sdin接口传输的24位数据  
	input  start, 		         //开始传输标志 ，这个标志置1开始发送
	output reg tr_end, 		   //传输结束标志 ，这个标志变成1说明发送接收（发送前为0）
	output reg [5:0]cyc_count, //计数器，受start信号控制
	output i2c_sclk, 	         //FPGA与wm8731时钟接口  
	inout  i2c_sdat //FPGA与wm8731数据接口  我现在明白为什么这个IO口要设置为inout模式了，
//因为IIC协议每发送一个字节就需要接收应答位，因此这个端口即需要输入也需要输出，因此设置为inout
//并且设置为输入的时候也很巧妙，用assign语句把信号设置为高阻状态，这样就可以接收到外部信号了！
); 
 
 
	reg reg_sdat;  
	reg sclk;  
	reg ack1,ack2,ack3;   
 

	assign ack = ack1|ack2|ack3;  
	assign i2c_sclk = sclk|(((cyc_count>=4) && (cyc_count<=30)) ? ~clock_i2c : 1'b0);  
    assign i2c_sdat = reg_sdat ? 1'bz : 1'b0;  
//为什么是高阻态？输入的时候用高阻态意味着这个位会根据外部变化而变化（相当于接收外部信号ACK）

 
  //IIC 写操作序列计数器（这个计数器意思是只计数一次，复位之后才会计数第二次，并且只有start信号才能开启计数）
	always@(posedge clock_i2c or negedge reset_n)  
	if(!reset_n)  
		cyc_count <= 6'b111111;  
	else begin  
		if(start == 0)  
			cyc_count <= 0;  
		else if(cyc_count < 6'b111111)  
			cyc_count <= cyc_count + 1'b1;  
	end 
 
	//IIC 配置主序列
	always@(posedge clock_i2c or negedge reset_n)  
	if(!reset_n) begin  
		tr_end<=0;  
		ack1<=1;  
		ack2<=1;  
		ack3<=1;  
		sclk<=1;  
		reg_sdat<=1;  
	end else begin
		case(cyc_count) //当cyc_count开始计数的时候就说明start信号置1了 
			0:begin ack1<=1;ack2<=1;ack3<=1;tr_end<=0;sclk<=1;reg_sdat<=1;end  
			1:reg_sdat<=0; //开始传输，在这个阶段刚好是i2cclk为高，i2cdat从高变低（I2C的开始标志）
			2:sclk<=0;  
			3:reg_sdat<=i2c_data[23];  
			4:reg_sdat<=i2c_data[22];  
			5:reg_sdat<=i2c_data[21];  
			6:reg_sdat<=i2c_data[20];  
			7:reg_sdat<=i2c_data[19];  
			8:reg_sdat<=i2c_data[18];  
			9:reg_sdat<=i2c_data[17];  
			10:reg_sdat<=i2c_data[16];  
			11:reg_sdat<=1; //应答信号
 
			12:begin reg_sdat<=i2c_data[15];ack1<=i2c_sdat;end  
			13:reg_sdat<=i2c_data[14];  
			14:reg_sdat<=i2c_data[13];  
			15:reg_sdat<=i2c_data[12];  
			16:reg_sdat<=i2c_data[11];  
			17:reg_sdat<=i2c_data[10];  
			18:reg_sdat<=i2c_data[9];  
			19:reg_sdat<=i2c_data[8];  
			20:reg_sdat<=1; //应答信号  
 
			21:begin reg_sdat<=i2c_data[7];ack2<=i2c_sdat;end  
			22:reg_sdat<=i2c_data[6];  
			23:reg_sdat<=i2c_data[5];  
			24:reg_sdat<=i2c_data[4];  
			25:reg_sdat<=i2c_data[3];  
			26:reg_sdat<=i2c_data[2];  
			27:reg_sdat<=i2c_data[1];  
			28:reg_sdat<=i2c_data[0];  
			29:reg_sdat<=1; //应答信号  
 
			30:begin ack3<=i2c_sdat;sclk<=0;reg_sdat<=0;end  
			31:sclk<=1;  
			32:begin reg_sdat<=1;tr_end<=1;end
 
		endcase  
	end 

endmodule