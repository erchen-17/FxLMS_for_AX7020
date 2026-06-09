//////////////////////////////////////////////////////////////////////////////////
// Write y[n] + e[n] to DDR
// Write one 64-bit word each time sample_valid_sync is asserted
//////////////////////////////////////////////////////////////////////////////////
module mem_test
#(
	parameter MEM_DATA_BITS = 64,
	parameter ADDR_BITS = 32
)
(
	input rst,                                 // Reset
	input mem_clk,                             // Interface clock

	// AXI write control
	output reg wr_burst_req,                   // Write request
	output reg[9:0] wr_burst_len,              // Write data length
	output reg[ADDR_BITS - 1:0] wr_burst_addr, // Write start address

	// Synchronised audio samples (mem_clk domain)
	input signed [31:0] y_n_sync,      // control output y(n)  Q16.16
	input signed [31:0] e_n_sync,      // error microphone e(n) Q16.16
	input sample_valid_sync,

	// AXI master interface
	input wr_burst_data_req,
	output [MEM_DATA_BITS - 1:0] wr_burst_data,
	input wr_burst_finish
);

//==============================
// ״̬������
//==============================
parameter IDLE      = 2'd0;
parameter MEM_WRITE = 2'd1;

parameter BURST_LEN = 1;  // ÿ��дһ��64bit

reg [1:0] state;

//==============================
// Data packing: upper 32-bit = y[n], lower 32-bit = e[n]
//==============================
assign wr_burst_data = {y_n_sync, e_n_sync};

//==============================
// ״̬��
//==============================
always @(posedge mem_clk or posedge rst)
begin
	if(rst)
	begin
		state <= IDLE;
		wr_burst_req <= 1'b0;
		wr_burst_len <= BURST_LEN;

		// DDR��ʼ��ַ
		wr_burst_addr <= 32'h0200_0000;
	end

	else
	begin
		case(state)

		//--------------------------------
		// �ȴ��µ���Ƶ����
		//--------------------------------
		IDLE:
		begin
			if(sample_valid_sync)
			begin
				wr_burst_req <= 1'b1;
				state <= MEM_WRITE;
			end
		end

		//--------------------------------
		// дDDR
		//--------------------------------
		MEM_WRITE:
		begin
			if(wr_burst_finish)
			begin
				wr_burst_req <= 1'b0;

				// ��ַ����8�ֽڣ�64bit��
				wr_burst_addr <= wr_burst_addr + 8;

				state <= IDLE;
			end
		end

		default:
			state <= IDLE;

		endcase
	end
end

endmodule