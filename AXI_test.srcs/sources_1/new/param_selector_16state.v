// param_selector_16state.v
// 1按键切换 16组参数：4种mu + 4种s_init_delay
// 修复：边沿检测 + 上电初始状态正确 + LED左高右低
module param_selector_16state #(
    parameter MU_WIDTH    = 32,
    parameter DELAY_WIDTH = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  key_press,    // 消抖后的按键（高电平有效）
    
    // 输出给 fxlms_core
    output reg  [MU_WIDTH-1:0]   mu_out,
    output reg  [DELAY_WIDTH-1:0] s_delay_out,
    
    // LED 显示 0~15 (LED1=最高位, LED4=最低位)
    output wire                  led_1st,
    output wire                  led_2nd,
    output wire                  led_3rd,
    output wire                  led_4th
);

// 4位计数器：0 ~ 15
reg [3:0] state_cnt;

// 边沿检测
reg key_press_delay;
wire key_press_edge;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        key_press_delay <= 1'b0;
    else
        key_press_delay <= key_press;
end

// 生成上升沿脉冲（松开时切换，最稳定）
assign key_press_edge = (~key_press_delay) & key_press;

// 按键边沿触发 → 状态+1
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state_cnt <= 4'd0;          // 上电一定是 0，全灭
    else if (key_press_edge)
        state_cnt <= state_cnt + 1'd1;
end

// ====================== 4 种步长 mu ======================
localparam [MU_WIDTH-1:0] MU0 = 32'sd66;    // 
localparam [MU_WIDTH-1:0] MU1 = 32'sd16;    // 
localparam [MU_WIDTH-1:0] MU2 = 32'sd393;   //
localparam [MU_WIDTH-1:0] MU3 = 32'sd655;   // 

// ====================== 4 种初始延迟 s_init_delay ======================
localparam [DELAY_WIDTH-1:0] DLY0 = 8'd0;
localparam [DELAY_WIDTH-1:0] DLY1 = 8'd32;
localparam [DELAY_WIDTH-1:0] DLY2 = 8'd56;
localparam [DELAY_WIDTH-1:0] DLY3 = 8'd80;

// ====================== 16 种组合 ======================
always @(*) begin
    case (state_cnt)
        4'd0:  begin mu_out = MU0; s_delay_out = DLY0; end
        4'd1:  begin mu_out = MU0; s_delay_out = DLY1; end
        4'd2:  begin mu_out = MU0; s_delay_out = DLY2; end
        4'd3:  begin mu_out = MU0; s_delay_out = DLY3; end
        
        4'd4:  begin mu_out = MU1; s_delay_out = DLY0; end
        4'd5:  begin mu_out = MU1; s_delay_out = DLY1; end
        4'd6:  begin mu_out = MU1; s_delay_out = DLY2; end
        4'd7:  begin mu_out = MU1; s_delay_out = DLY3; end
        
        4'd8:  begin mu_out = MU2; s_delay_out = DLY0; end
        4'd9:  begin mu_out = MU2; s_delay_out = DLY1; end
        4'd10: begin mu_out = MU2; s_delay_out = DLY2; end
        4'd11: begin mu_out = MU2; s_delay_out = DLY3; end
        
        4'd12: begin mu_out = MU3; s_delay_out = DLY0; end
        4'd13: begin mu_out = MU3; s_delay_out = DLY1; end
        4'd14: begin mu_out = MU3; s_delay_out = DLY2; end
        4'd15: begin mu_out = MU3; s_delay_out = DLY3; end
    endcase
end

// LED：左高右低（你要的顺序）
assign led_1st = state_cnt[3];  // LED1 = 最高位
assign led_2nd = state_cnt[2];
assign led_3rd = state_cnt[1];
assign led_4th = state_cnt[0];  // LED4 = 最低位

endmodule