`timescale 1ns / 1ps
`include "fxlms_params.vh"

// =============================================================================
// File: error_cosine_mu.v
// Description: Error-driven cosine learning-rate scheduler
//
// Implements:
//   mu(n) = beta * [cos(alpha * |e(n)| - pi) + 1]
//         = beta * [1 - cos(alpha * |e(n)|)]
//
// Notes:
//   - All math is in Q16.16
//   - theta = alpha*|e| is clamped to [0, pi] for LUT lookup
//   - cos(theta) uses the same 256-entry LUT style as cosine_annealing_mu
// =============================================================================
module error_cosine_mu #(
    parameter DATA_WIDTH      = `DATA_WIDTH,
    parameter FRAC_BITS       = `FRAC_BITS,
    parameter COS_TABLE_BITS  = `COS_TABLE_BITS
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         sample_tick,
    input  wire signed [DATA_WIDTH-1:0] error_in,
    input  wire signed [DATA_WIDTH-1:0] alpha,
    input  wire signed [DATA_WIDTH-1:0] beta,
    output reg  signed [DATA_WIDTH-1:0] mu_out
);

    localparam integer TABLE_SIZE = (1 << COS_TABLE_BITS);
    localparam signed [DATA_WIDTH-1:0] PI_Q16 = 32'sd205887; // pi * 65536

    function signed [DATA_WIDTH-1:0] cos_lut;
        input [COS_TABLE_BITS-1:0] idx;
        begin
            case (idx)
            8'd0: cos_lut = 32'sd65536;
            8'd1: cos_lut = 32'sd65531;
            8'd2: cos_lut = 32'sd65516;
            8'd3: cos_lut = 32'sd65491;
            8'd4: cos_lut = 32'sd65456;
            8'd5: cos_lut = 32'sd65412;
            8'd6: cos_lut = 32'sd65357;
            8'd7: cos_lut = 32'sd65292;
            8'd8: cos_lut = 32'sd65218;
            8'd9: cos_lut = 32'sd65134;
            8'd10: cos_lut = 32'sd65039;
            8'd11: cos_lut = 32'sd64935;
            8'd12: cos_lut = 32'sd64821;
            8'd13: cos_lut = 32'sd64697;
            8'd14: cos_lut = 32'sd64564;
            8'd15: cos_lut = 32'sd64420;
            8'd16: cos_lut = 32'sd64267;
            8'd17: cos_lut = 32'sd64104;
            8'd18: cos_lut = 32'sd63931;
            8'd19: cos_lut = 32'sd63749;
            8'd20: cos_lut = 32'sd63557;
            8'd21: cos_lut = 32'sd63355;
            8'd22: cos_lut = 32'sd63143;
            8'd23: cos_lut = 32'sd62923;
            8'd24: cos_lut = 32'sd62692;
            8'd25: cos_lut = 32'sd62452;
            8'd26: cos_lut = 32'sd62203;
            8'd27: cos_lut = 32'sd61944;
            8'd28: cos_lut = 32'sd61675;
            8'd29: cos_lut = 32'sd61398;
            8'd30: cos_lut = 32'sd61111;
            8'd31: cos_lut = 32'sd60814;
            8'd32: cos_lut = 32'sd60509;
            8'd33: cos_lut = 32'sd60194;
            8'd34: cos_lut = 32'sd59870;
            8'd35: cos_lut = 32'sd59537;
            8'd36: cos_lut = 32'sd59195;
            8'd37: cos_lut = 32'sd58844;
            8'd38: cos_lut = 32'sd58484;
            8'd39: cos_lut = 32'sd58116;
            8'd40: cos_lut = 32'sd57738;
            8'd41: cos_lut = 32'sd57352;
            8'd42: cos_lut = 32'sd56957;
            8'd43: cos_lut = 32'sd56553;
            8'd44: cos_lut = 32'sd56141;
            8'd45: cos_lut = 32'sd55720;
            8'd46: cos_lut = 32'sd55291;
            8'd47: cos_lut = 32'sd54853;
            8'd48: cos_lut = 32'sd54407;
            8'd49: cos_lut = 32'sd53953;
            8'd50: cos_lut = 32'sd53490;
            8'd51: cos_lut = 32'sd53020;
            8'd52: cos_lut = 32'sd52541;
            8'd53: cos_lut = 32'sd52055;
            8'd54: cos_lut = 32'sd51560;
            8'd55: cos_lut = 32'sd51058;
            8'd56: cos_lut = 32'sd50548;
            8'd57: cos_lut = 32'sd50030;
            8'd58: cos_lut = 32'sd49505;
            8'd59: cos_lut = 32'sd48972;
            8'd60: cos_lut = 32'sd48432;
            8'd61: cos_lut = 32'sd47884;
            8'd62: cos_lut = 32'sd47329;
            8'd63: cos_lut = 32'sd46767;
            8'd64: cos_lut = 32'sd46198;
            8'd65: cos_lut = 32'sd45622;
            8'd66: cos_lut = 32'sd45039;
            8'd67: cos_lut = 32'sd44449;
            8'd68: cos_lut = 32'sd43852;
            8'd69: cos_lut = 32'sd43249;
            8'd70: cos_lut = 32'sd42639;
            8'd71: cos_lut = 32'sd42023;
            8'd72: cos_lut = 32'sd41400;
            8'd73: cos_lut = 32'sd40771;
            8'd74: cos_lut = 32'sd40136;
            8'd75: cos_lut = 32'sd39494;
            8'd76: cos_lut = 32'sd38847;
            8'd77: cos_lut = 32'sd38194;
            8'd78: cos_lut = 32'sd37535;
            8'd79: cos_lut = 32'sd36870;
            8'd80: cos_lut = 32'sd36200;
            8'd81: cos_lut = 32'sd35524;
            8'd82: cos_lut = 32'sd34843;
            8'd83: cos_lut = 32'sd34156;
            8'd84: cos_lut = 32'sd33465;
            8'd85: cos_lut = 32'sd32768;
            8'd86: cos_lut = 32'sd32066;
            8'd87: cos_lut = 32'sd31360;
            8'd88: cos_lut = 32'sd30648;
            8'd89: cos_lut = 32'sd29932;
            8'd90: cos_lut = 32'sd29212;
            8'd91: cos_lut = 32'sd28487;
            8'd92: cos_lut = 32'sd27758;
            8'd93: cos_lut = 32'sd27024;
            8'd94: cos_lut = 32'sd26287;
            8'd95: cos_lut = 32'sd25545;
            8'd96: cos_lut = 32'sd24800;
            8'd97: cos_lut = 32'sd24050;
            8'd98: cos_lut = 32'sd23297;
            8'd99: cos_lut = 32'sd22541;
            8'd100: cos_lut = 32'sd21781;
            8'd101: cos_lut = 32'sd21018;
            8'd102: cos_lut = 32'sd20252;
            8'd103: cos_lut = 32'sd19482;
            8'd104: cos_lut = 32'sd18710;
            8'd105: cos_lut = 32'sd17935;
            8'd106: cos_lut = 32'sd17157;
            8'd107: cos_lut = 32'sd16376;
            8'd108: cos_lut = 32'sd15593;
            8'd109: cos_lut = 32'sd14808;
            8'd110: cos_lut = 32'sd14020;
            8'd111: cos_lut = 32'sd13231;
            8'd112: cos_lut = 32'sd12439;
            8'd113: cos_lut = 32'sd11645;
            8'd114: cos_lut = 32'sd10850;
            8'd115: cos_lut = 32'sd10053;
            8'd116: cos_lut = 32'sd9254;
            8'd117: cos_lut = 32'sd8454;
            8'd118: cos_lut = 32'sd7653;
            8'd119: cos_lut = 32'sd6850;
            8'd120: cos_lut = 32'sd6047;
            8'd121: cos_lut = 32'sd5243;
            8'd122: cos_lut = 32'sd4437;
            8'd123: cos_lut = 32'sd3631;
            8'd124: cos_lut = 32'sd2825;
            8'd125: cos_lut = 32'sd2018;
            8'd126: cos_lut = 32'sd1211;
            8'd127: cos_lut = 32'sd404;
            8'd128: cos_lut = -32'sd404;
            8'd129: cos_lut = -32'sd1211;
            8'd130: cos_lut = -32'sd2018;
            8'd131: cos_lut = -32'sd2825;
            8'd132: cos_lut = -32'sd3631;
            8'd133: cos_lut = -32'sd4437;
            8'd134: cos_lut = -32'sd5243;
            8'd135: cos_lut = -32'sd6047;
            8'd136: cos_lut = -32'sd6850;
            8'd137: cos_lut = -32'sd7653;
            8'd138: cos_lut = -32'sd8454;
            8'd139: cos_lut = -32'sd9254;
            8'd140: cos_lut = -32'sd10053;
            8'd141: cos_lut = -32'sd10850;
            8'd142: cos_lut = -32'sd11645;
            8'd143: cos_lut = -32'sd12439;
            8'd144: cos_lut = -32'sd13231;
            8'd145: cos_lut = -32'sd14020;
            8'd146: cos_lut = -32'sd14808;
            8'd147: cos_lut = -32'sd15593;
            8'd148: cos_lut = -32'sd16376;
            8'd149: cos_lut = -32'sd17157;
            8'd150: cos_lut = -32'sd17935;
            8'd151: cos_lut = -32'sd18710;
            8'd152: cos_lut = -32'sd19482;
            8'd153: cos_lut = -32'sd20252;
            8'd154: cos_lut = -32'sd21018;
            8'd155: cos_lut = -32'sd21781;
            8'd156: cos_lut = -32'sd22541;
            8'd157: cos_lut = -32'sd23297;
            8'd158: cos_lut = -32'sd24050;
            8'd159: cos_lut = -32'sd24800;
            8'd160: cos_lut = -32'sd25545;
            8'd161: cos_lut = -32'sd26287;
            8'd162: cos_lut = -32'sd27024;
            8'd163: cos_lut = -32'sd27758;
            8'd164: cos_lut = -32'sd28487;
            8'd165: cos_lut = -32'sd29212;
            8'd166: cos_lut = -32'sd29932;
            8'd167: cos_lut = -32'sd30648;
            8'd168: cos_lut = -32'sd31360;
            8'd169: cos_lut = -32'sd32066;
            8'd170: cos_lut = -32'sd32768;
            8'd171: cos_lut = -32'sd33465;
            8'd172: cos_lut = -32'sd34156;
            8'd173: cos_lut = -32'sd34843;
            8'd174: cos_lut = -32'sd35524;
            8'd175: cos_lut = -32'sd36200;
            8'd176: cos_lut = -32'sd36870;
            8'd177: cos_lut = -32'sd37535;
            8'd178: cos_lut = -32'sd38194;
            8'd179: cos_lut = -32'sd38847;
            8'd180: cos_lut = -32'sd39494;
            8'd181: cos_lut = -32'sd40136;
            8'd182: cos_lut = -32'sd40771;
            8'd183: cos_lut = -32'sd41400;
            8'd184: cos_lut = -32'sd42023;
            8'd185: cos_lut = -32'sd42639;
            8'd186: cos_lut = -32'sd43249;
            8'd187: cos_lut = -32'sd43852;
            8'd188: cos_lut = -32'sd44449;
            8'd189: cos_lut = -32'sd45039;
            8'd190: cos_lut = -32'sd45622;
            8'd191: cos_lut = -32'sd46198;
            8'd192: cos_lut = -32'sd46767;
            8'd193: cos_lut = -32'sd47329;
            8'd194: cos_lut = -32'sd47884;
            8'd195: cos_lut = -32'sd48432;
            8'd196: cos_lut = -32'sd48972;
            8'd197: cos_lut = -32'sd49505;
            8'd198: cos_lut = -32'sd50030;
            8'd199: cos_lut = -32'sd50548;
            8'd200: cos_lut = -32'sd51058;
            8'd201: cos_lut = -32'sd51560;
            8'd202: cos_lut = -32'sd52055;
            8'd203: cos_lut = -32'sd52541;
            8'd204: cos_lut = -32'sd53020;
            8'd205: cos_lut = -32'sd53490;
            8'd206: cos_lut = -32'sd53953;
            8'd207: cos_lut = -32'sd54407;
            8'd208: cos_lut = -32'sd54853;
            8'd209: cos_lut = -32'sd55291;
            8'd210: cos_lut = -32'sd55720;
            8'd211: cos_lut = -32'sd56141;
            8'd212: cos_lut = -32'sd56553;
            8'd213: cos_lut = -32'sd56957;
            8'd214: cos_lut = -32'sd57352;
            8'd215: cos_lut = -32'sd57738;
            8'd216: cos_lut = -32'sd58116;
            8'd217: cos_lut = -32'sd58484;
            8'd218: cos_lut = -32'sd58844;
            8'd219: cos_lut = -32'sd59195;
            8'd220: cos_lut = -32'sd59537;
            8'd221: cos_lut = -32'sd59870;
            8'd222: cos_lut = -32'sd60194;
            8'd223: cos_lut = -32'sd60509;
            8'd224: cos_lut = -32'sd60814;
            8'd225: cos_lut = -32'sd61111;
            8'd226: cos_lut = -32'sd61398;
            8'd227: cos_lut = -32'sd61675;
            8'd228: cos_lut = -32'sd61944;
            8'd229: cos_lut = -32'sd62203;
            8'd230: cos_lut = -32'sd62452;
            8'd231: cos_lut = -32'sd62692;
            8'd232: cos_lut = -32'sd62923;
            8'd233: cos_lut = -32'sd63143;
            8'd234: cos_lut = -32'sd63355;
            8'd235: cos_lut = -32'sd63557;
            8'd236: cos_lut = -32'sd63749;
            8'd237: cos_lut = -32'sd63931;
            8'd238: cos_lut = -32'sd64104;
            8'd239: cos_lut = -32'sd64267;
            8'd240: cos_lut = -32'sd64420;
            8'd241: cos_lut = -32'sd64564;
            8'd242: cos_lut = -32'sd64697;
            8'd243: cos_lut = -32'sd64821;
            8'd244: cos_lut = -32'sd64935;
            8'd245: cos_lut = -32'sd65039;
            8'd246: cos_lut = -32'sd65134;
            8'd247: cos_lut = -32'sd65218;
            8'd248: cos_lut = -32'sd65292;
            8'd249: cos_lut = -32'sd65357;
            8'd250: cos_lut = -32'sd65412;
            8'd251: cos_lut = -32'sd65456;
            8'd252: cos_lut = -32'sd65491;
            8'd253: cos_lut = -32'sd65516;
            8'd254: cos_lut = -32'sd65531;
            8'd255: cos_lut = -32'sd65536;
            default: cos_lut = -32'sd65536;
            endcase
        end
    endfunction

    wire signed [DATA_WIDTH-1:0] abs_e_w =
        error_in[DATA_WIDTH-1] ? (~error_in + 32'sd1) : error_in;
    wire signed [63:0] alpha_mul_abs_e_w = $signed(alpha) * $signed(abs_e_w);
    wire signed [DATA_WIDTH-1:0] theta_q16_w = alpha_mul_abs_e_w[FRAC_BITS +: DATA_WIDTH];
    wire [COS_TABLE_BITS-1:0] table_idx_w =
        (theta_q16_w >= PI_Q16) ? (TABLE_SIZE - 1) :
        ((theta_q16_w * (TABLE_SIZE - 1)) / PI_Q16);
    wire signed [DATA_WIDTH-1:0] cos_theta_w = cos_lut(table_idx_w);
    wire signed [DATA_WIDTH:0] one_minus_cos_w = 33'sd65536 - cos_theta_w;
    wire signed [63:0] beta_mul_term_w = $signed(beta) * $signed(one_minus_cos_w);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mu_out <= 32'sd0;
        end else if (sample_tick) begin
            mu_out <= beta_mul_term_w[FRAC_BITS +: DATA_WIDTH];
        end
    end

endmodule
