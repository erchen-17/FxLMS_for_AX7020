`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2025/12/22
// Design Name:
// Module Name: signal_subtractor_q16
// Project Name: FxLMS
// Target Devices:
// Tool Versions:
// Description: 32-bit Q16.16 format signal subtractor
//              Used to construct error signal in FxLMS test mode
//              error_signal = ref_signal - control_output (matching TB logic)
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - Changed from adder to subtractor to match TB logic
// Additional Comments:
//   - Maintain Q16.16 high-precision format, avoid precision loss from intermediate conversion
//   - Include saturation protection to prevent overflow
//   - Helps FxLMS algorithm converge faster
//   - Formula: e(n) = d(n) - y'(n), where d(n) is primary noise (ref_signal)
//              and y'(n) is secondary path output (control_output_delayed)
//
//////////////////////////////////////////////////////////////////////////////////

module signal_adder_q16 (
    input  wire signed [31:0] signal_a,    // Q16.16 input signal A (minuend)
    input  wire signed [31:0] signal_b,    // Q16.16 input signal B (subtrahend)
    output wire signed [31:0] sum_out      // Q16.16 output: A - B (with saturation)
);

    // =========================================================
    // 33-bit subtraction (extend sign bit to prevent overflow detection errors)
    // =========================================================
    wire signed [32:0] diff_extended;
    assign diff_extended = {signal_a[31], signal_a} - {signal_b[31], signal_b};

    // =========================================================
    // Saturation logic detection
    // =========================================================
    // Positive overflow: sign bit extended to 0, but result sign bit is 1
    wire overflow_pos = (diff_extended[32] == 1'b0) && (diff_extended[31] == 1'b1);

    // Negative overflow: sign bit extended to 1, but result sign bit is 0
    wire overflow_neg = (diff_extended[32] == 1'b1) && (diff_extended[31] == 1'b0);

    // =========================================================
    // Output saturation handling
    // =========================================================
    assign sum_out = overflow_pos ? 32'h7FFFFFFF :  // Saturate to max positive value (+32767.99998 in Q16.16)
                     overflow_neg ? 32'h80000000 :  // Saturate to max negative value (-32768.0 in Q16.16)
                     diff_extended[31:0];           // Normal range

endmodule
