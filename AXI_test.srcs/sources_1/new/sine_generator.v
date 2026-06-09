// =============================================================================
// File: sine_generator.v
// Description: Sine Wave Generator using Quarter-Wave LUT
//              Used for secondary path estimation in FxLMS system
//
// Algorithm: Phase accumulator + quarter-wave sine LUT
// Output Format: Q16.16 signed fixed-point
// Clock: Uses sample_clk (e.g., 48kHz LRCLK) directly as clock domain
// =============================================================================

`timescale 1ns/1ps

module sine_generator #(
    parameter DATA_WIDTH = 32,          // Q16.16 format
    parameter PHASE_WIDTH = 32,         // Phase accumulator width
    parameter FREQ_DIVIDER = 1          // Frequency divider (1 = no division, 2 = half freq, etc.)
)(
    input  wire                       sample_clk,    // Sample rate clock (e.g., 48kHz LRCLK)
    input  wire                       rst_n,
    input  wire                       enable,        // Enable generation
    input  wire [PHASE_WIDTH-1:0]     freq_word,     // Frequency control word
    input  wire signed [DATA_WIDTH-1:0] amplitude,   // Output amplitude (Q16.16)
    output reg  signed [DATA_WIDTH-1:0] sine_out     // Sine wave output (Q16.16)
);

    // =========================================================================
    // Frequency Divider (to reduce output frequency)
    // =========================================================================
    reg [$clog2(FREQ_DIVIDER > 0 ? FREQ_DIVIDER : 1)-1:0] div_counter;
    reg phase_update;

    always @(posedge sample_clk or negedge rst_n) begin
        if (!rst_n) begin
            div_counter <= 0;
            phase_update <= 1'b0;
        end else begin
            if (FREQ_DIVIDER <= 1) begin
                phase_update <= 1'b1;
            end else if (div_counter >= FREQ_DIVIDER - 1) begin
                div_counter <= 0;
                phase_update <= 1'b1;
            end else begin
                div_counter <= div_counter + 1;
                phase_update <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Phase Accumulator
    // =========================================================================
    reg [PHASE_WIDTH-1:0] phase_acc;

    always @(posedge sample_clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_acc <= 0;
        end else if (!enable) begin
            phase_acc <= 0;  // Reset phase when disabled
        end else if (phase_update) begin
            phase_acc <= phase_acc + freq_word;
        end
    end

    // =========================================================================
    // Quarter-wave lookup table (0 to π/2)
    // Using 256 entries for good precision
    // Values are sin(i * π/2 / 256) scaled to Q1.15 format (range -1 to +1)
    // =========================================================================
    localparam LUT_DEPTH = 256;
    localparam LUT_WIDTH = 16;  // Q1.15 format

    reg signed [LUT_WIDTH-1:0] sine_lut [0:LUT_DEPTH-1];

    // Initialize sine lookup table
    integer i;
    initial begin
        for (i = 0; i < LUT_DEPTH; i = i + 1) begin
            sine_lut[i] = $rtoi($sin(i * 3.14159265358979 / 2.0 / LUT_DEPTH) * 32767.0);
        end
    end

    // =========================================================================
    // Phase to sine conversion using symmetry
    // =========================================================================
    wire [1:0] quadrant;
    wire [7:0] lut_addr;
    wire       negate;

    assign quadrant = phase_acc[PHASE_WIDTH-1:PHASE_WIDTH-2];
    assign lut_addr = phase_acc[PHASE_WIDTH-3:PHASE_WIDTH-10];

    // Determine sign based on quadrant
    // Quadrant: 00 (0-π/2), 01 (π/2-π), 10 (π-3π/2), 11 (3π/2-2π)
    assign negate = (quadrant == 2'b10) || (quadrant == 2'b11);

    // Lookup table address (mirror for quadrants 01 and 11)
    wire [7:0] effective_addr;
    assign effective_addr = (quadrant == 2'b01 || quadrant == 2'b11) ?
                            (8'd255 - lut_addr) : lut_addr;

    // =========================================================================
    // Multiply by amplitude and convert to Q16.16
    // =========================================================================
    wire signed [LUT_WIDTH-1:0] sine_q15;
    assign sine_q15 = negate ? -sine_lut[effective_addr] : sine_lut[effective_addr];

    // Convert Q1.15 to Q16.16 and multiply by amplitude
    // sine_q15 is Q1.15 (-32768 to 32767 represents -1.0 to ~1.0)
    // amplitude is Q16.16
    // Result: (Q1.15 * Q16.16) >> 15 = Q16.16
    wire signed [47:0] mult_result;
    assign mult_result = sine_q15 * amplitude;

    // Output update on every sample_clk edge
    always @(posedge sample_clk or negedge rst_n) begin
        if (!rst_n) begin
            sine_out <= 0;
        end else begin
            // Right shift by 15 to convert from Q17.31 to Q16.16
            sine_out <= mult_result[46:15];  // Take bits [46:15] for Q16.16
        end
    end

endmodule
