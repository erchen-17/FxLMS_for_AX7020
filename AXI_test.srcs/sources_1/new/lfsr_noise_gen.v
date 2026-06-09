// =============================================================================
// File: lfsr_noise_gen.v
// Description: LFSR-based White Noise Generator for Secondary Path Estimation
//
// Algorithm: 32-bit Maximum Length LFSR (Galois form)
// Polynomial: x^32 + x^22 + x^2 + x + 1 (period = 2^32 - 1)
// Output Format: Q16.16 signed fixed-point
// Clock: Uses sample_clk (e.g., 48kHz LRCLK) directly as clock domain
//
// White noise provides uniform frequency content, which is essential for
// stable LMS convergence when estimating the secondary path S'(z).
// =============================================================================

`timescale 1ns/1ps

module lfsr_noise_gen #(
    parameter DATA_WIDTH = 32,          // Q16.16 format
    parameter LFSR_WIDTH = 32,          // LFSR register width
    parameter SEED = 32'hACE1_CAFE      // Non-zero seed (LFSR must not be all zeros)
)(
    input  wire                       sample_clk,    // Sample rate clock (e.g., 48kHz LRCLK)
    input  wire                       rst_n,
    input  wire                       enable,        // Enable generation
    input  wire signed [DATA_WIDTH-1:0] amplitude,   // Output amplitude (Q16.16)
    output reg  signed [DATA_WIDTH-1:0] noise_out    // White noise output (Q16.16)
);

    // =========================================================================
    // LFSR Register
    // =========================================================================
    reg [LFSR_WIDTH-1:0] lfsr_reg;

    // Feedback bit calculation (Galois LFSR)
    // Polynomial: x^32 + x^22 + x^2 + x + 1
    // Taps at positions: 32, 22, 2, 1 (1-indexed)
    // Feedback: bit[31] XOR bit[21] XOR bit[1] XOR bit[0]
    wire feedback;
    assign feedback = lfsr_reg[31] ^ lfsr_reg[21] ^ lfsr_reg[1] ^ lfsr_reg[0];

    // =========================================================================
    // LFSR State Machine
    // =========================================================================
    always @(posedge sample_clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr_reg <= SEED;
        end else if (!enable) begin
            lfsr_reg <= SEED;  // Reset to seed when disabled
        end else begin
            // Galois LFSR shift with feedback
            lfsr_reg <= {lfsr_reg[LFSR_WIDTH-2:0], feedback};
        end
    end

    // =========================================================================
    // Convert LFSR output to zero-mean Q16.16 format
    // =========================================================================
    // A max-length LFSR never produces all-zeros, so directly interpreting
    // the 32-bit register as signed gives mean ≈ -0.5 (DC bias).
    //
    // Fix: use bit[31] as sign control, bits[30:0] as magnitude.
    // When magnitude=0, output=0 regardless of sign → perfect symmetry.
    // Over the full LFSR period, +mag and -mag appear with equal frequency
    // (differ by at most 1 count out of 2^32-1), giving essentially zero DC.

    // Magnitude: bits[30:0], zero-extended to 32-bit signed (always >= 0)
    wire signed [LFSR_WIDTH-1:0] lfsr_mag;
    assign lfsr_mag = $signed({1'b0, lfsr_reg[LFSR_WIDTH-2:0]});

    // Multiplication: magnitude(Q0.31) * amplitude(Q16.16) = Q16.47
    // Right-shift by 31 to get Q16.16
    wire signed [63:0] mult_result;
    assign mult_result = lfsr_mag * amplitude;

    // Extract Q16.16 magnitude (always non-negative)
    wire signed [DATA_WIDTH-1:0] noise_mag;
    assign noise_mag = mult_result[62:31];

    always @(posedge sample_clk or negedge rst_n) begin
        if (!rst_n) begin
            noise_out <= 0;
        end else begin
            // Apply sign from LFSR MSB: +magnitude or -magnitude
            noise_out <= lfsr_reg[LFSR_WIDTH-1] ? noise_mag : -noise_mag;
        end
    end

endmodule
