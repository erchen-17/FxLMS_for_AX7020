`timescale 1ns/1ps
`include "fxlms_params.vh"
// =============================================================================
// File: primary_path_filter.v
// Description: Primary Path P(z) Filter for FxLMS Simulation
//
// This filter simulates the acoustic path from reference microphone to
// error microphone (the "primary path" in ANC terminology).
//
// Model: Exponential decay (matching fxlms_tb.v)
//   P(z) = sum of z^(-DELAY_P-i) * ALPHA_P^i for i = 0, 1, 2, ...
//   Impulse response: [0, 0, 0, 1, α, α², α³, ...]
//   This models multipath reflections in acoustic environments.
//
// In hardware, this would be replaced by the actual acoustic environment.
// This module is for simulation/testing purposes.
//
// Fixed-point format: Q16.16
// =============================================================================
module primary_path_filter #(
    parameter DATA_WIDTH   = `DATA_WIDTH,
    parameter FILTER_ORDER = 32,           // Smaller order for P(z), enough for simulation
    parameter FRAC_BITS    = `FRAC_BITS,
    parameter DELAY_P      = 3,            // Primary path delay (samples)
    parameter ALPHA_P_NUM  = 9,            // ALPHA_P numerator (0.9 = 9/10)
    parameter ALPHA_P_DEN  = 10            // ALPHA_P denominator
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         clear,        // Clear delay line (sync reset)
    input  wire                         sample_valid,
    input  wire signed [DATA_WIDTH-1:0] data_in,      // x(n) reference signal
    output reg  signed [DATA_WIDTH-1:0] data_out,     // d(n) = P(z) * x(n)
    output reg                          data_valid
);

    localparam ADDR_WIDTH = $clog2(FILTER_ORDER);
    localparam ACC_WIDTH  = 2*DATA_WIDTH + ADDR_WIDTH;

    // Delay line for input signal
    reg signed [DATA_WIDTH-1:0] delay_line [0:FILTER_ORDER-1];

    // P(z) coefficients - initialized to unit impulse (pure delay)
    // p_coeffs[DELAY_P] = 1.0, all others = 0
    reg signed [DATA_WIDTH-1:0] p_coeffs [0:FILTER_ORDER-1];

    // Control
    reg [ADDR_WIDTH-1:0] tap_counter;
    reg signed [ACC_WIDTH-1:0] accumulator;
    reg computing;

    // State machine
    reg [1:0] fir_state;
    localparam FIR_IDLE      = 2'd0;
    localparam FIR_COMPUTING = 2'd1;
    localparam FIR_LAST_ACC  = 2'd2;
    localparam FIR_OUTPUT    = 2'd3;

    integer i;

    // Multiplier
    wire signed [2*DATA_WIDTH-1:0] mult_result;
    assign mult_result = p_coeffs[tap_counter] * delay_line[tap_counter];

    // Saturation logic
    wire [ACC_WIDTH-DATA_WIDTH:0] overflow_bits;
    assign overflow_bits = accumulator[ACC_WIDTH-1:DATA_WIDTH-1];

    wire signed [DATA_WIDTH-1:0] saturated_output;
    assign saturated_output =
        (overflow_bits == {(ACC_WIDTH-DATA_WIDTH+1){accumulator[ACC_WIDTH-1]}}) ?
            accumulator[DATA_WIDTH-1:0] :
        accumulator[ACC_WIDTH-1] ?
            {1'b1, {(DATA_WIDTH-1){1'b0}}} :
            {1'b0, {(DATA_WIDTH-1){1'b1}}};

    // -------------------------------------------------------------------------
    // Coefficient initialization: Exponential decay model (matching fxlms_tb.v)
    // P(z) impulse response: [0, ..., 0, 1, α, α², α³, ...]
    //                         ^DELAY_P zeros
    // Pz[DELAY_P + i] = ALPHA_P^i for i = 0, 1, 2, ...
    // -------------------------------------------------------------------------
    // Pre-computed coefficients using integer arithmetic
    // ALPHA_P = ALPHA_P_NUM / ALPHA_P_DEN (e.g., 0.9 = 9/10)
    // Coefficients in Q16.16 format
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize coefficients: exponential decay starting at DELAY_P
            // p_coeffs[i] = 0 for i < DELAY_P
            // p_coeffs[DELAY_P + k] = ALPHA_P^k in Q16.16
            for (i = 0; i < FILTER_ORDER; i = i + 1) begin
                if (i < DELAY_P) begin
                    p_coeffs[i] <= 0;
                end else begin
                    // Compute ALPHA_P^(i-DELAY_P) using integer approximation
                    // For simplicity, pre-compute common values for α=0.9
                    case (i - DELAY_P)
                        0:  p_coeffs[i] <= (1 << FRAC_BITS);              // 1.0 = 65536
                        1:  p_coeffs[i] <= (ALPHA_P_NUM << FRAC_BITS) / ALPHA_P_DEN;  // 0.9 = 58982
                        2:  p_coeffs[i] <= (ALPHA_P_NUM * ALPHA_P_NUM << FRAC_BITS) / (ALPHA_P_DEN * ALPHA_P_DEN);  // 0.81
                        3:  p_coeffs[i] <= (ALPHA_P_NUM * ALPHA_P_NUM * ALPHA_P_NUM << FRAC_BITS) / (ALPHA_P_DEN * ALPHA_P_DEN * ALPHA_P_DEN);  // 0.729
                        4:  p_coeffs[i] <= 32'sd42998;  // 0.6561 * 65536
                        5:  p_coeffs[i] <= 32'sd38698;  // 0.59049 * 65536
                        6:  p_coeffs[i] <= 32'sd34828;  // 0.531441 * 65536
                        7:  p_coeffs[i] <= 32'sd31346;  // 0.4782969 * 65536
                        8:  p_coeffs[i] <= 32'sd28211;  // 0.43046721 * 65536
                        9:  p_coeffs[i] <= 32'sd25390;  // 0.387420489 * 65536
                        10: p_coeffs[i] <= 32'sd22851;  // 0.3486784401 * 65536
                        11: p_coeffs[i] <= 32'sd20566;  // 0.31381059609 * 65536
                        12: p_coeffs[i] <= 32'sd18509;  // 0.282429536481 * 65536
                        default: p_coeffs[i] <= 0;      // Truncate small values
                    endcase
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Delay line update
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < FILTER_ORDER; i = i + 1)
                delay_line[i] <= 0;
        end else if (clear) begin
            // Synchronous clear: reset delay line when FxLMS mode starts
            for (i = 0; i < FILTER_ORDER; i = i + 1)
                delay_line[i] <= 0;
        end else if (sample_valid && !computing) begin
            delay_line[0] <= data_in;
            for (i = 1; i < FILTER_ORDER; i = i + 1)
                delay_line[i] <= delay_line[i-1];
        end
    end

    // -------------------------------------------------------------------------
    // FIR state machine
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fir_state   <= FIR_IDLE;
            computing   <= 1'b0;
            tap_counter <= 0;
            accumulator <= 0;
            data_out    <= 0;
            data_valid  <= 1'b0;
        end else begin
            data_valid <= 1'b0;

            case (fir_state)
                FIR_IDLE: begin
                    if (sample_valid && !computing) begin
                        fir_state   <= FIR_COMPUTING;
                        computing   <= 1'b1;
                        tap_counter <= 0;
                        accumulator <= 0;
                    end
                end

                FIR_COMPUTING: begin
                    accumulator <= accumulator + (mult_result >>> FRAC_BITS);

                    if (tap_counter == FILTER_ORDER - 1) begin
                        fir_state <= FIR_LAST_ACC;
                    end else begin
                        tap_counter <= tap_counter + 1;
                    end
                end

                FIR_LAST_ACC: begin
                    fir_state   <= FIR_OUTPUT;
                    tap_counter <= 0;
                end

                FIR_OUTPUT: begin
                    computing  <= 1'b0;
                    data_out   <= saturated_output;
                    data_valid <= 1'b1;
                    fir_state  <= FIR_IDLE;
                end

                default: fir_state <= FIR_IDLE;
            endcase
        end
    end

endmodule
