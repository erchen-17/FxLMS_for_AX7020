`timescale 1ns/1ps
`include "fxlms_params.vh"
// =============================================================================
// File: true_secondary_path_filter.v
// Description: True Secondary Path S(z) Filter for FxLMS Simulation
//
// This filter simulates the ACTUAL acoustic path from speaker to error microphone
// (the "secondary path" in ANC terminology).
//
// Model: Exponential decay (matching testbench Sz_true)
//   S(z) = GAIN_S * sum of z^(-DELAY_S-i) * ALPHA_S^i for i = 0, 1, 2, ...
//   Impulse response: [0, G, G*alpha, G*alpha^2, G*alpha^3, ...]
//
// This is for SIMULATION purposes - models the physical speaker-to-mic path.
// In real hardware, this path is the actual acoustic environment.
//
// NOTE: This is different from secondary_path_filter.v which is S'(z) - the
//       estimated/adaptive filter used in FxLMS algorithm.
//
// Fixed-point format: Q16.16
// =============================================================================
module true_secondary_path_filter #(
    parameter DATA_WIDTH   = `DATA_WIDTH,
    parameter FILTER_ORDER = 32,           // Smaller order sufficient for S(z)
    parameter FRAC_BITS    = `FRAC_BITS,
    parameter DELAY_S      = 1,            // Secondary path delay (samples)
    parameter ALPHA_S_NUM  = 9,            // ALPHA_S numerator (0.9 = 9/10)
    parameter ALPHA_S_DEN  = 10,           // ALPHA_S denominator
    parameter GAIN_S_NUM   = 10,           // GAIN_S numerator (1.0 = 10/10)
    parameter GAIN_S_DEN   = 10            // GAIN_S denominator
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         clear,        // Clear delay line (sync reset)
    input  wire                         sample_valid,
    input  wire signed [DATA_WIDTH-1:0] data_in,      // y(n) control output
    output reg  signed [DATA_WIDTH-1:0] data_out,     // y'(n) = S(z) * y(n)
    output reg                          data_valid
);

    localparam ADDR_WIDTH = $clog2(FILTER_ORDER);
    localparam ACC_WIDTH  = 2*DATA_WIDTH + ADDR_WIDTH;

    // Delay line for input signal
    reg signed [DATA_WIDTH-1:0] delay_line [0:FILTER_ORDER-1];

    // S(z) coefficients - initialized to exponential decay
    reg signed [DATA_WIDTH-1:0] s_coeffs [0:FILTER_ORDER-1];

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
    assign mult_result = s_coeffs[tap_counter] * delay_line[tap_counter];

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
    // Coefficient initialization: Exponential decay model
    // S(z) impulse response: [0, G, G*alpha, G*alpha^2, ...]
    //                         ^DELAY_S zeros
    // s_coeffs[DELAY_S + i] = GAIN_S * ALPHA_S^i for i = 0, 1, 2, ...
    //
    // For GAIN_S=1.0, ALPHA_S=0.9, DELAY_S=1:
    //   s_coeffs[0] = 0
    //   s_coeffs[1] = 1.0 = 65536
    //   s_coeffs[2] = 0.9 = 58982
    //   s_coeffs[3] = 0.81 = 53084
    //   ...
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize coefficients: exponential decay starting at DELAY_S
            for (i = 0; i < FILTER_ORDER; i = i + 1) begin
                if (i < DELAY_S) begin
                    s_coeffs[i] <= 0;
                end else begin
                    // Compute GAIN_S * ALPHA_S^(i-DELAY_S) using integer approximation
                    // For GAIN_S=1.0, ALPHA_S=0.9
                    case (i - DELAY_S)
                        0:  s_coeffs[i] <= (GAIN_S_NUM << FRAC_BITS) / GAIN_S_DEN;  // G = 1.0 = 65536
                        1:  s_coeffs[i] <= (GAIN_S_NUM * ALPHA_S_NUM << FRAC_BITS) / (GAIN_S_DEN * ALPHA_S_DEN);  // G*0.9 = 58982
                        2:  s_coeffs[i] <= (GAIN_S_NUM * ALPHA_S_NUM * ALPHA_S_NUM << FRAC_BITS) / (GAIN_S_DEN * ALPHA_S_DEN * ALPHA_S_DEN);  // G*0.81
                        3:  s_coeffs[i] <= 32'sd47710;  // G * 0.729 * 65536
                        4:  s_coeffs[i] <= 32'sd42998;  // G * 0.6561 * 65536
                        5:  s_coeffs[i] <= 32'sd38698;  // G * 0.59049 * 65536
                        6:  s_coeffs[i] <= 32'sd34828;  // G * 0.531441 * 65536
                        7:  s_coeffs[i] <= 32'sd31346;  // G * 0.4782969 * 65536
                        8:  s_coeffs[i] <= 32'sd28211;  // G * 0.43046721 * 65536
                        9:  s_coeffs[i] <= 32'sd25390;  // G * 0.387420489 * 65536
                        10: s_coeffs[i] <= 32'sd22851;  // G * 0.3486784401 * 65536
                        11: s_coeffs[i] <= 32'sd20566;  // G * 0.31381059609 * 65536
                        12: s_coeffs[i] <= 32'sd18509;  // G * 0.282429536481 * 65536
                        default: s_coeffs[i] <= 0;      // Truncate small values
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
