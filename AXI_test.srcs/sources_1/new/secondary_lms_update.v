`timescale 1ns/1ps
`include "fxlms_params.vh"
// =============================================================================
// File: secondary_lms_update.v
// Description: LMS Update Module for Secondary Path Estimation
// Algorithm: S'(n+1) = S'(n) + mu_s * e_s(n) * w(n-i)
// Fixed-point format: Q16.16 for 32-bit data
//
// Structure matches lms_update.v for consistency
// =============================================================================
module secondary_lms_update #(
    parameter DATA_WIDTH   = `DATA_WIDTH,
    parameter FILTER_ORDER = `FILTER_ORDER_S,
    parameter MU_WIDTH     = `MU_WIDTH,
    parameter FRAC_BITS    = `FRAC_BITS
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,
    input  wire signed [MU_WIDTH-1:0]   mu,                   // Step size (Q16.16, runtime configurable)
    input  wire signed [DATA_WIDTH-1:0] estimation_error,     // e_s(n) = mic - S'*w(n)
    input  wire [7:0]                   s_init_delay,         // S' filter initial delay
    output reg                          done,

    // Coefficient output interface (to S' filter)
    output reg                              coeff_we,
    output reg  [$clog2(FILTER_ORDER)-1:0]  coeff_addr,
    output reg  signed [DATA_WIDTH-1:0]     coeff_dout,

    // External delay line interface (shared with secondary_path_filter)
    output reg  [$clog2(FILTER_ORDER)-1:0]  delay_read_addr_ext,
    input  wire signed [DATA_WIDTH-1:0]     delay_read_data_ext
);

    localparam ADDR_WIDTH = $clog2(FILTER_ORDER);

    // -------------------------------------------------------------------------
    // Width planning (FULL precision)
    // -------------------------------------------------------------------------
    localparam MU_MUL_WIDTH   = MU_WIDTH + DATA_WIDTH;
    localparam DELTA_FULL_WIDTH = MU_MUL_WIDTH + DATA_WIDTH;

    // =========================================================================
    // Memory for S'(z) coefficients
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] s_coeffs [0:FILTER_ORDER-1];

    // =========================================================================
    // Function to compute exponential decay coefficient (same as secondary_path_filter)
    // Returns GAIN * ALPHA^n in Q16.16 format
    // =========================================================================
    function signed [DATA_WIDTH-1:0] exp_decay_coeff;
        input integer n;
        reg signed [63:0] result;
        reg signed [31:0] alpha_power;
        integer k;
        begin
            // Start with alpha^0 = 1.0 in Q0.16
            alpha_power = 32'sd65536;  // 1.0 in Q0.16

            // Compute alpha^n
            for (k = 0; k < n; k = k + 1) begin
                // alpha_power = alpha_power * ALPHA >> 16
                alpha_power = (alpha_power * `S_INIT_ALPHA) >>> 16;
            end

            // result = GAIN * alpha^n >> 16 (GAIN is Q0.16, result is Q16.16)
            result = (`S_INIT_GAIN * alpha_power) >>> 16;
            exp_decay_coeff = result[DATA_WIDTH-1:0];
        end
    endfunction

    // Control
    reg [ADDR_WIDTH-1:0] tap_counter;
    reg computing;

    // State machine
    reg [2:0] state;
    localparam IDLE       = 3'd0;
    localparam LATCH_MU   = 3'd1;  // Latch mu*e
    localparam REQ_DELAY  = 3'd2;  // Issue delay-line read address
    localparam WAIT_DELAY = 3'd3;  // Wait external read latency
    localparam CAPTURE    = 3'd4;  // Capture delay + coeff for this tap
    localparam UPDATE     = 3'd5;  // Compute and write one tap
    localparam DONE_STATE = 3'd6;

    // Intermediate
    reg signed [MU_MUL_WIDTH-1:0] mu_error;

    // =========================================================================
    // Working registers for one-tap update
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] coeff_read_reg;
    reg signed [DATA_WIDTH-1:0] delay_read_reg;

    // -------------------------------------------------------------------------
    // Stage 1: mu_error = (mu * e_s) >>> FRAC_BITS
    // -------------------------------------------------------------------------
    wire signed [MU_MUL_WIDTH-1:0] mu_error_product_full;
    assign mu_error_product_full = mu * $signed(estimation_error);

    wire signed [MU_MUL_WIDTH-1:0] mu_error_scaled;
    assign mu_error_scaled = mu_error_product_full >>> FRAC_BITS;

    // -------------------------------------------------------------------------
    // Stage 2: delta_s_full = mu_error * white_noise_delay
    // -------------------------------------------------------------------------
    wire signed [DELTA_FULL_WIDTH-1:0] delta_s_full;
    assign delta_s_full = $signed(mu_error) * $signed(delay_read_reg);

    wire signed [DELTA_FULL_WIDTH-1:0] delta_s_shifted;
    assign delta_s_shifted = delta_s_full >>> FRAC_BITS;

    // -------------------------------------------------------------------------
    // Saturate delta_s_shifted to DATA_WIDTH
    // -------------------------------------------------------------------------
    wire [DELTA_FULL_WIDTH-DATA_WIDTH:0] delta_sign_ext_bits;
    assign delta_sign_ext_bits = delta_s_shifted[DELTA_FULL_WIDTH-1:DATA_WIDTH-1];

    wire delta_no_overflow;
    assign delta_no_overflow =
        (delta_sign_ext_bits == {(DELTA_FULL_WIDTH-DATA_WIDTH+1){delta_s_shifted[DATA_WIDTH-1]}});

    wire signed [DATA_WIDTH-1:0] delta_s_scaled;
    assign delta_s_scaled = delta_no_overflow ?
                            delta_s_shifted[DATA_WIDTH-1:0] :
                            (delta_s_shifted[DELTA_FULL_WIDTH-1] ?
                                {1'b1, {(DATA_WIDTH-1){1'b0}}} :
                                {1'b0, {(DATA_WIDTH-1){1'b1}}});

    // -------------------------------------------------------------------------
    // Coefficient update with saturation
    // EST_GRAD_DIRECTION: 0 = standard (+delta), 1 = reversed (-delta)
    // -------------------------------------------------------------------------
    wire signed [DATA_WIDTH:0] coeff_sum;
    assign coeff_sum = `EST_GRAD_DIRECTION ?
                       ($signed(coeff_read_reg) - $signed(delta_s_scaled)) :
                       ($signed(coeff_read_reg) + $signed(delta_s_scaled));

    wire signed [DATA_WIDTH-1:0] coeff_saturated;
    assign coeff_saturated =
        (coeff_sum[DATA_WIDTH] != coeff_sum[DATA_WIDTH-1]) ?
            (coeff_sum[DATA_WIDTH] ?
                {1'b1, {(DATA_WIDTH-1){1'b0}}} :
                {1'b0, {(DATA_WIDTH-1){1'b1}}}) :
            coeff_sum[DATA_WIDTH-1:0];

    // =========================================================================
    // Main LMS update state machine (tap-aligned, deterministic)
    // =========================================================================
    integer j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= IDLE;
            computing        <= 1'b0;
            tap_counter      <= 0;
            done             <= 1'b0;
            coeff_we         <= 1'b0;
            coeff_addr       <= 0;
            coeff_dout       <= 0;
            mu_error         <= 0;
            delay_read_addr_ext <= 0;
            coeff_read_reg   <= 0;
            delay_read_reg   <= 0;

            // Initialize S'(z) coefficients based on S_INIT_MODE
            for (j = 0; j < FILTER_ORDER; j = j + 1) begin
                if (`S_INIT_MODE == 0) begin
                    // Mode 0: All zeros
                    s_coeffs[j] <= 0;
                end else if (`S_INIT_MODE == 1) begin
                    // Mode 1: Exponential decay starting from S_INIT_DELAY
                    if (j < s_init_delay) begin
                        s_coeffs[j] <= 0;
                    end else begin
                        s_coeffs[j] <= exp_decay_coeff(j - s_init_delay);
                    end
                end else begin
                    // Mode 2: Unit impulse at index s_init_delay
                    if (j == s_init_delay) begin
                        s_coeffs[j] <= (1 << FRAC_BITS);  // 1.0 in Q16.16
                    end else begin
                        s_coeffs[j] <= 0;
                    end
                end
            end

        end else begin
            done     <= 1'b0;
            coeff_we <= 1'b0;

            case (state)
                IDLE: begin
                    if (start && !computing) begin
                        state       <= LATCH_MU;
                        computing   <= 1'b1;
                        tap_counter <= 0;
                        mu_error    <= mu_error_scaled;
                    end
                end

                LATCH_MU: begin
                    // Start from tap 0
                    tap_counter <= 0;
                    state <= REQ_DELAY;
                end

                REQ_DELAY: begin
                    // Request delay_line[tap_counter] from secondary_path_filter
                    delay_read_addr_ext <= tap_counter;
                    state <= WAIT_DELAY;
                end

                WAIT_DELAY: begin
                    // secondary_path_filter returns delay data with registered latency.
                    // Wait one extra cycle to guarantee aligned/stable capture.
                    state <= CAPTURE;
                end

                CAPTURE: begin
                    // Capture both operands for this exact tap.
                    coeff_read_reg <= s_coeffs[tap_counter];
                    delay_read_reg <= delay_read_data_ext;
                    state <= UPDATE;
                end

                UPDATE: begin
                    // Update one tap, then advance.
                    s_coeffs[tap_counter] <= coeff_saturated;
                    coeff_we   <= 1'b1;
                    coeff_addr <= tap_counter;
                    coeff_dout <= coeff_saturated;

                    if (tap_counter == FILTER_ORDER - 1) begin
                        state <= DONE_STATE;
                    end else begin
                        tap_counter <= tap_counter + 1'b1;
                        state <= REQ_DELAY;
                    end
                end

                DONE_STATE: begin
                    computing        <= 1'b0;
                    done             <= 1'b1;
                    tap_counter      <= 0;
                    state            <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
