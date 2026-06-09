`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2025/12/19 19:45:59
// Design Name:
// Module Name: WM8731
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

`include "../new/fxlms_params.vh"

module wm8731_top (
    input  wire clock_50m,
    input  wire reset_n,

    // ================= FxLMS Test Mode Button =================
    input  wire btn_fxlms_enable,  // 0=pass-through mode, 1=FxLMS test mode
    input  wire btn_weight_reset,  // Weight reset button (active-low)

    // ================= LED Status Indicator =================
    output wire led_fxlms_status,  // LED: 1=FxLMS mode (ON), 0=pass-through mode (OFF)
    output wire led_weight_reset,  // LED: Weight reset triggered

    // ================= I2C =================
    output wire aud_scl,
    inout  wire aud_sda,

    output wire aud_scl_1,
    inout  wire aud_sda_1,

    // ================= I2S : Codec 0 =================
    output wire bclk,
    output wire adclrc,
    output wire daclrc,
    input  wire adc_dat,

    // ================= I2S : Codec 1 =================
    output wire bclk_1,
    output wire adclrc_1,
    input  wire adc_dat_1,

    // ================= DAC =================
    output wire dac_dat
);

    // =========================================================
    // ILA Debug Signals (mark_debug for Vivado auto-detection)
    // =========================================================
    // Key audio signals
    (* mark_debug = "true" *) wire signed [23:0] dbg_adc_left_0;
    (* mark_debug = "true" *) wire signed [23:0] dbg_adc_left_1;
    (* mark_debug = "true" *) wire               dbg_adc_valid_0;
    (* mark_debug = "true" *) wire               dbg_adc_valid_1;
    (* mark_debug = "true" *) wire signed [23:0] dbg_fifo_rd_data;
    (* mark_debug = "true" *) wire               dbg_fifo_empty;
    (* mark_debug = "true" *) reg  signed [23:0] dbg_proc_out;

    // Control signals
    (* mark_debug = "true" *) wire               dbg_btn_fxlms_enable;
    (* mark_debug = "true" *) wire               dbg_btn_fxlms_enable_db;
    (* mark_debug = "true" *) wire               dbg_btn_weight_reset;
    (* mark_debug = "true" *) wire               dbg_btn_weight_reset_db;
    (* mark_debug = "true" *) wire               dbg_fxlms_processing_done;

    // FxLMS signals (Q16.16 format)
    (* mark_debug = "true" *) wire signed [31:0] dbg_ref_signal_q16;
    (* mark_debug = "true" *) wire signed [31:0] dbg_primary_noise_q16;
    (* mark_debug = "true" *) wire signed [31:0] dbg_control_output_q16;
    (* mark_debug = "true" *) wire signed [31:0] dbg_control_output_delayed;
    (* mark_debug = "true" *) wire signed [31:0] dbg_error_signal_q16;
    (* mark_debug = "true" *) wire signed [23:0] dbg_error_signal_24;
    (* mark_debug = "true" *) wire               dbg_adc_reg_valid;
    (* mark_debug = "true" *) wire               dbg_fxlms_estimation_active;

    // Overflow detection signals
    (* mark_debug = "true" *) wire               dbg_overflow_pos;
    (* mark_debug = "true" *) wire               dbg_overflow_neg;
    (* mark_debug = "true" *) reg                dbg_overflow_sticky;  // Latched overflow flag

    // =========================================================
    // I2S clocks: generated uniformly inside FPGA
    // =========================================================
    wire bclk_i;
    wire lrclk_i;
    wire bclk_pos;
    wire bclk_neg;

    // Multiple signals are from the same logical source
    assign bclk     = bclk_i;
    assign bclk_1   = bclk_i;

    assign adclrc   = lrclk_i;
    assign daclrc   = lrclk_i;
    assign adclrc_1 = lrclk_i;

    // =========================================================
    // ADC data (dual microphones)
    // =========================================================
    wire signed [23:0] adc_left_0;
    wire               adc_valid_0;

    wire signed [23:0] adc_left_1;
    wire               adc_valid_1;

    // =========================================================
    // Select test microphone source
    // =========================================================
    localparam integer TEST_MIC_SEL = 1; // 0: Mic0, 1: Mic1

    wire signed [23:0] adc_sel;
    wire               adc_sel_valid;

    assign adc_sel       = (TEST_MIC_SEL == 0) ? adc_left_0  : adc_left_1;
    assign adc_sel_valid = (TEST_MIC_SEL == 0) ? adc_valid_0 : adc_valid_1;

    // =========================================================
    // FxLMS Test Mode Signals
    // =========================================================
    // Button debouncing
    wire btn_fxlms_enable_db;
    wire btn_weight_reset_db;

    // FxLMS mode rising edge detection (for clearing delay lines)
    reg btn_fxlms_enable_db_d;  // Delayed version for edge detection
    wire fxlms_mode_start;      // Rising edge of FxLMS enable

    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n)
            btn_fxlms_enable_db_d <= 1'b0;
        else
            btn_fxlms_enable_db_d <= btn_fxlms_enable_db;
    end

    assign fxlms_mode_start = btn_fxlms_enable_db && !btn_fxlms_enable_db_d;

    // ADC data buffer (used in FxLMS mode)
    reg signed [23:0] adc_reg;
    reg adc_reg_valid;

    // Format conversion signals (Q16.16)
    wire signed [31:0] ref_signal_q16;      // ref_signal (24-bit -> Q16.16)
    reg  signed [31:0] ref_signal_q16_delayed; // ref_signal delayed to sync with primary_noise
    wire signed [31:0] primary_noise_q16;   // d(n) = P(z) * x(n) - primary path output
    wire               primary_valid;        // Primary path output valid
    wire signed [31:0] control_output_q16;  // FxLMS output (Q16.16)
    wire signed [31:0] error_signal_q16;    // error_signal (Q16.16)

    // Control output delay line (simulate S(z) path delay)
    reg signed [31:0] control_output_delayed;  // 1 sample delay to match TB behavior

    // Synchronized sample_valid for FxLMS (wait for primary_path_filter to complete)
    wire fxlms_sample_valid;

    // Output format conversion
    // (error_signal_24 is now declared and assigned in Q16.16 -> 24-bit conversion section)

    // FxLMS status
    wire fxlms_processing_done;
    wire fxlms_estimation_active;

    // =========================================================
    // FIFO
    // =========================================================
    wire signed [23:0] fifo_rd_data;
    wire               fifo_empty;

    // FIFO read enable: read whenever not empty (consistent with original version)
    audio_fifo_simple #(
        .DATA_WIDTH (24),
        .DEPTH      (16),
        .ADDR_WIDTH (4)
    ) u_audio_fifo (
        .clk      (clock_50m),
        .rst_n    (reset_n),
        .wr_en    (adc_sel_valid),
        .wr_data  (adc_sel),
        .rd_en    (~fifo_empty),
        .rd_data  (fifo_rd_data),
        .empty    (fifo_empty),
        .full     ()
    );

    // =========================================================
    // ADC data register buffer (used in FxLMS mode)
    // =========================================================
    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n) begin
            adc_reg <= 24'sd0;
            adc_reg_valid <= 1'b0;
        end else if (btn_fxlms_enable_db) begin
            // FxLMS test mode: cache microphone data
            if (adc_sel_valid) begin
                adc_reg <= adc_sel;
                adc_reg_valid <= 1'b1;
            end else begin
                adc_reg_valid <= 1'b0;
            end
        end else begin
            adc_reg_valid <= 1'b0;
        end
    end

    // =========================================================
    // Format conversion logic
    // =========================================================
    // 24-bit -> Q16.16 conversion (TEST: right-shift 8 bits to fill only fraction part)
    // Q16.16 format: 16-bit integer + 16-bit fraction
    // Right-shift 8 bits: 24-bit value fills lower 16 bits (fraction part only)
    // This makes the effective value = adc_reg / 256, keeping it in [-1, 1) range
    assign ref_signal_q16 = {{16{adc_reg[23]}}, adc_reg[23:8]};  // Right shift 8 bits

    // Use primary_valid as the synchronized sample_valid for FxLMS
    // This ensures primary_noise_q16 is ready when FxLMS starts processing
    assign fxlms_sample_valid = primary_valid;

    // =========================================================
    // Delay ref_signal to synchronize with primary_noise output
    // =========================================================
    // When primary_valid fires, we need the ref_signal that was input
    // to the primary_path_filter, not the current one
    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n)
            ref_signal_q16_delayed <= 32'sd0;
        else if (fxlms_mode_start)
            ref_signal_q16_delayed <= 32'sd0;  // Clear when FxLMS mode starts
        else if (primary_valid)
            ref_signal_q16_delayed <= ref_signal_q16_delayed;  // Hold value
        else if (adc_reg_valid)
            ref_signal_q16_delayed <= ref_signal_q16;  // Capture when new sample arrives
    end

    // =========================================================
    // Primary Path Filter P(z)
    // =========================================================
    // Simulates the acoustic path from reference mic to error mic
    // d(n) = P(z) * x(n)
    // Model: Pure delay (unit impulse)
    //   P(z) = z^(-DELAY_P) - only delay, no amplitude change
    primary_path_filter #(
        .DATA_WIDTH   (`DATA_WIDTH),
        .FILTER_ORDER (32),              // Smaller order sufficient for P(z)
        .FRAC_BITS    (`FRAC_BITS),
        .DELAY_P      (3)                // Primary path delay
    ) u_primary_path (
        .clk          (clock_50m),
        .rst_n        (reset_n),
        .clear        (fxlms_mode_start),  // Clear delay line when FxLMS mode starts
        .sample_valid (adc_reg_valid),
        .data_in      (ref_signal_q16),  // x(n)
        .data_out     (primary_noise_q16), // d(n) = P(z) * x(n)
        .data_valid   (primary_valid)
    );

    // =========================================================
    // Control output delay line (match TB timing)
    // =========================================================
    // In TB: error_signal uses control_delay[0] which contains PREVIOUS sample's output
    // TB timing:
    //   1. Compute error using control_delay[0] (old value)
    //   2. DUT processes and produces new control_output
    //   3. After processing done, update control_delay[0] = control_output
    //
    // So we need to update control_output_delayed AFTER fxlms finishes processing
    // This way, the NEXT sample's error calculation will use THIS sample's output
    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n)
            control_output_delayed <= 32'sd0;
        else if (fxlms_mode_start)
            control_output_delayed <= 32'sd0;  // Clear when FxLMS mode starts
        else if (fxlms_processing_done)
            control_output_delayed <= control_output_q16;
    end

    // =========================================================
    // Q16.16 -> 24-bit conversion (direct truncation)
    // =========================================================
    // Directly use lower 24 bits of error_signal_q16 as output
    // No shift operations - just truncate Q16.16 to 24 bits
    wire signed [23:0] error_signal_24;
    assign error_signal_24 = error_signal_q16[23:0];

    // =========================================================
    // Signal subtractor: construct error_signal (matching TB logic)
    // =========================================================
    // error_signal = d(n) - y'(n)
    //   d(n)  = P(z) * x(n) = primary_noise_q16 (from primary path filter)
    //   y'(n) = S(z) * y(n) ≈ control_output_delayed (simplified S(z) = z^(-1))
    // This matches the TB formula for proper FxLMS operation
    signal_adder_q16 u_signal_adder (
        .signal_a (primary_noise_q16),        // d(n) = P(z) * x(n) - primary path output
        .signal_b (control_output_delayed),   // y'(n) = S(z) * y(n) - secondary path output
        .sum_out  (error_signal_q16)          // e(n) = d(n) - y'(n)
    );

    // =========================================================
    // Button debounce module
    // =========================================================
    button_debounce #(
        .DEBOUNCE_CYCLES (250_000)  // 5ms @ 50MHz (recommended for audio systems)
    ) u_btn_fxlms_debounce (
        .clk     (clock_50m),
        .rst_n   (reset_n),
        .btn_in  (~btn_fxlms_enable),  // Invert: button is active-low (pressed=0)
        .btn_out (btn_fxlms_enable_db)
    );

    button_debounce #(
        .DEBOUNCE_CYCLES (250_000)  // 5ms @ 50MHz
    ) u_btn_weight_reset_debounce (
        .clk     (clock_50m),
        .rst_n   (reset_n),
        .btn_in  (~btn_weight_reset),  // Invert: button is active-low (pressed=0)
        .btn_out (btn_weight_reset_db)
    );

    // =========================================================
    // LED Status Indicator
    // =========================================================
    // LED ON when FxLMS mode is active (button pressed)
    // LED OFF when in pass-through mode (button not pressed)
    assign led_fxlms_status = btn_fxlms_enable_db;
    assign led_weight_reset = btn_weight_reset_db;

    // =========================================================
    // FxLMS Core module (debounce already done above)
    // =========================================================
    fxlms_core #(
        .DATA_WIDTH   (`DATA_WIDTH),
        .FILTER_ORDER (`FILTER_ORDER),
        .MU_WIDTH     (`MU_WIDTH),
        .MU_VALUE     (`MU_VALUE),
        .MU_S_VALUE   (`MU_S_VALUE)
    ) u_fxlms_core (
        .clk         (clock_50m),
        .rst_n       (reset_n),

        // Control signals (already debounced)
        .enable       (btn_fxlms_enable_db),  // Master enable
        .adapt_enable (1'b1),                 // Always enable adaptive mode
        .estimate_mode(1'b0),                 // Don't use estimation mode (use default S'(z))
        .weight_reset (btn_weight_reset_db),  // Weight reset button

        .sample_valid (fxlms_sample_valid),   // Synchronized sample_valid (from primary_path_filter)

        // Audio signals
        .ref_signal      (ref_signal_q16_delayed), // Delayed ref_signal (synced with primary_noise)
        .error_signal    (error_signal_q16),       // Manually constructed error signal (Q16.16)
        .white_noise_in  (32'sd0),                 // Don't use white noise

        .control_output  (control_output_q16),   // FxLMS output (Q16.16)
        .PROC_out        (),                     // Not used
        .processing_done (fxlms_processing_done),
        .estimation_active (fxlms_estimation_active)
    );

    // =========================================================
    // Output processing logic
    // =========================================================
    reg signed [23:0] proc_out;

    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n)
            proc_out <= 24'sd0;
        else if (btn_fxlms_enable_db) begin
            // ========== FxLMS Test Mode ==========
            // Output manually constructed error_signal
            // Reverse the input conversion: Q16.16 [15:0] -> shift left 8 bits -> 24-bit output
            // This matches the inverse of: 24-bit [23:8] -> Q16.16 [15:0]
            if (fxlms_processing_done)
                proc_out <= {error_signal_q16[15:0], 8'b0};  // Left shift 8 bits: restore 24-bit amplitude
        end else if (!fifo_empty) begin
            // ========== Pass-through Mode ==========
            // Continuously update from FIFO when data is available
            proc_out <= fifo_rd_data;
        end
    end

    // =========================================================
    // I2C configuration: configure WM8731
    // =========================================================
    reg_config u_reg_config_0 (
        .clock_50m (clock_50m),
        .reset_n   (reset_n),
        .i2c_sclk  (aud_scl),
        .i2c_sdat  (aud_sda)
    );

    reg_config_1 u_reg_config_1 (
        .clock_50m  (clock_50m),
        .reset_n    (reset_n),
        .i2c_sclk_1 (aud_scl_1),
        .i2c_sdat_1 (aud_sda_1)
    );

    // =========================================================
    // I2S clock distribution (unified inside FPGA)
    // =========================================================
    i2s_clk_gen u_i2s_clk_gen (
        .clock_50m (clock_50m),
        .rst_n     (reset_n),
        .bclk      (bclk_i),
        .lrclk     (lrclk_i),
        .bclk_pos  (bclk_pos),
        .bclk_neg  (bclk_neg)
    );

    // =========================================================
    // ADC RX (Mic 0)
    // =========================================================
    i2s_adc_rx u_i2s_adc_rx_0 (
        .rst_n     (reset_n),
        .bclk_neg  (bclk_neg),
        .lrclk     (lrclk_i),
        .adc_dat   (adc_dat),
        .adc_left  (adc_left_0),
        .adc_valid (adc_valid_0)
    );

    // =========================================================
    // ADC RX (Mic 1)
    // =========================================================
    i2s_adc_rx_1 u_i2s_adc_rx_1 (
        .rst_n        (reset_n),
        .bclk_neg     (bclk_neg),
        .lrclk        (lrclk_i),
        .adc_dat_1    (adc_dat_1),
        .adc_left_1   (adc_left_1),
        .adc_valid_1  (adc_valid_1)
    );

    // =========================================================
    // DAC TX
    // =========================================================
    i2s_dac_tx u_i2s_dac_tx (
        .rst_n     (reset_n),
        .bclk_neg  (bclk_neg),
        .lrclk     (lrclk_i),
        .dac_data  (proc_out),
        .dac_valid (1'b1),
        .dac_dat   (dac_dat)
    );

    // =========================================================
    // ILA Debug Signal Connections
    // =========================================================
    // Audio data signals
    assign dbg_adc_left_0  = adc_left_0;
    assign dbg_adc_left_1  = adc_left_1;
    assign dbg_adc_valid_0 = adc_valid_0;
    assign dbg_adc_valid_1 = adc_valid_1;
    assign dbg_fifo_rd_data = fifo_rd_data;
    assign dbg_fifo_empty  = fifo_empty;

    // Control signals
    assign dbg_btn_fxlms_enable    = btn_fxlms_enable;
    assign dbg_btn_fxlms_enable_db = btn_fxlms_enable_db;
    assign dbg_btn_weight_reset    = btn_weight_reset;
    assign dbg_btn_weight_reset_db = btn_weight_reset_db;
    assign dbg_fxlms_processing_done = fxlms_processing_done;

    // FxLMS signals
    assign dbg_ref_signal_q16       = ref_signal_q16_delayed;  // Use delayed version
    assign dbg_primary_noise_q16    = primary_noise_q16;
    assign dbg_control_output_q16   = control_output_q16;
    assign dbg_control_output_delayed = control_output_delayed;
    assign dbg_error_signal_q16     = error_signal_q16;
    assign dbg_error_signal_24      = error_signal_24;
    assign dbg_adc_reg_valid        = fxlms_sample_valid;  // Use synchronized sample_valid
    assign dbg_fxlms_estimation_active = fxlms_estimation_active;

    // Overflow detection (disabled - no longer using shift operations)
    assign dbg_overflow_pos = 1'b0;
    assign dbg_overflow_neg = 1'b0;

    // Sticky overflow flag (disabled)
    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n)
            dbg_overflow_sticky <= 1'b0;
    end

    // proc_out debug (directly assigned from reg)
    always @(posedge clock_50m) begin
        dbg_proc_out <= proc_out;
    end

endmodule
