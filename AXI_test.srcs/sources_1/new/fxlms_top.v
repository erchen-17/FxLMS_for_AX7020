`timescale 1ns / 1ps
`include "fxlms_params.vh"

// ==============================================================================
// Module: fxlms_top
// Description: Top-level wrapper for FxLMS system with button controls
//
// Features:
//   - Button debouncing for clean input signals
//   - Connects physical buttons to FxLMS control signals
//   - Optional LED status indicators
//
// Button Mapping:
//   - btn_adapt_enable (PS Button B13): Enable/disable adaptation
//   - btn_estimate_mode (PL Button N15): Trigger secondary path estimation
//
// I/O Interface:
//   - Buttons: Physical button inputs from FPGA board
//   - Audio I/O: Connect to ADC/DAC or simulation interfaces
//   - Status LEDs: Visual feedback (optional)
// ==============================================================================

module fxlms_top #(
    parameter DATA_WIDTH   = `DATA_WIDTH,
    parameter FILTER_ORDER = `FILTER_ORDER,
    parameter MU_WIDTH     = `MU_WIDTH,
    parameter MU_VALUE     = `MU_VALUE,
    parameter MU_S_VALUE   = `MU_S_VALUE,
    parameter CLK_FREQ_HZ  = 50_000_000,     // 50MHz default
    parameter DEBOUNCE_MS  = 20              // 20ms debounce time
)(
    // Clock and Reset
    input  wire clk,
    input  wire rst_n,

    // Button Inputs (from constraint file)
    input  wire btn_adapt_enable,      // PS Button (B13)
    input  wire btn_estimate_mode,     // PL Button (N15)
    input  wire btn_weight_reset,      // Weight reset button

    // FxLMS Core Control
    input  wire enable,                // Global enable
    input  wire sample_valid,          // Sample valid flag

    // Runtime configurable step sizes (default to .vh values if not driven externally)
    input  wire signed [MU_WIDTH-1:0]   mu_in,           // W filter step size (Q16.16)
    input  wire signed [MU_WIDTH-1:0]   mu_s_in,         // S' filter step size (Q16.16)

    // Audio Inputs
    input  wire signed [DATA_WIDTH-1:0] ref_signal,      // x(n) Reference signal
    input  wire signed [DATA_WIDTH-1:0] error_signal,    // e(n) Error signal
    input  wire signed [DATA_WIDTH-1:0] white_noise_in,  // w(n) White noise

    // Audio Outputs
    output wire signed [DATA_WIDTH-1:0] control_output,  // y(n) Control output
    output wire signed [DATA_WIDTH-1:0] PROC_out,        // Actual speaker output

    // Status Outputs
    output wire processing_done,
    output wire estimation_active,

    // Optional LED Status Indicators
    output wire led_adapt_status,      // LED: Adaptation active
    output wire led_estimate_status,   // LED: Estimation mode active
    output wire led_weight_reset       // LED: Weight reset triggered
);

    // Calculate debounce cycles from milliseconds
    localparam DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

    // Debounced button signals
    wire btn_adapt_enable_db;
    wire btn_estimate_mode_db;
    wire btn_weight_reset_db;

    // ==============================================================================
    // Button Debouncing
    // ==============================================================================

    // Debounce adapt_enable button
    button_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) debounce_adapt (
        .clk(clk),
        .rst_n(rst_n),
        .btn_in(btn_adapt_enable),
        .btn_out(btn_adapt_enable_db)
    );

    // Debounce estimate_mode button
    button_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) debounce_estimate (
        .clk(clk),
        .rst_n(rst_n),
        .btn_in(btn_estimate_mode),
        .btn_out(btn_estimate_mode_db)
    );

    // Debounce weight_reset button
    button_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) debounce_weight_reset (
        .clk(clk),
        .rst_n(rst_n),
        .btn_in(btn_weight_reset),
        .btn_out(btn_weight_reset_db)
    );

    // ==============================================================================
    // FxLMS Core Instantiation
    // ==============================================================================

    fxlms_core #(
        .DATA_WIDTH(DATA_WIDTH),
        .FILTER_ORDER(FILTER_ORDER),
        .MU_WIDTH(MU_WIDTH)
    ) u_fxlms_core (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .adapt_enable(btn_adapt_enable_db),     // Connect debounced button
        .estimate_mode(btn_estimate_mode_db),   // Connect debounced button
        .weight_reset(btn_weight_reset_db),     // Connect debounced weight reset button
        .sample_valid(sample_valid),
        .mu_in(mu_in),
        .mu_s_in(mu_s_in),
        .ref_signal(ref_signal),
        .error_signal(error_signal),
        .white_noise_in(white_noise_in),
        .control_output(control_output),
        .PROC_out(PROC_out),                    // Connect PROC_out output
        .processing_done(processing_done),
        .estimation_active(estimation_active)
    );

    // ==============================================================================
    // LED Status Indicators (optional)
    // ==============================================================================

    // LED shows adaptation status
    assign led_adapt_status = btn_adapt_enable_db && !estimation_active;

    // LED shows estimation mode status
    assign led_estimate_status = estimation_active;

    // LED shows weight reset status (lights up when button pressed)
    assign led_weight_reset = btn_weight_reset_db;

    // ==============================================================================
    // Debug Signals (for ILA probing)
    // ==============================================================================

    // Synthesis attributes to preserve signals for debugging
    (* mark_debug = "true" *) wire debug_adapt_raw = btn_adapt_enable;
    (* mark_debug = "true" *) wire debug_adapt_db = btn_adapt_enable_db;
    (* mark_debug = "true" *) wire debug_estimate_raw = btn_estimate_mode;
    (* mark_debug = "true" *) wire debug_estimate_db = btn_estimate_mode_db;
    (* mark_debug = "true" *) wire debug_weight_reset_raw = btn_weight_reset;
    (* mark_debug = "true" *) wire debug_weight_reset_db = btn_weight_reset_db;
    (* mark_debug = "true" *) wire debug_processing = processing_done;
    (* mark_debug = "true" *) wire debug_estimation = estimation_active;

endmodule
