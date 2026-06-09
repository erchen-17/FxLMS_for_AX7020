// =============================================================================
// File: fxlms_params.vh
// Description: FxLMS System Global Parameter Definitions
// Usage: `include "fxlms_params.vh" at the beginning of each module file
// =============================================================================

`ifndef FXLMS_PARAMS_VH
`define FXLMS_PARAMS_VH
`define SIM_DBG_MODE
// =============================================================================
// Debug / Simulation Mode
// =============================================================================
// Define SIM_DBG_MODE before including this header (e.g. in testbench) to use
// shorter debounce cycles suitable for simulation.  Leave undefined for
// production builds.
// =============================================================================
`ifdef SIM_DBG_MODE
    `define DEBOUNCE_CYCLES 4
`else
    `define DEBOUNCE_CYCLES 250_000
`endif

// Data bit width
`define DATA_WIDTH      32

// Fixed-point format: Q16.16 (16-bit integer + 16-bit fraction)
// Range: -32768.0 ~ +32767.99998 (approx. +/-32768)
// Precision: 2^(-16) approx. 1.53e-5
`define FRAC_BITS       16

// Filter orders
`define FILTER_ORDER    128     // W(z) control filter order
`define FILTER_ORDER_S  128     // S'(z) secondary path filter order (typically much smaller)

// LMS step size parameters
`define MU_WIDTH        32

// Common mu values in Q16.16 format (multiply by 65536 = 2^16):
// mu = 0.001 -> 32'sd66      (0.001 * 65536 ?????????????????? 66)
// mu = 0.005 -> 32'sd328     (0.005 * 65536 ?????????????????? 328)
// mu = 0.008 -> 32'sd524     (0.008 * 65536 ?????????????????? 524)
// mu = 0.05  -> 32'sd3277    (0.05 * 65536 ?????????????????? 3277)
// mu = 0.5   -> 32'sd32768   (0.5 * 65536 = 32768)
`define MU_VALUE        32'sd128     // mu ?????????????????? 0.0078 in Q16.16 (increased for faster learning)

`define MU_S_VALUE      32'sd2048    // mu_s ???????????????? 0.008 (8x larger for faster convergence)

// =============================================================================
// Secondary Path Estimation LMS Gradient Update Direction
// =============================================================================
// Controls the sign of the coefficient update step in secondary_lms_update:
//   S'(n+1) = S'(n) + mu_s * e_s(n) * w(n-i)   (EST_GRAD_DIRECTION = 0, standard LMS)
//   S'(n+1) = S'(n) - mu_s * e_s(n) * w(n-i)   (EST_GRAD_DIRECTION = 1, reversed)
// Use 1 if estimation diverges or error_signal polarity is inverted.
// 0 = Standard (gradient descent, add delta)
// 1 = Reversed  (gradient ascent,  subtract delta)
`define EST_GRAD_DIRECTION  0

// =============================================================================
// Secondary Path Estimation Signal Source Selection
// =============================================================================
// 0 = White Noise (LFSR-based) - Recommended for stable convergence
// 1 = Sine Wave - For single-frequency testing only
`define EST_SIGNAL_SRC  0

// =============================================================================
// W(z) Control Filter Initial Coefficients Mode
// =============================================================================
// 0 = All zeros (algorithm learns from scratch)
// 1 = Exponential decay: W[i] = GAIN * ALPHA^i (starting from DELAY index)
// 2 = Unit impulse: W[1] = 1.0, others = 0
`define W_INIT_MODE     0

// Exponential decay parameters (only used when W_INIT_MODE = 1)
`ifndef W_INIT_DELAY
    `define W_INIT_DELAY    74           // Initial delay (samples) before decay starts
`endif
`define W_INIT_GAIN     32'sd0  // Initial gain 0.5 in Q16.16
`define W_INIT_ALPHA    32'sd58982  // Decay factor 0.9 in Q16.16

// =============================================================================
// Secondary Path S'(z) Initial Coefficients Mode
// =============================================================================
// 0 = All zeros (rely on online estimation to learn S'(z))
// 1 = Exponential decay: S'[i] = GAIN * ALPHA^i (starting from DELAY index)
// 2 = Unit impulse: S'[1] = 1.0, others = 0 (assumes 1-sample delay)
`define S_INIT_MODE     0

// Exponential decay parameters (only used when S_INIT_MODE = 1)
`ifndef S_INIT_DELAY
    `define S_INIT_DELAY    74           // Initial delay (samples) before decay starts
`endif
`define S_INIT_GAIN     32'sd65536  // Initial gain 0.5 (half of true value for visible convergence)
`define S_INIT_ALPHA    32'sd58982  // Decay factor 0.9 (matches testbench TB_ALPHA_S)

// LFSR White Noise Generator Parameters 
`define LFSR_SEED       32'hACE1_CAFE    // Non-zero seed for LFSR
`define NOISE_AMPLITUDE 32'sd4096       // 0.3 amplitude in Q16.16 (matches TB_TONE_AMP and SINE_AMPLITUDE)

// =============================================================================
// Sine Wave Generator Parameters (for Secondary Path Estimation)
// =============================================================================
// Frequency control word for NCO (Numerically Controlled Oscillator)
// Formula: freq_word = (desired_freq / sample_rate) * 2^32
// For 48kHz sample rate and desired frequency f:
//   freq_word = (f / 48000) * 2^32
//
// Example frequencies:
//   100 Hz:  freq_word = (100 / 48000) * 2^32 = 8947848 (0x888889)
//   200 Hz:  freq_word = (200 / 48000) * 2^32 = 17895697 (0x1111111)
//   500 Hz:  freq_word = (500 / 48000) * 2^32 = 44739242 (0x2AAAAAA)
//   1000 Hz: freq_word = (1000 / 48000) * 2^32 = 89478485 (0x5555555)
//   2000 Hz: freq_word = (2000 / 48000) * 2^32 = 178956970 (0xAAAAAAA)
//
`define SINE_FREQ_WORD  32'd44739242    // 500 Hz @ 48kHz sample rate (matches TB_TONE_FREQ_HZ)

// Frequency divider for sine generator
// FREQ_DIVIDER = 1: No division (use freq_word directly)
// FREQ_DIVIDER = 2: Half frequency (e.g., 500Hz -> 250Hz)
// FREQ_DIVIDER = 4: Quarter frequency (e.g., 500Hz -> 125Hz)
// FREQ_DIVIDER = 10: 1/10 frequency (e.g., 500Hz -> 50Hz)
// This divider reduces the effective sample rate for phase accumulation
`define SINE_FREQ_DIVIDER  1            // Set to 1 for no division, increase to reduce frequency

// Sine wave amplitude in Q16.16 format
// amplitude = 0.8 -> 32'sd52429 (0.8 * 65536)
// amplitude = 0.5 -> 32'sd32768 (0.5 * 65536)
// amplitude = 0.3 -> 32'sd19661 (0.3 * 65536)
`define SINE_AMPLITUDE  32'sd2048      // 0.3 amplitude in Q16.16 (matches TB_TONE_AMP)

// =============================================================================
// Dynamic Learning-Rate Scheduler Selection
// =============================================================================
// 0 = Static mu from param_selector / MU_VALUE
// 1 = Cosine annealing by sample count:
//     mu(t) = mu_min + (mu_max - mu_min) * (1 + cos(pi * t / T)) / 2
// 2 = Error-driven cosine:
//     mu(n) = beta * [cos(alpha * |e(n)| - pi) + 1]
`define MU_SCHED_MODE           0
`define COS_ANNEAL_ENABLE       (`MU_SCHED_MODE == 1)

// Maximum (initial) learning rate in Q16.16
// mu = 0.01 -> 32'sd655
`define MU_COS_MAX              32'sd393

// Minimum (final) learning rate in Q16.16
// mu = 0.0005 -> 32'sd33
`define MU_COS_MIN              32'sd16

// Total number of audio sample ticks for one full annealing period.
// At 48 kHz sample rate:
//   48000  = 1 second
//   240000 = 5 seconds
//   480000 = 10 seconds
`define COS_ANNEAL_TOTAL_STEPS  6000

////////////////////////////////////////////////////////////////////////
// Cosine LUT address width (2^N entries). 8 = 256 entries is sufficient.
`define COS_TABLE_BITS          8

// Error-driven cosine parameters (used when MU_SCHED_MODE = 2)
// alpha scales |e(n)| to an angle in radians (Q16.16)
// beta sets mu amplitude (mu range is [0, 2*beta])
`define MU_ERR_COS_ALPHA        32'sd262144
`define MU_ERR_COS_BETA         32'sd512

`endif
