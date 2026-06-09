// =============================================================================
// File: tb_secondary_path_cfg.vh
// Description: Testbench Secondary Path Configuration
//
// This file controls how the secondary path S(z) is modeled in testbenches.
// Two modes are available:
//
//   Mode 0 (SIMPLE_DELAY): S(z) = GAIN_S * z^(-DELAY_S)
//     - Simple delay with gain, matching wm8731_top_tb style
//     - Easier for algorithm verification
//
//   Mode 1 (EXPONENTIAL_DECAY): S(z) = GAIN_S * z^(-DELAY_S) * sum(ALPHA_S^i)
//     - Exponential decay FIR filter
//     - More realistic acoustic model
//
// Usage:
//   `include "tb_secondary_path_cfg.vh"
//   Then use `ifdef TB_SECONDARY_PATH_SIMPLE_DELAY to select behavior
// =============================================================================

`ifndef TB_SECONDARY_PATH_CFG_VH
`define TB_SECONDARY_PATH_CFG_VH

// =============================================================================
// Secondary Path Mode Selection
// =============================================================================
// Uncomment ONE of the following lines to select the mode:

//`define TB_SECONDARY_PATH_SIMPLE_DELAY      // Mode 0: Simple delay (like wm8731_top_tb)
`define TB_SECONDARY_PATH_EXPONENTIAL_DECAY    // Mode 1: Exponential decay (more realistic)

// =============================================================================
// Secondary Path Parameters
// =============================================================================

// Delay (number of samples)
`define TB_DELAY_S      1

// Gain factor
`define TB_GAIN_S       1.0

// Decay factor (only used in EXPONENTIAL_DECAY mode)
`define TB_ALPHA_S      0.5

// =============================================================================
// Primary Path Parameters (for reference, modify if needed)
// =============================================================================

// Delay (number of samples)
`define TB_DELAY_P      3

// Decay factor (0.9 gives ~10x gain, 0.5 gives ~2x gain)
`define TB_ALPHA_P      0.5

// =============================================================================
// Signal Generation Parameters
// =============================================================================

// Sample rate (Hz)
`define TB_SAMPLE_RATE  48000.0

// Tone frequency (Hz)
`define TB_TONE_FREQ_HZ 500.0

// Tone amplitude (normalized, 0.0 ~ 1.0)
// Reduced to prevent saturation when combined with primary path gain
`define TB_TONE_AMP     0.3

// Noise standard deviation (Gaussian noise)
`define TB_NOISE_STD    0.01

// =============================================================================
// Test Duration Parameters (in samples)
// =============================================================================

`define TB_PASSTHROUGH_SAMPLES  500
`define TB_ESTIMATE_SAMPLES     3000
`define TB_FXLMS_SAMPLES        3000

`endif // TB_SECONDARY_PATH_CFG_VH
