// =============================================================================
// File: tb_stimulus_cfg.vh
// Description: Testbench stimulus configuration for tb_dual_mic_fxlms_v2
//
// Modify the parameters below to change audio frequencies and amplitudes
// used in each test phase without editing the testbench itself.
//
// Usage:
//   `include "tb_stimulus_cfg.vh"
// =============================================================================

`ifndef TB_STIMULUS_CFG_VH
`define TB_STIMULUS_CFG_VH

// =============================================================================
// Audio Frequencies (Hz) ?? one per test phase
// =============================================================================

// Phase 1: Mute check ?? tone injected while no button is pressed
`define TB_FREQ_MUTE          500.0

// Phase 2: Pass-through ?? speaker should follow ref mic
`define TB_FREQ_PASSTHROUGH   500.0

// Phase 4 & 5: ANC mode ?? used for both mu=0 and mu>0 phases
`define TB_FREQ_ANC           500.0

// =============================================================================
// Audio Amplitudes (normalised 0.0 ~ 1.0)
// =============================================================================

// Phase 1: Mute check amplitude
`define TB_AMP_MUTE           0.25

// Phase 2: Pass-through amplitude
`define TB_AMP_PASSTHROUGH    0.25

// Phase 4 & 5: ANC amplitude
`define TB_AMP_ANC            0.25

// =============================================================================
// Sample rate (Hz) ?? must match codec configuration
// =============================================================================
`define TB_FS                 48000.0

// =============================================================================
// White noise amplitude (normalised 0.0 ~ 1.0)
// Applied additively to x_ref in every test phase.
// Set to 0.0 to disable noise injection.
// =============================================================================
`define TB_AMP_NOISE          0

// =============================================================================
// Samples per test phase (number of audio frames driven in each phase)
// =============================================================================

// Phase 1: Mute check
`define TB_SAMPLES_MUTE        5

// Phase 2: Pass-through
`define TB_SAMPLES_PASSTHROUGH 5

// Phase 3: Secondary path estimation
`define TB_SAMPLES_EST         2400

// Phase 4: ANC with mu=0
`define TB_SAMPLES_ANC_MU0     20

// Phase 5: ANC with mu>0
`define TB_SAMPLES_ANC_MU1     10000

// Phase 3 enable flag: 1 = run estimation, 0 = skip
`define TB_ENABLE_EST          1

// Phase 5: number of btn_switch presses before ANC mu>0 loop
// Each press advances param_selector by one state (state0 -> stateN)
`define TB_BTN_SWITCH_PRESSES  1

`endif // TB_STIMULUS_CFG_VH
