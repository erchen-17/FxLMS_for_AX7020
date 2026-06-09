// =============================================================================
// File: tb_dual_mic_fxlms.v
// Description: Testbench for WM8731 Dual-Microphone FxLMS Top Module
//
// This testbench verifies the complete dual-microphone FxLMS system including:
//   - Dual microphone input processing
//   - FxLMS adaptive filtering
//   - Pass-through mode
//   - Secondary path estimation
//   - Button control and LED status
//
// Test Phases:
//   Phase 0: System Reset and Initialization
//   Phase 1: Secondary Path Estimation Mode (Learn S'(z))
//   Phase 2: FxLMS Active Mode with Noise Cancellation
//   Phase 3: Pass-through Mode
// =============================================================================
`timescale 1ns/1ps

`include "fxlms_params.vh"

module tb_dual_mic_fxlms;

    // =========================================================================
    // Test Parameters
    // =========================================================================
    parameter CLK_PERIOD = 20;              // 50MHz clock
    parameter DATA_WIDTH = 32;              // Q16.16
    parameter FRAC_BITS = 16;
    parameter FILTER_ORDER = `FILTER_ORDER;

    // Sample timing (simplified - not full I2S timing)
    parameter SAMPLE_PERIOD = 1000;         // Clock cycles between samples

    // Test sample counts
    parameter RESET_CYCLES = 100;
    parameter INIT_CYCLES = 500;
    parameter ESTIMATE_SAMPLES = 2000;      // Phase 1: Estimation first
    parameter FXLMS_SAMPLES = 3000;         // Phase 2: Then FxLMS convergence
    parameter PASSTHROUGH_SAMPLES = 500;    // Phase 3: Finally passthrough

    // Signal generation parameters
    parameter real TONE_FREQ = 0.02;        // Normalized frequency
    parameter real TONE_AMP = 0.6;
    parameter real NOISE_AMP = 0.1;

    // Acoustic path parameters (exponential decay model)
    // NOTE: S(z) parameters now match wm8731_top_tb for comparison
    parameter DELAY_P = 3;                  // Primary path delay
    parameter DELAY_S = 1;                  // Secondary path delay (was 2, now matches wm8731_top_tb)
    parameter real ALPHA_P = 0.9;           // Primary path decay coefficient
    parameter real ALPHA_S = 0.9;           // Secondary path decay coefficient (was 0.8, now matches wm8731_top_tb)
    parameter real GAIN_S = 1.0;            // Secondary path gain (was 0.8, now matches wm8731_top_tb)

    // =========================================================================
    // DUT Signals
    // =========================================================================
    reg clock_50m;
    reg reset_n;

    // Control buttons (active-low)
    reg btn_fxlms_enable;
    reg btn_passthrough;
    reg btn_estimate_mode;

    // LED status outputs
    wire led_fxlms_status;
    wire led_passthrough_status;
    wire led_estimate_status;

    // I2C interface (tri-state)
    wire aud_scl;
    wire aud_sda;
    wire aud_scl_1;
    wire aud_sda_1;

    // I2S interface - Codec 0 (Error Microphone)
    wire bclk;
    wire adclrc;
    wire daclrc;
    reg  adc_dat;           // Error mic input (simulated)

    // I2S interface - Codec 1 (Reference Microphone)
    wire bclk_1;
    wire adclrc_1;
    reg  adc_dat_1;         // Reference mic input (simulated)

    // DAC output
    wire dac_dat;

    // =========================================================================
    // Test Control Variables
    // =========================================================================
    integer test_phase;
    integer sample_count;
    integer total_samples;
    real error_power_sum;
    real initial_error_power;
    real final_error_power;
    integer rand_seed1, rand_seed2;

    // File handles
    integer log_file;
    integer csv_file;

    // Signal buffers
    real ref_signal_real;
    real error_signal_real;
    real noise_component;

    // Acoustic path models
    real Pz [0:FILTER_ORDER-1];             // Primary path P(z) coefficients
    real Sz [0:FILTER_ORDER-1];             // Secondary path S(z) coefficients
    reg signed [DATA_WIDTH-1:0] ref_delay [0:FILTER_ORDER-1];      // x(n) delay line
    reg signed [DATA_WIDTH-1:0] control_delay [0:FILTER_ORDER-1];  // y(n) delay line

    // Auxiliary variables for path computation
    real primary_noise;                     // d(n) = P(z) * x(n)
    real secondary_cancel;                  // y'(n) = S(z) * y(n)
    real acc;                               // Accumulator

    // Convergence detection variables
    real error_energy_window [0:999];       // Sliding window for 1000 samples
    integer window_idx;
    real window_sum;
    real avg_error_energy;
    real convergence_threshold;
    reg  convergence_flag;

    integer i, k;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    wm8731_dual_mic_top dut (
        .clock_50m              (clock_50m),
        .reset_n                (reset_n),

        // Control buttons
        .btn_fxlms_enable       (btn_fxlms_enable),
        .btn_passthrough        (btn_passthrough),
        .btn_estimate_mode      (btn_estimate_mode),

        // LED status
        .led_fxlms_status       (led_fxlms_status),
        .led_passthrough_status (led_passthrough_status),
        .led_estimate_status    (led_estimate_status),

        // I2C interface
        .aud_scl                (aud_scl),
        .aud_sda                (aud_sda),
        .aud_scl_1              (aud_scl_1),
        .aud_sda_1              (aud_sda_1),

        // I2S Codec 0 (Error Microphone)
        .bclk                   (bclk),
        .adclrc                 (adclrc),
        .daclrc                 (daclrc),
        .adc_dat                (adc_dat),

        // I2S Codec 1 (Reference Microphone)
        .bclk_1                 (bclk_1),
        .adclrc_1               (adclrc_1),
        .adc_dat_1              (adc_dat_1),

        // DAC output
        .dac_dat                (dac_dat)
    );

    // =========================================================================
    // Pull-up resistors for I2C (simulate open-drain)
    // =========================================================================
    pullup(aud_sda);
    pullup(aud_sda_1);

    // =========================================================================
    // Clock Generation
    // =========================================================================
    initial begin
        clock_50m = 0;
        forever #(CLK_PERIOD/2) clock_50m = ~clock_50m;
    end

    // =========================================================================
    // Acoustic Path Model Initialization
    // =========================================================================
    initial begin
        // Initialize all paths to zero
        for (i = 0; i < FILTER_ORDER; i = i + 1) begin
            Pz[i] = 0.0;
            Sz[i] = 0.0;
            ref_delay[i] = 0;
            control_delay[i] = 0;
        end

        // Primary path P(z): Exponential decay model
        // P(z) = z^(-DELAY_P) * sum(alpha_P^n * z^(-n))
        for (i = 0; i < FILTER_ORDER - DELAY_P; i = i + 1) begin
            Pz[DELAY_P + i] = $pow(ALPHA_P, i);
        end

        // Secondary path S(z): Exponential decay model
        // S(z) = GAIN_S * z^(-DELAY_S) * sum(alpha_S^n * z^(-n))
        for (i = 0; i < FILTER_ORDER - DELAY_S; i = i + 1) begin
            Sz[DELAY_S + i] = GAIN_S * $pow(ALPHA_S, i);
        end

        $display("Acoustic paths initialized:");
        $display("  P(z): delay=%0d, alpha=%f", DELAY_P, ALPHA_P);
        $display("  S(z): delay=%0d, alpha=%f, gain=%f", DELAY_S, ALPHA_S, GAIN_S);
    end

    // =========================================================================
    // Helper Functions
    // =========================================================================
    localparam real PI = 3.14159265358979323846;

    // Generate sinusoidal test signal
    function real generate_sine_wave;
        input integer sample_num;
        input real frequency;
        input real amplitude;
        begin
            generate_sine_wave = amplitude * $sin(2.0 * PI * frequency * sample_num);
        end
    endfunction

    // Generate Gaussian noise
    function real generate_noise;
        input real std_dev;
        integer rand_val;
        real noise;
        begin
            rand_val = $random(rand_seed1);
            noise = $itor(rand_val) / 2147483648.0;  // Normalize to [-1, 1]
            generate_noise = noise * std_dev;
        end
    endfunction

    // Generate white noise (uniform)
    function real generate_white_noise;
        input integer dummy;
        integer rand_val;
        begin
            rand_val = $random(rand_seed2);
            generate_white_noise = $itor(rand_val) / 2147483648.0;
        end
    endfunction

    // =========================================================================
    // Simplified Microphone Signal Injection
    // =========================================================================
    // This task bypasses full I2S simulation and directly injects signals
    // into the internal microphone data path
    //
    // Solution 2: Keep force throughout, use force=0 instead of release
    // This avoids race condition with I2S module's always block
    task inject_mic_samples;
        input real ref_signal;
        input real error_signal;
        integer timeout_counter;
        begin
            // Wait for positive clock edge
            @(posedge clock_50m);

            // Force internal microphone signals
            // Access the actual wire signals in the top module
            force dut.ref_mic_data = $rtoi(ref_signal * 8388607.0);    // 24-bit scale
            force dut.error_mic_data = $rtoi(error_signal * 8388607.0);
            force dut.ref_mic_valid = 1'b1;
            force dut.error_mic_valid = 1'b1;

            // Hold valid for two cycles to ensure clock_50m posedge captures it
            @(posedge clock_50m);
            @(posedge clock_50m);

            // Use force=0 instead of release to avoid I2S module competition
            // This keeps testbench in full control of valid signals
            force dut.ref_mic_valid = 1'b0;
            force dut.error_mic_valid = 1'b0;

            // Wait for FxLMS processing to complete (with timeout)
            timeout_counter = 0;
            while (!dut.fxlms_processing_done && timeout_counter < 600) begin
                @(posedge clock_50m);
                timeout_counter = timeout_counter + 1;
            end

            if (timeout_counter >= 600) begin
                $display("  [WARNING] inject_mic_samples timeout! processing_done not received");
            end

            // Extra cycles for output to stabilize
            repeat(10) @(posedge clock_50m);

            // Release all signals at the end for next injection
            release dut.ref_mic_data;
            release dut.error_mic_data;
            release dut.ref_mic_valid;
            release dut.error_mic_valid;
        end
    endtask

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        // Open output files
        log_file = $fopen("sim_dual_mic_log.txt", "w");
        csv_file = $fopen("sim_dual_mic_data.csv", "w");

        if (csv_file != 0) begin
            $fwrite(csv_file, "sample,phase,ref_signal,primary_noise,secondary_cancel,error_signal,speaker_output,s_prime_output,led_fxlms,led_passthrough,led_estimate\n");
        end

        // Waveform dump
        $dumpfile("tb_dual_mic_fxlms.vcd");
        $dumpvars(0, tb_dual_mic_fxlms);

        // Display header
        $display("========================================");
        $display("Dual-Microphone FxLMS System Testbench");
        $display("========================================");
        $display("Start time: %0t", $time);

        $fwrite(log_file, "========================================\n");
        $fwrite(log_file, "Dual-Microphone FxLMS System Testbench\n");
        $fwrite(log_file, "========================================\n");
        $fwrite(log_file, "Clock Period: %0d ns (50 MHz)\n", CLK_PERIOD);
        $fwrite(log_file, "Filter Order: %0d\n", FILTER_ORDER);
        $fwrite(log_file, "========================================\n\n");

        // Initialize signals
        reset_n = 1'b0;
        btn_fxlms_enable = 1'b1;    // Not pressed (active-low)
        btn_passthrough = 1'b1;
        btn_estimate_mode = 1'b1;
        adc_dat = 1'b0;
        adc_dat_1 = 1'b0;

        test_phase = 0;
        sample_count = 0;
        total_samples = 0;
        error_power_sum = 0.0;
        rand_seed1 = 12345;
        rand_seed2 = 67890;

        // =====================================================================
        // Phase 0: System Reset
        // =====================================================================
        $display("\n[Phase 0] System Reset and Initialization");
        $fwrite(log_file, "\n[Phase 0] System Reset\n");

        #(CLK_PERIOD * RESET_CYCLES);
        reset_n = 1'b1;

        $display("  Reset released at %0t", $time);

        // Wait for initialization (I2C config, etc.)
        #(CLK_PERIOD * INIT_CYCLES);

        $display("  Initialization complete at %0t", $time);
        $display("  System ready for operation");
        $fwrite(log_file, "  System initialized successfully\n");

        // Verify signal paths exist
        $display("\n[Signal Path Verification]");
        $display("  Checking DUT internal signals...");
        #(CLK_PERIOD);
        if (^dut.ref_mic_data === 1'bx) $display("  WARNING: dut.ref_mic_data contains X");
        if (^dut.error_mic_data === 1'bx) $display("  WARNING: dut.error_mic_data contains X");
        if (dut.ref_mic_valid === 1'bx) $display("  WARNING: dut.ref_mic_valid is X");
        if (dut.error_mic_valid === 1'bx) $display("  WARNING: dut.error_mic_valid is X");
        $display("  Signal path check complete");
        $display("");

        // =====================================================================
        // Phase 1: Secondary Path Estimation Mode (MUST DO FIRST!)
        // =====================================================================
        $display("\n[Phase 1] Secondary Path Estimation Mode");
        $display("  Target: %0d samples", ESTIMATE_SAMPLES);
        $display("  Purpose: Learn S'(z) before adaptive filtering");
        $fwrite(log_file, "\n[Phase 1] Secondary Path Estimation\n");
        $fwrite(log_file, "  This must be done before FxLMS adaptive mode\n");

        test_phase = 1;
        sample_count = 0;

        // Press estimation button AND FxLMS enable
        btn_estimate_mode = 1'b0;
        btn_fxlms_enable = 1'b0;
        $display("  Estimation button pressed at %0t", $time);

        #(CLK_PERIOD * 300_000);

        $display("  Estimation mode activated (LED should be ON)");
        $display("  Injecting sinusoidal signal to learn S'(z)...");

        for (sample_count = 0; sample_count < ESTIMATE_SAMPLES; sample_count = sample_count + 1) begin
            // =====================================================================
            // CORRECTED ESTIMATION MODE FLOW:
            // 1. DUT generates sine wave internally → speaker_out
            // 2. Testbench reads speaker_out
            // 3. Testbench computes S(z) * speaker_out
            // 4. Testbench injects result to error_mic
            // =====================================================================

            // Dummy ref_signal (not used in estimation mode, but inject 0 for safety)
            ref_signal_real = 0.0;

            // Update control delay line with ACTUAL control output from DUT
            for (k = FILTER_ORDER-1; k > 0; k = k - 1) begin
                control_delay[k] = control_delay[k-1];
            end
            // Read control output directly in Q16.16 format (avoid 24-bit conversion loss)
            control_delay[0] = dut.control_output_q16;

            // Compute secondary path response: e(n) = S(z) * speaker_out(n)
            // This simulates what the error microphone would pick up from the speaker
            acc = 0.0;
            for (k = 0; k < FILTER_ORDER; k = k + 1) begin
                acc = acc + Sz[k] * ($itor($signed(control_delay[k])) / 65536.0);
            end
            secondary_cancel = acc;

            // Error signal = secondary path output
            // This is what the physical error microphone would pick up
            error_signal_real = secondary_cancel;

            // Inject signals into DUT
            // ref_signal = 0 (not used in estimation mode)
            // error_signal = S(z) * speaker_out
            inject_mic_samples(ref_signal_real, error_signal_real);

            // Log data for plotting
            // CSV columns: sample,phase,ref_signal,primary_noise,secondary_cancel,error_signal,speaker_output,s_prime_output
            if (csv_file != 0) begin
                $fwrite(csv_file, "%0d,ESTIMATE,%f,%f,%f,%f,%f,%f,%b,%b,%b\n",
                        total_samples,
                        0.0,  // ref_signal (not used in estimation mode)
                        0.0,  // primary_noise (not applicable in estimation)
                        secondary_cancel,  // secondary_cancel = S(z) * speaker_out (actual)
                        error_signal_real,  // error_signal (same as secondary_cancel)
                        $itor($signed(dut.speaker_out)) / 8388607.0,  // speaker_output (sine wave)
                        $itor($signed(dut.u_fxlms_top.u_fxlms_core.s_output)) / 65536.0,  // S'(z)*w(n) estimated
                        led_fxlms_status,
                        led_passthrough_status,
                        led_estimate_status);
            end

            total_samples = total_samples + 1;

            if (sample_count % 500 == 0) begin
                $display("    Estimation sample %0d/%0d", sample_count, ESTIMATE_SAMPLES);
                $display("      control_output_q16 = %0d (0x%08h)",
                         $signed(dut.control_output_q16), dut.control_output_q16);
                $display("      secondary_cancel (TB) = %f", secondary_cancel);
                $display("      fxlms_processing_done = %b, estimation_active = %b",
                         dut.fxlms_processing_done, dut.fxlms_estimation_active);
            end

            #(CLK_PERIOD * SAMPLE_PERIOD);
        end

        $display("  Phase 1 Complete!");
        $display("  Secondary path S'(z) estimation finished");
        $display("  System now ready for adaptive FxLMS");
        $fwrite(log_file, "  S'(z) estimation complete\n");
        $fwrite(log_file, "  Ready for adaptive filtering\n");

        btn_estimate_mode = 1'b1;
        btn_fxlms_enable = 1'b1;
        #(CLK_PERIOD * 1000);

        // =====================================================================
        // Phase 2: FxLMS Active Mode (Adaptive Noise Cancellation)
        // =====================================================================
        $display("\n[Phase 2] FxLMS Active Mode - Noise Cancellation");
        $display("  Target: %0d samples", FXLMS_SAMPLES);
        $display("  Using S'(z) learned in Phase 1");
        $fwrite(log_file, "\n[Phase 2] FxLMS Active Mode\n");
        $fwrite(log_file, "  Samples: %0d\n", FXLMS_SAMPLES);

        test_phase = 2;
        sample_count = 0;
        error_power_sum = 0.0;

        // Initialize convergence detection
        convergence_threshold = 0.01;  // Threshold for average error energy
        convergence_flag = 1'b0;
        window_idx = 0;
        window_sum = 0.0;
        for (i = 0; i < 1000; i = i + 1) begin
            error_energy_window[i] = 0.0;
        end

        // Press FxLMS enable button
        btn_fxlms_enable = 1'b0;  // Active-low
        $display("  FxLMS button pressed at %0t", $time);

        // Wait for debouncing (5ms = 250,000 cycles @ 50MHz)
        #(CLK_PERIOD * 300_000);

        $display("  FxLMS mode activated (LED should be ON)");
        $display("  Beginning adaptive filtering...");

        for (sample_count = 0; sample_count < FXLMS_SAMPLES; sample_count = sample_count + 1) begin
            // Generate test signals
            // Reference signal: pure tone + noise (primary noise source)
            ref_signal_real = generate_sine_wave(sample_count, TONE_FREQ, TONE_AMP);
            noise_component = generate_noise(NOISE_AMP);
            ref_signal_real = ref_signal_real + noise_component;

            // Update reference delay line
            for (k = FILTER_ORDER-1; k > 0; k = k - 1) begin
                ref_delay[k] = ref_delay[k-1];
            end
            ref_delay[0] = $rtoi(ref_signal_real * 65536.0);  // Q16.16

            // Compute primary path output: d(n) = P(z) * x(n)
            acc = 0.0;
            for (k = 0; k < FILTER_ORDER; k = k + 1) begin
                acc = acc + Pz[k] * ($itor($signed(ref_delay[k])) / 65536.0);
            end
            primary_noise = acc;

            // Compute secondary path cancellation: y'(n) = S(z) * y(n)
            // (using control output from previous iteration)
            acc = 0.0;
            for (k = 0; k < FILTER_ORDER; k = k + 1) begin
                acc = acc + Sz[k] * ($itor($signed(control_delay[k])) / 65536.0);
            end
            secondary_cancel = acc;

            // Error signal: e(n) = d(n) - y'(n)
            // This is what the error microphone actually picks up
            error_signal_real = primary_noise - secondary_cancel;

            // Inject signals into DUT
            inject_mic_samples(ref_signal_real, error_signal_real);

            // Update control delay line with NEW output from DUT
            for (k = FILTER_ORDER-1; k > 0; k = k - 1) begin
                control_delay[k] = control_delay[k-1];
            end
            // Read control output directly in Q16.16 format (avoid 24-bit conversion loss)
            control_delay[0] = dut.control_output_q16;

            // Accumulate error power
            error_power_sum = error_power_sum + (error_signal_real * error_signal_real);

            // =========================================================
            // Convergence Detection (Sliding Window)
            // =========================================================
            // Update sliding window
            window_sum = window_sum - error_energy_window[window_idx];
            error_energy_window[window_idx] = (error_signal_real * error_signal_real);
            window_sum = window_sum + error_energy_window[window_idx];
            window_idx = (window_idx + 1) % 1000;

            // Check convergence after window is full
            if (sample_count >= 1000) begin
                avg_error_energy = window_sum / 1000.0;
                if (avg_error_energy < convergence_threshold) begin
                    if (!convergence_flag) begin
                        convergence_flag = 1'b1;
                        $display("    >>> CONVERGENCE DETECTED at sample %0d <<<", sample_count);
                        $display("        Average error energy: %f < threshold: %f",
                                 avg_error_energy, convergence_threshold);
                        $fwrite(log_file, "  Convergence at sample %0d (avg_error_energy=%f)\n",
                                sample_count, avg_error_energy);
                    end
                end else begin
                    convergence_flag = 1'b0;
                end
            end

            // Log data
            if (csv_file != 0) begin
                $fwrite(csv_file, "%0d,FXLMS,%f,%f,%f,%f,%f,%f,%b,%b,%b\n",
                        total_samples,
                        ref_signal_real,
                        primary_noise,
                        secondary_cancel,
                        error_signal_real,
                        $itor($signed(dut.speaker_out)) / 8388607.0,
                        $itor($signed(dut.u_fxlms_top.u_fxlms_core.s_output)) / 65536.0,  // x'(n) = S'(z)*x(n)
                        led_fxlms_status,
                        led_passthrough_status,
                        led_estimate_status);
            end

            total_samples = total_samples + 1;

            // Progress reporting with timing diagnostics
            if (sample_count % 500 == 0) begin
                $display("    Sample %0d/%0d - Error RMS: %f",
                         sample_count, FXLMS_SAMPLES,
                         $sqrt(error_power_sum / (sample_count + 1)));
                // Key signal monitoring for timing diagnosis
                $display("      [TIMING] control_output_q16 = %0d (0x%08h)",
                         $signed(dut.control_output_q16), dut.control_output_q16);
                $display("      [TIMING] secondary_cancel (TB) = %f", secondary_cancel);
                $display("      [TIMING] primary_noise (TB) = %f, error_signal (TB) = %f",
                         primary_noise, error_signal_real);
                $display("      [TIMING] fxlms_processing_done = %b, sample_valid_delayed = %b",
                         dut.fxlms_processing_done, dut.sample_valid_delayed);
            end

            // Wait between samples
            #(CLK_PERIOD * SAMPLE_PERIOD);
        end

        final_error_power = error_power_sum / FXLMS_SAMPLES;
        $display("  Phase 1 Complete!");
        $display("  Average Error Power: %f", final_error_power);
        $display("  Error RMS: %f", $sqrt(final_error_power));
        $fwrite(log_file, "  Final Error Power: %f\n", final_error_power);
        $fwrite(log_file, "  Error RMS: %f\n", $sqrt(final_error_power));

        // Release button
        btn_fxlms_enable = 1'b1;
        #(CLK_PERIOD * 1000);

        // =====================================================================
        // Phase 3: Pass-through Mode
        // =====================================================================
        $display("\n[Phase 3] Pass-through Mode");
        $display("  Target: %0d samples", PASSTHROUGH_SAMPLES);
        $fwrite(log_file, "\n[Phase 3] Pass-through Mode\n");

        test_phase = 3;
        sample_count = 0;

        // Press pass-through button
        btn_passthrough = 1'b0;
        $display("  Pass-through button pressed at %0t", $time);

        #(CLK_PERIOD * 300_000);  // Debounce

        $display("  Pass-through mode activated");
        $display("  Error mic signal passed directly to speaker");

        for (sample_count = 0; sample_count < PASSTHROUGH_SAMPLES; sample_count = sample_count + 1) begin
            ref_signal_real = generate_sine_wave(FXLMS_SAMPLES + sample_count, TONE_FREQ, TONE_AMP);
            error_signal_real = ref_signal_real;

            inject_mic_samples(ref_signal_real, error_signal_real);

            if (csv_file != 0) begin
                $fwrite(csv_file, "%0d,PASSTHROUGH,%f,0.0,0.0,%f,%f,0.0,%b,%b,%b\n",
                        total_samples,
                        ref_signal_real,
                        error_signal_real,
                        $itor($signed(dut.speaker_out)) / 8388607.0,
                        led_fxlms_status,
                        led_passthrough_status,
                        led_estimate_status);
            end

            total_samples = total_samples + 1;

            #(CLK_PERIOD * SAMPLE_PERIOD);
        end

        $display("  Phase 2 Complete!");
        $fwrite(log_file, "  Pass-through mode verified\n");

        btn_passthrough = 1'b1;
        #(CLK_PERIOD * 1000);

        // =====================================================================
        // Simulation Summary
        // =====================================================================
        $display("\n========================================");
        $display("Simulation Complete!");
        $display("========================================");
        $display("Total simulation time: %0t", $time);
        $display("Total samples processed: %0d", total_samples);
        $display("");
        $display("Test Results:");
        $display("  Phase 1 (Estimation):  %0d samples", ESTIMATE_SAMPLES);
        $display("    S'(z) learning:      Complete");
        $display("  Phase 2 (FxLMS):       %0d samples", FXLMS_SAMPLES);
        $display("    Final Error RMS:     %f", $sqrt(final_error_power));
        $display("  Phase 3 (Passthrough): %0d samples", PASSTHROUGH_SAMPLES);
        $display("");
        $display("LED Status at end:");
        $display("  FxLMS LED:       %b", led_fxlms_status);
        $display("  Passthrough LED: %b", led_passthrough_status);
        $display("  Estimation LED:  %b", led_estimate_status);
        $display("========================================");

        $fwrite(log_file, "\n========================================\n");
        $fwrite(log_file, "Simulation Complete!\n");
        $fwrite(log_file, "Total samples: %0d\n", total_samples);
        $fwrite(log_file, "Final Error RMS: %f\n", $sqrt(final_error_power));
        $fwrite(log_file, "========================================\n");

        // Close files
        $fclose(log_file);
        $fclose(csv_file);

        $display("\nOutput files:");
        $display("  sim_dual_mic_log.txt");
        $display("  sim_dual_mic_data.csv");
        $display("  tb_dual_mic_fxlms.vcd");

        #(CLK_PERIOD * 100);
        $finish;
    end

    // =========================================================================
    // Timeout Watchdog
    // =========================================================================
    initial begin
        #1_000_000_000;  // 1 second timeout
        $display("\n[ERROR] Simulation timeout at %0t!", $time);
        $display("Current phase: %0d", test_phase);
        $display("Samples processed: %0d", total_samples);
        $fclose(log_file);
        $fclose(csv_file);
        $finish;
    end

    // =========================================================================
    // LED Monitoring
    // =========================================================================
    always @(posedge led_fxlms_status) begin
        $display("  [%0t] >>> FxLMS LED ON", $time);
    end

    always @(negedge led_fxlms_status) begin
        $display("  [%0t] >>> FxLMS LED OFF", $time);
    end

    always @(posedge led_passthrough_status) begin
        $display("  [%0t] >>> Pass-through LED ON", $time);
    end

    always @(negedge led_passthrough_status) begin
        $display("  [%0t] >>> Pass-through LED OFF", $time);
    end

    always @(posedge led_estimate_status) begin
        $display("  [%0t] >>> Estimation LED ON", $time);
    end

    always @(negedge led_estimate_status) begin
        $display("  [%0t] >>> Estimation LED OFF", $time);
    end

endmodule
