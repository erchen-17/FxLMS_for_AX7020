`timescale 1ns/1ps
`include "fxlms_params.vh"
`include "tb_stimulus_cfg.vh"

// =============================================================================
// Full-system testbench for wm8731_dual_mic_top
//
// Coverage:
//  1) No button pressed   -> mute check
//  2) Pass-through mode   -> speaker follows ref microphone
//  3) Estimation mode     -> S'(z) update activity & error convergence
//  4) ANC mode (mu>0)     -> control output activity increases (merged Phase4&5)
//
// I2S injection method is aligned to i2s_adc_rx / i2s_adc_rx_1 timing:
//  - Start left frame at LRCLK falling edge
//  - 1 dummy bit, then 24-bit MSB-first payload
// =============================================================================
module tb_dual_mic_fxlms_v2;

    // -------------------------------------------------------------------------
    // Basic parameters
    // -------------------------------------------------------------------------
    localparam integer CLK_PERIOD_NS = 20;            // 50 MHz
    localparam integer AUDIO_WIDTH   = 24;
    localparam integer FRAC_BITS     = 16;
    localparam integer FILTER_ORDER  = `FILTER_ORDER;
    localparam integer DB_CYCLES     = 100;             // debounce wait (SIM_DBG_MODE uses 4)
    localparam integer PROC_WAIT_CYC =1000;            // ~1 audio sample @48kHz on 50MHz clk

    localparam integer S_MUTE        = `TB_SAMPLES_MUTE;
    localparam integer S_PASSTHROUGH = `TB_SAMPLES_PASSTHROUGH;
    localparam integer S_EST         = `TB_SAMPLES_EST;
    localparam integer S_ANC_MU1     = `TB_SAMPLES_ANC_MU0 + `TB_SAMPLES_ANC_MU1; // 合并Phase4�???5的时�???

    localparam integer ENABLE_EST    = `TB_ENABLE_EST;

    localparam real PI = 3.14159265358979323846;

    // -------------------------------------------------------------------------
    // DUT I/O
    // -------------------------------------------------------------------------
    reg         clock_50m;

    reg         btn_fxlms_enable;   // active-low on pin
    reg         btn_passthrough;    // active-low on pin
    reg         btn_estimate_mode;  // active-low on pin
    reg         btn_switch;         // active-low on pin

    wire        led_1st;
    wire        led_2nd;
    wire        led_3rd;
    wire        led_4th;

    wire        aud_scl;
    wire        aud_sda;
    wire        aud_scl_1;
    wire        aud_sda_1;

    wire        bclk;
    wire        adclrc;
    wire        daclrc;
    reg         adc_dat;

    wire        bclk_1;
    wire        adclrc_1;
    reg         adc_dat_1;

    wire        dac_dat;

    wire [14:0] DDR_addr;
    wire [2:0]  DDR_ba;
    wire        DDR_cas_n;
    wire        DDR_ck_n;
    wire        DDR_ck_p;
    wire        DDR_cke;
    wire        DDR_cs_n;
    wire [3:0]  DDR_dm;
    wire [31:0] DDR_dq;
    wire [3:0]  DDR_dqs_n;
    wire [3:0]  DDR_dqs_p;
    wire        DDR_odt;
    wire        DDR_ras_n;
    wire        DDR_reset_n;
    wire        DDR_we_n;
    wire        FIXED_IO_ddr_vrn;
    wire        FIXED_IO_ddr_vrp;
    wire [53:0] FIXED_IO_mio;
    wire        FIXED_IO_ps_clk;
    wire        FIXED_IO_ps_porb;
    wire        FIXED_IO_ps_srstb;

    wm8731_dual_mic_top_simfix dut (
        .clock_50m          (clock_50m),
        .btn_fxlms_enable   (btn_fxlms_enable),
        .btn_passthrough    (btn_passthrough),
        .btn_estimate_mode  (btn_estimate_mode),
        .btn_switch         (btn_switch),
        .led_1st            (led_1st),
        .led_2nd            (led_2nd),
        .led_3rd            (led_3rd),
        .led_4th            (led_4th),
        .aud_scl            (aud_scl),
        .aud_sda            (aud_sda),
        .aud_scl_1          (aud_scl_1),
        .aud_sda_1          (aud_sda_1),
        .bclk               (bclk),
        .adclrc             (adclrc),
        .daclrc             (daclrc),
        .adc_dat            (adc_dat),
        .bclk_1             (bclk_1),
        .adclrc_1           (adclrc_1),
        .adc_dat_1          (adc_dat_1),
        .dac_dat            (dac_dat),
        .DDR_addr           (DDR_addr),
        .DDR_ba             (DDR_ba),
        .DDR_cas_n          (DDR_cas_n),
        .DDR_ck_n           (DDR_ck_n),
        .DDR_ck_p           (DDR_ck_p),
        .DDR_cke            (DDR_cke),
        .DDR_cs_n           (DDR_cs_n),
        .DDR_dm             (DDR_dm),
        .DDR_dq             (DDR_dq),
        .DDR_dqs_n          (DDR_dqs_n),
        .DDR_dqs_p          (DDR_dqs_p),
        .DDR_odt            (DDR_odt),
        .DDR_ras_n          (DDR_ras_n),
        .DDR_reset_n        (DDR_reset_n),
        .DDR_we_n           (DDR_we_n),
        .FIXED_IO_ddr_vrn   (FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp   (FIXED_IO_ddr_vrp),
        .FIXED_IO_mio       (FIXED_IO_mio),
        .FIXED_IO_ps_clk    (FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb   (FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb  (FIXED_IO_ps_srstb)
    );

    // I2C pullups in TB
    pullup(aud_sda);
    pullup(aud_sda_1);

    // -------------------------------------------------------------------------
    // Clock
    // -------------------------------------------------------------------------
    initial begin
        clock_50m = 1'b0;
        forever #(CLK_PERIOD_NS/2) clock_50m = ~clock_50m;
    end

    // -------------------------------------------------------------------------
    // I2S stimulus engine (same protocol style as your existing TB)
    // -------------------------------------------------------------------------
    reg adclrc_d;
    reg adclrc1_d;

    reg        tx_active_0;
    reg [5:0]  tx_bitcnt_0;
    reg [23:0] tx_shift_0;

    reg        tx_active_1;
    reg [5:0]  tx_bitcnt_1;
    reg [23:0] tx_shift_1;

    reg signed [AUDIO_WIDTH-1:0] adc_test_data_0;  // error mic  (codec0)
    reg signed [AUDIO_WIDTH-1:0] adc_test_data_1;  // ref mic    (codec1)

    always @(negedge bclk or negedge dut.reset_n) begin
        if (!dut.reset_n) adclrc_d <= 1'b1;
        else              adclrc_d <= adclrc;
    end

    always @(negedge bclk_1 or negedge dut.reset_n) begin
        if (!dut.reset_n) adclrc1_d <= 1'b1;
        else              adclrc1_d <= adclrc_1;
    end

    // Codec0 serial injection
    always @(negedge bclk or negedge dut.reset_n) begin
        if (!dut.reset_n) begin
            tx_active_0 <= 1'b0;
            tx_bitcnt_0 <= 6'd0;
            tx_shift_0  <= 24'd0;
            adc_dat     <= 1'b0;
        end else begin
            if ((adclrc_d == 1'b1) && (adclrc == 1'b0)) begin
                tx_active_0 <= 1'b1;
                tx_bitcnt_0 <= 6'd0;
                tx_shift_0  <= adc_test_data_0[23:0];
                adc_dat     <= 1'b0; // dummy bit
            end else if (tx_active_0 && (adclrc == 1'b0)) begin
                tx_bitcnt_0 <= tx_bitcnt_0 + 1'b1;

                if (tx_bitcnt_0 >= 6'd1 && tx_bitcnt_0 <= 6'd24)
                    adc_dat <= tx_shift_0[23];

                if (tx_bitcnt_0 >= 6'd2 && tx_bitcnt_0 <= 6'd25)
                    tx_shift_0 <= {tx_shift_0[22:0], 1'b0};

                if (tx_bitcnt_0 >= 6'd25) begin
                    tx_active_0 <= 1'b0;
                    adc_dat <= 1'b0;
                end else if (tx_bitcnt_0 == 6'd0) begin
                    adc_dat <= 1'b0;
                end
            end else begin
                tx_active_0 <= 1'b0;
                adc_dat <= 1'b0;
            end
        end
    end

    // Codec1 serial injection
    always @(negedge bclk_1 or negedge dut.reset_n) begin
        if (!dut.reset_n) begin
            tx_active_1 <= 1'b0;
            tx_bitcnt_1 <= 6'd0;
            tx_shift_1  <= 24'd0;
            adc_dat_1   <= 1'b0;
        end else begin
            if ((adclrc1_d == 1'b1) && (adclrc_1 == 1'b0)) begin
                tx_active_1 <= 1'b1;
                tx_bitcnt_1 <= 6'd0;
                tx_shift_1  <= adc_test_data_1[23:0];
                adc_dat_1   <= 1'b0; // dummy bit
            end else if (tx_active_1 && (adclrc_1 == 1'b0)) begin
                tx_bitcnt_1 <= tx_bitcnt_1 + 1'b1;

                if (tx_bitcnt_1 >= 6'd1 && tx_bitcnt_1 <= 6'd24)
                    adc_dat_1 <= tx_shift_1[23];

                if (tx_bitcnt_1 >= 6'd2 && tx_bitcnt_1 <= 6'd25)
                    tx_shift_1 <= {tx_shift_1[22:0], 1'b0};

                if (tx_bitcnt_1 >= 6'd25) begin
                    tx_active_1 <= 1'b0;
                    adc_dat_1 <= 1'b0;
                end else if (tx_bitcnt_1 == 6'd0) begin
                    adc_dat_1 <= 1'b0;
                end
            end else begin
                tx_active_1 <= 1'b0;
                adc_dat_1 <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Helpers: numeric conversion + plant model
    // -------------------------------------------------------------------------
    integer i, k;
    integer seed0, seed1;

    real Pz [0:FILTER_ORDER-1];
    real Sz [0:FILTER_ORDER-1];
    real x_hist [0:FILTER_ORDER-1];
    real y_hist [0:FILTER_ORDER-1];

    function automatic signed [AUDIO_WIDTH-1:0] real_to_audio;
        input real x;
        real scaled;
        begin
            scaled = x * 8388607.0; // 2^23-1
            if (scaled > 8388607.0)  scaled = 8388607.0;
            if (scaled < -8388608.0) scaled = -8388608.0;
            real_to_audio = $rtoi(scaled);
        end
    endfunction

    function automatic real q16_to_norm;
        input signed [31:0] q16;
        begin
            q16_to_norm = ($itor(q16) / 65536.0) * 2.0;
        end
    endfunction

    task automatic shift_histories;
        input real x_in;
        input real y_in;
        begin
            for (k = FILTER_ORDER-1; k > 0; k = k - 1) begin
                x_hist[k] = x_hist[k-1];
                y_hist[k] = y_hist[k-1];
            end
            x_hist[0] = x_in;
            y_hist[0] = y_in;
        end
    endtask

    task automatic compute_paths;
        input  real x_in;
        input  real y_in;
        output real d_out;
        output real s_out;
        real acc_d;
        real acc_s;
        begin
            shift_histories(x_in, y_in);
            acc_d = 0.0;
            acc_s = 0.0;
            for (k = 0; k < FILTER_ORDER; k = k + 1) begin
                acc_d = acc_d + Pz[k] * x_hist[k];
                acc_s = acc_s + Sz[k] * y_hist[k];
            end
            d_out = acc_d;
            s_out = acc_s;
        end
    endtask

    // Model true secondary path for estimation: error_mic = Sz * excitation
    // Reuses y_hist[] as the excitation delay line (reset before Phase 4)
    task automatic compute_true_secondary;
        input  real exc_in;
        output real s_out;
        real acc;
        begin
            for (k = FILTER_ORDER-1; k > 0; k = k - 1)
                y_hist[k] = y_hist[k-1];
            y_hist[0] = exc_in;
            acc = 0.0;
            for (k = 0; k < FILTER_ORDER; k = k + 1)
                acc = acc + Sz[k] * y_hist[k];
            s_out = acc;
        end
    endtask

    task automatic wait_btn_debounce;
        begin
            repeat(DB_CYCLES) @(posedge clock_50m);
        end
    endtask

    task automatic pulse_btn_switch;
        begin
            btn_switch <= 1'b0;  // press
            repeat(3) wait_btn_debounce();
            btn_switch <= 1'b1;  // release
            repeat(3) wait_btn_debounce();
        end
    endtask

    task automatic drive_one_audio_frame;
        input real ref_norm;
        input real err_norm;
        begin
            adc_test_data_1 <= real_to_audio(ref_norm); // ref mic
            adc_test_data_0 <= real_to_audio(err_norm); // error mic
            @(negedge adclrc);
            repeat(PROC_WAIT_CYC) @(posedge clock_50m);
        end
    endtask

    // -------------------------------------------------------------------------
    // SCV文件导出相关（简化版：写死简单文件名，无参数，导出en、dn、yn�???
    // -------------------------------------------------------------------------
    integer scv_file;  // SCV文件句柄
    // �???化：直接写死文件名（可自行修改引号内的名称，默认与仿真文件同目录�???
    // 文件名：sin400_mu1.scv（与你Python代码读取路径匹配�???

    // 初始化SCV文件（仿真开始时创建并写入表头）
    task automatic init_scv_file;
        begin
            // 直接使用�???单文件名，无�???参数，引号内可直接修�???
            scv_file = $fopen("sin500_mu1_long_2.scv", "w"); // 打开文件�???"w"表示覆盖写入
            if (scv_file == 0) begin
                $error("wrong");
            end else begin
                // 写入表头：en(e_mic)、dn(d_primary)、yn(算法直接输出y_ctrl)、mu(effective_mu)
                $fwrite(scv_file, "en,dn,yn,mu\n");
                $display("success");
            end
        end
    endtask

    // 写入单组信号数据到SCV文件（仅在ANC模式下调用）
    task automatic write_scv_data;
        input real mu;
        input real en;       // en信号（e_mic，剩余误差）
        input real dn;       // dn信号（d_primary，主路径噪声�???
        input real yn;       // yn信号（算法直接输出，y_ctrl�???
        begin
            if (scv_file != 0) begin
                // 以浮点数格式写入，用逗号分隔，�?�配Python读取（可直接用pandas.read_csv读取�???
                $fwrite(scv_file, "%.10f,%.10f,%.10f,%.10f\n", en, dn, yn, mu);
            end
        end
    endtask

    // 关闭SCV文件（仿真结束前调用，确保数据写入完成）
    task automatic close_scv_file;
        begin
            if (scv_file != 0) begin
                $fclose(scv_file);
                $display("success close");
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------------------
    integer n;
    integer active_cnt_est;
    integer active_cnt_mu1;
    integer mute_err_cnt;
    integer passthrough_ok_cnt;
    integer est_update_cnt;

    real x_ref;
    real white_noise;
    real y_ctrl;         // yn：算法直接输出（control_output_q16转换后）
    real d_primary;      // dn：主路径噪声
    real s_secondary;
    real e_mic;          // en：剩余误�???
    real abs_err;
    real rms_mu1_acc;
    real rms_mu1;
    real exc_norm;
    real est_plant_out;
    real est_err_early;
    real est_err_late;
    real effective_mu_real;

    initial begin
        $dumpfile("tb_dual_mic_fxlms_v2.vcd");
        $dumpvars(0, tb_dual_mic_fxlms_v2);

        // 初始化SCV文件（仿真开始时执行�???
        init_scv_file;

        seed0 = 32'h1234_5678;
        seed1 = 32'h9abc_def0;

        // Buttons idle (not pressed)
        btn_fxlms_enable  = 1'b1;
        btn_passthrough   = 1'b1;
        btn_estimate_mode = 1'b1;
        btn_switch        = 1'b1;
        adc_test_data_0   = 24'sd0;
        adc_test_data_1   = 24'sd0;
        adc_dat           = 1'b0;
        adc_dat_1         = 1'b0;

        // Acoustic path initialization
        for (i = 0; i < FILTER_ORDER; i = i + 1) begin
            Pz[i] = 0.0;
            Sz[i] = 0.0;
            x_hist[i] = 0.0;
            y_hist[i] = 0.0;
        end

        // Primary path: pure delay of 5 samples
        Pz[5] = 1.0;

        // Secondary path: impulse at tap 32, gain=8.0
        // Larger gain ensures mu_s * estimation_error survives Q16.16 truncation
        Sz[32] = 1.0;

        // Wait for internal POR and I2S ready
        wait (dut.por_counter == 8'hff);
        repeat(50) @(posedge clock_50m);

        $display("============================================================");
        $display("Full TB start: wm8731_dual_mic_top");
        $display("============================================================");

        // ---------------------------------------------------------------------
        // Phase 1: no buttons -> speaker should stay muted
        // ---------------------------------------------------------------------
        mute_err_cnt = 0;
        for (n = 0; n < S_MUTE; n = n + 1) begin
            white_noise = `TB_AMP_NOISE * ($random / 2147483648.0);
            x_ref = `TB_AMP_MUTE * $sin(2.0 * PI * `TB_FREQ_MUTE * 2.0 * n / `TB_FS) + white_noise;
            drive_one_audio_frame(x_ref, x_ref);
            if ($signed(dut.speaker_out) !== 24'sd0)
                mute_err_cnt = mute_err_cnt + 1;
        end
        $display("[P1 MUTE] non-zero speaker samples = %0d / %0d", mute_err_cnt, S_MUTE);

        // ---------------------------------------------------------------------
        // Phase 2: passthrough mode -> speaker follows ref mic path
        // ---------------------------------------------------------------------
        btn_passthrough = 1'b0; // press B
        wait_btn_debounce();

        passthrough_ok_cnt = 0;
        for (n = 0; n < S_PASSTHROUGH; n = n + 1) begin
            white_noise = `TB_AMP_NOISE * ($random / 2147483648.0);
            x_ref = `TB_AMP_PASSTHROUGH * $sin(2.0 * PI * `TB_FREQ_PASSTHROUGH * 2.0 * n / `TB_FS) + white_noise;
            drive_one_audio_frame(x_ref, 0.0);

            abs_err = ($itor($signed(dut.speaker_out)) - $itor(real_to_audio(x_ref)));
            if (abs_err < 0.0) abs_err = -abs_err;
            if (abs_err < 12000.0)
                passthrough_ok_cnt = passthrough_ok_cnt + 1;
        end
        $display("[P2 PASSTHROUGH] approx-follow count = %0d / %0d", passthrough_ok_cnt, S_PASSTHROUGH);

        // Exit passthrough
        btn_passthrough = 1'b1;
        wait_btn_debounce();

        // ---------------------------------------------------------------------
        // Phase 3: Secondary path estimation (controlled by ENABLE_EST)
        // ---------------------------------------------------------------------
        if (ENABLE_EST) begin
            btn_estimate_mode = 1'b0; // press (active-low)
            wait_btn_debounce();

            // Clear y_hist before using it as estimation delay line
            for (i = 0; i < FILTER_ORDER; i = i + 1)
                y_hist[i] = 0.0;

            est_update_cnt = 0;
            est_err_early  = 0.0;
            est_err_late   = 0.0;
            est_plant_out  = 0.0;

            for (n = 0; n < S_EST; n = n + 1) begin
                // Compute plant response BEFORE driving: e(n) = Sz * w(n)
                exc_norm = q16_to_norm($signed(dut.control_output_q16));
                compute_true_secondary(exc_norm, est_plant_out);
                drive_one_audio_frame(0.0, est_plant_out);

                if (dut.u_fxlms_core.est_lms_done)
                    est_update_cnt = est_update_cnt + 1;

                abs_err = $itor($signed(dut.u_fxlms_core.estimation_error_reg));
                if (abs_err < 0.0) abs_err = -abs_err;
                if (n < (S_EST / 4))
                    est_err_early = est_err_early + abs_err;
                else if (n >= (S_EST * 3 / 4))
                    est_err_late = est_err_late + abs_err;
            end
            est_err_early = est_err_early / (S_EST / 4);
            est_err_late  = est_err_late  / (S_EST / 4);

            $display("[P3 ESTIMATION] S' update cycles = %0d / %0d", est_update_cnt, S_EST);
            $display("[P3 ESTIMATION] avg |est_error| early=%f late=%f", est_err_early, est_err_late);

            // Exit estimation mode
            btn_estimate_mode = 1'b1;
            wait_btn_debounce();

            // Clear histories before ANC phases reuse them
            for (i = 0; i < FILTER_ORDER; i = i + 1) begin
                x_hist[i] = 0.0;
                y_hist[i] = 0.0;
            end
        end else begin
            $display("[P3 ESTIMATION] SKIPPED (ENABLE_EST=0)");
        end

        // ---------------------------------------------------------------------
        // Phase 4: ANC mode (mu>0) -> control output activity increases (合并原Phase4&5)
        // ---------------------------------------------------------------------
        btn_fxlms_enable = 1'b0; // 按下fxlms按钮，启动ANC模式
        wait_btn_debounce();

        // 仅在ANC测试�???始时按指定次数切换mu&gt;0状�??
        for (i = 0; i < `TB_BTN_SWITCH_PRESSES; i = i + 1)
            pulse_btn_switch();

        active_cnt_mu1 = 0;
        rms_mu1_acc = 0.0;
        // 测试时长为原Phase4 + Phase5的�?�时�???
        for (n = 0; n < S_ANC_MU1; n = n + 1) begin
            white_noise = `TB_AMP_NOISE * ($random / 2147483648.0);
            x_ref = `TB_AMP_ANC * $sin(2.0 * PI * `TB_FREQ_ANC * 2.0 * n / `TB_FS) + white_noise;
            y_ctrl = q16_to_norm($signed(dut.control_output_q16)); // yn：算法直接输出（control_output_q16转换为归�???化�?�）
            effective_mu_real = q16_to_norm($signed(dut.effective_mu));
            compute_paths(x_ref, y_ctrl, d_primary, s_secondary);
            e_mic = d_primary + s_secondary; // en：剩余误�???
            drive_one_audio_frame(x_ref, e_mic);

            // 仅在ANC模式下，�???3个目标信号（en、dn、yn）写入SCV文件
            write_scv_data(effective_mu_real, e_mic, d_primary, y_ctrl);

            rms_mu1_acc = rms_mu1_acc + y_ctrl * y_ctrl;
            if ((y_ctrl > 0.01) || (y_ctrl < -0.01))
                active_cnt_mu1 = active_cnt_mu1 + 1;
        end
        rms_mu1 = $sqrt(rms_mu1_acc / S_ANC_MU1);
        $display("[P4 ANC mu>0] active y samples = %0d / %0d, y_rms=%f",
                 active_cnt_mu1, S_ANC_MU1, rms_mu1);

        // Release ANC button
        btn_fxlms_enable = 1'b1;
        wait_btn_debounce();

        // ---------------------------------------------------------------------
        // Summary + basic checks
        // ---------------------------------------------------------------------
        $display("============================================================");
        $display("SUMMARY");
        $display("  mute non-zero samples     : %0d / %0d", mute_err_cnt, S_MUTE);
        $display("  passthrough follow count  : %0d / %0d", passthrough_ok_cnt, S_PASSTHROUGH);
        $display("  est S' update count       : %0d / %0d", est_update_cnt, S_EST);
        $display("  est error early/late      : %f / %f", est_err_early, est_err_late);
        $display("  mu>0 active count         : %0d / %0d (rms=%f)", active_cnt_mu1, S_ANC_MU1, rms_mu1);
        $display("  LED state (mu selector)   : %b%b%b%b", led_1st, led_2nd, led_3rd, led_4th);
        $display("============================================================");

        if (mute_err_cnt > (S_MUTE/10))
            $error("Mute phase failed: too many non-zero speaker samples.");

        if (passthrough_ok_cnt < (S_PASSTHROUGH*2/3))
            $error("Pass-through phase failed: speaker does not track mic well.");

        if (ENABLE_EST && est_update_cnt == 0)
            $error("Estimation phase failed: no S' coefficient updates detected.");

        // 关闭SCV文件（仿真结束前执行，确保数据完整写入）
        close_scv_file;

        #10000;
        $finish;
    end

    // -------------------------------------------------------------------------
    // Timeout watchdog
    // -------------------------------------------------------------------------
    initial begin
        #(64'd10_000_000_000);
        // 超时情况下也关闭SCV文件，避免数据丢�???
        close_scv_file;
        $error("TB timeout.");
        $finish;
    end

endmodule
