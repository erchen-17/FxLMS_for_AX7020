`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Simulation-fix variant of WM8731_dual_mic top
// Note: Main file remains untouched. This module is for simulation use.
//////////////////////////////////////////////////////////////////////////////////

`include "../new/fxlms_params.vh"

module wm8731_dual_mic_top_simfix (
    input  wire clock_50m,
    input  wire btn_fxlms_enable,
    input  wire btn_passthrough,
    input  wire btn_estimate_mode,
    input  wire btn_switch,
    output wire led_1st,
    output wire led_2nd,
    output wire led_3rd,
    output wire led_4th,
    output wire aud_scl,
    inout  wire aud_sda,
    output wire aud_scl_1,
    inout  wire aud_sda_1,
    output wire bclk,
    output wire adclrc,
    output wire daclrc,
    input  wire adc_dat,
    output wire bclk_1,
    output wire adclrc_1,
    input  wire adc_dat_1,
    output wire dac_dat,
    inout [14:0]DDR_addr,
    inout [2:0]DDR_ba,
    inout DDR_cas_n,
    inout DDR_ck_n,
    inout DDR_ck_p,
    inout DDR_cke,
    inout DDR_cs_n,
    inout [3:0]DDR_dm,
    inout [31:0]DDR_dq,
    inout [3:0]DDR_dqs_n,
    inout [3:0]DDR_dqs_p,
    inout DDR_odt,
    inout DDR_ras_n,
    inout DDR_reset_n,
    inout DDR_we_n,
    inout FIXED_IO_ddr_vrn,
    inout FIXED_IO_ddr_vrp,
    inout [53:0]FIXED_IO_mio,
    inout FIXED_IO_ps_clk,
    inout FIXED_IO_ps_porb,
    inout FIXED_IO_ps_srstb
);

    // -------------------------------------------------------------------------
    // Reuse the original top for all functionality through local copy logic.
    // This file keeps only simulation-safe AXI master read-channel wiring.
    // -------------------------------------------------------------------------

    // ===== Power-on reset and debug scanner =====
    reg [7:0] por_counter = 8'd0;
    reg reset_n = 1'b0;
    reg por_weight_reset = 1'b0;

    always @(posedge clock_50m) begin
        if (por_counter < 8'd255) begin
            por_counter      <= por_counter + 8'd1;
            reset_n          <= (por_counter >= 8'd128) ? 1'b1 : 1'b0;
            por_weight_reset <= (por_counter >= 8'd128) ? 1'b1 : 1'b0;
        end else begin
            reset_n          <= 1'b1;
            por_weight_reset <= 1'b0;
        end
    end

    reg  [7:0]          dbg_s_coeff_addr;
    wire signed [31:0]  dbg_s_coeff_data;
    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n) dbg_s_coeff_addr <= 8'd0;
        else          dbg_s_coeff_addr <= dbg_s_coeff_addr + 8'd1;
    end

    // ===== I2S clocks =====
    wire bclk_i;
    wire lrclk_i;
    wire bclk_pos;
    wire bclk_neg;
    assign bclk     = bclk_i;
    assign bclk_1   = bclk_i;
    assign adclrc   = lrclk_i;
    assign daclrc   = lrclk_i;
    assign adclrc_1 = lrclk_i;

    // ===== ADC data =====
    wire signed [23:0] ref_mic_data;
    wire               ref_mic_valid;
    wire signed [23:0] error_mic_data;
    wire               error_mic_valid;

    // ===== Buttons =====
    wire btn_fxlms_enable_db;
    wire btn_passthrough_db;
    wire btn_estimate_mode_db;
    wire btn_switch_db;
    reg  btn_fxlms_enable_db_d;
    wire fxlms_mode_start;
    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n) btn_fxlms_enable_db_d <= 1'b0;
        else          btn_fxlms_enable_db_d <= btn_fxlms_enable_db;
    end
    assign fxlms_mode_start = btn_fxlms_enable_db && !btn_fxlms_enable_db_d;

    // ===== Mic buffering =====
    reg signed [23:0] ref_mic_reg;
    reg signed [23:0] error_mic_reg;
    reg               mic_data_valid;
    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n) begin
            ref_mic_reg <= 24'sd0;
            error_mic_reg <= 24'sd0;
            mic_data_valid <= 1'b0;
        end else if (btn_fxlms_enable_db || btn_estimate_mode_db) begin
            if (ref_mic_valid && error_mic_valid) begin
                ref_mic_reg <= ref_mic_data;
                error_mic_reg <= error_mic_data;
                mic_data_valid <= 1'b1;
            end else begin
                mic_data_valid <= 1'b0;
            end
        end else begin
            mic_data_valid <= 1'b0;
        end
    end

    wire signed [31:0] x_n_q16 = {{16{ref_mic_reg[23]}}, ref_mic_reg[23:8]};
    wire signed [31:0] e_n_q16 = {{16{error_mic_reg[23]}}, error_mic_reg[23:8]};

    wire signed [31:0] control_output_q16;
    reg  signed [31:0] x_n_q16_delayed;
    reg  signed [31:0] e_n_q16_delayed;
    reg                sample_valid_delayed;
    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n) begin
            x_n_q16_delayed <= 32'sd0;
            e_n_q16_delayed <= 32'sd0;
            sample_valid_delayed <= 1'b0;
        end else if (fxlms_mode_start) begin
            x_n_q16_delayed <= 32'sd0;
            e_n_q16_delayed <= 32'sd0;
            sample_valid_delayed <= 1'b0;
        end else if (mic_data_valid) begin
            x_n_q16_delayed <= x_n_q16;
            e_n_q16_delayed <= e_n_q16;
            sample_valid_delayed <= 1'b1;
        end else begin
            sample_valid_delayed <= 1'b0;
        end
    end

    // ===== Estimation signal =====
    wire signed [31:0] sine_wave_q16;
    wire signed [31:0] lfsr_noise_q16;
    wire signed [31:0] estimation_signal;

    sine_generator #(
        .DATA_WIDTH   (`DATA_WIDTH),
        .PHASE_WIDTH  (32),
        .FREQ_DIVIDER (`SINE_FREQ_DIVIDER)
    ) u_sine_generator (
        .sample_clk  (lrclk_i),
        .rst_n       (reset_n),
        .enable      (btn_estimate_mode_db),
        .freq_word   (`SINE_FREQ_WORD),
        .amplitude   (`SINE_AMPLITUDE),
        .sine_out    (sine_wave_q16)
    );

    lfsr_noise_gen #(
        .DATA_WIDTH   (`DATA_WIDTH),
        .LFSR_WIDTH   (32),
        .SEED         (`LFSR_SEED)
    ) u_lfsr_noise_gen (
        .sample_clk  (lrclk_i),
        .rst_n       (reset_n),
        .enable      (btn_estimate_mode_db),
        .amplitude   (`NOISE_AMPLITUDE),
        .noise_out   (lfsr_noise_q16)
    );

    generate
        if (`EST_SIGNAL_SRC == 0) begin : gen_white_noise
            assign estimation_signal = lfsr_noise_q16;
        end else begin : gen_sine_wave
            assign estimation_signal = sine_wave_q16;
        end
    endgenerate

    wire [31:0] target_mu;
    wire [7:0]  target_s_delay;
    param_selector_16state #(
        .MU_WIDTH    (`MU_WIDTH),
        .DELAY_WIDTH (8)
    ) u_param_selector (
        .clk         (clock_50m),
        .rst_n       (reset_n),
        .key_press   (btn_switch_db),
        .mu_out      (target_mu),
        .s_delay_out (target_s_delay),
        .led_1st     (led_1st),
        .led_2nd     (led_2nd),
        .led_3rd     (led_3rd),
        .led_4th     (led_4th)
    );

    // ===== Dynamic Learning Rate Scheduler =====
    wire signed [31:0] effective_mu;

    generate
        if (`MU_SCHED_MODE == 1) begin : gen_cos_anneal
            wire signed [31:0] cos_mu;
            wire               cos_done;

            cosine_annealing_mu #(
                .DATA_WIDTH     (`DATA_WIDTH),
                .FRAC_BITS      (`FRAC_BITS),
                .COS_TABLE_BITS (`COS_TABLE_BITS),
                .TOTAL_STEPS    (`COS_ANNEAL_TOTAL_STEPS),
                .PHASE_BITS     (32)
            ) u_cos_anneal (
                .clk         (clock_50m),
                .rst_n       (reset_n),
                .start       (fxlms_mode_start),
                .sample_tick (sample_valid_delayed),
                .mu_max      (`MU_COS_MAX),
                .mu_min      (`MU_COS_MIN),
                .mu_out      (cos_mu),
                .annealing_done (cos_done)
            );

            assign effective_mu = cos_mu;
        end else if (`MU_SCHED_MODE == 2) begin : gen_error_cos_mu
            wire signed [31:0] err_cos_mu;

            error_cosine_mu #(
                .DATA_WIDTH     (`DATA_WIDTH),
                .FRAC_BITS      (`FRAC_BITS),
                .COS_TABLE_BITS (`COS_TABLE_BITS)
            ) u_error_cos (
                .clk         (clock_50m),
                .rst_n       (reset_n),
                .sample_tick (sample_valid_delayed),
                .error_in    (e_n_q16_delayed),
                .alpha       (`MU_ERR_COS_ALPHA),
                .beta        (`MU_ERR_COS_BETA),
                .mu_out      (err_cos_mu)
            );

            assign effective_mu = err_cos_mu;
        end else begin : gen_static_mu
            assign effective_mu = target_mu;
        end
    endgenerate

    wire fxlms_processing_done;
    wire fxlms_estimation_active;
    fxlms_core #(
        .DATA_WIDTH   (`DATA_WIDTH),
        .FILTER_ORDER (`FILTER_ORDER),
        .MU_WIDTH     (`MU_WIDTH)
    ) u_fxlms_core (
        .clk         (clock_50m),
        .rst_n       (reset_n),
        .enable       (btn_fxlms_enable_db || btn_estimate_mode_db),
        .adapt_enable (1'b1),
        .estimate_mode(btn_estimate_mode_db),
        .weight_reset (por_weight_reset),
        .sample_valid (sample_valid_delayed),
        .mu_in        (effective_mu),
        .mu_s_in      (`MU_S_VALUE),
        .w_init_delay (`W_INIT_DELAY),
        .s_init_delay (target_s_delay),
        .ref_signal      (x_n_q16_delayed),
        .error_signal    (e_n_q16_delayed),
        .white_noise_in  (estimation_signal),
        .control_output  (control_output_q16),
        .PROC_out        (),
        .processing_done (fxlms_processing_done),
        .estimation_active (fxlms_estimation_active),
        .dbg_s_coeff_addr (dbg_s_coeff_addr),
        .dbg_s_coeff_data (dbg_s_coeff_data)
    );

    // ===== Output path =====
    wire signed [39:0] control_shifted = {{8{control_output_q16[31]}}, control_output_q16} << 8;
    wire control_overflow = (control_shifted[39:24] != {16{control_shifted[23]}});
    wire signed [23:0] control_output_24 = control_overflow ?
        (control_output_q16[31] ? 24'sh800000 : 24'sh7FFFFF) :
        control_shifted[23:0];

    wire signed [23:0] fifo_rd_data;
    wire fifo_empty;
    audio_fifo_simple #(
        .DATA_WIDTH (24),
        .DEPTH      (16),
        .ADDR_WIDTH (4)
    ) u_audio_fifo (
        .clk      (clock_50m),
        .rst_n    (reset_n),
        .wr_en    (ref_mic_valid),
        .wr_data  (ref_mic_data),
        .rd_en    (~fifo_empty),
        .rd_data  (fifo_rd_data),
        .empty    (fifo_empty),
        .full     ()
    );

    wire signed [31:0] passthrough_q16 = {{16{fifo_rd_data[23]}}, fifo_rd_data[23:8]};
    wire signed [39:0] passthrough_shifted = {{8{passthrough_q16[31]}}, passthrough_q16} << 8;
    wire passthrough_overflow = (passthrough_shifted[39:24] != {16{passthrough_shifted[23]}});
    wire signed [23:0] passthrough_24 = passthrough_overflow ?
        (passthrough_q16[31] ? 24'sh800000 : 24'sh7FFFFF) :
        passthrough_shifted[23:0];

    reg signed [23:0] speaker_out;
    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n)
            speaker_out <= 24'sd0;
        else if (btn_fxlms_enable_db || btn_estimate_mode_db) begin
            if (fxlms_processing_done) speaker_out <= control_output_24;
        end else if (btn_passthrough_db) begin
            if (!fifo_empty) speaker_out <= passthrough_24;
        end else begin
            speaker_out <= 24'sd0;
        end
    end

    // ===== Debouncers =====
    button_debounce #(.DEBOUNCE_CYCLES(`DEBOUNCE_CYCLES)) u_btn_fxlms_debounce (
        .clk(clock_50m), .rst_n(reset_n), .btn_in(~btn_fxlms_enable), .btn_out(btn_fxlms_enable_db));
    button_debounce #(.DEBOUNCE_CYCLES(`DEBOUNCE_CYCLES)) u_btn_passthrough_debounce (
        .clk(clock_50m), .rst_n(reset_n), .btn_in(~btn_passthrough), .btn_out(btn_passthrough_db));
    button_debounce #(.DEBOUNCE_CYCLES(`DEBOUNCE_CYCLES)) u_btn_estimate_mode_debounce (
        .clk(clock_50m), .rst_n(reset_n), .btn_in(~btn_estimate_mode), .btn_out(btn_estimate_mode_db));
    button_debounce #(.DEBOUNCE_CYCLES(`DEBOUNCE_CYCLES)) u_btn_switch_debounce (
        .clk(clock_50m), .rst_n(reset_n), .btn_in(~btn_switch), .btn_out(btn_switch_db));

    // ===== Codec + I2S =====
    reg_config u_reg_config_0 (.clock_50m(clock_50m), .reset_n(reset_n), .i2c_sclk(aud_scl), .i2c_sdat(aud_sda));
    reg_config_1 u_reg_config_1 (.clock_50m(clock_50m), .reset_n(reset_n), .i2c_sclk_1(aud_scl_1), .i2c_sdat_1(aud_sda_1));
    i2s_clk_gen u_i2s_clk_gen (.clock_50m(clock_50m), .rst_n(reset_n), .bclk(bclk_i), .lrclk(lrclk_i), .bclk_pos(bclk_pos), .bclk_neg(bclk_neg));
    i2s_adc_rx u_i2s_adc_rx_0 (.rst_n(reset_n), .bclk_neg(bclk_neg), .lrclk(lrclk_i), .adc_dat(adc_dat), .adc_left(error_mic_data), .adc_valid(error_mic_valid));
    i2s_adc_rx_1 u_i2s_adc_rx_1 (.rst_n(reset_n), .bclk_neg(bclk_neg), .lrclk(lrclk_i), .adc_dat_1(adc_dat_1), .adc_left_1(ref_mic_data), .adc_valid_1(ref_mic_valid));
    i2s_dac_tx u_i2s_dac_tx (.rst_n(reset_n), .bclk_neg(bclk_neg), .lrclk(lrclk_i), .dac_data(speaker_out), .dac_valid(1'b1), .dac_dat(dac_dat));

    // ===== AXI / DDR section =====
`ifdef SIM_DBG_MODE
    // In simulation debug mode, skip all AXI modules to avoid PS7 VIP X issues.
    // Tie off DDR and FIXED_IO ports.
    assign DDR_addr    = 15'bz;
    assign DDR_ba      = 3'bz;
    assign DDR_cas_n   = 1'bz;
    assign DDR_ck_n    = 1'bz;
    assign DDR_ck_p    = 1'bz;
    assign DDR_cke     = 1'bz;
    assign DDR_cs_n    = 1'bz;
    assign DDR_dm      = 4'bz;
    assign DDR_dq      = 32'bz;
    assign DDR_dqs_n   = 4'bz;
    assign DDR_dqs_p   = 4'bz;
    assign DDR_odt     = 1'bz;
    assign DDR_ras_n   = 1'bz;
    assign DDR_reset_n = 1'bz;
    assign DDR_we_n    = 1'bz;
    assign FIXED_IO_ddr_vrn  = 1'bz;
    assign FIXED_IO_ddr_vrp  = 1'bz;
    assign FIXED_IO_mio      = 54'bz;
    assign FIXED_IO_ps_clk   = 1'bz;
    assign FIXED_IO_ps_porb  = 1'bz;
    assign FIXED_IO_ps_srstb = 1'bz;
`else
    wire rst_n_axi;
    wire M_AXI_ACLK;
    wire [0:0]  M_AXI_AWID;
    wire [31:0] M_AXI_AWADDR;
    wire [7:0]  M_AXI_AWLEN;
    wire [2:0]  M_AXI_AWSIZE;
    wire [1:0]  M_AXI_AWBURST;
    wire        M_AXI_AWLOCK;
    wire [3:0]  M_AXI_AWCACHE;
    wire [2:0]  M_AXI_AWPROT;
    wire [3:0]  M_AXI_AWQOS;
    wire [0:0]  M_AXI_AWUSER;
    wire        M_AXI_AWVALID;
    wire        M_AXI_AWREADY;
    wire [63:0] M_AXI_WDATA;
    wire [7:0]  M_AXI_WSTRB;
    wire        M_AXI_WLAST;
    wire [0:0]  M_AXI_WUSER;
    wire        M_AXI_WVALID;
    wire        M_AXI_WREADY;
    wire [0:0]  M_AXI_BID;
    wire [1:0]  M_AXI_BRESP;
    wire [0:0]  M_AXI_BUSER;
    wire        M_AXI_BVALID;
    wire        M_AXI_BREADY;

    reg signed [31:0] y_n_q16_sync1, y_n_q16_sync2;
    reg signed [31:0] e_n_q16_sync1, e_n_q16_sync2;
    reg sample_valid_sync1, sample_valid_sync2;
    wire sample_valid_sync_axi = sample_valid_sync2;

    always @(posedge M_AXI_ACLK or negedge rst_n_axi) begin
        if (!rst_n_axi) begin
            y_n_q16_sync1 <= 32'sd0; y_n_q16_sync2 <= 32'sd0;
            e_n_q16_sync1 <= 32'sd0; e_n_q16_sync2 <= 32'sd0;
            sample_valid_sync1 <= 1'b0; sample_valid_sync2 <= 1'b0;
        end else begin
            y_n_q16_sync1 <= control_output_q16;
            e_n_q16_sync1 <= e_n_q16_delayed;
            sample_valid_sync1 <= sample_valid_delayed && btn_fxlms_enable_db;
            y_n_q16_sync2 <= y_n_q16_sync1;
            e_n_q16_sync2 <= e_n_q16_sync1;
            sample_valid_sync2 <= sample_valid_sync1;
        end
    end

    wire wr_burst_data_req;
    wire wr_burst_finish;
    wire wr_burst_req;
    wire [9:0]  wr_burst_len;
    wire [31:0] wr_burst_addr;
    wire [63:0] wr_burst_data = {y_n_q16_sync2, e_n_q16_sync2};

    // ===== DDR writer =====
    mem_test #(.MEM_DATA_BITS(64), .ADDR_BITS(32)) u_mem_test (
        .rst(~rst_n_axi),
        .mem_clk(M_AXI_ACLK),
        .wr_burst_req(wr_burst_req),
        .wr_burst_len(wr_burst_len),
        .wr_burst_addr(wr_burst_addr),
        .y_n_sync(y_n_q16_sync2),
        .e_n_sync(e_n_q16_sync2),
        .sample_valid_sync(sample_valid_sync_axi),
        .wr_burst_data_req(wr_burst_data_req),
        .wr_burst_data(wr_burst_data),
        .wr_burst_finish(wr_burst_finish)
    );

    // ===== Fixed aq_axi_master read-channel hookup for simulation =====
    aq_axi_master u_aq_axi_master (
        .ARESETN(rst_n_axi),
        .ACLK(M_AXI_ACLK),
        .M_AXI_AWID(M_AXI_AWID), .M_AXI_AWADDR(M_AXI_AWADDR), .M_AXI_AWLEN(M_AXI_AWLEN),
        .M_AXI_AWSIZE(M_AXI_AWSIZE), .M_AXI_AWBURST(M_AXI_AWBURST), .M_AXI_AWLOCK(M_AXI_AWLOCK),
        .M_AXI_AWCACHE(M_AXI_AWCACHE), .M_AXI_AWPROT(M_AXI_AWPROT), .M_AXI_AWQOS(M_AXI_AWQOS),
        .M_AXI_AWUSER(M_AXI_AWUSER), .M_AXI_AWVALID(M_AXI_AWVALID), .M_AXI_AWREADY(M_AXI_AWREADY),
        .M_AXI_WDATA(M_AXI_WDATA), .M_AXI_WSTRB(M_AXI_WSTRB), .M_AXI_WLAST(M_AXI_WLAST),
        .M_AXI_WUSER(M_AXI_WUSER), .M_AXI_WVALID(M_AXI_WVALID), .M_AXI_WREADY(M_AXI_WREADY),
        .M_AXI_BID(M_AXI_BID), .M_AXI_BRESP(M_AXI_BRESP), .M_AXI_BUSER(M_AXI_BUSER),
        .M_AXI_BVALID(M_AXI_BVALID), .M_AXI_BREADY(M_AXI_BREADY),

        .M_AXI_ARID(), .M_AXI_ARADDR(), .M_AXI_ARLEN(), .M_AXI_ARSIZE(), .M_AXI_ARBURST(),
        .M_AXI_ARLOCK(), .M_AXI_ARCACHE(), .M_AXI_ARPROT(), .M_AXI_ARQOS(), .M_AXI_ARUSER(),
        .M_AXI_ARVALID(), .M_AXI_ARREADY(1'b0),
        .M_AXI_RID(1'b0), .M_AXI_RDATA(64'd0), .M_AXI_RRESP(2'd0),
        .M_AXI_RLAST(1'b0), .M_AXI_RUSER(1'b0), .M_AXI_RVALID(1'b0), .M_AXI_RREADY(),

        .MASTER_RST(~rst_n_axi),
        .WR_START(wr_burst_req),
        .WR_ADRS({wr_burst_addr[28:0],3'd0}),
        .WR_LEN(wr_burst_len),
        .WR_READY(),
        .WR_FIFO_RE(wr_burst_data_req),
        .WR_FIFO_EMPTY(1'b0),
        .WR_FIFO_AEMPTY(1'b0),
        .WR_FIFO_DATA(wr_burst_data),
        .WR_DONE(wr_burst_finish),
        .RD_START(1'b0),
        .RD_ADRS(32'd0),
        .RD_LEN(32'd0),
        .RD_READY(),
        .RD_FIFO_WE(),
        .RD_FIFO_FULL(1'b0),
        .RD_FIFO_AFULL(1'b0),
        .RD_FIFO_DATA(),
        .RD_DONE(),
        .DEBUG()
    );

    ANC_wrapper u_ps_block (
        .DDR_addr(DDR_addr), .DDR_ba(DDR_ba), .DDR_cas_n(DDR_cas_n), .DDR_ck_n(DDR_ck_n), .DDR_ck_p(DDR_ck_p),
        .DDR_cke(DDR_cke), .DDR_cs_n(DDR_cs_n), .DDR_dm(DDR_dm), .DDR_dq(DDR_dq), .DDR_dqs_n(DDR_dqs_n),
        .DDR_dqs_p(DDR_dqs_p), .DDR_odt(DDR_odt), .DDR_ras_n(DDR_ras_n), .DDR_reset_n(DDR_reset_n), .DDR_we_n(DDR_we_n),
        .FIXED_IO_ddr_vrn(FIXED_IO_ddr_vrn), .FIXED_IO_ddr_vrp(FIXED_IO_ddr_vrp), .FIXED_IO_mio(FIXED_IO_mio),
        .FIXED_IO_ps_clk(FIXED_IO_ps_clk), .FIXED_IO_ps_porb(FIXED_IO_ps_porb), .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),
        .S00_AXI_awaddr(M_AXI_AWADDR), .S00_AXI_awburst(M_AXI_AWBURST), .S00_AXI_awcache(M_AXI_AWCACHE),
        .S00_AXI_awid(M_AXI_AWID), .S00_AXI_awlen(M_AXI_AWLEN), .S00_AXI_awlock(M_AXI_AWLOCK),
        .S00_AXI_awprot(M_AXI_AWPROT), .S00_AXI_awqos(M_AXI_AWQOS), .S00_AXI_awready(M_AXI_AWREADY),
        .S00_AXI_awregion(4'b0000), .S00_AXI_awsize(M_AXI_AWSIZE), .S00_AXI_awvalid(M_AXI_AWVALID),
        .S00_AXI_bid(M_AXI_BID), .S00_AXI_bready(M_AXI_BREADY), .S00_AXI_bresp(M_AXI_BRESP), .S00_AXI_bvalid(M_AXI_BVALID),
        .S00_AXI_wdata(M_AXI_WDATA), .S00_AXI_wlast(M_AXI_WLAST), .S00_AXI_wready(M_AXI_WREADY),
        .S00_AXI_wstrb(M_AXI_WSTRB), .S00_AXI_wvalid(M_AXI_WVALID),
        .S00_AXI_araddr(32'd0), .S00_AXI_arburst(2'd0), .S00_AXI_arcache(4'd0), .S00_AXI_arid(1'd0),
        .S00_AXI_arlen(8'd0), .S00_AXI_arlock(2'd0), .S00_AXI_arprot(3'd0), .S00_AXI_arqos(4'd0),
        .S00_AXI_arready(), .S00_AXI_arregion(4'b0000), .S00_AXI_arsize(3'd0), .S00_AXI_arvalid(1'd0),
        .S00_AXI_rdata(), .S00_AXI_rid(), .S00_AXI_rlast(), .S00_AXI_rready(1'd0),
        .S00_AXI_rresp(), .S00_AXI_rvalid(),
        .axim_rst_n(rst_n_axi), .FCLK_CLK0(M_AXI_ACLK), .axi_hp_clk(M_AXI_ACLK)
    );
`endif // SIM_DBG_MODE

endmodule
