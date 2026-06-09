`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/03/13
// Module Name: sine_capture_top
// Project Name: ANC Data Collection
// Description: Sine wave generator + microphone capture with AXI DDR data write
//              - yn (x_n_sync to mem_test): Q16.16 sine wave played through speaker
//              - en (e_n_sync to mem_test): Error microphone (Codec 0) capture
//              - AXI: Writes {yn, en} Q16.16 pairs to DDR
//
//              Clock domains:
//                lrclk_i (48kHz) : sine_generator output (sine_out)
//                clock_50m (50MHz): main logic; sine_out synced here via 2-stage FF
//                M_AXI_ACLK     : AXI HP port; yn/en synced here via 2-stage FF
//
// Frequency word for sine_generator (sample rate = 48kHz):
//   freq_word = round(freq_hz / 48000 * 2^32)
//   500 Hz -> 32'd44_739_243  (0x02AA_AAAB)  [active]
//   1 kHz  -> 32'd89_478_485  (0x0555_5555)
//
// Amplitude note (Q16.16):
//   SINE_AMPLITUDE = 32'h0000_4E20 => 0.3052 (20000/65536)
//   => max sine_out = 20000 (raw integer units)
//   => fits in 24-bit signed. No saturation needed.
//////////////////////////////////////////////////////////////////////////////////

module sine_capture_top (
    input  wire clock_50m,

    // ================= Control Buttons =================
    input  wire btn_fxlms_enable,     // Button A: Enable FxLMS mode with weight update (active-low)
    input  wire btn_passthrough,      // Button B: Enable pass-through mode (active-low)
    input  wire btn_estimate_mode,    // Button C: Enable secondary path estimation mode (active-low)
    input  wire btn_switch,

    // ================= LED Status Indicators =================
    output wire led_1st,      
    output wire led_2nd, 
    output wire led_3rd, 
    output wire led_4th,  

    // ================= I2C =================
    output wire aud_scl,
    inout  wire aud_sda,

    output wire aud_scl_1,
    inout  wire aud_sda_1,

    // ================= I2S : Codec 0 (Error Microphone) =================
    output wire bclk,
    output wire adclrc,
    output wire daclrc,
    input  wire adc_dat,        // Error microphone input

    // ================= I2S : Codec 1 (Reference Microphone) =================
    output wire bclk_1,
    output wire adclrc_1,
    input  wire adc_dat_1,      // Reference microphone input

    // ================= DAC (Speaker Output) =================
    output wire dac_dat,          // Speaker output (anti-noise signal)
    
    // ================= DDR=================
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

    // =========================================================
    // I2S clocks: generated uniformly inside FPGA
    // =========================================================
    wire bclk_i;
    wire lrclk_i;
    wire bclk_pos;
    wire bclk_neg;

    assign bclk     = bclk_i;
    assign bclk_1   = bclk_i;
    assign adclrc   = lrclk_i;
    assign daclrc   = lrclk_i;
    assign adclrc_1 = lrclk_i;

    // =========================================================
    // ADC data from dual microphones
    // =========================================================
    wire signed [23:0] ref_mic_data;    // Codec 1 - captured but not used for AXI
    wire               ref_mic_valid;

    wire signed [23:0] error_mic_data;  // Codec 0 - en signal
    wire               error_mic_valid;

    // =========================================================
    // Button debouncing
    // =========================================================
    wire btn_fxlms_enable_db;
    wire btn_passthrough_db;
    wire btn_estimate_mode_db;

    // =========================================================
    // Format conversion: error mic 24-bit -> Q16.16 (en)
    // =========================================================
    wire signed [31:0] error_mic_q16;
    assign error_mic_q16 = {{16{error_mic_data[23]}}, error_mic_data[23:8]};

    // =========================================================
    // Sine Wave Generator (lrclk domain = 48 kHz)
    // yn = sine wave output, sent to speaker and AXI
    //
    // SINE_FREQ_WORD = round(500 / 48000 * 2^32) = 44_739_243
    // SINE_AMPLITUDE = 32'h0000_00C8 => 200/65536 = 0.003052 (1/100 of original 20000)
    // =========================================================
    localparam [31:0]        SINE_FREQ_WORD = 32'd44_739_243;
    localparam signed [31:0] SINE_AMPLITUDE = 32'h0000_00C8;

    // sine_out is in the lrclk (48 kHz) clock domain
    wire signed [31:0] sine_out;

    sine_generator #(
        .DATA_WIDTH  (32),
        .PHASE_WIDTH (32),
        .FREQ_DIVIDER(1)
    ) u_sine_gen (
        .sample_clk (lrclk_i),
        .rst_n      (reset_n),
        .enable     (1'b1),
        .freq_word  (SINE_FREQ_WORD),
        .amplitude  (SINE_AMPLITUDE),
        .sine_out   (sine_out)
    );
    wire reset_n = 1'b1;
    // =========================================================
    // CDC: lrclk domain -> clock_50m domain (2-stage synchronizer)
    // sine_out changes at 48 kHz; sampled >1000x per period at 50 MHz.
    // =========================================================
    reg signed [31:0] sine_50m_s1, sine_50m_s2;

    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n) begin
            sine_50m_s1 <= 32'sd0;
            sine_50m_s2 <= 32'sd0;
        end else begin
            sine_50m_s1 <= sine_out;
            sine_50m_s2 <= sine_50m_s1;
        end
    end

    // =========================================================
    // Convert Q16.16 sine (clock_50m domain) -> 24-bit audio for DAC
    // With SINE_AMPLITUDE = 127.0, max absolute value = 8_323_072 < 2^23,
    // so sine_50m_s2[23:0] is always a valid 24-bit signed sample.
    // Saturation guard is kept for safety if amplitude is changed.
    // =========================================================
    wire sine_overflow;
    assign sine_overflow = (sine_50m_s2[31:23] != 9'b000000000) &&
                           (sine_50m_s2[31:23] != 9'b111111111);

    wire signed [23:0] sine_24bit;
    assign sine_24bit = sine_overflow ?
        (sine_50m_s2[31] ? 24'sh800000 : 24'sh7FFFFF) :
        sine_50m_s2[23:0];

    // =========================================================
    // Speaker output register (clock_50m domain)
    // =========================================================
    reg signed [23:0] speaker_out;

    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n)
            speaker_out <= 24'sd0;
        else
            speaker_out <= sine_24bit;
    end

    // =========================================================
    // yn in Q16.16 for AXI (sine wave, clock_50m domain)
    // =========================================================
    wire signed [31:0] yn_q16;
    assign yn_q16 = sine_50m_s2;

    // =========================================================
    // LED Status Indicators
    // =========================================================
    assign led_fxlms_status      = btn_fxlms_enable_db;
    assign led_passthrough_status = btn_passthrough_db;
    assign led_estimate_status   = btn_estimate_mode_db;

    // ========== AXI write channel signals ==========
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

    // ========== 2-cycle delay for error_mic_valid in 50MHz domain ==========
    reg error_mic_valid_d1, error_mic_valid_d2;
    always @(posedge clock_50m or negedge reset_n) begin
        if (!reset_n) begin
            error_mic_valid_d1 <= 1'b0;
            error_mic_valid_d2 <= 1'b0;
        end else begin
            error_mic_valid_d1 <= error_mic_valid;
            error_mic_valid_d2 <= error_mic_valid_d1;
        end
    end

    // ========== CDC sync: clock_50m -> AXI clock domain ==========
    reg signed [31:0] y_n_q16_sync1, y_n_q16_sync2;   // yn: sine wave
    reg signed [31:0] e_n_q16_sync1, e_n_q16_sync2;   // en: error mic
    reg sample_valid_sync1, sample_valid_sync2;
    wire sample_valid_sync_axi;

    always @(posedge M_AXI_ACLK or negedge rst_n_axi) begin
        if (!rst_n_axi) begin
            y_n_q16_sync1      <= 32'sd0;
            y_n_q16_sync2      <= 32'sd0;
            e_n_q16_sync1      <= 32'sd0;
            e_n_q16_sync2      <= 32'sd0;
            sample_valid_sync1 <= 1'b0;
            sample_valid_sync2 <= 1'b0;
        end else begin
            y_n_q16_sync1      <= yn_q16;
            y_n_q16_sync2      <= y_n_q16_sync1;
            e_n_q16_sync1      <= error_mic_q16;
            e_n_q16_sync2      <= e_n_q16_sync1;
            sample_valid_sync1 <= error_mic_valid_d2;
            sample_valid_sync2 <= sample_valid_sync1;
        end
    end
    assign sample_valid_sync_axi = sample_valid_sync2;

    // ========== AXI burst control wires (driven by mem_test) ==========
    wire wr_burst_data_req;
    wire wr_burst_finish;
    wire wr_burst_req;
    wire [9:0]  wr_burst_len;
    wire [31:0] wr_burst_addr;
    wire [63:0] wr_burst_data;

    // =========================================================
    // Button debounce modules
    // =========================================================
    button_debounce #(
        .DEBOUNCE_CYCLES(250_000)   // 5 ms @ 50 MHz
    ) u_btn_fxlms_debounce (
        .clk    (clock_50m),
        .rst_n  (reset_n),
        .btn_in (~btn_fxlms_enable),
        .btn_out(btn_fxlms_enable_db)
    );

    button_debounce #(
        .DEBOUNCE_CYCLES(250_000)
    ) u_btn_passthrough_debounce (
        .clk    (clock_50m),
        .rst_n  (reset_n),
        .btn_in (~btn_passthrough),
        .btn_out(btn_passthrough_db)
    );

    button_debounce #(
        .DEBOUNCE_CYCLES(250_000)
    ) u_btn_estimate_mode_debounce (
        .clk    (clock_50m),
        .rst_n  (reset_n),
        .btn_in (~btn_estimate_mode),
        .btn_out(btn_estimate_mode_db)
    );

    // =========================================================
    // I2C configuration: configure both WM8731 codecs
    // =========================================================
    reg_config u_reg_config_0 (
        .clock_50m(clock_50m),
        .reset_n  (reset_n),
        .i2c_sclk (aud_scl),
        .i2c_sdat (aud_sda)
    );

    reg_config_1 u_reg_config_1 (
        .clock_50m  (clock_50m),
        .reset_n    (reset_n),
        .i2c_sclk_1 (aud_scl_1),
        .i2c_sdat_1 (aud_sda_1)
    );

    // =========================================================
    // I2S clock generation (unified inside FPGA)
    // =========================================================
    i2s_clk_gen u_i2s_clk_gen (
        .clock_50m(clock_50m),
        .rst_n    (reset_n),
        .bclk     (bclk_i),
        .lrclk    (lrclk_i),
        .bclk_pos (bclk_pos),
        .bclk_neg (bclk_neg)
    );

    // =========================================================
    // ADC RX (Error Microphone - Mic 0) -> en
    // =========================================================
    i2s_adc_rx u_i2s_adc_rx_0 (
        .rst_n    (reset_n),
        .bclk_neg (bclk_neg),
        .lrclk    (lrclk_i),
        .adc_dat  (adc_dat),
        .adc_left (error_mic_data),
        .adc_valid(error_mic_valid)
    );

    // =========================================================
    // ADC RX (Reference Microphone - Mic 1, hardware present, not used for AXI)
    // =========================================================
    i2s_adc_rx_1 u_i2s_adc_rx_1 (
        .rst_n      (reset_n),
        .bclk_neg   (bclk_neg),
        .lrclk      (lrclk_i),
        .adc_dat_1  (adc_dat_1),
        .adc_left_1 (ref_mic_data),
        .adc_valid_1(ref_mic_valid)
    );

    // =========================================================
    // DAC TX (Speaker Output - plays sine wave yn)
    // =========================================================
    i2s_dac_tx u_i2s_dac_tx (
        .rst_n    (reset_n),
        .bclk_neg (bclk_neg),
        .lrclk    (lrclk_i),
        .dac_data (speaker_out),
        .dac_valid(1'b1),
        .dac_dat  (dac_dat)
    );

    // =========================================================
    // mem_test: generates AXI burst write requests
    // Packs {yn, en} Q16.16 pairs into 64-bit burst data
    // =========================================================
    mem_test #(
        .MEM_DATA_BITS(64),
        .ADDR_BITS    (32)
    ) u_mem_test (
        .rst              (~rst_n_axi),
        .mem_clk          (M_AXI_ACLK),
        .wr_burst_req     (wr_burst_req),
        .wr_burst_len     (wr_burst_len),
        .wr_burst_addr    (wr_burst_addr),
        .y_n_sync         (y_n_q16_sync2),      // yn: sine wave (Q16.16)
        .e_n_sync         (e_n_q16_sync2),      // en: error mic (Q16.16)
        .sample_valid_sync(sample_valid_sync_axi),
        .wr_burst_data_req(wr_burst_data_req),
        .wr_burst_data    (wr_burst_data),
        .wr_burst_finish  (wr_burst_finish)
    );

    // =========================================================
    // aq_axi_master: AXI write protocol bridge
    // =========================================================
    aq_axi_master u_aq_axi_master (
        .ARESETN(rst_n_axi),
        .ACLK   (M_AXI_ACLK),

        .M_AXI_AWID   (M_AXI_AWID),
        .M_AXI_AWADDR (M_AXI_AWADDR),
        .M_AXI_AWLEN  (M_AXI_AWLEN),
        .M_AXI_AWSIZE (M_AXI_AWSIZE),
        .M_AXI_AWBURST(M_AXI_AWBURST),
        .M_AXI_AWLOCK (M_AXI_AWLOCK),
        .M_AXI_AWCACHE(M_AXI_AWCACHE),
        .M_AXI_AWPROT (M_AXI_AWPROT),
        .M_AXI_AWQOS  (M_AXI_AWQOS),
        .M_AXI_AWUSER (M_AXI_AWUSER),
        .M_AXI_AWVALID(M_AXI_AWVALID),
        .M_AXI_AWREADY(M_AXI_AWREADY),

        .M_AXI_WDATA (M_AXI_WDATA),
        .M_AXI_WSTRB (M_AXI_WSTRB),
        .M_AXI_WLAST (M_AXI_WLAST),
        .M_AXI_WUSER (M_AXI_WUSER),
        .M_AXI_WVALID(M_AXI_WVALID),
        .M_AXI_WREADY(M_AXI_WREADY),

        .M_AXI_BID   (M_AXI_BID),
        .M_AXI_BRESP (M_AXI_BRESP),
        .M_AXI_BUSER (M_AXI_BUSER),
        .M_AXI_BVALID(M_AXI_BVALID),
        .M_AXI_BREADY(M_AXI_BREADY),

        // Read channel: unused
        .M_AXI_ARID   (1'b0),
        .M_AXI_ARADDR (32'd0),
        .M_AXI_ARLEN  (8'd0),
        .M_AXI_ARSIZE (3'd0),
        .M_AXI_ARBURST(2'd0),
        .M_AXI_ARLOCK (1'b0),
        .M_AXI_ARCACHE(4'd0),
        .M_AXI_ARPROT (3'd0),
        .M_AXI_ARQOS  (4'd0),
        .M_AXI_ARUSER (1'b0),
        .M_AXI_ARVALID(1'b0),
        .M_AXI_ARREADY(),
        .M_AXI_RID    (),
        .M_AXI_RDATA  (),
        .M_AXI_RRESP  (),
        .M_AXI_RLAST  (),
        .M_AXI_RUSER  (),
        .M_AXI_RVALID (),
        .M_AXI_RREADY (1'b0),

        .MASTER_RST    (~rst_n_axi),
        .WR_START      (wr_burst_req),
        .WR_ADRS       ({wr_burst_addr[28:0], 3'd0}),
        .WR_LEN        (wr_burst_len),
        .WR_READY      (),
        .WR_FIFO_RE    (wr_burst_data_req),
        .WR_FIFO_EMPTY (1'b0),
        .WR_FIFO_AEMPTY(1'b0),
        .WR_FIFO_DATA  (wr_burst_data),
        .WR_DONE       (wr_burst_finish),

        // Read control: unused
        .RD_START    (1'b0),
        .RD_ADRS     (32'd0),
        .RD_LEN      (32'd0),
        .RD_READY    (),
        .RD_FIFO_WE  (),
        .RD_FIFO_FULL (1'b0),
        .RD_FIFO_AFULL(1'b0),
        .RD_FIFO_DATA (),
        .RD_DONE     (),
        .DEBUG       ()
    );

    // =========================================================
    // Zynq PS block (ANC_wrapper) - provides AXI clock and DDR interface
    // =========================================================
    ANC_wrapper u_ps_block (
        .DDR_addr   (DDR_addr),
        .DDR_ba     (DDR_ba),
        .DDR_cas_n  (DDR_cas_n),
        .DDR_ck_n   (DDR_ck_n),
        .DDR_ck_p   (DDR_ck_p),
        .DDR_cke    (DDR_cke),
        .DDR_cs_n   (DDR_cs_n),
        .DDR_dm     (DDR_dm),
        .DDR_dq     (DDR_dq),
        .DDR_dqs_n  (DDR_dqs_n),
        .DDR_dqs_p  (DDR_dqs_p),
        .DDR_odt    (DDR_odt),
        .DDR_ras_n  (DDR_ras_n),
        .DDR_reset_n(DDR_reset_n),
        .DDR_we_n   (DDR_we_n),
        .FIXED_IO_ddr_vrn (FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp (FIXED_IO_ddr_vrp),
        .FIXED_IO_mio     (FIXED_IO_mio),
        .FIXED_IO_ps_clk  (FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb (FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb(FIXED_IO_ps_srstb),

        // AXI write channel
        .S00_AXI_awaddr  (M_AXI_AWADDR),
        .S00_AXI_awburst (M_AXI_AWBURST),
        .S00_AXI_awcache (M_AXI_AWCACHE),
        .S00_AXI_awid    (M_AXI_AWID),
        .S00_AXI_awlen   (M_AXI_AWLEN),
        .S00_AXI_awlock  (M_AXI_AWLOCK),
        .S00_AXI_awprot  (M_AXI_AWPROT),
        .S00_AXI_awqos   (M_AXI_AWQOS),
        .S00_AXI_awready (M_AXI_AWREADY),
        .S00_AXI_awregion(4'b0000),
        .S00_AXI_awsize  (M_AXI_AWSIZE),
        .S00_AXI_awvalid (M_AXI_AWVALID),
        .S00_AXI_bid     (M_AXI_BID),
        .S00_AXI_bready  (M_AXI_BREADY),
        .S00_AXI_bresp   (M_AXI_BRESP),
        .S00_AXI_bvalid  (M_AXI_BVALID),
        .S00_AXI_wdata   (M_AXI_WDATA),
        .S00_AXI_wlast   (M_AXI_WLAST),
        .S00_AXI_wready  (M_AXI_WREADY),
        .S00_AXI_wstrb   (M_AXI_WSTRB),
        .S00_AXI_wvalid  (M_AXI_WVALID),

        // AXI read channel: unused
        .S00_AXI_araddr  (32'd0),
        .S00_AXI_arburst (2'd0),
        .S00_AXI_arcache (4'd0),
        .S00_AXI_arid    (1'd0),
        .S00_AXI_arlen   (8'd0),
        .S00_AXI_arlock  (2'd0),
        .S00_AXI_arprot  (3'd0),
        .S00_AXI_arqos   (4'd0),
        .S00_AXI_arready (),
        .S00_AXI_arregion(4'b0000),
        .S00_AXI_arsize  (3'd0),
        .S00_AXI_arvalid (1'd0),
        .S00_AXI_rdata   (64'd0),
        .S00_AXI_rid     (1'd0),
        .S00_AXI_rlast   (1'd0),
        .S00_AXI_rready  (1'd0),
        .S00_AXI_rresp   (2'd0),
        .S00_AXI_rvalid  (1'd0),

        // Clock and reset outputs to PL
        .axim_rst_n(rst_n_axi),
        .FCLK_CLK0 (M_AXI_ACLK),
        .axi_hp_clk(M_AXI_ACLK)
    );

endmodule
