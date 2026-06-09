`timescale 1ns/1ps
`include "fxlms_params.vh"

// =============================================================================
// File: fxlms_core.v
// Description: FxLMS Algorithm Core Module with Secondary Path Estimation
//
// Operation Modes:
//   - estimate_mode = 1: Secondary path estimation mode
//     * Injects white noise to speaker
//     * Updates S'(z) coefficients using LMS (reuses lms_update module)
//     * Typically activated by a button/switch
//   - estimate_mode = 0, adapt_enable = 1: Normal FxLMS adaptive mode
//   - estimate_mode = 0, adapt_enable = 0: Fixed filter mode
// =============================================================================
module fxlms_core #(
    parameter DATA_WIDTH   = `DATA_WIDTH,
    parameter FILTER_ORDER = `FILTER_ORDER,
    parameter MU_WIDTH     = `MU_WIDTH
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         enable,
    input  wire                         adapt_enable,      // Adaptation enable
    input  wire                         estimate_mode,     // Secondary path estimation mode
    input  wire                         weight_reset,      // Reset all W filter weights to zero
    input  wire                         sample_valid,      // Sample valid flag
    input  wire signed [MU_WIDTH-1:0]   mu_in,             // W filter step size (runtime configurable)
    input  wire signed [MU_WIDTH-1:0]   mu_s_in,           // S' filter step size (runtime configurable)
    input  wire [7:0]                   w_init_delay,      // W filter initial delay
    input  wire [7:0]                   s_init_delay,      // S' filter initial delay
    input  wire signed [DATA_WIDTH-1:0] ref_signal,        // x(n) Reference signal
    input  wire signed [DATA_WIDTH-1:0] error_signal,      // e(n) Error signal
    input  wire signed [DATA_WIDTH-1:0] white_noise_in,    // w(n) White noise for estimation
    output reg  signed [DATA_WIDTH-1:0] control_output,    // y(n) Control output
    output reg  signed [DATA_WIDTH-1:0] PROC_out,          // Actual output to speaker (inverted or white noise)
    output wire                         processing_done,
    output wire                         estimation_active,  // Indicates estimation mode is running

    // Debug: S' coefficient readback
    input  wire [$clog2(`FILTER_ORDER_S)-1:0]  dbg_s_coeff_addr,
    output wire signed [DATA_WIDTH-1:0]        dbg_s_coeff_data
);

    // State machine states
    localparam IDLE       = 3'd0;
    localparam FILTER_W   = 3'd1;
    localparam FILTER_S   = 3'd2;
    localparam UPDATE_W   = 3'd3;
    localparam DONE       = 3'd4;
    // States for secondary path estimation
    localparam EST_FILTER_S = 3'd5;  // Filter white noise through S' during estimation
    localparam EST_UPDATE_S = 3'd6;  // Update S' coefficients

    reg [2:0] state, next_state;

    // W filter signals
    wire signed [DATA_WIDTH-1:0] w_output;
    wire w_done;
    reg w_start;

    // S' filter signals
    wire signed [DATA_WIDTH-1:0] s_output;
    wire s_done;
    reg s_start;

    // LMS update signals (for W filter)
    reg lms_start;
    wire lms_done;
    wire lms_coeff_we;
    wire [$clog2(FILTER_ORDER)-1:0] lms_coeff_addr;
    wire signed [DATA_WIDTH-1:0] lms_coeff_dout;

    // Secondary path estimation LMS signals
    reg est_lms_start;
    wire est_lms_done;
    wire est_lms_coeff_we;
    wire [$clog2(FILTER_ORDER)-1:0] est_lms_coeff_addr;
    wire signed [DATA_WIDTH-1:0] est_lms_coeff_dout;

    // Shared delay line between secondary_path_filter and secondary_lms_update
    wire [$clog2(`FILTER_ORDER_S)-1:0] est_delay_read_addr;
    wire signed [DATA_WIDTH-1:0]       est_delay_read_data;

    // Registers
    reg signed [DATA_WIDTH-1:0] ref_reg;
    reg signed [DATA_WIDTH-1:0] error_reg;
    reg signed [DATA_WIDTH-1:0] filtered_x_reg;
    reg signed [DATA_WIDTH-1:0] white_noise_reg;
    reg signed [DATA_WIDTH-1:0] estimation_error_reg;  // e_s(n) for S' estimation

    // Pending flags to delay LMS start by 1 cycle
    reg adapt_pending;
    reg est_pending;

    // Sample tick signal for W filter LMS delay line synchronization
    reg sample_tick;

    // State machine - current state register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    // State machine - combinational logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (sample_valid && enable) begin
                    if (estimate_mode)
                        next_state = EST_FILTER_S;  // Go to estimation path
                    else
                        next_state = FILTER_W;      // Normal FxLMS path
                end
            end
            // Normal FxLMS states
            FILTER_W: begin
                if (w_done)
                    next_state = FILTER_S;
            end
            FILTER_S: begin
                if (s_done)
                    next_state = adapt_enable ? UPDATE_W : DONE;
            end
            UPDATE_W: begin
                if (lms_done)
                    next_state = DONE;
            end
            // Estimation mode states
            EST_FILTER_S: begin
                if (s_done)
                    next_state = EST_UPDATE_S;
            end
            EST_UPDATE_S: begin
                if (est_lms_done)
                    next_state = DONE;
            end
            DONE: begin
                next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end

    // State machine - output logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_start <= 1'b0;
            s_start <= 1'b0;
            lms_start <= 1'b0;
            est_lms_start <= 1'b0;
            sample_tick <= 1'b0;
            ref_reg <= 0;
            error_reg <= 0;
            filtered_x_reg <= 0;
            white_noise_reg <= 0;
            estimation_error_reg <= 0;
            control_output <= 0;
            PROC_out <= 0;
            adapt_pending <= 1'b0;
            est_pending <= 1'b0;

        end else begin
            w_start <= 1'b0;
            s_start <= 1'b0;
            lms_start <= 1'b0;
            est_lms_start <= 1'b0;
            sample_tick <= 1'b0;

            case (state)
                IDLE: begin
                    // Clear pending when idle
                    adapt_pending <= 1'b0;
                    est_pending <= 1'b0;

                    if (sample_valid && enable) begin
                        ref_reg <= ref_signal;
                        error_reg <= error_signal;
                        white_noise_reg <= white_noise_in;

                        if (estimate_mode) begin
                            // Estimation mode: filter white noise through S'
                            s_start <= 1'b1;
                            // Output white noise to speaker for estimation
                            control_output <= white_noise_in;
                            PROC_out <= -white_noise_in;  // Direct white noise output
                        end else begin
                            // Normal FxLMS mode
                            w_start <= 1'b1;
                        end
                    end
                end

                // Normal FxLMS states
                FILTER_W: begin
                    if (w_done) begin
                        control_output <= w_output;
                        PROC_out <= w_output;  // Direct output to speaker (no inversion)
                        s_start <= 1'b1;
                    end
                end

                FILTER_S: begin
                    if (s_done) begin
                        // 1) Lock in x'(n)
                        filtered_x_reg <= s_output;

                        // 2) Generate sample_tick for LMS delay line update
                        sample_tick <= 1'b1;

                        // 3) Set pending flag for LMS start
                        if (adapt_enable)
                            adapt_pending <= 1'b1;
                    end
                end

                UPDATE_W: begin
                    // Fire LMS start ONE cycle after FILTER_S done
                    if (adapt_pending) begin
                        lms_start <= 1'b1;
                        adapt_pending <= 1'b0;
                    end
                end

                // Estimation mode states
                EST_FILTER_S: begin
                    if (s_done) begin
                        // S' output: predicted response to white noise
                        // estimation_error = actual mic signal - S' * w(n)
                        estimation_error_reg <= error_reg - s_output;

                        est_pending <= 1'b1;
                    end
                end

                EST_UPDATE_S: begin
                    if (est_pending) begin
                        est_lms_start <= 1'b1;
                        est_pending <= 1'b0;
                    end
                end

                default: ;
            endcase
        end
    end

    assign processing_done = (state == IDLE);
    assign estimation_active = estimate_mode;

    // Instantiate W(z) control filter
    fir_filter #(
        .DATA_WIDTH(DATA_WIDTH),
        .FILTER_ORDER(FILTER_ORDER),
        .IS_ADAPTIVE(1)
    ) u_w_filter (
        .clk(clk),
        .rst_n(rst_n),
        .start(w_start),
        .data_in(ref_reg),
        .w_init_delay(w_init_delay),
        .coeff_we(lms_coeff_we),
        .coeff_addr(lms_coeff_addr),
        .coeff_din(lms_coeff_dout),
        .coeff_reset(weight_reset),
        .data_out(w_output),
        .done(w_done)
    );

    // Instantiate S'(z) secondary path estimation filter
    // In estimation mode, input is white noise; in normal mode, input is reference signal
    wire signed [DATA_WIDTH-1:0] s_filter_input;
    assign s_filter_input = estimate_mode ? white_noise_reg : ref_reg;

    secondary_path_filter #(
        .DATA_WIDTH(DATA_WIDTH),
        .FILTER_ORDER(`FILTER_ORDER_S),  // Use S-specific order from header
        .FRAC_BITS(`FRAC_BITS)
    ) u_s_filter (
        .clk(clk),
        .rst_n(rst_n),
        .start(s_start),
        .data_in(s_filter_input),
        .s_init_delay(s_init_delay),
        .data_out(s_output),
        .done(s_done),
        // Coefficient write interface from estimation LMS
        .coeff_we(est_lms_coeff_we),
        .coeff_addr(est_lms_coeff_addr),
        .coeff_din(est_lms_coeff_dout),
        // Shared delay line read interface (used by secondary_lms_update)
        .delay_read_addr_ext(est_delay_read_addr),
        .delay_read_data_ext(est_delay_read_data),
        .delay_write_ptr_out(),
        // Debug: coefficient readback
        .dbg_coeff_rd_addr(dbg_s_coeff_addr),
        .dbg_coeff_rd_data(dbg_s_coeff_data)
    );

    // Instantiate LMS weight update module (for W filter)
    lms_update #(
        .DATA_WIDTH(DATA_WIDTH),
        .FILTER_ORDER(FILTER_ORDER),
        .MU_WIDTH(MU_WIDTH)
    ) u_lms_update (
        .clk(clk),
        .rst_n(rst_n),
        .start(lms_start),
        .sample_tick(sample_tick),
        .mu(mu_in),
        .error_signal(error_reg),
        .filtered_x(filtered_x_reg),
        .weight_reset(weight_reset),
        .w_init_delay(w_init_delay),
        .done(lms_done),
        .coeff_we(lms_coeff_we),
        .coeff_addr(lms_coeff_addr),
        .coeff_dout(lms_coeff_dout)
    );

    // Instantiate secondary path LMS update module (for S' estimation)
    // Shares delay line with secondary_path_filter for sample-accurate alignment
    secondary_lms_update #(
        .DATA_WIDTH(DATA_WIDTH),
        .FILTER_ORDER(`FILTER_ORDER_S),
        .MU_WIDTH(MU_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_s_lms_update (
        .clk(clk),
        .rst_n(rst_n),
        .start(est_lms_start),
        .mu(mu_s_in),
        .estimation_error(estimation_error_reg),
        .s_init_delay(s_init_delay),
        .done(est_lms_done),
        .coeff_we(est_lms_coeff_we),
        .coeff_addr(est_lms_coeff_addr),
        .coeff_dout(est_lms_coeff_dout),
        // Shared delay line from secondary_path_filter
        .delay_read_addr_ext(est_delay_read_addr),
        .delay_read_data_ext(est_delay_read_data)
    );

endmodule
