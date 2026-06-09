`timescale 1ns/1ps
`include "fxlms_params.vh"
// =============================================================================
// File: secondary_path_filter.v
// Description: Secondary Path Estimation Filter S'(z)
// Fixed-point format: Q16.16
//
// This is a standard FIR filter with coefficient write interface.
// Used for both:
//   1. Filtering x(n) to get x'(n) in normal FxLMS mode
//   2. Filtering w(n) to get S'*w(n) in estimation mode
//
// Coefficients can be updated by external LMS module via coeff_we interface.
//
// Initialization Mode (controlled by S_INIT_MODE in fxlms_params.vh):
//   0 = All zeros - rely on online estimation to learn S'(z)
//   1 = Exponential decay - S'[i] = GAIN * ALPHA^(i-DELAY) for i >= DELAY
//   2 = Unit impulse - S'[1] = 1.0, others = 0 (assumes 1-sample delay)
// =============================================================================
module secondary_path_filter #(
    parameter DATA_WIDTH   = `DATA_WIDTH,
    parameter FILTER_ORDER = `FILTER_ORDER_S,  // Use S-specific order
    parameter FRAC_BITS    = `FRAC_BITS
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,
    input  wire signed [DATA_WIDTH-1:0] data_in,
    input  wire [7:0]                   s_init_delay,    // S' filter initial delay
    output reg  signed [DATA_WIDTH-1:0] data_out,
    output reg                          done,

    // Coefficient write interface (from LMS estimator)
    input  wire                              coeff_we,
    input  wire [$clog2(FILTER_ORDER)-1:0]   coeff_addr,
    input  wire signed [DATA_WIDTH-1:0]      coeff_din,

    // Delay line read interface (for LMS update to share delay line)
    input  wire [$clog2(FILTER_ORDER)-1:0]   delay_read_addr_ext,
    output wire signed [DATA_WIDTH-1:0]      delay_read_data_ext,
    output wire [$clog2(FILTER_ORDER)-1:0]   delay_write_ptr_out,

    // Debug: coefficient readback interface
    input  wire [$clog2(FILTER_ORDER)-1:0]   dbg_coeff_rd_addr,
    output wire signed [DATA_WIDTH-1:0]      dbg_coeff_rd_data
);

    localparam ADDR_WIDTH = $clog2(FILTER_ORDER);
    localparam ACC_WIDTH  = 2*DATA_WIDTH + ADDR_WIDTH;

    // Delay line for input signal
    reg signed [DATA_WIDTH-1:0] delay_line [0:FILTER_ORDER-1];

    // S'(z) coefficients
    reg signed [DATA_WIDTH-1:0] s_coeffs [0:FILTER_ORDER-1];

    // Control
    reg [ADDR_WIDTH-1:0] tap_counter;
    reg signed [ACC_WIDTH-1:0] accumulator;
    reg computing;

    integer i;

    // -------------------------------------------------------------------------
    // Coefficient write logic (from external LMS module)
    // Initialization mode controlled by S_INIT_MODE parameter:
    //   0 = All zeros (rely on online estimation)
    //   1 = Exponential decay: S'[i] = GAIN * ALPHA^i
    //   2 = Unit impulse: S'[1] = 1.0, others = 0
    // -------------------------------------------------------------------------

    // Function to compute exponential decay coefficient
    // Returns GAIN * ALPHA^n in Q16.16 format
    function signed [DATA_WIDTH-1:0] exp_decay_coeff;
        input integer n;
        reg signed [63:0] result;
        reg signed [31:0] alpha_power;
        integer k;
        begin
            // Start with alpha^0 = 1.0 in Q0.16
            alpha_power = 32'sd65536;  // 1.0 in Q0.16

            // Compute alpha^n
            for (k = 0; k < n; k = k + 1) begin
                // alpha_power = alpha_power * ALPHA >> 16
                alpha_power = (alpha_power * `S_INIT_ALPHA) >>> 16;
            end

            // result = GAIN * alpha^n >> 16 (GAIN is Q0.16, result is Q16.16)
            result = (`S_INIT_GAIN * alpha_power) >>> 16;
            exp_decay_coeff = result[DATA_WIDTH-1:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize based on S_INIT_MODE
            for (i = 0; i < FILTER_ORDER; i = i + 1) begin
                if (`S_INIT_MODE == 0) begin
                    // Mode 0: All zeros
                    s_coeffs[i] <= 0;
                end else if (`S_INIT_MODE == 1) begin
                    // Mode 1: Exponential decay starting from S_INIT_DELAY
                    if (i < s_init_delay) begin
                        s_coeffs[i] <= 0;
                    end else begin
                        s_coeffs[i] <= exp_decay_coeff(i - s_init_delay);
                    end
                end else begin
                    // Mode 2: Unit impulse at index s_init_delay
                    if (i == s_init_delay) begin
                        s_coeffs[i] <= (1 << FRAC_BITS);  // 1.0 in Q16.16
                    end else begin
                        s_coeffs[i] <= 0;
                    end
                end
            end
        end else if (coeff_we) begin
            s_coeffs[coeff_addr] <= coeff_din;
        end
    end

    // -------------------------------------------------------------------------
    // Delay line update
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < FILTER_ORDER; i = i + 1)
                delay_line[i] <= 0;
        end else if (start && !computing) begin
            delay_line[0] <= data_in;
            for (i = 1; i < FILTER_ORDER; i = i + 1)
                delay_line[i] <= delay_line[i-1];
        end
    end

    // =========================================================================
    // External delay line read interface (for LMS update)
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] delay_read_data_ext_reg;
    always @(posedge clk) begin
        delay_read_data_ext_reg <= delay_line[delay_read_addr_ext];
    end
    assign delay_read_data_ext = delay_read_data_ext_reg;
    assign delay_write_ptr_out = 0;  // Shift-register delay line always writes to index 0

    // Debug: coefficient readback (1-cycle latency)
    reg signed [DATA_WIDTH-1:0] dbg_coeff_rd_data_reg;
    always @(posedge clk) begin
        dbg_coeff_rd_data_reg <= s_coeffs[dbg_coeff_rd_addr];
    end
    assign dbg_coeff_rd_data = dbg_coeff_rd_data_reg;

    // Multiplier
    wire signed [2*DATA_WIDTH-1:0] mult_result;
    assign mult_result = s_coeffs[tap_counter] * delay_line[tap_counter];

    // -------------------------------------------------------------------------
    // FIR state machine
    // -------------------------------------------------------------------------
    reg [1:0] fir_state;
    localparam FIR_IDLE      = 2'd0;
    localparam FIR_COMPUTING = 2'd1;
    localparam FIR_LAST_ACC  = 2'd2;
    localparam FIR_OUTPUT    = 2'd3;

    // Saturation logic
    wire [ACC_WIDTH-DATA_WIDTH:0] overflow_bits;
    assign overflow_bits = accumulator[ACC_WIDTH-1:DATA_WIDTH-1];

    wire signed [DATA_WIDTH-1:0] saturated_output;
    assign saturated_output =
        (overflow_bits == {(ACC_WIDTH-DATA_WIDTH+1){accumulator[ACC_WIDTH-1]}}) ?
            accumulator[DATA_WIDTH-1:0] :
        accumulator[ACC_WIDTH-1] ?
            {1'b1, {(DATA_WIDTH-1){1'b0}}} :
            {1'b0, {(DATA_WIDTH-1){1'b1}}};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fir_state   <= FIR_IDLE;
            computing   <= 1'b0;
            tap_counter <= 0;
            accumulator <= 0;
            data_out    <= 0;
            done        <= 1'b0;
        end else begin
            done <= 1'b0;

            case (fir_state)
                FIR_IDLE: begin
                    if (start && !computing) begin
                        fir_state   <= FIR_COMPUTING;
                        computing   <= 1'b1;
                        tap_counter <= 0;
                        accumulator <= 0;
                    end
                end

                FIR_COMPUTING: begin
                    accumulator <= accumulator + (mult_result >>> FRAC_BITS);

                    if (tap_counter == FILTER_ORDER - 1) begin
                        fir_state <= FIR_LAST_ACC;
                    end else begin
                        tap_counter <= tap_counter + 1;
                    end
                end

                FIR_LAST_ACC: begin
                    fir_state   <= FIR_OUTPUT;
                    tap_counter <= 0;
                end

                FIR_OUTPUT: begin
                    computing <= 1'b0;
                    data_out  <= saturated_output;
                    done      <= 1'b1;
                    fir_state <= FIR_IDLE;
                end

                default: fir_state <= FIR_IDLE;
            endcase
        end
    end

endmodule
