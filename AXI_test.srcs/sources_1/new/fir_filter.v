`timescale 1ns/1ps
`include "fxlms_params.vh"
// =============================================================================
// File: fir_filter.v
// Description: Adaptive FIR Filter (W Filter) - BRAM Version with Pipelining
// Fixed-point format: Q16.16
//
// Changes from register version:
//   - delay_line and coeffs stored in Block RAM (inferred)
//   - Circular buffer for delay line (no shifting needed)
//   - Pipelined BRAM reads: same cycle count as original register version
//
// Timing: Same as original (~FILTER_ORDER + 4 cycles per sample)
// =============================================================================
module fir_filter #(
    parameter DATA_WIDTH   = `DATA_WIDTH,
    parameter FILTER_ORDER = `FILTER_ORDER,
    parameter FRAC_BITS    = `FRAC_BITS,
    parameter IS_ADAPTIVE  = 1
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,
    input  wire signed [DATA_WIDTH-1:0] data_in,
    input  wire [7:0]                   w_init_delay,

    // Coefficient update interface
    input  wire                         coeff_we,
    input  wire [$clog2(FILTER_ORDER)-1:0] coeff_addr,
    input  wire signed [DATA_WIDTH-1:0] coeff_din,

    // Coefficient reset interface
    input  wire                         coeff_reset,

    output reg  signed [DATA_WIDTH-1:0] data_out,
    output reg                          done
);
    localparam ADDR_WIDTH = $clog2(FILTER_ORDER);
    localparam ACC_WIDTH = 2*DATA_WIDTH + ADDR_WIDTH;

    // =========================================================================
    // BRAM for delay line (circular buffer)
    // =========================================================================
    (* ram_style = "block" *) reg signed [DATA_WIDTH-1:0] delay_line [0:FILTER_ORDER-1];

    // =========================================================================
    // BRAM for coefficients
    // Note: Using distributed RAM due to complex write pattern (reset + LMS update)
    // =========================================================================
    (* ram_style = "distributed" *) reg signed [DATA_WIDTH-1:0] coeffs [0:FILTER_ORDER-1];

    // Circular buffer write pointer
    reg [ADDR_WIDTH-1:0] write_ptr;

    // Control signals
    reg [ADDR_WIDTH-1:0] tap_counter;
    reg signed [ACC_WIDTH-1:0] accumulator;
    reg computing;

    // =========================================================================
    // Pipeline registers for BRAM read (1 cycle latency)
    // Stage 1: Address issued (tap_counter)
    // Stage 2: Data available (delay_reg, coeff_reg)
    // Stage 3: Multiply result ready, accumulate
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] delay_reg;
    reg signed [DATA_WIDTH-1:0] coeff_reg;
    reg [1:0] pipe_valid;  // 2-stage pipeline valid bits

    // BRAM read address for circular buffer
    // After FIR_WRITE: delay_line[write_ptr] contains x(n), write_ptr unchanged
    // We want: tap=0 �? x(n), tap=1 �? x(n-1), tap=k �? x(n-k)
    // So read address = write_ptr - tap_counter (with wrap-around)
    wire [ADDR_WIDTH-1:0] delay_read_addr;
    assign delay_read_addr = (write_ptr >= tap_counter) ?
                             (write_ptr - tap_counter) :
                             (FILTER_ORDER + write_ptr - tap_counter);

    // State machine
    reg [2:0] fir_state;
    localparam FIR_IDLE      = 3'd0;
    localparam FIR_WRITE     = 3'd1;  // Write new sample
    localparam FIR_WAIT      = 3'd2;  // Wait for BRAM write to complete
    localparam FIR_PRIME     = 3'd3;  // Prime the pipeline
    localparam FIR_COMPUTING = 3'd4;  // Pipelined MAC
    localparam FIR_DRAIN     = 3'd5;  // Drain: accumulate last tap (N-1)
    localparam FIR_OUTPUT    = 3'd6;

    // Coefficient reset
    reg [ADDR_WIDTH-1:0] reset_counter;
    reg resetting;

    // =========================================================================
    // W filter initial coefficient function (compile-time elaboration)
    // Computes W_INIT_GAIN * W_INIT_ALPHA^n in Q16.16 fixed-point
    // =========================================================================
    function automatic signed [DATA_WIDTH-1:0] w_exp_decay_coeff;
        input integer n;
        integer k;
        reg signed [63:0] val;
        begin
            val = `W_INIT_GAIN;
            for (k = 0; k < n; k = k + 1)
                val = (val * `W_INIT_ALPHA) >>> `FRAC_BITS;
            w_exp_decay_coeff = val[DATA_WIDTH-1:0];
        end
    endfunction

    // =========================================================================
    // Coefficient write/reset logic
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            resetting <= 1'b0;
            reset_counter <= 0;
        end else if (coeff_reset && !resetting && !computing) begin
            resetting <= 1'b1;
            reset_counter <= 0;
        end else if (resetting) begin
            if (`W_INIT_MODE == 0) begin
                coeffs[reset_counter] <= 0;
            end else if (`W_INIT_MODE == 1) begin
                if (reset_counter < w_init_delay)
                    coeffs[reset_counter] <= 0;
                else
                    coeffs[reset_counter] <= w_exp_decay_coeff(reset_counter - w_init_delay);
            end else begin
                // Mode 2: unit impulse at index 1
                coeffs[reset_counter] <= (reset_counter == 1) ? (1 << `FRAC_BITS) : 0;
            end
            if (reset_counter == FILTER_ORDER - 1) begin
                resetting <= 1'b0;
            end else begin
                reset_counter <= reset_counter + 1;
            end
        end else if (coeff_we && !resetting) begin
            coeffs[coeff_addr] <= coeff_din;
        end
    end

    // =========================================================================
    // BRAM read - registered outputs (1 cycle latency)
    // =========================================================================
    always @(posedge clk) begin
        delay_reg <= delay_line[delay_read_addr];
        coeff_reg <= coeffs[tap_counter];
    end

    // Multiplier
    wire signed [2*DATA_WIDTH-1:0] mult_result;
    assign mult_result = coeff_reg * delay_reg;

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

    // =========================================================================
    // Pipelined FIR filter state machine
    //
    // Timeline (write_ptr = P):
    //   T0 IDLE:      start=1
    //   T1 WRITE:     delay_line[P] <= x(n), tap_counter=0
    //   T2 WAIT:      BRAM write completes, issue read addr P (tap 0)
    //   T3 PRIME:     delay_reg = x(n), issue read addr P-1 (tap 1)
    //   T4 COMPUTING: delay_reg = x(n-1), acc += c[0]*x(n), issue tap 2
    //   ...
    //   T(N+3) DRAIN: acc += c[N-2]*x(n-N+2)
    //   T(N+4) OUTPUT: acc += c[N-1]*x(n-N+1), done=1
    //
    // Total: N + 5 cycles
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fir_state   <= FIR_IDLE;
            computing   <= 1'b0;
            tap_counter <= 0;
            accumulator <= 0;
            data_out    <= 0;
            done        <= 1'b0;
            write_ptr   <= 0;
            pipe_valid  <= 2'b00;
        end else begin
            done <= 1'b0;

            case (fir_state)
                FIR_IDLE: begin
                    pipe_valid <= 2'b00;
                    if (start && !computing && !resetting) begin
                        fir_state   <= FIR_WRITE;
                        computing   <= 1'b1;
                        accumulator <= 0;
                    end
                end

                FIR_WRITE: begin
                    // Write x(n) to delay_line[write_ptr]
                    delay_line[write_ptr] <= data_in;
                    tap_counter <= 0;
                    fir_state <= FIR_WAIT;
                end

                FIR_WAIT: begin
                    // BRAM write now complete
                    // Issue read for tap 0: addr = write_ptr
                    // tap_counter already 0
                    fir_state <= FIR_PRIME;
                end

                FIR_PRIME: begin
                    // delay_reg now has x(n) from tap 0
                    // Issue read for tap 1
                    tap_counter <= 1;
                    pipe_valid <= 2'b01;
                    fir_state <= FIR_COMPUTING;
                end

                FIR_COMPUTING: begin
                    // Accumulate previous tap's result
                    accumulator <= accumulator + (mult_result >>> FRAC_BITS);

                    if (tap_counter == FILTER_ORDER - 1) begin
                        fir_state <= FIR_DRAIN;
                    end else begin
                        tap_counter <= tap_counter + 1;
                    end
                end

                FIR_DRAIN: begin
                    // Accumulate last tap (N-1)
                    accumulator <= accumulator + (mult_result >>> FRAC_BITS);
                    fir_state <= FIR_OUTPUT;
                end

                FIR_OUTPUT: begin
                    computing <= 1'b0;
                    data_out  <= saturated_output;
                    done      <= 1'b1;
                    write_ptr <= (write_ptr == FILTER_ORDER - 1) ? 0 : write_ptr + 1;
                    fir_state <= FIR_IDLE;
                end

                default: fir_state <= FIR_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Initialize delay_line and coeffs (for simulation only)
    // Coeffs follow W_INIT_MODE; hardware uses coeff_reset signal instead
    // =========================================================================
    integer i;
    initial begin
        for (i = 0; i < FILTER_ORDER; i = i + 1) begin
            delay_line[i] = 0;
            if (`W_INIT_MODE == 0) begin
                coeffs[i] = 0;
            end else if (`W_INIT_MODE == 1) begin
                if (i < w_init_delay)
                    coeffs[i] = 0;
                else
                    coeffs[i] = w_exp_decay_coeff(i - w_init_delay);
            end else begin
                // Mode 2: unit impulse at index 1
                coeffs[i] = (i == 1) ? (1 << `FRAC_BITS) : 0;
            end
        end
    end

endmodule
