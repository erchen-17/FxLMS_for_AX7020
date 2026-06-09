`include "fxlms_params.vh"
// =============================================================================
// File: lms_update.v  (BRAM VERSION with Pipelining)
// Description: LMS Weight Update Module
// Algorithm: w(n+1) = w(n) - mu * e(n) * x'(n)
// Fixed-point format: Q16.16 for 32-bit data
//
// Changes from register version:
//   - weights and x_delay_filtered stored in Block RAM
//   - Uses circular buffer for delay line
//   - Pipelined BRAM reads for improved throughput
//
// Timing: ~FILTER_ORDER + 5 cycles per update
// =============================================================================
module lms_update #(
    parameter DATA_WIDTH   = `DATA_WIDTH,
    parameter FILTER_ORDER = `FILTER_ORDER,
    parameter MU_WIDTH     = `MU_WIDTH,
    parameter FRAC_BITS    = `FRAC_BITS
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         start,
    input  wire                         sample_tick,     // Pulse every sample (for x' delay line update)
    input  wire signed [MU_WIDTH-1:0]   mu,              // Step size (Q16.16, runtime configurable)
    input  wire signed [DATA_WIDTH-1:0] error_signal,    // e(n), Q16.16
    input  wire signed [DATA_WIDTH-1:0] filtered_x,      // x'(n), Q16.16
    input  wire                         weight_reset,    // Reset all weights to zero
    input  wire [7:0]                   w_init_delay,    // W filter initial delay
    output reg                          done,

    // Coefficient output interface (to W filter)
    output reg                              coeff_we,    // Coefficient write enable
    output reg  [$clog2(FILTER_ORDER)-1:0]  coeff_addr,
    output reg  signed [DATA_WIDTH-1:0]     coeff_dout
);

    localparam ADDR_WIDTH = $clog2(FILTER_ORDER);

    // -------------------------------------------------------------------------
    // Width planning (FULL precision)
    // -------------------------------------------------------------------------
    localparam MU_MUL_WIDTH   = MU_WIDTH + DATA_WIDTH;
    localparam DELTA_FULL_WIDTH = MU_MUL_WIDTH + DATA_WIDTH;

    // =========================================================================
    // Memory for weights
    // =========================================================================
    
    reg signed [DATA_WIDTH-1:0] weights [0:FILTER_ORDER-1];

    // =========================================================================
    // Memory for x' delay line (circular buffer)
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] x_delay_filtered [0:FILTER_ORDER-1];

    // Circular buffer write pointer
    reg [ADDR_WIDTH-1:0] delay_write_ptr;

    // Control
    reg [ADDR_WIDTH-1:0] tap_counter;
    reg computing;

    // State machine
    reg [2:0] state;
    localparam IDLE       = 3'd0;
    localparam LATCH_MU   = 3'd1;  // Latch mu*e
    localparam PRIME      = 3'd2;  // Prime pipeline
    localparam COMPUTING  = 3'd3;  // Pipelined compute
    localparam DRAIN      = 3'd4;  // Drain pipeline (tap N-2)
    localparam DRAIN2     = 3'd5;  // Drain pipeline (tap N-1)
    localparam DONE_STATE = 3'd6;

    // Intermediate
    reg signed [MU_MUL_WIDTH-1:0] mu_error;

    // Weight reset state
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
    // Pipeline registers for BRAM read (1 cycle latency)
    // Stage 1: Address issued (tap_counter)
    // Stage 2: Data available (weight_read_reg, x_delay_read_reg)
    // Stage 3: Compute delta and update
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] weight_read_reg;
    reg signed [DATA_WIDTH-1:0] x_delay_read_reg;
    reg [1:0] pipe_valid;
    reg [ADDR_WIDTH-1:0] weight_read_addr;  // Address that weight_read_reg came from (synchronized)

    // Circular buffer read address calculation
    // After sample_tick: x'(n) written to delay_write_ptr-1 (ptr already incremented)
    // We want: tap=0 �???? x'(n), tap=1 �???? x'(n-1), tap=k �???? x'(n-k)
    // So read address = (delay_write_ptr - 1) - tap_counter (with wrap-around)
    wire [ADDR_WIDTH-1:0] delay_read_addr;
    wire [ADDR_WIDTH-1:0] read_base;
    // read_base = delay_write_ptr - 1 (with wrap-around)
    assign read_base = (delay_write_ptr == 0) ? (FILTER_ORDER - 1) : (delay_write_ptr - 1);
    // delay_read_addr = read_base - tap_counter (with wrap-around)
    assign delay_read_addr = (read_base >= tap_counter) ?
                             (read_base - tap_counter) :
                             (FILTER_ORDER + read_base - tap_counter);

    // -------------------------------------------------------------------------
    // Stage 1: mu_error = (mu * e) >>> FRAC_BITS
    // -------------------------------------------------------------------------
    wire signed [MU_MUL_WIDTH-1:0] mu_error_product_full;
    assign mu_error_product_full = mu * $signed(error_signal);

    wire signed [MU_MUL_WIDTH-1:0] mu_error_scaled;
    assign mu_error_scaled = mu_error_product_full >>> FRAC_BITS;

    // -------------------------------------------------------------------------
    // Stage 2: delta_w_full = mu_error * x_delay_filtered
    // -------------------------------------------------------------------------
    wire signed [DELTA_FULL_WIDTH-1:0] delta_w_full;
    assign delta_w_full = $signed(mu_error) * $signed(x_delay_read_reg);

    wire signed [DELTA_FULL_WIDTH-1:0] delta_w_shifted;
    assign delta_w_shifted = delta_w_full >>> FRAC_BITS;

    // -------------------------------------------------------------------------
    // Saturate delta_w_shifted to DATA_WIDTH
    // -------------------------------------------------------------------------
    wire [DELTA_FULL_WIDTH-DATA_WIDTH:0] delta_sign_ext_bits;
    assign delta_sign_ext_bits = delta_w_shifted[DELTA_FULL_WIDTH-1:DATA_WIDTH-1];

    wire delta_no_overflow;
    assign delta_no_overflow =
        (delta_sign_ext_bits == {(DELTA_FULL_WIDTH-DATA_WIDTH+1){delta_w_shifted[DATA_WIDTH-1]}});

    wire signed [DATA_WIDTH-1:0] delta_w_scaled;
    assign delta_w_scaled = delta_no_overflow ?
                            delta_w_shifted[DATA_WIDTH-1:0] :
                            (delta_w_shifted[DELTA_FULL_WIDTH-1] ?
                                {1'b1, {(DATA_WIDTH-1){1'b0}}} :
                                {1'b0, {(DATA_WIDTH-1){1'b1}}});

    // -------------------------------------------------------------------------
    // Weight update with saturation
    // -------------------------------------------------------------------------
    wire signed [DATA_WIDTH:0] weight_sum;
    assign weight_sum = $signed(weight_read_reg) - $signed(delta_w_scaled);  //!!!!!!!

    wire signed [DATA_WIDTH-1:0] weight_saturated;
    assign weight_saturated =
        (weight_sum[DATA_WIDTH] != weight_sum[DATA_WIDTH-1]) ?
            (weight_sum[DATA_WIDTH] ?
                {1'b1, {(DATA_WIDTH-1){1'b0}}} :
                {1'b0, {(DATA_WIDTH-1){1'b1}}}) :
            weight_sum[DATA_WIDTH-1:0];

    // =========================================================================
    // Weight memory write enable and data (directly from state machine outputs)
    // =========================================================================
    reg                        weight_we;
    reg [ADDR_WIDTH-1:0]       weight_waddr;
    reg signed [DATA_WIDTH-1:0] weight_wdata;

    // =========================================================================
    // BRAM for weights - proper dual-port template
    // Port A: Write (from LMS update or reset)
    // Port B: Read (for computation)
    // weight_read_addr is updated in sync with weight_read_reg to track source address
    // =========================================================================
    always @(posedge clk) begin
        if (weight_we)
            weights[weight_waddr] <= weight_wdata;
        weight_read_reg <= weights[tap_counter];
        weight_read_addr <= tap_counter;  // Record which address we're reading from
    end

    // =========================================================================
    // BRAM read for x_delay_filtered
    // =========================================================================
    always @(posedge clk) begin
        x_delay_read_reg <= x_delay_filtered[delay_read_addr];
    end

    // =========================================================================
    // Delay line update on sample_tick (circular buffer write)
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            delay_write_ptr <= 0;
        end else if (sample_tick && !computing) begin
            x_delay_filtered[delay_write_ptr] <= filtered_x;
            delay_write_ptr <= (delay_write_ptr == FILTER_ORDER - 1) ? 0 : delay_write_ptr + 1;
        end
    end

    // =========================================================================
    // Weight reset logic (controls reset_counter and resetting flag only)
    // Actual writes go through weight_we/waddr/wdata in state machine
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            resetting <= 1'b0;
            reset_counter <= 0;
        end else if (weight_reset && !resetting && !computing) begin
            resetting <= 1'b1;
            reset_counter <= 0;
        end else if (resetting) begin
            if (reset_counter == FILTER_ORDER - 1) begin
                resetting <= 1'b0;
            end else begin
                reset_counter <= reset_counter + 1;
            end
        end
    end

    // =========================================================================
    // Main LMS update state machine (Pipelined)
    // Total cycles: 1(LATCH_MU) + 1(PRIME) + N(COMPUTING) + 1(DRAIN) + 1(DONE)
    //             = N + 4 cycles
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state           <= IDLE;
            computing       <= 1'b0;
            tap_counter     <= 0;
            done            <= 1'b0;
            coeff_we        <= 1'b0;
            coeff_addr      <= 0;
            coeff_dout      <= 0;
            mu_error        <= 0;
            pipe_valid      <= 2'b00;
            weight_we       <= 1'b0;
            weight_waddr    <= 0;
            weight_wdata    <= 0;

        end else begin
            done     <= 1'b0;
            coeff_we <= 1'b0;
            weight_we <= 1'b0;

            // Handle reset writes through the centralized write port
            if (resetting) begin
                weight_we    <= 1'b1;
                weight_waddr <= reset_counter;
                if (`W_INIT_MODE == 0) begin
                    weight_wdata <= 0;
                end else if (`W_INIT_MODE == 1) begin
                    if (reset_counter < w_init_delay)
                        weight_wdata <= 0;
                    else
                        weight_wdata <= w_exp_decay_coeff(reset_counter - w_init_delay);
                end else begin
                    // Mode 2: unit impulse at index 1
                    weight_wdata <= (reset_counter == 1) ? (1 << `FRAC_BITS) : 0;
                end
            end

            case (state)
                IDLE: begin
                    pipe_valid <= 2'b00;
                    if (start && !computing && !resetting) begin
                        state       <= LATCH_MU;
                        computing   <= 1'b1;
                        tap_counter <= 0;
                        mu_error    <= mu_error_scaled;
                    end
                end

                LATCH_MU: begin
                    // mu_error is now latched, start priming pipeline
                    // Issue first BRAM read (tap_counter = 0)
                    tap_counter <= 0;
                    state <= PRIME;
                end

                PRIME: begin
                    // Pipeline priming: tap 0 data will arrive next cycle
                    // Issue read for tap 1
                    tap_counter <= 1;
                    pipe_valid <= 2'b01;    // First stage valid
                    state <= COMPUTING;
                end

                COMPUTING: begin
                    // Pipelined operation:
                    // - weight_read_reg contains data from weight_read_addr
                    // - Compute delta and write back to weight_read_addr
                    // - Issue next read address (tap_counter)

                    pipe_valid <= {pipe_valid[0], 1'b1};

                    // Write back when pipeline stage 2 is valid
                    // Use weight_read_addr which is synchronized with weight_read_reg
                    if (pipe_valid[0]) begin
                        // Write updated weight back to BRAM
                        weight_we    <= 1'b1;
                        weight_waddr <= weight_read_addr;
                        weight_wdata <= weight_saturated;

                        // Write-through to adaptive FIR (W filter)
                        coeff_we   <= 1'b1;
                        coeff_addr <= weight_read_addr;
                        coeff_dout <= weight_saturated;
                    end

                    if (tap_counter == FILTER_ORDER - 1) begin
                        // Last address issued, go to drain
                        state <= DRAIN;
                    end else begin
                        tap_counter <= tap_counter + 1;
                    end
                end

                DRAIN: begin
                    // Drain: write back result for weight_read_addr
                    weight_we    <= 1'b1;
                    weight_waddr <= weight_read_addr;
                    weight_wdata <= weight_saturated;
                    coeff_we   <= 1'b1;
                    coeff_addr <= weight_read_addr;
                    coeff_dout <= weight_saturated;

                    state <= DONE_STATE;
                end

                DONE_STATE: begin
                    computing       <= 1'b0;
                    done            <= 1'b1;
                    tap_counter     <= 0;
                    state           <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

    // =========================================================================
    // Initialize BRAMs (for simulation)
    // W filter weights follow W_INIT_MODE; hardware uses weight_reset signal
    // =========================================================================
    integer i;
    initial begin
        for (i = 0; i < FILTER_ORDER; i = i + 1) begin
            if (`W_INIT_MODE == 0) begin
                weights[i] = 0;
            end else if (`W_INIT_MODE == 1) begin
                if (i < w_init_delay)
                    weights[i] = 0;
                else
                    weights[i] = w_exp_decay_coeff(i - w_init_delay);
            end else begin
                // Mode 2: unit impulse at index 1
                weights[i] = (i == 1) ? (1 << `FRAC_BITS) : 0;
            end
            x_delay_filtered[i] = 0;
        end
    end

endmodule
