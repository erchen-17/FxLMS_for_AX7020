`timescale 1ns / 1ps

// ==============================================================================
// Module: button_debounce
// Description: Debounces a mechanical button input signal
//
// Operation:
//   - Samples input button at a slower rate (determined by counter)
//   - Only changes output when input is stable for a full debounce period
//   - Configurable debounce time via DEBOUNCE_CYCLES parameter
//
// Parameters:
//   - DEBOUNCE_CYCLES: Number of clock cycles for debounce period
//     Example: For 50MHz clock and 20ms debounce: 50MHz * 0.02s = 1_000_000
//
// Timing:
//   - Input: Raw button signal (may have bounces)
//   - Output: Clean, debounced signal (delayed by debounce period)
// ==============================================================================

module button_debounce #(
    parameter DEBOUNCE_CYCLES = 1_000_000  // Default: 20ms at 50MHz
)(
    input  wire clk,           // System clock
    input  wire rst_n,         // Active-low reset
    input  wire btn_in,        // Raw button input
    output reg  btn_out        // Debounced button output
);

    // State registers
    reg [31:0] counter;
    reg btn_sync_0, btn_sync_1;  // Synchronizer flip-flops
    reg btn_stable;               // Stable button state

    // Synchronize button input to clock domain (prevent metastability)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_sync_0 <= 1'b0;
            btn_sync_1 <= 1'b0;
        end else begin
            btn_sync_0 <= btn_in;
            btn_sync_1 <= btn_sync_0;
        end
    end

    // Debounce logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 0;
            btn_stable <= 1'b0;
            btn_out <= 1'b0;
        end else begin
            // If synchronized button matches the stable state, reset counter
            if (btn_sync_1 == btn_stable) begin
                counter <= 0;
            end
            // If button state is different, increment counter
            else begin
                counter <= counter + 1;

                // If counter reaches debounce threshold, update stable state and output
                if (counter >= DEBOUNCE_CYCLES) begin
                    btn_stable <= btn_sync_1;
                    btn_out <= btn_sync_1;
                    counter <= 0;
                end
            end
        end
    end

endmodule
