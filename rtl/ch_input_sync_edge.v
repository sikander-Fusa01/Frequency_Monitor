// -----------------------------------------------------------------------------
// Module      : ch_input_sync_edge
// Description : Combines the Input Clock Synchronizer (ICS) and Edge Detector
//               (ED)
//            
//
//               ICS  : Parameterizable multi-stage synchronizer (default
//                      2-flop, SYNC_STAGES configurable 2-4) bringing the
//                      asynchronous clk_in signal into the ref_clk domain.
//               ED   : 2-flop shift register performing combinational 0->1
//                      transition detection, producing a single ref_clk-cycle
//                      wide pulse per rising edge of the input clock. Duty
//                      cycle independent (works for 10%-90% duty cycle).

// -----------------------------------------------------------------------------
module ch_input_sync_edge #(
    parameter SYNC_STAGES = 2
) (
    input  wire ref_clk,
    input  wire ref_rst_n,
    input  wire clk_in,        // asynchronous input clock (treated as data)
    output wire edge_pulse     // single ref_clk-cycle pulse per rising edge
);

    wire clk_in_sync;

    // Input Clock Synchronizer (ICS)
    sync2ff #(
        .WIDTH  (1),
        .STAGES (SYNC_STAGES)
    ) u_ics (
        .clk   (ref_clk),
        .rst_n (ref_rst_n),
        .din   (clk_in),
        .dout  (clk_in_sync)
    );

    // Edge Detector (ED) : 2-flop shift register, combinational 0->1 detect
    reg d1, d2;

    always @(posedge ref_clk or negedge ref_rst_n) begin
        if (!ref_rst_n) begin
            d1 <= 1'b0;
            d2 <= 1'b0;
        end else begin
            d1 <= clk_in_sync;
            d2 <= d1;
        end
    end

    assign edge_pulse = d1 & ~d2;

endmodule
