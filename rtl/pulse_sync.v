// -----------------------------------------------------------------------------
// Module      : pulse_sync
// Description : Toggle-based single-cycle pulse synchronizer. Converts a
//               single source-clock-cycle pulse into a toggle signal, safely
//               synchronizes the toggle into the destination clock domain
//               using a 3-stage flop chain, then edge-detects the toggle to
//               regenerate a single destination-clock-cycle pulse.
//               Used for crossing START/ABORT/IRQ-event/DONE type pulses
//               between the ref_clk and bus_clk domains 
// -----------------------------------------------------------------------------
module pulse_sync (
    input  wire src_clk,
    input  wire src_rst_n,
    input  wire src_pulse,

    input  wire dst_clk,
    input  wire dst_rst_n,
    output wire dst_pulse
);

    reg toggle_src;

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n)
            toggle_src <= 1'b0;
        else if (src_pulse)
            toggle_src <= ~toggle_src;
    end

    reg [2:0] sync_dst;

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n)
            sync_dst <= 3'b000;
        else
            sync_dst <= {sync_dst[1:0], toggle_src};
    end

    assign dst_pulse = sync_dst[2] ^ sync_dst[1];

endmodule
