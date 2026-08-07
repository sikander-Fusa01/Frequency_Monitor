// -----------------------------------------------------------------------------
// Module      : sync2ff
// Description : Generic parameterizable multi-flop synchronizer used to bring
//               quasi-static / level signals from one clock domain safely
//               into another clock domain. Used throughout the Frequency
//               Monitor IP for Input Clock Synchronizer (ICS) chains and for
//               synchronizing single-bit / multi-bit control and status
//               signals across the ref_clk <-> bus_clk boundary.
// -----------------------------------------------------------------------------
module sync2ff #(
    parameter WIDTH  = 1,
    parameter STAGES = 2
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire [WIDTH-1:0] din,
    output wire [WIDTH-1:0] dout
);

    // synchronizer flop bank : sync_ff[0] is the first (asynchronous-input)
    // stage, sync_ff[STAGES-1] is the final synchronized stage.
    reg [WIDTH-1:0] sync_ff [0:STAGES-1];
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < STAGES; i = i + 1) begin
                sync_ff[i] <= {WIDTH{1'b0}};
            end
        end else begin
            sync_ff[0] <= din;
            for (i = 1; i < STAGES; i = i + 1) begin
                sync_ff[i] <= sync_ff[i-1];
            end
        end
    end

    assign dout = sync_ff[STAGES-1];

endmodule
