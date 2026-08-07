// -----------------------------------------------------------------------------
// Module      : interrupt_gen
// Description : Interrupt Generator (IG) sub-block,
//               Combines measurement-done, monitor-fault and monitor-clear
//               event pulses (already synchronized into the bus_clk domain)
//               into the single irq output, configurable as edge-sensitive
//               (single pulse) or level-sensitive (asserted until cleared).
//
//               FSM state encoding (matches specification exactly):
//                 IRQ_IDLE    = 2'b00
//                 IRQ_ASSERT  = 2'b01
//                 IRQ_PENDING = 2'b10   (level mode only)
// -----------------------------------------------------------------------------
module interrupt_gen (
    input  wire bus_clk,
    input  wire bus_rst_n,

    input  wire irq_global_en,
    input  wire irq_edge,          // 1 = edge/pulse mode, 0 = level mode

    input  wire en_measure_done,
    input  wire en_mon_fault,
    input  wire en_mon_clear,

    input  wire measure_done_evt,  // 1 bus_clk-cycle pulse
    input  wire mon_fault_evt,     // 1 bus_clk-cycle pulse
    input  wire mon_clear_evt,     // 1 bus_clk-cycle pulse

    input  wire pending_masked,    // OR of enabled IRQ_STATUS bits still set

    output reg  irq
);

    localparam IRQ_IDLE    = 2'b00;
    localparam IRQ_ASSERT  = 2'b01;
    localparam IRQ_PENDING = 2'b10;

    reg [1:0] state, next_state;
    wire      any_masked_event;

    assign any_masked_event = (measure_done_evt & en_measure_done) |
                              (mon_fault_evt    & en_mon_fault)    |
                              (mon_clear_evt    & en_mon_clear);

    always @(*) begin
        next_state = state;
        case (state)
            IRQ_IDLE   : begin
                if (irq_global_en && any_masked_event)
                    next_state = IRQ_ASSERT;
            end
            IRQ_ASSERT : begin
                if (irq_edge)
                    next_state = IRQ_IDLE;
                else
                    next_state = IRQ_PENDING;
            end
            IRQ_PENDING: begin
                if (!pending_masked)
                    next_state = IRQ_IDLE;
            end
            default    : next_state = IRQ_IDLE;
        endcase
    end

    always @(posedge bus_clk or negedge bus_rst_n) begin
        if (!bus_rst_n)
            state <= IRQ_IDLE;
        else
            state <= next_state;
    end

    always @(*) begin
        irq = (state == IRQ_ASSERT) || (state == IRQ_PENDING);
    end

endmodule
