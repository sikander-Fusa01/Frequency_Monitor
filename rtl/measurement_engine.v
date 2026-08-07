// -----------------------------------------------------------------------------
// Module      : measurement_engine
// Description : Measurement Engine (ME) sub-block.
//               Performs on-demand frequency measurement of a selected input
//               channel using an N:1 channel multiplexer, a 32-bit edge
//               counter, a 32-bit gate (down) counter and a result latch.
//
//               FSM state encoding (matches specification exactly):
//                 IDLE  = 2'b00
//                 ARM   = 2'b01
//                 COUNT = 2'b10
//                 LATCH = 2'b11
//
//               All logic operates synchronously to ref_clk. start_pulse and
//               abort_pulse are expected to already be synchronized 1-cycle
//               pulses in the ref_clk domain (see pulse_sync). measure_sel,
//               measure_gate and measure_continuous are expected to already
//               be synchronized (quasi-static) into the ref_clk domain.
// -----------------------------------------------------------------------------
module measurement_engine #(
    parameter NUM_CHANNELS = 4,
    parameter RESULT_WIDTH = 32
) (
    input  wire                          ref_clk,
    input  wire                          ref_rst_n,

    input  wire                          global_en,
    input  wire [NUM_CHANNELS-1:0]       edge_pulse,       // per-channel edge pulses
    input  wire [3:0]                    measure_sel,      // selected channel (0..N-1)
    input  wire [RESULT_WIDTH-1:0]       measure_gate,      // gate time in ref_clk cycles
    input  wire                          measure_continuous,

    input  wire                          start_pulse,       // synced START (self-clearing)
    input  wire                          abort_pulse,       // synced ABORT (self-clearing)

    output reg                           measure_busy,
    output reg                           measure_done_pulse, // 1 ref_clk-cycle pulse on LATCH completion
    output reg                           result_valid,
    output reg  [RESULT_WIDTH-1:0]       measure_result
);

    // FSM state encoding - matches specification Section 3.4.1 exactly
    localparam ME_IDLE  = 2'b00;
    localparam ME_ARM   = 2'b01;
    localparam ME_COUNT = 2'b10;
    localparam ME_LATCH = 2'b11;

    reg [1:0]                 state, next_state;
    reg [RESULT_WIDTH-1:0]    edge_cnt;
    reg [RESULT_WIDTH-1:0]    gate_cnt;
    reg                       gate_open;
    wire                      sel_edge_pulse;

    // N:1 channel multiplexer for the currently selected channel
    assign sel_edge_pulse = edge_pulse[measure_sel];

    // -------------------------------------------------------------------
    // Next-state logic
    // -------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            ME_IDLE : begin
                if (global_en && start_pulse)
                    next_state = ME_ARM;
            end
            ME_ARM  : begin
                next_state = ME_COUNT;
            end
            ME_COUNT: begin
                if (abort_pulse)
                    next_state = ME_IDLE;
                else if (gate_cnt == {RESULT_WIDTH{1'b0}})
                    next_state = ME_LATCH;
            end
            ME_LATCH: begin
                if (measure_continuous)
                    next_state = ME_ARM;
                else
                    next_state = ME_IDLE;
            end
            default : next_state = ME_IDLE;
        endcase
    end

    // -------------------------------------------------------------------
    // State register
    // -------------------------------------------------------------------
    always @(posedge ref_clk or negedge ref_rst_n) begin
        if (!ref_rst_n)
            state <= ME_IDLE;
        else
            state <= next_state;
    end

    // -------------------------------------------------------------------
    // Datapath : edge counter, gate counter, result latch
    // -------------------------------------------------------------------
    always @(posedge ref_clk or negedge ref_rst_n) begin
        if (!ref_rst_n) begin
            edge_cnt            <= {RESULT_WIDTH{1'b0}};
            gate_cnt            <= {RESULT_WIDTH{1'b0}};
            gate_open           <= 1'b0;
            measure_result      <= {RESULT_WIDTH{1'b0}};
            measure_done_pulse  <= 1'b0;
            result_valid        <= 1'b0;
        end else begin
            measure_done_pulse <= 1'b0; // default: 1-cycle pulse

            case (state)
                ME_IDLE : begin
                    gate_open <= 1'b0;
                end

                ME_ARM  : begin
                    edge_cnt  <= {RESULT_WIDTH{1'b0}}; // cleared in ARM
                    gate_cnt  <= measure_gate;          // loaded from MEASURE_GATE
                    gate_open <= 1'b0;
                end

                ME_COUNT: begin
                    gate_open <= 1'b1;
                    if (sel_edge_pulse)
                        edge_cnt <= edge_cnt + 1'b1;

                    if (gate_cnt != {RESULT_WIDTH{1'b0}})
                        gate_cnt <= gate_cnt - 1'b1;
                end

                ME_LATCH: begin
                    gate_open          <= 1'b0;
                    measure_result     <= edge_cnt;   // freeze/store final count
                    measure_done_pulse <= 1'b1;
                    result_valid       <= 1'b1;
                end

                default : begin
                    gate_open <= 1'b0;
                end
            endcase

            // RESULT_VALID cleared whenever a new measurement is (re)armed
            if (state == ME_ARM)
                result_valid <= 1'b0;
        end
    end

    // measure_busy reflects ARM/COUNT/LATCH activity
    always @(*) begin
        measure_busy = (state != ME_IDLE);
    end

endmodule
