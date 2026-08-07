// -----------------------------------------------------------------------------
// Module      : monitor_comparator
// Description : Monitor Comparator (MC) sub-block.
//               Implements a round-robin scheduler that sequentially samples
//               all enabled input channels, opens a gate window (MON_PERIOD),
//               counts edges, and compares the raw edge count against the
//               per-channel MON_MIN / MON_MAX thresholds.
//
//               FSM state encoding (matches specification exactly):
//                 MON_IDLE    = 3'b000
//                 MON_SELECT  = 3'b001
//                 MON_ARM     = 3'b010
//                 MON_COUNT   = 3'b011
//                 MON_COMPARE = 3'b100
//                 MON_WAIT    = 3'b101
//
//               Per-channel threshold buses are passed in flattened form
//               (WIDTH*NUM_CHANNELS bits) since standard Verilog module ports
//               do not support unpacked array ports. mon_period is expected
//               to already be synchronized (quasi-static) into ref_clk.
// -----------------------------------------------------------------------------
module monitor_comparator #(
    parameter NUM_CHANNELS = 4,
    parameter RESULT_WIDTH = 32,
    parameter THRESH_WIDTH = 32,
    parameter CH_IDX_WIDTH = 5   // supports up to 32 channels (spec max 16)
) (
    input  wire                                    ref_clk,
    input  wire                                    ref_rst_n,

    input  wire                                    global_en,
    input  wire                                    mon_mode,        // CTRL.MON_MODE
    input  wire [NUM_CHANNELS-1:0]                 edge_pulse,      // per-channel edge pulses
    input  wire [NUM_CHANNELS-1:0]                 mon_enable,      // MON_ENABLE bitmask
    input  wire [RESULT_WIDTH-1:0]                 mon_period,      // MON_PERIOD gate cycles

    input  wire [NUM_CHANNELS*THRESH_WIDTH-1:0]    mon_min_flat,    // per-channel MON_MIN
    input  wire [NUM_CHANNELS*THRESH_WIDTH-1:0]    mon_max_flat,    // per-channel MON_MAX

    output reg                                     mon_active,      // STATUS.MON_ACTIVE

    output reg  [CH_IDX_WIDTH-1:0]                 mon_channel_active, // 0x1F pattern if idle

    // per-channel results & fault event pulses (ref_clk domain, 1-cycle)
    output reg  [NUM_CHANNELS*RESULT_WIDTH-1:0]    last_result_flat,
    output reg  [NUM_CHANNELS-1:0]                 underflow_evt,
    output reg  [NUM_CHANNELS-1:0]                 overflow_evt,
    output reg  [NUM_CHANNELS-1:0]                 loss_of_clock_evt,
    output reg  [NUM_CHANNELS-1:0]                 recovered_evt,
    output reg  [NUM_CHANNELS-1:0]                 fault_active_level // live, held until next compare
);

    // FSM state encoding - matches specification Section 3.4.2 exactly
    localparam MON_IDLE    = 3'b000;
    localparam MON_SELECT  = 3'b001;
    localparam MON_ARM     = 3'b010;
    localparam MON_COUNT   = 3'b011;
    localparam MON_COMPARE = 3'b100;
    localparam MON_WAIT    = 3'b101;

    reg [2:0]                  state, next_state;
    reg [CH_IDX_WIDTH-1:0]     ch_idx;
    reg [RESULT_WIDTH-1:0]     edge_cnt;
    reg [RESULT_WIDTH-1:0]     gate_cnt;

    integer k;
    reg [CH_IDX_WIDTH-1:0]     cand;
    reg [CH_IDX_WIDTH-1:0]     next_ch;
    reg                        next_ch_valid;

    wire                       any_enabled;
    assign any_enabled = |mon_enable;

    // -------------------------------------------------------------------
    // Round-robin next-channel finder (priority scan starting after ch_idx)
    // -------------------------------------------------------------------
    always @(*) begin
        next_ch_valid = 1'b0;
        next_ch       = ch_idx;
        for (k = 1; k <= NUM_CHANNELS; k = k + 1) begin
            cand = (ch_idx + k[CH_IDX_WIDTH-1:0]) % NUM_CHANNELS;
            if (!next_ch_valid && mon_enable[cand]) begin
                next_ch       = cand;
                next_ch_valid = 1'b1;
            end
        end
    end

    wire [THRESH_WIDTH-1:0] sel_min = mon_min_flat[ch_idx*THRESH_WIDTH +: THRESH_WIDTH];
    wire [THRESH_WIDTH-1:0] sel_max = mon_max_flat[ch_idx*THRESH_WIDTH +: THRESH_WIDTH];
    wire                    sel_edge_pulse = edge_pulse[ch_idx];

    // -------------------------------------------------------------------
    // Next-state logic
    // -------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            MON_IDLE   : begin
                if (global_en && mon_mode && any_enabled)
                    next_state = MON_SELECT;
            end
            MON_SELECT : begin
                if (next_ch_valid)
                    next_state = MON_ARM;
                else
                    next_state = MON_IDLE;
            end
            MON_ARM    : begin
                next_state = MON_COUNT;
            end
            MON_COUNT  : begin
                if (gate_cnt == {RESULT_WIDTH{1'b0}})
                    next_state = MON_COMPARE;
            end
            MON_COMPARE: begin
                next_state = MON_WAIT;
            end
            MON_WAIT   : begin
                if (mon_mode)
                    next_state = MON_SELECT;
                else
                    next_state = MON_IDLE;
            end
            default    : next_state = MON_IDLE;
        endcase
    end

    always @(posedge ref_clk or negedge ref_rst_n) begin
        if (!ref_rst_n)
            state <= MON_IDLE;
        else
            state <= next_state;
    end

    // -------------------------------------------------------------------
    // Datapath
    // -------------------------------------------------------------------
    always @(posedge ref_clk or negedge ref_rst_n) begin
        if (!ref_rst_n) begin
            ch_idx              <= {CH_IDX_WIDTH{1'b0}};
            edge_cnt            <= {RESULT_WIDTH{1'b0}};
            gate_cnt            <= {RESULT_WIDTH{1'b0}};
            last_result_flat    <= {(NUM_CHANNELS*RESULT_WIDTH){1'b0}};
            underflow_evt       <= {NUM_CHANNELS{1'b0}};
            overflow_evt        <= {NUM_CHANNELS{1'b0}};
            loss_of_clock_evt   <= {NUM_CHANNELS{1'b0}};
            recovered_evt       <= {NUM_CHANNELS{1'b0}};
            fault_active_level  <= {NUM_CHANNELS{1'b0}};
        end else begin
            // event pulses default deasserted every cycle (1-cycle pulses)
            underflow_evt     <= {NUM_CHANNELS{1'b0}};
            overflow_evt      <= {NUM_CHANNELS{1'b0}};
            loss_of_clock_evt <= {NUM_CHANNELS{1'b0}};
            recovered_evt     <= {NUM_CHANNELS{1'b0}};

            case (state)
                MON_SELECT : begin
                    if (next_ch_valid)
                        ch_idx <= next_ch;
                end

                MON_ARM    : begin
                    edge_cnt <= {RESULT_WIDTH{1'b0}};
                    gate_cnt <= mon_period; // loaded from MON_PERIOD
                end

                MON_COUNT  : begin
                    if (sel_edge_pulse)
                        edge_cnt <= edge_cnt + 1'b1;
                    if (gate_cnt != {RESULT_WIDTH{1'b0}})
                        gate_cnt <= gate_cnt - 1'b1;
                end

                MON_COMPARE: begin
                    last_result_flat[ch_idx*RESULT_WIDTH +: RESULT_WIDTH] <= edge_cnt;

                    if (edge_cnt == {RESULT_WIDTH{1'b0}}) begin
                        // Loss-of-Clock
                        loss_of_clock_evt[ch_idx] <= 1'b1;
                        fault_active_level[ch_idx] <= 1'b1;
                    end else if (edge_cnt < sel_min) begin
                        // Underflow
                        underflow_evt[ch_idx] <= 1'b1;
                        fault_active_level[ch_idx] <= 1'b1;
                    end else if (edge_cnt > sel_max) begin
                        // Overflow
                        overflow_evt[ch_idx] <= 1'b1;
                        fault_active_level[ch_idx] <= 1'b1;
                    end else begin
                        // Within thresholds : check for recovery transition
                        if (fault_active_level[ch_idx])
                            recovered_evt[ch_idx] <= 1'b1;
                        fault_active_level[ch_idx] <= 1'b0;
                    end
                end

                default : begin
                    // MON_IDLE / MON_WAIT : hold state
                end
            endcase
        end
    end

    // mon_active status and current active channel index (combinational)
    always @(*) begin
        mon_active = (state != MON_IDLE);
        if (state == MON_IDLE)
            mon_channel_active = {CH_IDX_WIDTH{1'b1}}; // 0x1F pattern when idle
        else
            mon_channel_active = ch_idx;
    end

endmodule
