// -----------------------------------------------------------------------------
// Module      : apb_regs
// Description : APB Register Interface (ARI) sub-block, 
//               Implements a zero-wait-state APB4 slave (pready is always
//               asserted the same cycle as psel & penable) with 16-bit
//               addressing (64 KB space) and 32-bit data width, complete
//               address decode, and the entire register map described in
//               Section 5 of the specification:
//                 0x00-0x2F : Global Control & Status registers
//                 0x100-0x1FF (0x10 stride) : Per-channel monitor registers
//                 0x200-0x21B : Global Monitor Summary registers
// -----------------------------------------------------------------------------
module apb_regs #(
    parameter NUM_CHANNELS = 4,
    parameter ADDR_WIDTH   = 16,
    parameter RESULT_WIDTH = 32,
    parameter THRESH_WIDTH = 32
) (
    input  wire                                 bus_clk,
    input  wire                                 bus_rst_n,

    // ---------------- APB4 slave interface -----------------------------
    input  wire [ADDR_WIDTH-1:0]                paddr,
    input  wire [31:0]                          pwdata,
    output reg  [31:0]                          prdata,
    input  wire                                 pwrite,
    input  wire                                 psel,
    input  wire                                 penable,
    output wire                                 pready,
    output reg                                  pslverr,

    // ---------------- Config outputs (bus_clk domain values) -----------
    // (synchronized into ref_clk domain externally, at the top level)
    output reg                                  global_en_o,
    output reg                                  mon_mode_o,
    output reg                                  irq_global_en_o,
    output reg                                  irq_edge_o,
    output reg                                  measure_continuous_o,
    output reg  [RESULT_WIDTH-1:0]              mon_period_o,
    output reg  [3:0]                           measure_sel_o,
    output reg  [RESULT_WIDTH-1:0]              measure_gate_o,
    output reg  [NUM_CHANNELS-1:0]              mon_enable_o,
    output reg  [NUM_CHANNELS*THRESH_WIDTH-1:0] mon_min_flat_o,
    output reg  [NUM_CHANNELS*THRESH_WIDTH-1:0] mon_max_flat_o,
    output reg                                  start_pulse_o,   // 1 bus_clk-cycle pulse
    output reg                                  abort_pulse_o,   // 1 bus_clk-cycle pulse

    // ---------------- Status / result inputs ----------------------------
    // (already synchronized into bus_clk domain at the top level)
    input  wire                                 mon_active_i,
    input  wire                                 measure_busy_i,
    input  wire                                 ref_clk_ok_i,
    input  wire [RESULT_WIDTH-1:0]              measure_result_i,
    input  wire                                 measure_done_evt_i,   // bus_clk pulse
    input  wire [NUM_CHANNELS*RESULT_WIDTH-1:0] last_result_flat_i,
    input  wire [NUM_CHANNELS-1:0]              fault_active_i,       // live level
    input  wire [NUM_CHANNELS-1:0]              underflow_evt_i,      // bus_clk pulses
    input  wire [NUM_CHANNELS-1:0]              overflow_evt_i,
    input  wire [NUM_CHANNELS-1:0]              loss_of_clock_evt_i,
    input  wire [NUM_CHANNELS-1:0]              recovered_evt_i,
    input  wire [4:0]                           mon_channel_active_i,

    // ---------------- Interrupt output -----------------------------------
    output wire                                 irq
);

    // ================================================================
    // Address map constants
    // ================================================================
    localparam ADDR_CTRL            = 8'h00;
    localparam ADDR_STATUS          = 8'h04;
    localparam ADDR_IRQ_EN          = 8'h08;
    localparam ADDR_IRQ_STATUS      = 8'h0C;
    localparam ADDR_IRQ_CLR         = 8'h10;
    localparam ADDR_MON_PERIOD      = 8'h14;
    localparam ADDR_MEASURE_SEL     = 8'h18;
    localparam ADDR_MEASURE_GATE    = 8'h1C;
    localparam ADDR_MEASURE_CTRL    = 8'h20;
    localparam ADDR_MEASURE_RESULT  = 8'h24;
    localparam ADDR_REVISION        = 8'h28;
    localparam ADDR_NUM_CHANNELS    = 8'h2C;

    localparam ADDR_MON_ENABLE          = 8'h00; // within page 0x02xx
    localparam ADDR_FAULT_SUMMARY       = 8'h04;
    localparam ADDR_LOC_SUMMARY         = 8'h08;
    localparam ADDR_UNDERFLOW_SUMMARY   = 8'h0C;
    localparam ADDR_OVERFLOW_SUMMARY    = 8'h10;
    localparam ADDR_RECOVERED_SUMMARY   = 8'h14;
    localparam ADDR_MON_CHANNEL_ACTIVE  = 8'h18;

    wire [7:0] page = paddr[15:8];
    wire [7:0] low  = paddr[7:0];

    wire [3:0] ch_index = paddr[7:4];
    wire [1:0] ch_reg   = paddr[3:2];
    wire       ch_in_range = (ch_index < NUM_CHANNELS);

    // ================================================================
    // Address validity decode
    // ================================================================
    reg addr_valid;
    always @(*) begin
        addr_valid = 1'b0;
        case (page)
            8'h00 : begin
                case (low)
                    ADDR_CTRL, ADDR_STATUS, ADDR_IRQ_EN, ADDR_IRQ_STATUS,
                    ADDR_IRQ_CLR, ADDR_MON_PERIOD, ADDR_MEASURE_SEL,
                    ADDR_MEASURE_GATE, ADDR_MEASURE_CTRL, ADDR_MEASURE_RESULT,
                    ADDR_REVISION, ADDR_NUM_CHANNELS : addr_valid = 1'b1;
                    default : addr_valid = 1'b0;
                endcase
            end
            8'h01 : begin
                addr_valid = ch_in_range && (paddr[1:0] == 2'b00);
            end
            8'h02 : begin
                case (low)
                    ADDR_MON_ENABLE, ADDR_FAULT_SUMMARY, ADDR_LOC_SUMMARY,
                    ADDR_UNDERFLOW_SUMMARY, ADDR_OVERFLOW_SUMMARY,
                    ADDR_RECOVERED_SUMMARY, ADDR_MON_CHANNEL_ACTIVE : addr_valid = 1'b1;
                    default : addr_valid = 1'b0;
                endcase
            end
            default : addr_valid = 1'b0;
        endcase
    end

    // Zero-wait-state APB4 slave : ready combinationally with the access phase
    assign pready = 1'b1;

    wire access_phase = psel && penable;
    wire wr_en = access_phase && pwrite;
    wire rd_en = access_phase && !pwrite;

    // ================================================================
    // Internal register storage
    // ================================================================
    // IRQ_STATUS sticky bits (RW1C)
    reg meas_done_irq_sticky, mon_fault_irq_sticky, mon_clear_irq_sticky;
    // IRQ_EN bits
    reg en_measure_done, en_mon_fault, en_mon_clear;

    // MEASURE_CTRL status bits (sticky, cleared on new START or IRQ_CLR)
    reg measure_done_sticky, result_valid_sticky;

    // Per-channel RW1C fault sticky bits
    reg [NUM_CHANNELS-1:0] ch_underflow_sticky;
    reg [NUM_CHANNELS-1:0] ch_overflow_sticky;
    reg [NUM_CHANNELS-1:0] ch_loc_sticky;
    reg [NUM_CHANNELS-1:0] ch_recovered_sticky;

    // Global monitor summary RW1C sticky bits
    reg loc_summary_sticky, underflow_summary_sticky;
    reg overflow_summary_sticky, recovered_summary_sticky;

    integer c;

    // ================================================================
    // Event detection : OR-reduce per-channel event pulses for
    // IRQ_STATUS / summary register update
    // ================================================================
    wire mon_fault_evt_any  = (|underflow_evt_i) | (|overflow_evt_i) | (|loss_of_clock_evt_i);
    wire mon_clear_evt_any  = (|recovered_evt_i);

    // ANY_FAULT (live) : OR of all live per-channel fault_active bits
    wire any_fault_live = |fault_active_i;

    // ================================================================
    // Synchronous write / event-update logic
    // ================================================================
    always @(posedge bus_clk or negedge bus_rst_n) begin
        if (!bus_rst_n) begin
            global_en_o            <= 1'b0;
            mon_mode_o              <= 1'b0;
            irq_global_en_o         <= 1'b0;
            irq_edge_o              <= 1'b0;
            measure_continuous_o    <= 1'b0;

            mon_period_o            <= 32'h000186A0;
            measure_sel_o           <= 4'h0;
            measure_gate_o          <= 32'h000186A0;
            mon_enable_o            <= {NUM_CHANNELS{1'b0}};

            start_pulse_o           <= 1'b0;
            abort_pulse_o           <= 1'b0;

            en_measure_done         <= 1'b0;
            en_mon_fault            <= 1'b0;
            en_mon_clear            <= 1'b0;

            meas_done_irq_sticky    <= 1'b0;
            mon_fault_irq_sticky    <= 1'b0;
            mon_clear_irq_sticky    <= 1'b0;

            measure_done_sticky     <= 1'b0;
            result_valid_sticky     <= 1'b0;

            ch_underflow_sticky     <= {NUM_CHANNELS{1'b0}};
            ch_overflow_sticky      <= {NUM_CHANNELS{1'b0}};
            ch_loc_sticky           <= {NUM_CHANNELS{1'b0}};
            ch_recovered_sticky     <= {NUM_CHANNELS{1'b0}};

            loc_summary_sticky        <= 1'b0;
            underflow_summary_sticky  <= 1'b0;
            overflow_summary_sticky   <= 1'b0;
            recovered_summary_sticky  <= 1'b0;

            for (c = 0; c < NUM_CHANNELS; c = c + 1) begin
                mon_min_flat_o[c*THRESH_WIDTH +: THRESH_WIDTH] <= {THRESH_WIDTH{1'b0}};
                mon_max_flat_o[c*THRESH_WIDTH +: THRESH_WIDTH] <= {THRESH_WIDTH{1'b1}};
            end
        end else begin
            // ---- self-clearing / 1-cycle pulses : default deassert ----
            start_pulse_o <= 1'b0;
            abort_pulse_o <= 1'b0;

            // ---- sticky bits SET on synchronized hardware events -------
            if (measure_done_evt_i) begin
                measure_done_sticky <= 1'b1;
                result_valid_sticky <= 1'b1;
                meas_done_irq_sticky <= 1'b1;
            end
            if (mon_fault_evt_any)
                mon_fault_irq_sticky <= 1'b1;
            if (mon_clear_evt_any)
                mon_clear_irq_sticky <= 1'b1;

            for (c = 0; c < NUM_CHANNELS; c = c + 1) begin
                if (underflow_evt_i[c])
                    ch_underflow_sticky[c] <= 1'b1;
                if (overflow_evt_i[c])
                    ch_overflow_sticky[c] <= 1'b1;
                if (loss_of_clock_evt_i[c])
                    ch_loc_sticky[c] <= 1'b1;
                if (recovered_evt_i[c])
                    ch_recovered_sticky[c] <= 1'b1;
            end

            if (|underflow_evt_i)      underflow_summary_sticky <= 1'b1;
            if (|overflow_evt_i)       overflow_summary_sticky  <= 1'b1;
            if (|loss_of_clock_evt_i)  loc_summary_sticky        <= 1'b1;
            if (|recovered_evt_i)      recovered_summary_sticky  <= 1'b1;

            // ---- APB register writes -----------------------------------
            if (wr_en && addr_valid) begin
                case (page)
                    8'h00 : begin
                        case (low)
                            ADDR_CTRL : begin
                                global_en_o          <= pwdata[0];
                                mon_mode_o           <= pwdata[1];
                                irq_global_en_o      <= pwdata[2];
                                irq_edge_o           <= pwdata[3];
                                measure_continuous_o <= pwdata[4];
                            end
                            ADDR_IRQ_EN : begin
                                en_measure_done <= pwdata[0];
                                en_mon_fault    <= pwdata[1];
                                en_mon_clear    <= pwdata[2];
                            end
                            ADDR_IRQ_STATUS : begin
                                // RW1C : write 1 clears, write 0 no effect
                                if (pwdata[0]) meas_done_irq_sticky <= 1'b0;
                                if (pwdata[1]) mon_fault_irq_sticky <= 1'b0;
                                if (pwdata[2]) mon_clear_irq_sticky <= 1'b0;
                            end
                            ADDR_IRQ_CLR : begin
                                // WO : write 1 clears corresponding IRQ_STATUS bit
                                if (pwdata[0]) meas_done_irq_sticky <= 1'b0;
                                if (pwdata[1]) mon_fault_irq_sticky <= 1'b0;
                                if (pwdata[2]) mon_clear_irq_sticky <= 1'b0;
                            end
                            ADDR_MON_PERIOD :
                                mon_period_o <= pwdata[RESULT_WIDTH-1:0];
                            ADDR_MEASURE_SEL :
                                measure_sel_o <= pwdata[3:0];
                            ADDR_MEASURE_GATE :
                                measure_gate_o <= pwdata[RESULT_WIDTH-1:0];
                            ADDR_MEASURE_CTRL : begin
                                // START / ABORT : self-clearing WO pulses
                                if (pwdata[0]) begin
                                    start_pulse_o       <= 1'b1;
                                    measure_done_sticky <= 1'b0;
                                    result_valid_sticky <= 1'b0;
                                end
                                if (pwdata[1])
                                    abort_pulse_o <= 1'b1;
                            end
                            // ADDR_MEASURE_RESULT, ADDR_REVISION,
                            // ADDR_NUM_CHANNELS, ADDR_STATUS : read-only,
                            // writes ignored with no side effects.
                            default : ; // reserved / RO : ignored
                        endcase
                    end

                    8'h01 : begin
                        if (ch_in_range) begin
                            case (ch_reg)
                                2'b00 : mon_min_flat_o[ch_index*THRESH_WIDTH +: THRESH_WIDTH] <= pwdata;
                                2'b01 : mon_max_flat_o[ch_index*THRESH_WIDTH +: THRESH_WIDTH] <= pwdata;
                                2'b10 : ; // LAST_RESULT : RO, ignored
                                2'b11 : begin
                                    // CH[i]_FAULT : RW1C bits, FAULT_ACTIVE is RO
                                    if (pwdata[0]) ch_underflow_sticky[ch_index] <= 1'b0;
                                    if (pwdata[1]) ch_overflow_sticky[ch_index]  <= 1'b0;
                                    if (pwdata[2]) ch_loc_sticky[ch_index]       <= 1'b0;
                                    if (pwdata[4]) ch_recovered_sticky[ch_index] <= 1'b0;
                                end
                            endcase
                        end
                    end

                    8'h02 : begin
                        case (low)
                            ADDR_MON_ENABLE :
                                mon_enable_o <= pwdata[NUM_CHANNELS-1:0];
                            ADDR_LOC_SUMMARY :
                                if (pwdata[0]) loc_summary_sticky <= 1'b0;
                            ADDR_UNDERFLOW_SUMMARY :
                                if (pwdata[0]) underflow_summary_sticky <= 1'b0;
                            ADDR_OVERFLOW_SUMMARY :
                                if (pwdata[0]) overflow_summary_sticky <= 1'b0;
                            ADDR_RECOVERED_SUMMARY :
                                if (pwdata[0]) recovered_summary_sticky <= 1'b0;
                            // ADDR_FAULT_SUMMARY, ADDR_MON_CHANNEL_ACTIVE : RO
                            default : ; // ignored
                        endcase
                    end

                    default : ; // unreachable (addr_valid already gates page)
                endcase
            end

          
        end
    end

    // ================================================================
    // pslverr : combinational, asserted for invalid address on any access
    // ================================================================
    always @(*) begin
        pslverr = access_phase && !addr_valid;
    end

    // ================================================================
    // Read data mux (combinational)
    // ================================================================
    always @(*) begin
        prdata = 32'h00000000;
        if (addr_valid) begin
            case (page)
                8'h00 : begin
                    case (low)
                        ADDR_CTRL : prdata = {27'b0, measure_continuous_o, irq_edge_o,
                                               irq_global_en_o, mon_mode_o, global_en_o};
                        ADDR_STATUS : prdata = {27'b0, any_fault_live,
                                                 ref_clk_ok_i, measure_done_sticky,
                                                 measure_busy_i, mon_active_i};
                        ADDR_IRQ_EN : prdata = {29'b0, en_mon_clear, en_mon_fault, en_measure_done};
                        ADDR_IRQ_STATUS : prdata = {29'b0, mon_clear_irq_sticky,
                                                     mon_fault_irq_sticky, meas_done_irq_sticky};
                        ADDR_IRQ_CLR : prdata = 32'h00000000; // WO reads as 0
                        ADDR_MON_PERIOD : prdata = mon_period_o;
                        ADDR_MEASURE_SEL : prdata = {28'b0, measure_sel_o};
                        ADDR_MEASURE_GATE : prdata = measure_gate_o;
                        ADDR_MEASURE_CTRL : prdata = {28'b0, result_valid_sticky,
                                                       measure_done_sticky, 2'b00};
                        ADDR_MEASURE_RESULT : prdata = measure_result_i;
                        ADDR_REVISION : prdata = 32'h00010000;
                        ADDR_NUM_CHANNELS : prdata = {27'b0, NUM_CHANNELS[4:0]};
                        default : prdata = 32'h00000000;
                    endcase
                end
                8'h01 : begin
                    if (ch_in_range) begin
                        case (ch_reg)
                            2'b00 : prdata = mon_min_flat_o[ch_index*THRESH_WIDTH +: THRESH_WIDTH];
                            2'b01 : prdata = mon_max_flat_o[ch_index*THRESH_WIDTH +: THRESH_WIDTH];
                            2'b10 : prdata = last_result_flat_i[ch_index*RESULT_WIDTH +: RESULT_WIDTH];
                            2'b11 : prdata = {27'b0, ch_recovered_sticky[ch_index],
                                               fault_active_i[ch_index], ch_loc_sticky[ch_index],
                                               ch_overflow_sticky[ch_index], ch_underflow_sticky[ch_index]};
                        endcase
                    end
                end
                8'h02 : begin
                    case (low)
                        ADDR_MON_ENABLE : prdata = {{(32-NUM_CHANNELS){1'b0}}, mon_enable_o};
                        ADDR_FAULT_SUMMARY : prdata = {31'b0, (|fault_active_i)};
                        ADDR_LOC_SUMMARY : prdata = {31'b0, loc_summary_sticky};
                        ADDR_UNDERFLOW_SUMMARY : prdata = {31'b0, underflow_summary_sticky};
                        ADDR_OVERFLOW_SUMMARY : prdata = {31'b0, overflow_summary_sticky};
                        ADDR_RECOVERED_SUMMARY : prdata = {31'b0, recovered_summary_sticky};
                        ADDR_MON_CHANNEL_ACTIVE : prdata = {27'b0, mon_channel_active_i};
                        default : prdata = 32'h00000000;
                    endcase
                end
                default : prdata = 32'h00000000;
            endcase
        end
    end

    // ================================================================
    // Interrupt Generator instance
    // ================================================================
    interrupt_gen u_irq_gen (
        .bus_clk         (bus_clk),
        .bus_rst_n       (bus_rst_n),
        .irq_global_en   (irq_global_en_o),
        .irq_edge        (irq_edge_o),
        .en_measure_done (en_measure_done),
        .en_mon_fault    (en_mon_fault),
        .en_mon_clear    (en_mon_clear),
        .measure_done_evt(measure_done_evt_i),
        .mon_fault_evt   (mon_fault_evt_any),
        .mon_clear_evt   (mon_clear_evt_any),
        .pending_masked  ((meas_done_irq_sticky & en_measure_done) |
                           (mon_fault_irq_sticky & en_mon_fault)    |
                           (mon_clear_irq_sticky & en_mon_clear)),
        .irq             (irq)
    );

endmodule
