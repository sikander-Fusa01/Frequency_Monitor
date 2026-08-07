// -----------------------------------------------------------------------------
// Module      : freq_monitor_top
// Description : Top-level Frequency Monitor IP, 
//               Instantiates and connects all eight major functional blocks:
//                 1. Input Clock Synchronizer (ICS)      -> ch_input_sync_edge
//                 2. Edge Detector (ED)                  -> ch_input_sync_edge
//                 3. Reference Timer / Gate Generator     -> (inside ME / MC)
//                 4. Measurement Engine (ME)              -> measurement_engine
//                 5. Monitor Comparator (MC)              -> monitor_comparator
//                 6. Fault Accumulator & Status (FAS)     -> apb_regs (sticky
//                                                            storage) + summary
//                                                            read-mux logic
//                 7. Interrupt Generator (IG)             -> interrupt_gen
//                                                            (instantiated
//                                                            inside apb_regs)
//                 8. APB Register Interface (ARI)         -> apb_regs
//
//               Three clock domains are used : clk_in[0..N-1] (asynchronous input clocks),
//               ref_clk (reference / measurement domain) and bus_clk (APB
//               register domain). All CDC crossings use the sync2ff (level)
//               and pulse_sync (toggle-based single pulse) primitives.

// -----------------------------------------------------------------------------
module freq_monitor_top #(
    parameter NUM_CHANNELS    = 4,           // 1-16 input channels
    parameter REF_CLK_FREQ_HZ = 100_000_000, // informational / software use
    parameter SYNC_STAGES     = 2,           // 2-4 synchronizer stages
    parameter RESULT_WIDTH    = 32,
    parameter THRESH_WIDTH    = 32,
    parameter ADDR_WIDTH      = 16
) (
    // ---------------- Clock & Reset Interface ---------------------------
    input  wire                          ref_clk,
    input  wire                          ref_rst_n,
    input  wire                          bus_clk,
    input  wire                          bus_rst_n,

    // ---------------- Input Clock Interface ------------------------------
    input  wire [NUM_CHANNELS-1:0]       clk_in,

    // ---------------- APB4 Slave Interface --------------------------------
    input  wire [ADDR_WIDTH-1:0]         paddr,
    input  wire [31:0]                   pwdata,
    output wire [31:0]                   prdata,
    input  wire                          pwrite,
    input  wire                          psel,
    input  wire                          penable,
    output wire                          pready,
    output wire                          pslverr,

    // ---------------- Interrupt Interface ----------------------------------
    output wire                          irq
);

    // ================================================================
    // 1 & 2. Input Clock Synchronizer + Edge Detector (per channel)
    // ================================================================
    wire [NUM_CHANNELS-1:0] edge_pulse_ref;

    genvar gi;
    generate
        for (gi = 0; gi < NUM_CHANNELS; gi = gi + 1) begin : g_ch_sync
            ch_input_sync_edge #(
                .SYNC_STAGES (SYNC_STAGES)
            ) u_ch_sync_edge (
                .ref_clk    (ref_clk),
                .ref_rst_n  (ref_rst_n),
                .clk_in     (clk_in[gi]),
                .edge_pulse (edge_pulse_ref[gi])
            );
        end
    endgenerate

    // ================================================================
    // Reference-clock domain "REF_CLK_OK" heartbeat generator
    // (toggles every ref_clk cycle; synchronized + toggle-detected in
    // bus_clk domain to prove the reference clock is actively toggling)
    // ================================================================
    reg ref_heartbeat;
    always @(posedge ref_clk or negedge ref_rst_n) begin
        if (!ref_rst_n)
            ref_heartbeat <= 1'b0;
        else
            ref_heartbeat <= ~ref_heartbeat;
    end

    wire ref_heartbeat_sync;
    reg  ref_heartbeat_sync_d;
    wire ref_clk_ok_bus;

    sync2ff #(.WIDTH(1), .STAGES(2)) u_sync_heartbeat (
        .clk   (bus_clk),
        .rst_n (bus_rst_n),
        .din   (ref_heartbeat),
        .dout  (ref_heartbeat_sync)
    );

    always @(posedge bus_clk or negedge bus_rst_n) begin
        if (!bus_rst_n)
            ref_heartbeat_sync_d <= 1'b0;
        else
            ref_heartbeat_sync_d <= ref_heartbeat_sync;
    end

    // toggling detected => clock alive; held true once toggling observed
    assign ref_clk_ok_bus = ref_heartbeat_sync ^ ref_heartbeat_sync_d;

    reg ref_clk_ok_latched;
    always @(posedge bus_clk or negedge bus_rst_n) begin
        if (!bus_rst_n)
            ref_clk_ok_latched <= 1'b0;
        else if (ref_clk_ok_bus)
            ref_clk_ok_latched <= 1'b1;
    end

    // ================================================================
    // Config signals (bus_clk domain, from apb_regs) synchronized into
    // the ref_clk domain (Section 3.3.3 : APB Control -> Reference Clock)
    // ================================================================
    wire global_en_bus, mon_mode_bus, irq_global_en_bus, irq_edge_bus, measure_continuous_bus;
    wire [RESULT_WIDTH-1:0] mon_period_bus, measure_gate_bus;
    wire [3:0]              measure_sel_bus;
    wire [NUM_CHANNELS-1:0] mon_enable_bus;
    wire [NUM_CHANNELS*THRESH_WIDTH-1:0] mon_min_flat_bus, mon_max_flat_bus;
    wire start_pulse_bus, abort_pulse_bus;

    wire global_en_ref, mon_mode_ref, measure_continuous_ref;
    wire [RESULT_WIDTH-1:0] mon_period_ref, measure_gate_ref;
    wire [3:0]              measure_sel_ref;
    wire [NUM_CHANNELS-1:0] mon_enable_ref;
    wire [NUM_CHANNELS*THRESH_WIDTH-1:0] mon_min_flat_ref, mon_max_flat_ref;
    wire start_pulse_ref, abort_pulse_ref;

    // Single-bit control synchronizers (double-flop, Section 3.3.3)
    sync2ff #(.WIDTH(1), .STAGES(2)) u_sync_global_en (
        .clk(ref_clk), .rst_n(ref_rst_n), .din(global_en_bus), .dout(global_en_ref));

    sync2ff #(.WIDTH(1), .STAGES(2)) u_sync_mon_mode (
        .clk(ref_clk), .rst_n(ref_rst_n), .din(mon_mode_bus), .dout(mon_mode_ref));

    sync2ff #(.WIDTH(1), .STAGES(2)) u_sync_meas_cont (
        .clk(ref_clk), .rst_n(ref_rst_n), .din(measure_continuous_bus), .dout(measure_continuous_ref));

    // Multi-bit quasi-static configuration buses (double-flop synchronized;
    // software is required to hold configuration stable while GLOBAL_EN /
    // MON_MODE is active, consistent with Section 3.3.3 guidance)
    sync2ff #(.WIDTH(RESULT_WIDTH), .STAGES(2)) u_sync_mon_period (
        .clk(ref_clk), .rst_n(ref_rst_n), .din(mon_period_bus), .dout(mon_period_ref));

    sync2ff #(.WIDTH(RESULT_WIDTH), .STAGES(2)) u_sync_measure_gate (
        .clk(ref_clk), .rst_n(ref_rst_n), .din(measure_gate_bus), .dout(measure_gate_ref));

    sync2ff #(.WIDTH(4), .STAGES(2)) u_sync_measure_sel (
        .clk(ref_clk), .rst_n(ref_rst_n), .din(measure_sel_bus), .dout(measure_sel_ref));

    sync2ff #(.WIDTH(NUM_CHANNELS), .STAGES(2)) u_sync_mon_enable (
        .clk(ref_clk), .rst_n(ref_rst_n), .din(mon_enable_bus), .dout(mon_enable_ref));

    sync2ff #(.WIDTH(NUM_CHANNELS*THRESH_WIDTH), .STAGES(2)) u_sync_mon_min (
        .clk(ref_clk), .rst_n(ref_rst_n), .din(mon_min_flat_bus), .dout(mon_min_flat_ref));

    sync2ff #(.WIDTH(NUM_CHANNELS*THRESH_WIDTH), .STAGES(2)) u_sync_mon_max (
        .clk(ref_clk), .rst_n(ref_rst_n), .din(mon_max_flat_bus), .dout(mon_max_flat_ref));

    // Single-cycle pulse synchronizers (Section 3.3.2 handshake style)
    pulse_sync u_sync_start (
        .src_clk(bus_clk), .src_rst_n(bus_rst_n), .src_pulse(start_pulse_bus),
        .dst_clk(ref_clk), .dst_rst_n(ref_rst_n), .dst_pulse(start_pulse_ref));

    pulse_sync u_sync_abort (
        .src_clk(bus_clk), .src_rst_n(bus_rst_n), .src_pulse(abort_pulse_bus),
        .dst_clk(ref_clk), .dst_rst_n(ref_rst_n), .dst_pulse(abort_pulse_ref));

    // ================================================================
    // 4. Measurement Engine (ME)
    // ================================================================
    wire                     measure_busy_ref, measure_done_pulse_ref, result_valid_ref;
    wire [RESULT_WIDTH-1:0]  measure_result_ref;

    measurement_engine #(
        .NUM_CHANNELS (NUM_CHANNELS),
        .RESULT_WIDTH (RESULT_WIDTH)
    ) u_measurement_engine (
        .ref_clk             (ref_clk),
        .ref_rst_n           (ref_rst_n),
        .global_en           (global_en_ref),
        .edge_pulse          (edge_pulse_ref),
        .measure_sel         (measure_sel_ref),
        .measure_gate        (measure_gate_ref),
        .measure_continuous  (measure_continuous_ref),
        .start_pulse         (start_pulse_ref),
        .abort_pulse         (abort_pulse_ref),
        .measure_busy        (measure_busy_ref),
        .measure_done_pulse  (measure_done_pulse_ref),
        .result_valid        (result_valid_ref),
        .measure_result      (measure_result_ref)
    );

    // ================================================================
    // 5. Monitor Comparator (MC)
    // ================================================================
    wire                          mon_active_ref;
    wire [4:0]                    mon_channel_active_ref;
    wire [NUM_CHANNELS*RESULT_WIDTH-1:0] last_result_flat_ref;
    wire [NUM_CHANNELS-1:0]       underflow_evt_ref, overflow_evt_ref;
    wire [NUM_CHANNELS-1:0]       loss_of_clock_evt_ref, recovered_evt_ref;
    wire [NUM_CHANNELS-1:0]       fault_active_ref;

    monitor_comparator #(
        .NUM_CHANNELS (NUM_CHANNELS),
        .RESULT_WIDTH (RESULT_WIDTH),
        .THRESH_WIDTH (THRESH_WIDTH),
        .CH_IDX_WIDTH (5)
    ) u_monitor_comparator (
        .ref_clk             (ref_clk),
        .ref_rst_n           (ref_rst_n),
        .global_en           (global_en_ref),
        .mon_mode            (mon_mode_ref),
        .edge_pulse          (edge_pulse_ref),
        .mon_enable          (mon_enable_ref),
        .mon_period          (mon_period_ref),
        .mon_min_flat        (mon_min_flat_ref),
        .mon_max_flat        (mon_max_flat_ref),
        .mon_active          (mon_active_ref),
        .mon_channel_active  (mon_channel_active_ref),
        .last_result_flat    (last_result_flat_ref),
        .underflow_evt       (underflow_evt_ref),
        .overflow_evt        (overflow_evt_ref),
        .loss_of_clock_evt   (loss_of_clock_evt_ref),
        .recovered_evt       (recovered_evt_ref),
        .fault_active_level  (fault_active_ref)
    );

    // ================================================================
    // Status / result CDC : ref_clk domain -> bus_clk domain
    // ================================================================
    wire measure_busy_bus, mon_active_bus;
    wire [RESULT_WIDTH-1:0] measure_result_bus;
    wire [NUM_CHANNELS*RESULT_WIDTH-1:0] last_result_flat_bus;
    wire [NUM_CHANNELS-1:0] fault_active_bus;
    wire [4:0] mon_channel_active_bus;
    wire measure_done_evt_bus;
    wire [NUM_CHANNELS-1:0] underflow_evt_bus, overflow_evt_bus;
    wire [NUM_CHANNELS-1:0] loss_of_clock_evt_bus, recovered_evt_bus;

    sync2ff #(.WIDTH(1), .STAGES(2)) u_sync_measure_busy (
        .clk(bus_clk), .rst_n(bus_rst_n), .din(measure_busy_ref), .dout(measure_busy_bus));

    sync2ff #(.WIDTH(1), .STAGES(2)) u_sync_mon_active (
        .clk(bus_clk), .rst_n(bus_rst_n), .din(mon_active_ref), .dout(mon_active_bus));

    sync2ff #(.WIDTH(RESULT_WIDTH), .STAGES(2)) u_sync_measure_result (
        .clk(bus_clk), .rst_n(bus_rst_n), .din(measure_result_ref), .dout(measure_result_bus));

    sync2ff #(.WIDTH(NUM_CHANNELS*RESULT_WIDTH), .STAGES(2)) u_sync_last_result (
        .clk(bus_clk), .rst_n(bus_rst_n), .din(last_result_flat_ref), .dout(last_result_flat_bus));

    sync2ff #(.WIDTH(NUM_CHANNELS), .STAGES(2)) u_sync_fault_active (
        .clk(bus_clk), .rst_n(bus_rst_n), .din(fault_active_ref), .dout(fault_active_bus));

    sync2ff #(.WIDTH(5), .STAGES(2)) u_sync_mon_ch_active (
        .clk(bus_clk), .rst_n(bus_rst_n), .din(mon_channel_active_ref), .dout(mon_channel_active_bus));

    pulse_sync u_sync_measure_done (
        .src_clk(ref_clk), .src_rst_n(ref_rst_n), .src_pulse(measure_done_pulse_ref),
        .dst_clk(bus_clk), .dst_rst_n(bus_rst_n), .dst_pulse(measure_done_evt_bus));

    generate
        for (gi = 0; gi < NUM_CHANNELS; gi = gi + 1) begin : g_fault_pulse_sync
            pulse_sync u_sync_underflow (
                .src_clk(ref_clk), .src_rst_n(ref_rst_n), .src_pulse(underflow_evt_ref[gi]),
                .dst_clk(bus_clk), .dst_rst_n(bus_rst_n), .dst_pulse(underflow_evt_bus[gi]));

            pulse_sync u_sync_overflow (
                .src_clk(ref_clk), .src_rst_n(ref_rst_n), .src_pulse(overflow_evt_ref[gi]),
                .dst_clk(bus_clk), .dst_rst_n(bus_rst_n), .dst_pulse(overflow_evt_bus[gi]));

            pulse_sync u_sync_loc (
                .src_clk(ref_clk), .src_rst_n(ref_rst_n), .src_pulse(loss_of_clock_evt_ref[gi]),
                .dst_clk(bus_clk), .dst_rst_n(bus_rst_n), .dst_pulse(loss_of_clock_evt_bus[gi]));

            pulse_sync u_sync_recovered (
                .src_clk(ref_clk), .src_rst_n(ref_rst_n), .src_pulse(recovered_evt_ref[gi]),
                .dst_clk(bus_clk), .dst_rst_n(bus_rst_n), .dst_pulse(recovered_evt_bus[gi]));
        end
    endgenerate

    // ================================================================
    // 8. APB Register Interface (ARI) + Fault Accumulator/Status (FAS)
    //    + Interrupt Generator (IG, instantiated inside apb_regs)
    // ================================================================
    apb_regs #(
        .NUM_CHANNELS (NUM_CHANNELS),
        .ADDR_WIDTH   (ADDR_WIDTH),
        .RESULT_WIDTH (RESULT_WIDTH),
        .THRESH_WIDTH (THRESH_WIDTH)
    ) u_apb_regs (
        .bus_clk               (bus_clk),
        .bus_rst_n              (bus_rst_n),

        .paddr                  (paddr),
        .pwdata                 (pwdata),
        .prdata                 (prdata),
        .pwrite                 (pwrite),
        .psel                   (psel),
        .penable                (penable),
        .pready                 (pready),
        .pslverr                (pslverr),

        .global_en_o            (global_en_bus),
        .mon_mode_o             (mon_mode_bus),
        .irq_global_en_o        (irq_global_en_bus),
        .irq_edge_o             (irq_edge_bus),
        .measure_continuous_o   (measure_continuous_bus),
        .mon_period_o           (mon_period_bus),
        .measure_sel_o          (measure_sel_bus),
        .measure_gate_o         (measure_gate_bus),
        .mon_enable_o           (mon_enable_bus),
        .mon_min_flat_o         (mon_min_flat_bus),
        .mon_max_flat_o         (mon_max_flat_bus),
        .start_pulse_o          (start_pulse_bus),
        .abort_pulse_o          (abort_pulse_bus),

        .mon_active_i           (mon_active_bus),
        .measure_busy_i         (measure_busy_bus),
        .ref_clk_ok_i           (ref_clk_ok_latched),
        .measure_result_i       (measure_result_bus),
        .measure_done_evt_i     (measure_done_evt_bus),
        .last_result_flat_i     (last_result_flat_bus),
        .fault_active_i         (fault_active_bus),
        .underflow_evt_i        (underflow_evt_bus),
        .overflow_evt_i         (overflow_evt_bus),
        .loss_of_clock_evt_i    (loss_of_clock_evt_bus),
        .recovered_evt_i        (recovered_evt_bus),
        .mon_channel_active_i   (mon_channel_active_bus),

        .irq                    (irq)
    );

endmodule
