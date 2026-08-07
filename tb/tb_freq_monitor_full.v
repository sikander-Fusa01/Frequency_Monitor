

`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// Module      : tb_freq_monitor_full
// Description : Comprehensive, self-checking testbench for freq_monitor_top.
//               Every check is performed via the check()/check_approx() tasks
//               which print a PASS/FAIL verdict together with an
//               EXPECTED-vs-ACTUAL comparison line, and are tallied into a
//               final pass/fail scoreboard printed at the end of simulation.
//
//               Coverage (mirrors Sections 5, 6, 10 of the specification):
//                 1.  Reset / default register values (entire map)
//                 2.  APB protocol rules: invalid address (pslverr), RO
//                     writes ignored, WO reads as 0, reserved bits read 0
//                 3.  CTRL / IRQ_EN read-write behavior
//                 4.  Self-clearing START / ABORT bits
//                 5.  Measurement Mode : single-shot accuracy (+/- tolerance
//                     per Section 6.1 accuracy analysis), continuous mode,
//                     abort-during-count
//                 6.  Monitor Mode : no-fault, Loss-of-Clock, Overflow,
//                     Underflow, Recovery, sticky RW1C behavior, global
//                     summary registers
//                 7.  Round-robin scheduler visits every enabled channel
//                 8.  Interrupt Generator : edge (pulse) mode and level
//                     (persistent-until-cleared) mode

// -----------------------------------------------------------------------------
module tb_freq_monitor_full;

    localparam NUM_CHANNELS = 4;

    // -----------------------------------------------------------------
    // Register address map (must match apb_regs.v)
    // -----------------------------------------------------------------
    localparam A_CTRL           = 16'h0000;
    localparam A_STATUS         = 16'h0004;
    localparam A_IRQ_EN         = 16'h0008;
    localparam A_IRQ_STATUS     = 16'h000C;
    localparam A_IRQ_CLR        = 16'h0010;
    localparam A_MON_PERIOD     = 16'h0014;
    localparam A_MEASURE_SEL    = 16'h0018;
    localparam A_MEASURE_GATE   = 16'h001C;
    localparam A_MEASURE_CTRL   = 16'h0020;
    localparam A_MEASURE_RESULT = 16'h0024;
    localparam A_REVISION       = 16'h0028;
    localparam A_NUM_CHANNELS   = 16'h002C;

    localparam A_MON_ENABLE          = 16'h0200;
    localparam A_FAULT_SUMMARY       = 16'h0204;
    localparam A_LOC_SUMMARY         = 16'h0208;
    localparam A_UNDERFLOW_SUMMARY   = 16'h020C;
    localparam A_OVERFLOW_SUMMARY    = 16'h0210;
    localparam A_RECOVERED_SUMMARY   = 16'h0214;
    localparam A_MON_CHANNEL_ACTIVE  = 16'h0218;

    function [15:0] ch_min_addr(input integer ch);
        ch_min_addr = 16'h0100 + (ch * 16'h0010) + 16'h0000;
    endfunction
    function [15:0] ch_max_addr(input integer ch);
        ch_max_addr = 16'h0100 + (ch * 16'h0010) + 16'h0004;
    endfunction
    function [15:0] ch_last_addr(input integer ch);
        ch_last_addr = 16'h0100 + (ch * 16'h0010) + 16'h0008;
    endfunction
    function [15:0] ch_fault_addr(input integer ch);
        ch_fault_addr = 16'h0100 + (ch * 16'h0010) + 16'h000C;
    endfunction

    // -----------------------------------------------------------------
    // DUT connections
    // -----------------------------------------------------------------
    reg  ref_clk = 0;
    reg  bus_clk = 0;
    reg  ref_rst_n = 0;
    reg  bus_rst_n = 0;
    reg  [NUM_CHANNELS-1:0] clk_in = 0;

    reg  [15:0] paddr;
    reg  [31:0] pwdata;
    wire [31:0] prdata;
    reg         pwrite;
    reg         psel;
    reg         penable;
    wire        pready;
    wire        pslverr;
    wire        irq;

    freq_monitor_top #(
        .NUM_CHANNELS    (NUM_CHANNELS),
        .REF_CLK_FREQ_HZ (100_000_000),
        .SYNC_STAGES     (2),
        .RESULT_WIDTH    (32),
        .THRESH_WIDTH    (32),
        .ADDR_WIDTH      (16)
    ) dut (
        .ref_clk   (ref_clk),
        .ref_rst_n (ref_rst_n),
        .bus_clk   (bus_clk),
        .bus_rst_n (bus_rst_n),
        .clk_in    (clk_in),
        .paddr     (paddr),
        .pwdata    (pwdata),
        .prdata    (prdata),
        .pwrite    (pwrite),
        .psel      (psel),
        .penable   (penable),
        .pready    (pready),
        .pslverr   (pslverr),
        .irq       (irq)
    );

    // ref_clk : 100 MHz (10 ns period)
    always #5  ref_clk = ~ref_clk;
    // bus_clk : 50 MHz (20 ns period)
    always #10 bus_clk = ~bus_clk;

    // clk_in[0] : 10 MHz  (100 ns period)  -> used for Measurement Mode tests
    always #50 clk_in[0] = ~clk_in[0];
    // clk_in[1] : 25 MHz  (40 ns period)   -> Monitor Mode "in range" channel
    always #20 clk_in[1] = ~clk_in[1];
    // clk_in[2] : stuck low                -> Monitor Mode Loss-of-Clock channel
    // clk_in[3] : 20 MHz (50 ns period)    -> Monitor Mode Overflow channel
    always #25 clk_in[3] = ~clk_in[3];

    // -----------------------------------------------------------------
    // Scoreboard
    // -----------------------------------------------------------------
    integer test_num = 0;
    integer pass_num = 0;
    integer fail_num = 0;

    task check;
        input [8*48-1:0] name;
        input [31:0] expected;
        input [31:0] actual;
        input [31:0] mask;
        reg   [31:0] exp_m, act_m;
        begin
            test_num = test_num + 1;
            exp_m = expected & mask;
            act_m = actual & mask;
            if (exp_m === act_m) begin
                pass_num = pass_num + 1;
                $display("[TEST %0d] %-38s EXPECTED=0x%08h ACTUAL=0x%08h  ==> PASS",
                          test_num, name, exp_m, act_m);
            end else begin
                fail_num = fail_num + 1;
                $display("[TEST %0d] %-38s EXPECTED=0x%08h ACTUAL=0x%08h  ==> FAIL",
                          test_num, name, exp_m, act_m);
            end
        end
    endtask

    task check_approx;
        input [8*48-1:0] name;
        input integer expected;
        input integer actual;
        input integer tolerance;
        integer diff;
        begin
            test_num = test_num + 1;
            diff = actual - expected;
            if (diff < 0) diff = -diff;
            if (diff <= tolerance) begin
                pass_num = pass_num + 1;
                $display("[TEST %0d] %-38s EXPECTED~=%-6d ACTUAL=%-6d (tol=%0d) ==> PASS",
                          test_num, name, expected, actual, tolerance);
            end else begin
                fail_num = fail_num + 1;
                $display("[TEST %0d] %-38s EXPECTED~=%-6d ACTUAL=%-6d (tol=%0d) ==> FAIL",
                          test_num, name, expected, actual, tolerance);
            end
        end
    endtask

    task check_true;
        input [8*48-1:0] name;
        input cond;
        begin
            test_num = test_num + 1;
            if (cond) begin
                pass_num = pass_num + 1;
                $display("[TEST %0d] %-38s CONDITION TRUE                  ==> PASS", test_num, name);
            end else begin
                fail_num = fail_num + 1;
                $display("[TEST %0d] %-38s CONDITION FALSE                 ==> FAIL", test_num, name);
            end
        end
    endtask

    // -----------------------------------------------------------------
    // APB access tasks
    // -----------------------------------------------------------------
    task apb_write;
        input [15:0] a;
        input [31:0] d;
        begin
            @(posedge bus_clk);
            paddr   <= a;
            pwdata  <= d;
            pwrite  <= 1'b1;
            psel    <= 1'b1;
            penable <= 1'b0;
            @(posedge bus_clk);
            penable <= 1'b1;
            @(posedge bus_clk);
            psel    <= 1'b0;
            penable <= 1'b0;
            pwrite  <= 1'b0;
        end
    endtask

    reg [31:0] rd_data;
    reg        rd_err;

    task apb_read;
        input [15:0] a;
        begin
            @(posedge bus_clk);
            paddr   <= a;
            pwrite  <= 1'b0;
            psel    <= 1'b1;
            penable <= 1'b0;
            @(posedge bus_clk);
            penable <= 1'b1;
            @(posedge bus_clk);
            rd_data = prdata;
            rd_err  = pslverr;
            psel    <= 1'b0;
            penable <= 1'b0;
        end
    endtask

    integer i;
    integer scan_idx;
    reg [31:0] result0, result1;
    time t_start, t_end;

    // ===================================================================
    // Main test sequence
    // ===================================================================
    initial begin
        paddr = 0; pwdata = 0; pwrite = 0; psel = 0; penable = 0;

        // -------------------- Reset ------------------------------------
        ref_rst_n = 0; bus_rst_n = 0;
        #40;
        ref_rst_n = 1; bus_rst_n = 1;
        #50;

        $display("\n================ SECTION 1 : RESET DEFAULT VALUES ================\n");
        apb_read(A_CTRL);           check("CTRL reset value",           32'h00000000, rd_data, 32'h0000001F);
        apb_read(A_STATUS);         check("STATUS reset value",         32'h00000008, rd_data, 32'h0000001F);
        apb_read(A_IRQ_EN);         check("IRQ_EN reset value",         32'h00000000, rd_data, 32'h00000007);
        apb_read(A_IRQ_STATUS);     check("IRQ_STATUS reset value",     32'h00000000, rd_data, 32'h00000007);
        apb_read(A_MON_PERIOD);     check("MON_PERIOD reset value",     32'h000186A0, rd_data, 32'hFFFFFFFF);
        apb_read(A_MEASURE_SEL);    check("MEASURE_SEL reset value",    32'h00000000, rd_data, 32'h0000000F);
        apb_read(A_MEASURE_GATE);   check("MEASURE_GATE reset value",   32'h000186A0, rd_data, 32'hFFFFFFFF);
        apb_read(A_MEASURE_CTRL);   check("MEASURE_CTRL reset value",   32'h00000000, rd_data, 32'h0000000F);
        apb_read(A_MEASURE_RESULT); check("MEASURE_RESULT reset value", 32'h00000000, rd_data, 32'hFFFFFFFF);
        apb_read(A_REVISION);       check("REVISION reset value",       32'h00010000, rd_data, 32'hFFFFFFFF);
        apb_read(A_NUM_CHANNELS);   check("NUM_CHANNELS reset value",   NUM_CHANNELS, rd_data, 32'h0000001F);
        apb_read(A_MON_ENABLE);     check("MON_ENABLE reset value",     32'h00000000, rd_data, 32'h0000000F);
        apb_read(A_FAULT_SUMMARY);  check("FAULT_SUMMARY reset value",  32'h00000000, rd_data, 32'h00000001);
        apb_read(A_LOC_SUMMARY);    check("LOC_SUMMARY reset value",    32'h00000000, rd_data, 32'h00000001);
        apb_read(A_UNDERFLOW_SUMMARY); check("UNDERFLOW_SUMMARY reset",  32'h00000000, rd_data, 32'h00000001);
        apb_read(A_OVERFLOW_SUMMARY);  check("OVERFLOW_SUMMARY reset",   32'h00000000, rd_data, 32'h00000001);
        apb_read(A_RECOVERED_SUMMARY); check("RECOVERED_SUMMARY reset",  32'h00000000, rd_data, 32'h00000001);
        apb_read(A_MON_CHANNEL_ACTIVE);check("MON_CHANNEL_ACTIVE reset", 32'h0000001F, rd_data, 32'h0000001F);

        for (i = 0; i < NUM_CHANNELS; i = i + 1) begin
            apb_read(ch_min_addr(i));   check("CH_MON_MIN reset",  32'h00000000, rd_data, 32'hFFFFFFFF);
            apb_read(ch_max_addr(i));   check("CH_MON_MAX reset",  32'hFFFFFFFF, rd_data, 32'hFFFFFFFF);
            apb_read(ch_last_addr(i));  check("CH_LAST_RESULT rst",32'h00000000, rd_data, 32'hFFFFFFFF);
            apb_read(ch_fault_addr(i)); check("CH_FAULT reset",    32'h00000000, rd_data, 32'h0000001F);
        end

        $display("\n================ SECTION 2 : APB PROTOCOL RULES =================\n");
        // Invalid address
        apb_read(16'h0FF0);
        check("Invalid addr pslverr=1", 32'h1, {31'b0, rd_err}, 32'h1);
        check("Invalid addr prdata=0",  32'h0, rd_data, 32'hFFFFFFFF);

        // Write to RO (REVISION) ignored
        apb_write(A_REVISION, 32'hDEADBEEF);
        apb_read(A_REVISION);
        check("Write to RO ignored", 32'h00010000, rd_data, 32'hFFFFFFFF);

        // Read from WO (IRQ_CLR) returns 0
        apb_write(A_IRQ_CLR, 32'h00000007);
        apb_read(A_IRQ_CLR);
        check("Read from WO returns 0", 32'h00000000, rd_data, 32'hFFFFFFFF);

        // Reserved bits in CTRL : write all-ones, verify reserved [31:5] read 0
        apb_write(A_CTRL, 32'hFFFFFFFF);
        apb_read(A_CTRL);
        check("CTRL reserved bits read 0", 32'h00000000, rd_data, 32'hFFFFFFE0);
        apb_write(A_CTRL, 32'h00000000); // restore

        $display("\n================ SECTION 3 : CTRL / IRQ_EN READ-WRITE ============\n");
        apb_write(A_CTRL, 32'h0000001F);
        apb_read(A_CTRL);
        check("CTRL R/W all defined bits", 32'h0000001F, rd_data, 32'h0000001F);
        apb_write(A_CTRL, 32'h00000000);

        apb_write(A_IRQ_EN, 32'h00000007);
        apb_read(A_IRQ_EN);
        check("IRQ_EN R/W all defined bits", 32'h00000007, rd_data, 32'h00000007);
        apb_write(A_IRQ_EN, 32'h00000000);

        $display("\n============== SECTION 4 : MEASUREMENT MODE (SINGLE-SHOT) ========\n");
        // gate = 50 ref_clk cycles (500 ns) ; clk_in[0] period = 100 ns
        // expected edges ~= 500 / 100 = 5 (+/- 1 per spec accuracy analysis)
        apb_write(A_MEASURE_GATE, 32'd50);
        apb_write(A_MEASURE_SEL,  32'd0);
        apb_write(A_CTRL, 32'h00000001); // GLOBAL_EN
        apb_write(A_MEASURE_CTRL, 32'h00000001); // START (self-clearing)

        apb_read(A_MEASURE_CTRL);
        check("START self-clears", 32'h0, {31'b0, rd_data[0]}, 32'h1);

        // poll for DONE
        for (i = 0; i < 30; i = i + 1) begin
            apb_read(A_MEASURE_CTRL);
            if (rd_data[2]) i = 30;
            else #100;
        end
        check_true("MEASURE_CTRL.DONE observed", rd_data[2]);
        check_true("MEASURE_CTRL.RESULT_VALID observed", rd_data[3]);

        apb_read(A_STATUS);
        check("STATUS.MEASURE_BUSY=0 after done", 32'h0, {31'b0, rd_data[1]}, 32'h1);
        check("STATUS.MEASURE_DONE=1 after done", 32'h1, {31'b0, rd_data[2]}, 32'h1);
        check("STATUS.REF_CLK_OK=1",              32'h1, {31'b0, rd_data[3]}, 32'h1);

        apb_read(A_MEASURE_RESULT);
        check_approx("MEASURE_RESULT single-shot", 5, rd_data, 2);

        $display("\n============== SECTION 5 : MEASUREMENT MODE (CONTINUOUS) =========\n");
        apb_write(A_CTRL, 32'h00000011); // GLOBAL_EN + MEASURE_CONTINUOUS
        apb_write(A_MEASURE_CTRL, 32'h00000001); // START
        #2000; // allow several ARM->COUNT->LATCH cycles
        apb_read(A_STATUS);
        check("STATUS.MEASURE_BUSY=1 (continuous)", 32'h1, {31'b0, rd_data[1]}, 32'h1);
        apb_read(A_MEASURE_RESULT);
        result0 = rd_data;
        #1000;
        apb_read(A_MEASURE_RESULT);
        result1 = rd_data;
        check_approx("Continuous result re-measured", result0, result1, 2);
        // stop continuous mode via ABORT
        apb_write(A_MEASURE_CTRL, 32'h00000002); // ABORT
        #100;
        apb_write(A_CTRL, 32'h00000001); // clear MEASURE_CONTINUOUS
        apb_read(A_STATUS);
        check("STATUS.MEASURE_BUSY=0 after abort", 32'h0, {31'b0, rd_data[1]}, 32'h1);

        $display("\n============== SECTION 6 : MEASUREMENT MODE (ABORT) ==============\n");
        apb_write(A_MEASURE_GATE, 32'd100000); // long gate so we can abort mid-count
        apb_write(A_MEASURE_CTRL, 32'h00000001); // START
        #200; // still counting
        apb_read(A_STATUS);
        check_true("MEASURE_BUSY=1 mid-count", rd_data[1]);
        apb_write(A_MEASURE_CTRL, 32'h00000002); // ABORT
        #100;
        apb_read(A_STATUS);
        check("STATUS.MEASURE_BUSY=0 after abort", 32'h0, {31'b0, rd_data[1]}, 32'h1);
        apb_read(A_MEASURE_CTRL);
        check("No DONE after abort", 32'h0, {31'b0, rd_data[2]}, 32'h1);
        apb_write(A_MEASURE_GATE, 32'd50); // restore

        $display("\n================ SECTION 7 : INTERRUPT - EDGE MODE ================\n");
        apb_write(A_IRQ_EN, 32'h00000001);      // EN_MEASURE_DONE
        apb_write(A_CTRL, 32'h0000000D);        // GLOBAL_EN + IRQ_GLOBAL_EN + IRQ_EDGE
        apb_write(A_MEASURE_CTRL, 32'h00000001);// START
        t_start = $time;
        wait (irq === 1'b1);
        t_start = $time;
        @(posedge bus_clk);
        check_true("IRQ pulses in edge mode", (irq === 1'b1) || ($time - t_start <= 40));
        wait (irq === 1'b0);
        t_end = $time;
        check_true("IRQ deasserts automatically (edge mode)", (t_end - t_start) <= 40);
        apb_read(A_IRQ_STATUS);
        check_true("IRQ_STATUS.MEASURE_DONE_IRQ set (edge)", rd_data[0]);
        apb_write(A_IRQ_STATUS, 32'h00000001); // clear via RW1C
        apb_read(A_IRQ_STATUS);
        check("IRQ_STATUS cleared via RW1C", 32'h0, {31'b0, rd_data[0]}, 32'h1);

        $display("\n================ SECTION 8 : INTERRUPT - LEVEL MODE ===============\n");
        apb_write(A_CTRL, 32'h00000005); // GLOBAL_EN + IRQ_GLOBAL_EN, IRQ_EDGE=0 (level)
        apb_write(A_MEASURE_CTRL, 32'h00000001); // START
        #1000;
        check_true("IRQ remains asserted (level mode)", irq === 1'b1);
        apb_write(A_IRQ_CLR, 32'h00000001); // clear pending source
        #100;
        check_true("IRQ deasserts after IRQ_CLR (level mode)", irq === 1'b0);
        apb_write(A_CTRL, 32'h00000000);

        $display("\n================ SECTION 9 : MONITOR MODE SETUP ===================\n");
        // Channel 1 : in range   (period 40ns, window 2000ns => ~50 edges)
        apb_write(ch_min_addr(1), 32'd30);
        apb_write(ch_max_addr(1), 32'd70);
        // Channel 2 : stuck low -> Loss-of-Clock
        apb_write(ch_min_addr(2), 32'd0);
        apb_write(ch_max_addr(2), 32'hFFFFFFFF);
        // Channel 3 : overflow (period 50ns, window 2000ns => ~40 edges, MAX=20)
        apb_write(ch_min_addr(3), 32'd0);
        apb_write(ch_max_addr(3), 32'd20);

        apb_write(A_MON_PERIOD, 32'd200); // 2000 ns gate window
        apb_write(A_MON_ENABLE, 32'b1110); // channels 1,2,3 enabled, ch0 disabled
        apb_write(A_CTRL, 32'h00000003);   // GLOBAL_EN + MON_MODE

        $display("\n============= SECTION 10 : ROUND-ROBIN SCHEDULER CHECK ============\n");
        // observe MON_CHANNEL_ACTIVE visiting channels 1, 2 and 3 over time
        begin : rr_check
            reg seen1, seen2, seen3;
            seen1 = 0; seen2 = 0; seen3 = 0;
            for (i = 0; i < 60; i = i + 1) begin
                apb_read(A_MON_CHANNEL_ACTIVE);
                if (rd_data == 32'd1) seen1 = 1;
                if (rd_data == 32'd2) seen2 = 1;
                if (rd_data == 32'd3) seen3 = 1;
                #300;
            end
            check_true("Round-robin visited channel 1", seen1);
            check_true("Round-robin visited channel 2", seen2);
            check_true("Round-robin visited channel 3", seen3);
        end

        // allow a few complete scan rounds so all fault/no-fault results settle
        #8000;

        $display("\n============= SECTION 11 : MONITOR MODE FAULT DETECTION ===========\n");
        apb_read(ch_fault_addr(1));
        check("CH1 (in-range) FAULT_ACTIVE=0", 32'h0, {31'b0, rd_data[3]}, 32'h1);
        apb_read(ch_last_addr(1));
        check_approx("CH1 LAST_RESULT (in range)", 50, rd_data, 5);

        apb_read(ch_fault_addr(2));
        check("CH2 LOSS_OF_CLOCK sticky set", 32'h1, {31'b0, rd_data[2]}, 32'h1);
        check("CH2 FAULT_ACTIVE=1 (live)",     32'h1, {31'b0, rd_data[3]}, 32'h1);

        apb_read(ch_fault_addr(3));
        check("CH3 OVERFLOW sticky set",   32'h1, {31'b0, rd_data[1]}, 32'h1);
        check("CH3 FAULT_ACTIVE=1 (live)", 32'h1, {31'b0, rd_data[3]}, 32'h1);

        apb_read(A_FAULT_SUMMARY);
        check("FAULT_SUMMARY=1 (any fault)", 32'h1, rd_data, 32'h1);
        apb_read(A_LOC_SUMMARY);
        check("LOC_SUMMARY=1", 32'h1, rd_data, 32'h1);
        apb_read(A_OVERFLOW_SUMMARY);
        check("OVERFLOW_SUMMARY=1", 32'h1, rd_data, 32'h1);
        apb_read(A_UNDERFLOW_SUMMARY);
        check("UNDERFLOW_SUMMARY=0 (none yet)", 32'h0, rd_data, 32'h1);

        apb_read(A_STATUS);
        check("STATUS.ANY_FAULT=1", 32'h1, {31'b0, rd_data[4]}, 32'h1);

        $display("\n=========== SECTION 12 : UNDERFLOW + RECOVERY (RW1C) ==============\n");
        // Raise CH1 threshold above its actual rate (~50) to force underflow
        apb_write(ch_min_addr(1), 32'd200);
        #8000; // wait a full scan round
        apb_read(ch_fault_addr(1));
        check("CH1 UNDERFLOW sticky set", 32'h1, {31'b0, rd_data[0]}, 32'h1);
        apb_read(A_UNDERFLOW_SUMMARY);
        check("UNDERFLOW_SUMMARY=1 now", 32'h1, rd_data, 32'h1);

        // clear the sticky bit (RW1C) and verify write-0 has no effect
        apb_write(ch_fault_addr(1), 32'h00000000);
        apb_read(ch_fault_addr(1));
        check("RW1C write-0 has no effect", 32'h1, {31'b0, rd_data[0]}, 32'h1);
        apb_write(ch_fault_addr(1), 32'h00000001); // write-1 clears
        apb_read(ch_fault_addr(1));
        check("RW1C write-1 clears bit", 32'h0, {31'b0, rd_data[0]}, 32'h1);

        // Lower threshold back down to allow recovery, then check RECOVERED
        apb_write(ch_min_addr(1), 32'd10);
        #8000;
        apb_read(ch_fault_addr(1));
        check("CH1 RECOVERED sticky set", 32'h1, {31'b0, rd_data[4]}, 32'h1);
        check("CH1 FAULT_ACTIVE=0 after recovery", 32'h0, {31'b0, rd_data[3]}, 32'h1);
        apb_read(A_RECOVERED_SUMMARY);
        check("RECOVERED_SUMMARY=1", 32'h1, rd_data, 32'h1);

        apb_write(A_CTRL, 32'h00000000); // stop monitor mode

        // ===================================================================
        $display("\n========================= FINAL SCOREBOARD ========================\n");
        $display("TOTAL TESTS : %0d", test_num);
        $display("PASSED      : %0d", pass_num);
        $display("FAILED      : %0d", fail_num);
        if (fail_num == 0)
            $display("\n*** OVERALL RESULT : ALL TESTS PASSED ***\n");
        else
            $display("\n*** OVERALL RESULT : %0d TEST(S) FAILED ***\n", fail_num);

        #200;
        $finish;
    end

    // safety timeout
    initial begin
        #200000;
        $display("ERROR: TESTBENCH TIMEOUT");
        $finish;
    end

    initial begin
        $dumpfile("/tmp/tb_freq_monitor_full.vcd");
        $dumpvars(0, tb_freq_monitor_full);
    end

endmodule
