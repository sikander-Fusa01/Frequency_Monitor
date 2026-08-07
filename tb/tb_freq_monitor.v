`timescale 1ns/1ps
module tb_freq_monitor;

    localparam NUM_CHANNELS = 4;

    reg ref_clk = 0;
    reg bus_clk = 0;
    reg ref_rst_n = 0;
    reg bus_rst_n = 0;
    reg [NUM_CHANNELS-1:0] clk_in = 0;

    reg  [15:0] paddr;
    reg  [31:0] pwdata;
    wire [31:0] prdata;
    reg         pwrite;
    reg         psel;
    reg         penable;
    wire        pready;
    wire        pslverr;
    wire        irq;

    integer i;

    freq_monitor_top #(
        .NUM_CHANNELS (NUM_CHANNELS),
        .REF_CLK_FREQ_HZ (100_000_000),
        .SYNC_STAGES (2),
        .RESULT_WIDTH (32),
        .THRESH_WIDTH (32),
        .ADDR_WIDTH (16)
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

    // ref_clk : 100 MHz
    always #5 ref_clk = ~ref_clk;
    // bus_clk : 50 MHz
    always #10 bus_clk = ~bus_clk;
    // clk_in[0] : ~10 MHz toggle to exercise edge detector / measurement
    always #50 clk_in[0] = ~clk_in[0];

    task apb_write(input [15:0] a, input [31:0] d);
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
    end
    endtask

    task apb_read(input [15:0] a);
    begin
        @(posedge bus_clk);
        paddr   <= a;
        pwrite  <= 1'b0;
        psel    <= 1'b1;
        penable <= 1'b0;
        @(posedge bus_clk);
        penable <= 1'b1;
        @(posedge bus_clk);
        $display("READ addr=0x%04h data=0x%08h pslverr=%b", a, prdata, pslverr);
        psel    <= 1'b0;
        penable <= 1'b0;
    end
    endtask

    initial begin
        paddr = 0; pwdata = 0; pwrite = 0; psel = 0; penable = 0;

        #20 ref_rst_n = 0; bus_rst_n = 0;
        #40 ref_rst_n = 1; bus_rst_n = 1;

        #50;
        // read REVISION
        apb_read(16'h0028);
        // read NUM_CHANNELS
        apb_read(16'h002C);

        // program MEASURE_GATE to a small value for quick simulation
        apb_write(16'h001C, 32'd50);
        // select channel 0
        apb_write(16'h0018, 32'd0);
        // enable global
        apb_write(16'h0000, 32'h00000001);
        // start measurement
        apb_write(16'h0020, 32'h00000001);

        // wait and poll MEASURE_CTRL for DONE
        for (i = 0; i < 40; i = i + 1) begin
            apb_read(16'h0020);
            #100;
        end

        apb_read(16'h0024); // MEASURE_RESULT
        apb_read(16'h0004); // STATUS

        // program a channel monitor and enable monitor mode
        apb_write(16'h0114, 32'h00000000); // CH1 MON_MIN = 0
        apb_write(16'h0110 + 4, 32'h00000005); // just exercise write path
        apb_write(16'h0200, 32'h0000000F); // MON_ENABLE all channels
        apb_write(16'h0014, 32'd50);        // MON_PERIOD small
        apb_write(16'h0000, 32'h00000003);  // GLOBAL_EN + MON_MODE

        #5000;
        apb_read(16'h0218); // MON_CHANNEL_ACTIVE
        apb_read(16'h0204); // FAULT_SUMMARY

        // invalid address check
        apb_read(16'h0FF0);

        #200;
        $display("TEST COMPLETE");
        $finish;
    end

    initial begin
        $dumpfile("/tmp/tb_freq_monitor.vcd");
        $dumpvars(0, tb_freq_monitor);
    end

endmodule
