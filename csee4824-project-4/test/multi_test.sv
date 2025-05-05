`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"          // <-- needed for ALU_FUNC / ALU_MUL

module testbench;

    logic [63:0] a, b, result, cres;
    logic quit, clock, start, reset, done, correct;
    ALU_FUNC mult_func;             // <-- new signal
    integer i;

    multi dut(
        .clock      (clock),
        .reset      (reset),
        .mcand      (a),
        .mplier     (b),
        .mult_func  (mult_func),    // <-- connect it
        .start      (start),
        .product    (result),
        .done       (done)
    );

    // CLOCK_PERIOD is defined on the command line by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    assign cres    = a * b;
    assign correct = ~done || (cres === result);

    always @(posedge clock) begin
        #(`CLOCK_PERIOD*0.2);        // let signals settle
        if (!correct) begin
            $display("@@@ Incorrect at time %4.0f", $time);
            $display("@@@ done:%b a:%h b:%h result:%h", done, a, b, result);
            $display("@@@ Expected result:%h", cres);
            $finish;
        end
    end

    // wait-until-done helper (unchanged)
    task wait_until_done;
        forever begin : wait_loop
            @(posedge done);
            if (done) disable wait_until_done;
        end
    endtask

    initial begin
        // choose multiply-function once for all tests
        mult_func = ALU_MUL;        // low-64-bit product

        // NOTE: monitor starts using 5-digit decimal values for printing
        $monitor("Time:%4.0f done:%b a:%5d b:%5d result:%5d correct:%5d",
                 $time, done, a, b, result, cres);

        $display("\nBeginning edge-case testing:");

        reset = 1;
        clock = 0;
        a = 2;  b = 3;  start = 1;
        @(posedge clock);
        reset = 0;
        @(posedge clock);
        start = 0;
        wait_until_done();

        start = 1;  a = 5;  b = 50;
        @(posedge clock); start =0; wait_until_done();

        start = 1;  a = 0;  b = 257;
        @(posedge clock); start =0; wait_until_done();

        // switch monitor to hex
        $monitor("Time:%4.0f done:%b a:%h b:%h result:%h correct:%h",
                 $time, done, a, b, result, cres);

        start = 1;  a = 64'hFFFF_FFFF_FFFF_FFFF;  b = 64'hFFFF_FFFF_FFFF_FFFF;
        @(posedge clock);  start = 0;  wait_until_done();

        start = 1;  a = 64'hFFFF_FFFF_FFFF_FFFF;  b = 3;
        @(posedge clock);  start = 0;  wait_until_done();

        start = 1;  a = 64'hFFFF_FFFF_FFFF_FFFF;  b = 0;
        @(posedge clock);  start = 0;  wait_until_done();

        start = 1;  a = 64'h5555_5555_5555_5555;  b = 64'hCCCC_CCCC_CCCC_CCCC;
        @(posedge clock);  start = 0;  wait_until_done();

        $monitor(); // turn off monitor for the for-loop
        $display("\nBeginning random testing:");

        for (i = 0; i <= 15; i = i + 1) begin
            start = 1;
            a = {$random, $random};
            b = {$random, $random};
            @(posedge clock);
            start = 0;
            wait_until_done();
            $display("Time:%4.0f done:%b a:%h b:%h result:%h correct:%h",
                     $time, done, a, b, result, cres);
        end

        $display("@@@ Passed\n");
        $finish;
    end
endmodule
