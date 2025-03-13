
`include "verilog/sys_defs.svh"

module testbench;

    logic [63:0] a, b, result, cres;
    logic quit, clock, start, reset, done, correct;
    integer i;

    mult dut(
        .clock(clock),
        .reset(reset),
        .mcand(a),
        .mplier(b),
        .start(start),
        .product(result),
        .done(done)
    );


    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end


    assign cres = a * b;
    assign correct = ~done || (cres === result);


    always @(posedge clock) begin
        #(`CLOCK_PERIOD*0.2); // a short wait to let signals stabilize
        if (!correct) begin
            $display("@@@ Incorrect at time %4.0f", $time);
            $display("@@@ done:%b a:%h b:%h result:%h", done, a, b, result);
            $display("@@@ Expected result:%h", cres);
            $finish;
        end
    end


    // Some students have had problems just using "@(posedge done)" because their
    // "done" signals glitch (even though they are the output of a register). This
    // prevents that by making sure "done" is high at the clock edge.
    task wait_until_done;
        forever begin : wait_loop
            @(posedge done);
            @(negedge clock);
            if (done) begin
                disable wait_until_done;
            end
        end
    endtask


    initial begin
        // NOTE: monitor starts using 5-digit decimal values for printing
        $monitor("Time:%4.0f done:%b a:%5d b:%5d result:%5d correct:%5d",
                 $time, done, a, b, result, cres);

        $display("\nBeginning edge-case testing:");

        reset = 1;
        clock = 0;
        a = 2;
        b = 3;
        start = 1;
        @(negedge clock);
        reset = 0;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 5;
        b = 50;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 0;
        b = 257;
        @(negedge clock);
        start = 0;
        wait_until_done();

        // change the monitor to hex for these values
        $monitor("Time:%4.0f done:%b a:%h b:%h result:%h correct:%h",
                 $time, done, a, b, result, cres);

        start = 1;
        a = 64'hFFFF_FFFF_FFFF_FFFF;
        b = 64'hFFFF_FFFF_FFFF_FFFF;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 64'hFFFF_FFFF_FFFF_FFFF;
        b = 3;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 64'hFFFF_FFFF_FFFF_FFFF;
        b = 0;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 64'h5555_5555_5555_5555;
        b = 64'hCCCC_CCCC_CCCC_CCCC;
        @(negedge clock);
        start = 0;
        wait_until_done();

        $monitor(); // turn off monitor for the for-loop
        $display("\nBeginning random testing:");

        for (i = 0; i <= 15; i = i+1) begin
            start = 1;
            a = {$random, $random}; // multiply random 64-bit numbers
            b = {$random, $random};
            @(negedge clock);
            start = 0;
            wait_until_done();
            $display("Time:%4.0f done:%b a:%h b:%h result:%h correct:%h",
                     $time, done, a, b, result, cres);
        end

        $display("@@@ Passed\n");
        $finish;
    end

endmodule
