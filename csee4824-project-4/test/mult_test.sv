`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

module testbench();

    logic [63:0] a, b, result, cres;
    logic quit, clock, start, reset, done, correct;
    mult_type_t mult_type;
    logic [127:0] full_product;
    integer i;

    mult dut(
        .clock(clock),
        .reset(reset),
        .mcand(a),
        .mplier(b),
        .mult_type(mult_type),
        .start(start),
        .product(result),
        .done(done)
    );

    // Clock generation
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    // Calculate expected product and correct signal
    always_comb begin
        case (mult_type)
            MUL_ALU_MUL:    full_product = $signed(a) * $signed(b);
            MUL_ALU_MULH:   full_product = $signed(a) * $signed(b);
            MUL_ALU_MULHSU: full_product = $signed(a) * $unsigned(b);
            MUL_ALU_MULHU:  full_product = $unsigned(a) * $unsigned(b);
            default:        full_product = 128'b0;
        endcase

        case (mult_type)
            MUL_ALU_MUL:    cres = full_product[63:0];
            default:        cres = full_product[127:64];
        endcase

        correct = ~done || (result === cres);
    end

    // Correctness check
    always @(posedge clock) begin
        #(`CLOCK_PERIOD*0.2); // allow signals to settle
        if (done && !correct) begin
            $display("@@@ Incorrect at time %4.0f", $time);
            $display("@@@ done:%b mult_type:%b a:%h b:%h result:%h", done, mult_type, a, b, result);
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

    // MAIN TESTING SEQUENCE
    initial begin
        $monitor("Time:%4.0f done:%b a:%h b:%h result:%h correct:%h",
                 $time, done, a, b, result, cres);

        $display("\n=== Beginning multiplier test ===");

        reset = 1;
        clock = 0;
        a = 2;
        b = 3;
        mult_type = MUL_ALU_MUL;
        start = 1;
        @(negedge clock);
        reset = 0;
        @(negedge clock);
        start = 0;
        wait_until_done();

        $display("\n--- Edge Case Testing ---");

        start = 1; a = 5; b = 50; mult_type = MUL_ALU_MUL;
        @(negedge clock); start = 0; wait_until_done();

        start = 1; a = 0; b = 257; mult_type = MUL_ALU_MUL;
        @(negedge clock); start = 0; wait_until_done();

        start = 1; a = 64'hFFFF_FFFF_FFFF_FFFF; b = 64'hFFFF_FFFF_FFFF_FFFF; mult_type = MUL_ALU_MUL;
        @(negedge clock); start = 0; wait_until_done();

        start = 1; a = 64'hFFFF_FFFF_FFFF_FFFF; b = 3; mult_type = MUL_ALU_MULH;
        @(negedge clock); start = 0; wait_until_done();

        start = 1; a = 64'hFFFF_FFFF_FFFF_FFFF; b = 0; mult_type = MUL_ALU_MULH;
        @(negedge clock); start = 0; wait_until_done();

        start = 1; a = 64'h5555_5555_5555_5555; b = 64'hCCCC_CCCC_CCCC_CCCC; mult_type = MUL_ALU_MULHU;
        @(negedge clock); start = 0; wait_until_done();

        $display("\n--- Random Testing ---");

        for (i = 0; i <= 15; i = i+1) begin
            start = 1;
            a = {$random, $random};
            b = {$random, $random};
            mult_type = MUL_ALU_MUL;
            @(negedge clock);
            start = 0;
            wait_until_done();
            $display("Time:%4.0f done:%b a:%h b:%h result:%h expected:%h",
                     $time, done, a, b, result, cres);
        end

        $display("@@@ Passed\n");
        $finish;
    end

endmodule
