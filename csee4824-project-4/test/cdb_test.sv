`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

`timescale 1ns/1ps

module testbench;

    logic clock, reset;
    logic [4:0] fu_tag, cdb_tag;
    logic [63:0] fu_result, cdb_data;
    logic fu_valid, cdb_valid;

    cdb dut (
        .clock(clock),
        .reset(reset),
        .fu_tag(fu_tag),
        .fu_result(fu_result),
        .fu_valid(fu_valid),
        .cdb_data(cdb_data),
        .cdb_tag(cdb_tag),
        .cdb_valid(cdb_valid)
    );

    initial begin
        clock = 0;
        forever #5 clock = ~clock;  // 100MHz clock
    end

    initial begin
        // Initialize
        reset = 1;
        fu_tag = 0;
        fu_result = 0;
        fu_valid = 0;
        #20 reset = 0;

        // --- Test 1: Basic Valid Broadcast ---
        fu_tag = 5'b10101;
        fu_result = 64'hDEADBEEF12345678;
        fu_valid = 1;
        #10;
        check_output("Test 1: Valid Broadcast", 
                     fu_tag, fu_result, 1);

        // --- Test 2: Invalid Data (fu_valid=0) ---
        fu_valid = 0;
        #10;
        check_output("Test 2: Invalid Data", 
                     5'b0, 64'b0, 0);

        // --- Test 3: Back-to-Back Valid/Invalid ---
        fu_valid = 1;
        fu_tag = 5'b01010;
        fu_result = 64'hAABBCCDDEEFF0011;
        #5;  // Half-cycle delay
        fu_valid = 0;
        #5;
        check_output("Test 3: Valid-to-Invalid Transition", 
                     fu_tag, fu_result, 0);  // Should latch invalid

        // --- Test 4: Reset During Operation ---
        fu_valid = 1;
        #5;
        reset = 1;
        #10;
        check_output("Test 4: Reset Assertion", 
                     5'b0, 64'b0, 0);
        reset = 0;

        // --- Test 5: Max/Min Values ---
        fu_tag = 5'b11111;       // Max tag
        fu_result = 64'hFFFFFFFFFFFFFFFF;  // Max data
        fu_valid = 1;
        #10;
        check_output("Test 5: Max Values", 
                     fu_tag, fu_result, 1);

        fu_tag = 5'b00000;       // Min tag
        fu_result = 64'h0;       // Min data
        #10;
        check_output("Test 6: Min Values", 
                     fu_tag, fu_result, 1);

        // --- Test 6: Random Stimulus ---
        $display("=== Random Testing ===");
        repeat (10) begin
            fu_tag = $urandom_range(0, 31);
            fu_result = {$urandom(), $urandom()};
            fu_valid = $urandom_range(0, 1);
            #10;
            if (fu_valid) begin
                check_output($sformatf("Random Valid %0d", $time), 
                             fu_tag, fu_result, 1);
            end else begin
                check_output($sformatf("Random Invalid %0d", $time), 
                             5'b0, 64'b0, 0);
            end
        end

        $display("=== All Tests Passed ===");
        $finish;
    end

    // ===== Verification Task =====
    task check_output(
        input string test_name,
        input [4:0]  exp_tag,
        input [63:0] exp_data,
        input        exp_valid
    );
        if (cdb_valid !== exp_valid) begin
            $error("%s: VALID mismatch (Got %b, Exp %b)",
                   test_name, cdb_valid, exp_valid);
        end
        else if (cdb_valid && (cdb_tag !== exp_tag || cdb_data !== exp_data)) begin
            $error("%s: DATA/TAG mismatch (Got Tag=%h, Data=%h | Exp Tag=%h, Data=%h)",
                   test_name, cdb_tag, cdb_data, exp_tag, exp_data);
        end
        else begin
            $display("%s: PASS", test_name);
        end
    endtask

endmodule
