`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

`timescale 1ns/1ps

module testbench;

    logic clock, reset;
    logic [4:0] cdb_tag;
    logic [31:0] cdb_data; //originally was 64 bit, but cdb.sv shows 32 so changed to 32
    logic cdb_valid;

    CDB_PACKET cdb_in;

    cdb dut (
        .clock(clock),
        .reset(reset),
        .cdb_in(cdb_in),
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
        cdb_in.tag = 0;
        cdb_in.value = 0;
        cdb_in.valid = 0;
        #20 reset = 0;

        // --- Test 1: Basic Valid Broadcast ---
        cdb_in.tag = 5'b10101;
        cdb_in.value = 64'hDEADBEEF12345678;
        cdb_in.valid = 1;
        #10;
        check_output("Test 1: Valid Broadcast", 
                    cdb_in.tag, cdb_in.value, 1);

        // --- Test 2: Invalid Data (fu_valid/cdb_in.valid =0) ---
        cdb_in.valid = 0;
        #10;
        check_output("Test 2: Invalid Data", 
                     5'b0, 64'b0, 0);

        // --- Test 3: Back-to-Back Valid/Invalid ---
        cdb_in.valid = 1;
        cdb_in.tag = 5'b01010;
        cdb_in.value = 64'hAABBCCDDEEFF0011;
        #5;  // Half-cycle delay
        cdb_in.valid = 0;
        #5;
        check_output("Test 3: Valid-to-Invalid Transition", 
                    cdb_in.tag, cdb_in.value, 0);  // Should latch invalid

        // --- Test 4: Reset During Operation ---
        cdb_in.valid = 1;
        #5;
        reset = 1;
        #10;
        check_output("Test 4: Reset Assertion", 
                     5'b0, 64'b0, 0);
        reset = 0;

        // --- Test 5: Max/Min Values ---
        cdb_in.tag = 5'b11111;       // Max tag
        cdb_in.value = 64'hFFFFFFFFFFFFFFFF;  // Max data
        cdb_in.valid = 1;
        #10;
        check_output("Test 5: Max Values", 
                     cdb_in.tag, cdb_in.value, 1);

        cdb_in.tag = 5'b00000;       // Min tag
        cdb_in.value = 64'h0;       // Min data
        #10;
        check_output("Test 6: Min Values", 
                    cdb_in.tag, cdb_in.value, 1);

        // --- Test 6: Random Stimulus ---
        $display("=== Random Testing ===");
        repeat (10) begin
            cdb_in.tag = $urandom_range(0, 31);
            cdb_in.value = {$urandom(), $urandom()};
            cdb_in.valid = $urandom_range(0, 1);
            #10;
            if (cdb_in.valid) begin
                check_output($sformatf("Random Valid %0d", $time), 
                            cdb_in.tag, cdb_in.value, 1);
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
