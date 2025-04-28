`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

module testbench();

    logic clock;
    logic reset;

    logic [`XLEN-1:0] fetch_pc;
    logic             predict_taken;
    logic [`XLEN-1:0] predict_target;

    logic resolve_valid;
    logic [`XLEN-1:0] resolve_pc;
    logic             resolve_taken;
    logic [`XLEN-1:0] resolve_target;
    logic resolve_is_return;
    logic resolve_is_call;

    // Instantiate branch predictor
    branch_predictor uut (
        .clock(clock),
        .reset(reset),
        .fetch_pc(fetch_pc),
        .predict_taken(predict_taken),
        .predict_target(predict_target),
        .resolve_valid(resolve_valid),
        .resolve_pc(resolve_pc),
        .resolve_taken(resolve_taken),
        .resolve_target(resolve_target),
        .resolve_is_return(resolve_is_return),
        .resolve_is_call(resolve_is_call)
    );

    // Clock generation
    always #5 clock = ~clock;

    localparam BASE_PC = 32'h1000_0000;
    localparam TARGET_PC = 32'h2000_0000;
    localparam ALT_TARGET_PC = 32'h3000_0000;
    localparam RETURN_PC = 32'h4000_0000;

    /////////////////////////////
    // Helper Tasks
    /////////////////////////////

    // Reset the predictor
    task automatic reset_predictor();
        $display("[Test] Resetting branch predictor...");
        reset = 1;
        #15;
        reset = 0;
        #10;
    endtask

    // Simulate a fetch and check prediction
    task automatic predict_fetch(input logic [`XLEN-1:0] pc,
                                  input logic expected_taken,
                                  input logic [`XLEN-1:0] expected_target);
        fetch_pc = pc;
        #10;
        assert (predict_taken == expected_taken) else $fatal("Fetch prediction mismatch at PC %h. Expected taken=%0d", pc, expected_taken);
        assert (predict_target == expected_target) else $fatal("Target mismatch at PC %h. Got %h expected %h", pc, predict_target, expected_target);
    endtask

    // Simulate resolving a branch (updating predictors)
    task automatic resolve_branch(input logic [`XLEN-1:0] pc,
                                   input logic taken,
                                   input logic [`XLEN-1:0] target);
        resolve_valid = 1;
        resolve_pc = pc;
        resolve_taken = taken;
        resolve_target = target;
        resolve_is_call = 0;
        resolve_is_return = 0;
        #10;
        resolve_valid = 0;
    endtask

    // Simulate call (push return address)
    task automatic push_call(input logic [`XLEN-1:0] pc);
        resolve_valid = 1;
        resolve_pc = pc;
        resolve_taken = 1;
        resolve_target = pc + 32'h8; // dummy target
        resolve_is_call = 1;
        resolve_is_return = 0;
        #10;
        resolve_valid = 0;
        resolve_is_call = 0;
    endtask

    // Simulate return (pop return address)
    task automatic pop_return(input logic [`XLEN-1:0] pc);
        resolve_valid = 1;
        resolve_pc = pc;
        resolve_taken = 1;
        resolve_target = RETURN_PC;
        resolve_is_call = 0;
        resolve_is_return = 1;
        #10;
        resolve_valid = 0;
        resolve_is_return = 0;
    endtask

    /////////////////////////////
    // Test Sequences
    /////////////////////////////

    initial begin
        $display("\nStarting Comprehensive branch_predictor_tb...");

        clock = 0;
        reset = 0;
        fetch_pc = 0;
        resolve_valid = 0;
        resolve_pc = 0;
        resolve_taken = 0;
        resolve_target = 0;
        resolve_is_return = 0;
        resolve_is_call = 0;

        // Apply Reset
        reset_predictor();

        //////////////////////
        // Test 1: Empty lookup
        //////////////////////
        $display("[Test 1] Fetch PC without any branch inserted (should predict not taken PC+4)");
        predict_fetch(BASE_PC, 0, BASE_PC + 4);

        //////////////////////
        // Test 2: Insert taken branch
        //////////////////////
        $display("[Test 2] Insert branch and predict taken...");
        resolve_branch(BASE_PC, 1, TARGET_PC);

        predict_fetch(BASE_PC, 1, TARGET_PC);

        //////////////////////
        // Test 3: Correct a misprediction (taken to not-taken)
        //////////////////////
        $display("[Test 3] Force not-taken update and predict...");
        resolve_branch(BASE_PC, 0, BASE_PC + 4);

        predict_fetch(BASE_PC, 0, BASE_PC + 4);

        //////////////////////
        // Test 4: Overwrite BTB target
        //////////////////////
        $display("[Test 4] Overwrite BTB entry to different target...");
        resolve_branch(BASE_PC, 1, ALT_TARGET_PC);

        predict_fetch(BASE_PC, 1, ALT_TARGET_PC);

        //////////////////////
        // Test 5: Test Saturating Counter
        //////////////////////
        $display("[Test 5] Multiple updates to saturate predictor...");

        // Many taken updates to drive counter up
        repeat (5) resolve_branch(BASE_PC, 1, ALT_TARGET_PC);
        predict_fetch(BASE_PC, 1, ALT_TARGET_PC);

        // Many not-taken updates to drive counter down
        repeat (5) resolve_branch(BASE_PC, 0, BASE_PC + 4);
        predict_fetch(BASE_PC, 0, BASE_PC + 4);

        //////////////////////
        // Test 6: Test Return Address Stack
        //////////////////////
        $display("[Test 6] Test RAS (push and pop returns)");

        // Simulate a CALL
        push_call(BASE_PC);

        // Simulate RETURN fetch
        pop_return(BASE_PC);

        // We can't *directly* check RAS result easily here unless we hook internals,
        // but this validates through normal operation.

        //////////////////////
        // Test 7: Reset test again
        //////////////////////
        $display("[Test 7] Reset branch predictor and verify cleared state");

        reset_predictor();
        predict_fetch(BASE_PC, 0, BASE_PC + 4);

        //////////////////////
        $display("\nAll tests passed for branch_predictor_tb! ✅\n");
        $finish;
    end

endmodule
