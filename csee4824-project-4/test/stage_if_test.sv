`timescale 1ns/1ps

`include "verilog/sys_defs.svh"

module testbench();

// Inputs
    logic clock;
    logic reset;
    logic if_valid;
    logic take_branch;
    logic [`XLEN-1:0] branch_target;
    logic [63:0] Icache_data_out;
    logic Icache_valid_out;

    // Outputs
    IF_ID_PACKET if_packet;
    logic [`XLEN-1:0] proc2Icache_addr;
    logic stall_if;

    //expected outputs: 
    logic [`XLEN-1:0] expected_PC;
    logic [`XLEN-1:0] expected_NPC;
    logic [31:0] expected_inst;
    logic expected_valid;

    // DUT
    stage_if dut (
        .clock(clock),
        .reset(reset),
        .if_valid(if_valid),
        .take_branch(take_branch),
        .branch_target(branch_target),
        .Icache_data_out(Icache_data_out),
        .Icache_valid_out(Icache_valid_out),
        .if_packet(if_packet),
        .proc2Icache_addr(proc2Icache_addr),
        .stall_if(stall_if)
    );

    // Clock generator
    always #5 clock = ~clock;

   // Observation + Assertions
    always @(posedge clock) begin
        expected_valid = Icache_valid_out && if_valid;

        if (reset) begin
            expected_PC = 0;
            expected_NPC = 4;
        end else if (take_branch) begin
            expected_PC = branch_target;
            expected_NPC = branch_target + 4;
        end else if (if_valid && Icache_valid_out) begin
            expected_PC = expected_PC + 4;
            expected_NPC = expected_PC + 4;
        end

        expected_inst = (expected_PC[2]) ? Icache_data_out[63:32] : Icache_data_out[31:0];

        if (if_packet.valid !== expected_valid) begin
            $fatal("Mismatch @%0t: valid = %b, expected = %b", $time, if_packet.valid, expected_valid);
        end

        if (if_packet.PC !== expected_PC) begin
            $fatal("Mismatch @%0t: PC = 0x%h, expected = 0x%h", $time, if_packet.PC, expected_PC);
        end

        if (if_packet.valid && if_packet.inst !== expected_inst) begin
            $fatal("Mismatch @%0t: inst = 0x%h, expected = 0x%h", $time, if_packet.inst, expected_inst);
        end

        $display("Time %0t | PC=0x%h NPC=0x%h inst=0x%h valid=%b stall=%b",
                 $time, if_packet.PC, if_packet.NPC, if_packet.inst, if_packet.valid, stall_if);
    end

    initial begin
        $display("Starting IF stage test...");

        clock = 0;
        reset = 1;
        if_valid = 0;
        take_branch = 0;
        branch_target = 0;
        Icache_data_out = 64'hDEADBEEF_F00DFACE;
        Icache_valid_out = 0;

        // Hold reset
        #10;
        reset = 0;

        //  No valid fetch yet
        #10;
        if_valid = 1;
        Icache_valid_out = 0;

        // Still stall, cache not ready
        #10;
        Icache_valid_out = 1;
        Icache_data_out = 64'hAABBCCDD_11223344;

        //  Should accept instruction (lower half)
        #10;

        // Next fetch, should go to PC+4
        Icache_data_out = 64'hFFEEDDCC_BBAA9988;
        #10;

        // Simulate cache miss (stall)
        Icache_valid_out = 0;
        #10;

        // Deliver upper instruction
        Icache_valid_out = 1;
        Icache_data_out = 64'h12345678_9ABCDEF0;
        #10;

        // Take branch
        take_branch = 1;
        branch_target = 32'h100;
        #10;

        // Clear branch
        take_branch = 0;
        #10;

        // Finish
        $display("Test complete.");
        $finish;
    end

endmodule
