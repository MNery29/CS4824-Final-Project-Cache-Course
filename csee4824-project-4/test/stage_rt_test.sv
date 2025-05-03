`timescale 1ns/1ps

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module testbench();

    // Inputs
    logic clock;
    logic reset;
    ROB_RETIRE_PACKET rob_retire_packet;
    logic branch_mispredict;
    logic rob_valid;
    logic rob_ready;

    // Outputs
    logic [63:0] retire_value;
    logic [4:0] retire_dest;
    logic retire_valid_out;
    logic [63:0] mem_addr;
    logic mem_valid;

    //reset toggle check: 
    logic after_reset; // used to ensure asserts don't start till after we are actually up and running.

    // Expected Outputs
    logic [63:0] expected_retire_value;
    logic [4:0] expected_retire_dest;
    logic expected_retire_valid_out;
    logic [63:0] expected_mem_addr;
    logic expected_mem_valid;

    // DUT
    stage_rt dut (
        .clock(clock),
        .reset(reset),
        .rob_retire_packet(rob_retire_packet),
        .branch_mispredict(branch_mispredict),
        .rob_valid(rob_valid),
        .rob_ready(rob_ready),
        .retire_value(retire_value),
        .retire_dest(retire_dest),
        .retire_valid_out(retire_valid_out),
        .mem_addr(mem_addr),
        .mem_valid(mem_valid)
    );

    // Clock generator
    always #5 clock = ~clock;

    // Observation + Assertions
    always @(posedge clock) begin
        // Determine what we *expect* based on inputs
        if (reset || branch_mispredict) begin
            expected_retire_value = 64'b0;
            expected_retire_dest = 5'b0;
            expected_retire_valid_out = 1'b0;
            expected_mem_addr = 64'b0;
            expected_mem_valid = 1'b0;
        end else if (rob_valid && rob_ready) begin
            expected_retire_value = rob_retire_packet.value;
            expected_retire_dest = rob_retire_packet.dest_reg;
            expected_retire_valid_out = 1'b1;
            expected_mem_addr = rob_retire_packet.mem_addr;
            expected_mem_valid = rob_retire_packet.mem_valid;
        end else begin
            expected_retire_value = 64'b0;
            expected_retire_dest = 5'b0;
            expected_retire_valid_out = 1'b0;
            expected_mem_addr = 64'b0;
            expected_mem_valid = 1'b0;
        end
        // Assertions
        if (after_reset) begin
            if (retire_valid_out !== expected_retire_valid_out) begin
            $fatal("Mismatch @%0t: retire_valid_out = %b, expected = %b", $time, retire_valid_out, expected_retire_valid_out);
        end

        if (retire_dest !== expected_retire_dest) begin
            $fatal("Mismatch @%0t: retire_dest = %0d, expected = %0d", $time, retire_dest, expected_retire_dest);
        end

        if (retire_value !== expected_retire_value) begin
            $fatal("Mismatch @%0t: retire_value = 0x%h, expected = 0x%h", $time, retire_value, expected_retire_value);
        end

        if (mem_valid !== expected_mem_valid) begin
            $fatal("Mismatch @%0t: mem_valid = %b, expected = %b", $time, mem_valid, expected_mem_valid);
        end

        if (mem_addr !== expected_mem_addr) begin
            $fatal("Mismatch @%0t: mem_addr = 0x%h, expected = 0x%h", $time, mem_addr, expected_mem_addr);
        end

        // Display cycle results
        $display("Time %0t | retire_valid=%b retire_dest=%0d retire_value=0x%h mem_valid=%b mem_addr=0x%h",
            $time, retire_valid_out, retire_dest, retire_value, mem_valid, mem_addr);
        end  
    end

    // Stimulus
    initial begin
        $display("Starting Retire Stage Test...");

        // Initial state
        clock = 0;
        reset = 1;
        branch_mispredict = 0;
        rob_valid = 0;
        rob_ready = 0;
        rob_retire_packet = '{default:0};

        // Hold reset
        #10;
        reset = 0;
        after_reset = 1;

        // Cycle 1: No valid retire (empty ROB)
        rob_valid = 0;
        rob_ready = 0;
        rob_retire_packet = '{tag:6'd0, dest_reg:5'd0, value:64'h0, reg_valid:1'b0, mem_valid:1'b0, mem_addr:64'h0};
        #10;

        // Cycle 2: Valid instruction but not ready yet
        rob_valid = 1;
        rob_ready = 0;
        rob_retire_packet = '{tag:6'd1, dest_reg:5'd5, value:64'h1111_1111_1111_1111, reg_valid:1'b1, mem_valid:1'b0, mem_addr:64'h0};
        #10;

        // Cycle 3: Ready now — should retire
        rob_valid = 1;
        rob_ready = 1;
        rob_retire_packet = '{tag:6'd1, dest_reg:5'd5, value:64'h1111_1111_1111_1111, reg_valid:1'b1, mem_valid:1'b0, mem_addr:64'h0};
        #10;

        // Cycle 4: Memory operation, valid and ready
        rob_valid = 1;
        rob_ready = 1;
        rob_retire_packet = '{tag:6'd2, dest_reg:5'd8, value:64'h8000_0040, reg_valid:1'b1, mem_valid:1'b1, mem_addr:64'h8000_0040};
        #10;

        // Cycle 5: No valid instruction (empty head)
        rob_valid = 0;
        rob_ready = 0;
        rob_retire_packet = '{default:0};
        #10;

        // Cycle 6: Simulate branch mispredict flush
        branch_mispredict = 1;
        #10;

        // Cycle 7: Recovery after branch
        branch_mispredict = 0;
        rob_valid = 1;
        rob_ready = 1;
        rob_retire_packet = '{tag:6'd3, dest_reg:5'd12, value:64'hDEAD_BEEF_CAFE_BABE, reg_valid:1'b1, mem_valid:1'b0, mem_addr:64'h0};
        #10;

        $display("Test complete.");
        $finish;
    end

endmodule
