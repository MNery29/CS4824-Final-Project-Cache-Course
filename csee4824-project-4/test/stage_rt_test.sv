`timescale 1ns/1ps

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module tb_stage_rt();

    // Inputs
    logic clock;
    logic reset;
    ROB_RETIRE_PACKET rob_retire_packet;
    logic branch_mispredict;

    // Outputs
    logic [63:0] retire_value;
    logic [4:0] retire_dest;
    logic retire_valid_out;
    logic [63:0] mem_addr;
    logic mem_valid;

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
        if (reset || branch_mispredict) begin
            expected_retire_value = 64'b0;
            expected_retire_dest = 5'b0;
            expected_retire_valid_out = 1'b0;
            expected_mem_addr = 64'b0;
            expected_mem_valid = 1'b0;
        end else if (rob_retire_packet.reg_valid) begin
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

        // Display info each cycle
        $display("Time %0t | retire_valid=%b retire_dest=%0d retire_value=0x%h mem_valid=%b mem_addr=0x%h",
            $time, retire_valid_out, retire_dest, retire_value, mem_valid, mem_addr);
    end

    // Stimulus
    initial begin
        $display("Starting Retire Stage Test...");

        // Initial state
        clock = 0;
        reset = 1;
        branch_mispredict = 0;
        rob_retire_packet = '{default:0};

        // Hold reset
        #10;
        reset = 0;

        // Cycle 1: No valid retire (reg_valid = 0)
        rob_retire_packet.reg_valid = 0;
        rob_retire_packet.mem_valid = 0;
        #10;

        // Cycle 2: Retire valid instruction
        rob_retire_packet = '{tag:6'd5, dest_reg:5'd10, value:64'h1234_5678_ABCD_EF01,
                              reg_valid:1'b1, mem_valid:1'b0, mem_addr:64'h0};
        #10;

        // Cycle 3: Retire memory valid instruction
        rob_retire_packet = '{tag:6'd6, dest_reg:5'd12, value:64'hAABB_CCDD_EEFF_0011,
                              reg_valid:1'b1, mem_valid:1'b1, mem_addr:64'h8000_0040};
        #10;

        // Cycle 4: Another no-op (reg_valid = 0 again)
        rob_retire_packet = '{default:0};
        #10;

        // Cycle 5: Branch mispredict — clear outputs
        branch_mispredict = 1;
        #10;

        // Cycle 6: Clear branch mispredict
        branch_mispredict = 0;
        rob_retire_packet = '{tag:6'd7, dest_reg:5'd15, value:64'h1111_2222_3333_4444,
                              reg_valid:1'b1, mem_valid:1'b0, mem_addr:64'h0};
        #10;

        $display("Test complete.");
        $finish;
    end

endmodule
