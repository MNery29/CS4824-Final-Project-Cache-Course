`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

`timescale 1ns/1ps

module testbench;

    logic clock, reset;
    logic rob_to_rs_read1, rob_to_rs_read2;
    logic [4:0] rob_read_tag1, rob_read_tag2;
    logic retire_entry, rob_clear;
    logic rob_full;
    logic [31:0] rob_to_rs_value1, rob_to_rs_value2;

    logic [11:0] rob_pointers; 
    logic [45:0] rob_debug[31:0];

    // Local Values
    logic        dispatch_valid;
    logic [4:0]  dispatch_dest;
    logic [6:0]  dispatch_opcode;

    logic        cdb_valid;
    logic [5:0]  cdb_tag;
    logic [31:0] cdb_value;

    ROB_DISPATCH_PACKET rob_dispatch_out;
    ROB_RETIRE_PACKET   rob_retire_out;

    DISPATCH_ROB_PACKET rob_dispatch_in_temp;
    CDB_ROB_PACKET      rob_cdb_in_temp;
    //lsq input
    logic store_retire;
    logic [4:0] store_tag;
    logic rob_ready;
    logic rob_valid;
    
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    // DUT
    reorder_buffer rob (
        .clock(clock),
        .reset(reset),

        .rob_dispatch_in(rob_dispatch_in_temp),
        .rob_cdb_in(rob_cdb_in_temp),

        .rob_to_rs_read1(rob_to_rs_read1),
        .rob_read_tag1(rob_read_tag1),
        .rob_to_rs_read2(rob_to_rs_read2),
        .rob_read_tag2(rob_read_tag2),

        .retire_entry(retire_entry),
        .rob_clear(rob_clear),
        .store_retire(store_retire),
        .store_tag(store_tag),
        .rob_ready(rob_ready),
        .rob_valid(rob_valid),


        .rob_dispatch_out(rob_dispatch_out),
        .rob_retire_out(rob_retire_out),
        .rob_to_rs_value1(rob_to_rs_value1),
        .rob_to_rs_value2(rob_to_rs_value2),
        .rob_full(rob_full),
        .rob_debug(rob_debug),
        .rob_pointers(rob_pointers)
    );

    // Struct Assembly
    always_comb begin
        rob_dispatch_in_temp = '{valid: dispatch_valid, dest_reg: dispatch_dest, opcode: dispatch_opcode, is_branch:0};
        rob_cdb_in_temp       = '{valid: cdb_valid, tag: cdb_tag, value: cdb_value};
    end

    // Debug
    task print_contents;
        input [45:0] array [31:0];
        input [11:0] pointers;
        integer i;
        begin
            $display("ROB contents:");
            for (i = 0; i < 32; i++) begin
                $display("Status:%b Opcode:%b Dest:%b Value:%h", array[i][45:44], array[i][43:37], array[i][36:32], array[i][31:0]);
            end
            $display("H/T pointers: Head:%b Tail:%b", pointers[11:6], pointers[5:0]);
        end
    endtask

    // Output
    initial begin
        $monitor("Time:%4.0f | clk:%b rst:%b | Dispatch: valid=%b dest=%b opcode=%b | CDB: valid=%b tag=%b value=%h | Retire: tag=%b dest=%b value=%h reg_valid=%b mem_valid=%b | rob_full=%b",
            $time, clock, reset,
            dispatch_valid, dispatch_dest, dispatch_opcode,
            cdb_valid, cdb_tag, cdb_value,
            rob_retire_out.tag, rob_retire_out.dest_reg, rob_retire_out.value,
            rob_retire_out.reg_valid, rob_retire_out.mem_valid,
            rob_full);
    end

    initial begin
        // reset
        $display("Starting testbench... RESETING ALL VALUES");
        clock = 1;
        reset = 1;
        dispatch_valid = 0;
        dispatch_dest  = 0;
        dispatch_opcode = 0;
        cdb_valid = 0;
        cdb_tag = 0;
        cdb_value = 0;
        retire_entry = 0;
        rob_clear = 0;
        rob_to_rs_read1 = 0;
        rob_to_rs_read2 = 0;

        @(negedge clock);
        reset = 0;
        $display("Starting testbench... RESET COMPLETE");

        // Instruction 1
        $display("Dispatching instruction 1");
        dispatch_valid = 1;
        dispatch_dest = 5'd1;
        dispatch_opcode = `RV32_ADD;
        print_contents(rob_debug, rob_pointers);

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Instruction 2 
        $display("Dispatching instruction 2");
        dispatch_dest = 5'd2;
        print_contents(rob_debug, rob_pointers);

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Instruction 3
        $display("Dispatching instruction 3");
        dispatch_dest = 5'd8;
        dispatch_opcode = `RV32_SUB;
        print_contents(rob_debug, rob_pointers);

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Broadcast CDB tag 1
        $display("Broadcasting CDB tag 1");
        dispatch_valid = 0;
        cdb_valid = 1;
        cdb_tag = 6'd1;
        cdb_value = 32'hFFFF_FFFF;
        print_contents(rob_debug, rob_pointers);

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Retire instruction 
        $display("Retiring instruction 1");
        cdb_valid = 0;
        retire_entry = 1;
        print_contents(rob_debug, rob_pointers);

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Attempt another retire (may stall)
        $display("Attempting to retire instruction 2 (may stall)");
        retire_entry = 1;
        print_contents(rob_debug, rob_pointers);

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Instruction 4
        $display("Dispatching instruction 4");
        retire_entry = 0;
        dispatch_valid = 1;
        dispatch_dest = 5'd31;
        dispatch_opcode = `RV32_SUB;
        print_contents(rob_debug, rob_pointers);

        @(negedge clock);
        print_contents(rob_debug, rob_pointers);
        //print_contents(rob_debug, rob_pointers);

        $finish;
    end

endmodule