`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

`timescale 1ns/1ps

module testbench;

    logic clock, reset;
    logic rob_to_rs_read1, rob_to_rs_read2;
    logic [5:0] rob_read_tag1, rob_read_tag2;
    logic retire_entry, rob_clear;
    logic rob_full;
    logic [31:0] rob_to_rs_value1, rob_to_rs_value2;
    logic [11:0] rob_pointers;
    logic [45:0] rob_debug[31:0];

    DISPATCH_ROB_PACKET rob_dispatch_in;
    CDB_ROB_PACKET cdb_rob_in;
    ROB_DISPATCH_PACKET rob_dispatch_out;
    ROB_RETIRE_PACKET rob_retire_out;


    reorder_buffer rob (
        .clock(clock),
        .reset(reset),

        .rob_dispatch_in(rob_dispatch_in),
        .rob_cdb_in(rob_cdb_in),

        .rob_to_rs_read1(rob_to_rs_read1),
        .rob_read_tag1(rob_read_tag1),
        .rob_to_rs_read2(rob_to_rs_read2),
        .rob_read_tag2(rob_read_tag2),

        .retire_entry(retire_entry),
        .rob_clear(rob_clear),

        .rob_dispatch_out(rob_dispatch_out),
        .rob_retire_out(rob_retire_out),
        .rob_to_rs_value1(rob_to_rs_value1),
        .rob_to_rs_value2(rob_to_rs_value2),
        .rob_full(rob_full),
        .rob_debug(rob_debug),
        .rob_pointers(rob_pointers)
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    task print_contents;
        input [45:0] array [31:0];
        input [11:0] pointers;
        integer i;
        begin
            $display("ROB contents:");
            for (i = 0; i < 32; i++) begin
                $display("Status:%b Opcode:%b Dest:%b Value:%h", array[i][45:44], array[i][43:37], array[i][36:32], array[i][31:0]);
            end
            $display("H/T pointers:");
            $display("Head:%b Tail:%b", pointers[11:6], pointers[5:0]);
        end

    endtask

    initial begin
        $monitor("Time:%4.0f | clk:%b rst:%b | Dispatch: valid=%b dest=%b opcode=%b | CDB: valid=%b tag=%b value=%h | Retire: tag=%b dest=%b value=%h valid=%b mem=%b | rob_full=%b",
                 $time, clock, reset, 
                 rob_dispatch_in.valid, rob_dispatch_in.dest_reg, rob_dispatch_in.opcode,
                 rob_cdb_in.valid, rob_cdb_in.tag, rob_cdb_in.value,
                 rob_retire_out.tag, rob_retire_out.dest_reg, rob_retire_out.value,
                 rob_retire_out.reg_valid, rob_retire_out.mem_valid, rob_full);

        // Initial Reset
        clock = 1;
        reset = 1;
        retire_entry = 0;
        rob_clear = 0;
        rob_to_rs_read1 = 0;
        rob_to_rs_read2 = 0;

        rob_dispatch_in = '{default: 0};
        rob_cdb_in      = '{default: 0};

        @(negedge clock);
        reset = 0;

        // Cycle 1 
        rob_dispatch_in.valid    = 1;
        rob_dispatch_in.dest_reg = 5'b00001;
        rob_dispatch_in.opcode   = `RV32_ADD;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers);

        // Cycle 2 
        rob_dispatch_in.dest_reg = 5'b00010;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers);

        // Cycle 3 
        rob_dispatch_in.dest_reg = 5'b01000;
        rob_dispatch_in.opcode   = `RV32_SUB;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers);

        // Cycle 4 
        rob_dispatch_in.valid = 0;
        rob_cdb_in.valid = 1;
        rob_cdb_in.tag   = 6'd1;
        rob_cdb_in.value = 32'hFFFF_FFFF;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers);

        // Cycle 5 
        rob_cdb_in.valid = 0;
        retire_entry     = 1;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers);

        // Cycle 6 
        retire_entry = 1;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers);

        // Cycle 7
        retire_entry = 0;
        rob_dispatch_in.valid    = 1;
        rob_dispatch_in.dest_reg = 5'b11111;
        rob_dispatch_in.opcode   = `RV32_SUB;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers);

        $finish;
    end


endmodule
