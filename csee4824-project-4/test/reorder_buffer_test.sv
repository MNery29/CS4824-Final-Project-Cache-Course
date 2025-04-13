`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

`timescale 1ns/1ps

module testbench;

    logic clock, reset, load_entry, rob_to_rs_read1, rob_to_rs_read2, cdb_valid, retire_entry, rob_clear, rob_full, rob_out_valid, reg_valid;
    logic [4:0] dispatch_dest_reg, reg_dest;
    logic [5:0] rob_read_tag1, rob_read_tag2, cdb_tag, rob_tag_out, rob_retire_tag_out;
    logic [6:0] dispatch_opcode;
    logic [11:0] rob_pointers;
    logic [31:0] cdb_value, reg_value, rob_to_rs_value1, rob_to_rs_value2;
    logic [45:0] rob_debug[31:0];

    reorder_buffer rob(
        .clock(clock), 
        .reset(reset),
        .load_entry(load_entry),
        .rob_to_rs_read1(rob_to_rs_read1),
        .rob_to_rs_read2(rob_to_rs_read2),
        .cdb_valid(cdb_valid),
        .retire_entry(retire_entry),
        .rob_clear(rob_clear),
        .rob_full(rob_full),
        .rob_out_valid(rob_out_valid),
        .reg_valid(reg_valid),
        .dispatch_dest_reg(dispatch_dest_reg),
        .rob_read_tag1(rob_read_tag1),
        .rob_read_tag2(rob_read_tag2),
        .cdb_tag(cdb_tag),
        .reg_dest(reg_dest),
        .rob_tag_out(rob_tag_out),
        .rob_retire_tag_out(rob_retire_tag_out),
        .dispatch_opcode(dispatch_opcode),
        .cdb_value(cdb_value),
        .reg_value(reg_value),
        .rob_to_rs_value1(rob_to_rs_value1),
        .rob_to_rs_value2(rob_to_rs_value2),
        .rob_pointers(rob_pointers),
        .rob_debug(rob_debug)
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
        $monitor("Time:%4.0f clock:%b reset:%b\
                 |Inputs| cdb_valid:%b cdb_tag:%b cdb_value:%h load_entry:%b dispatch_dest_reg:%b dispatch_opcode:%b read1:%b read2:%b read_tag1:%b read_tag2:%b retire_entry:%b rob_clear:%b\
                 |Outputs| reg_dest:%b reg_value:%h reg_valid:%b rob_out_valid:%b rob_tag_out:%b rob_retire_tag_out:%b read_value1:%h read_value2:%h rob_full:%b",
                 $time, clock, reset, cdb_valid, cdb_tag, cdb_value, load_entry, dispatch_dest_reg, dispatch_opcode, rob_to_rs_read1, rob_to_rs_read2, 
                 rob_read_tag1, rob_read_tag2, retire_entry, rob_clear, reg_dest, reg_value, reg_valid, rob_out_valid, rob_tag_out, rob_retire_tag_out, rob_to_rs_value1, rob_to_rs_value2, rob_full);

        //Reset 
        clock = 1;
        reset = 1; //Pull reset high
        load_entry = 0;
        retire_entry = 0;
        cdb_valid = 0;
        rob_clear = 0;
        rob_to_rs_read1 = 0;
        rob_to_rs_read2 = 0;

        @(negedge clock);
        reset = 0; //Initialize inputs

        cdb_valid = 0;
        cdb_tag = 5'b00000;
        cdb_value = 32'h0000_0000;

        load_entry = 1; //Dispatch an instruction 
        dispatch_dest_reg = 5'b00001;
        dispatch_opcode = `RV32_ADD;

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers); 
        cdb_valid = 0; //Don't CDB broadcast
        cdb_tag = 5'b00000;
        cdb_value = 32'h0000_0000;

        load_entry = 1; //Dispatch another instruction 
        dispatch_dest_reg = 5'b00010;
        dispatch_opcode = `RV32_ADD;

        rob_to_rs_read1 = 0; 
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers); 
        cdb_valid = 0; //Don't CDB broadcast
        cdb_tag = 5'b00000;
        cdb_value = 32'h0000_0000;

        load_entry = 1; //Dispatch another instruction 
        dispatch_dest_reg = 5'b01000;
        dispatch_opcode = `RV32_SUB;

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers); 
        cdb_valid = 1; //CDB broadcast with tag 0
        cdb_tag = 5'b00001; 
        cdb_value = 32'hFFFF_FFFF;

        load_entry = 0; //Don't dispatch
        dispatch_dest_reg = 5'b01000; 
        dispatch_opcode = `RV32_SUB;

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers); 
        cdb_valid = 0; //Don't CDB broadcast
        cdb_tag = 5'b00000;
        cdb_value = 32'h0000_0000;

        load_entry = 0; //Don't dispatch
        dispatch_dest_reg = 5'b00000;
        dispatch_opcode = 0;

        retire_entry = 1; //Retire instruction! FFFF_FFFF should be stored to register 00001

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers); 
        cdb_valid = 0; 
        cdb_tag = 5'b00000;
        cdb_value = 32'h0000_0000;

        load_entry = 0;
        dispatch_dest_reg = 5'b00000;
        dispatch_opcode = 0;

        retire_entry = 1; //Retire instruction again. Nothing should happen since entry and head pointer is not ready!

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;
        
        @(negedge clock);
        print_contents(rob_debug, rob_pointers); 
        cdb_valid = 0; //Don't CDB broadcast
        cdb_tag = 5'b00000; 
        cdb_value = 32'h0000_0000;

        load_entry = 1; //Dispatch another instruction 
        dispatch_dest_reg = 5'b11111; 
        dispatch_opcode = `RV32_SUB;

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;

        @(negedge clock);
        print_contents(rob_debug, rob_pointers); 

        $finish;

    end

endmodule
