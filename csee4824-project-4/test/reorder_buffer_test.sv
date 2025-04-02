`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

`timescale 1ns/1ps

module testbench;

    logic clock, reset, dispatch_valid, rob_to_rs_read1, rob_to_rs_read2, cdb_valid, retire_valid, clear_rob, rob_full, rob_out_valid;
    logic [4:0] dispatch_dest_reg, rob_read_tag1, rob_read_tag2, cdb_tag, reg_dest, rob_tag_out;
    logic [6:0] dispatch_opcode;
    logic [31:0] cdb_value, reg_value, rob_to_rs_value1, rob_to_rs_value2;
    logic [11:0] rob_debug;

    reorder_buffer rob(
        .clock(clock), 
        .reset(reset),
        .dispatch_valid(dispatch_valid),
        .rob_to_rs_read1(rob_to_rs_read1),
        .rob_to_rs_read2(rob_to_rs_read2),
        .cdb_valid(cdb_valid),
        .retire_valid(retire_valid),
        .clear_rob(clear_rob),
        .rob_full(rob_full),
        .rob_out_valid(rob_out_valid),
        .dispatch_dest_reg(dispatch_dest_reg),
        .rob_read_tag1(rob_read_tag1),
        .rob_read_tag2(rob_read_tag2),
        .cdb_tag(cdb_tag),
        .reg_dest(reg_dest),
        .rob_tag_out(rob_tag_out),
        .dispatch_opcode(dispatch_opcode),
        .cdb_value(cdb_value),
        .reg_value(reg_value),
        .rob_to_rs_value1(rob_to_rs_value1),
        .rob_to_rs_value2(rob_to_rs_value2),
        .rob_debug(rob_debug)
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    initial begin
        $monitor("Time:%4.0f clock:%b reset:%b\
                 |Inputs| cdb_valid:%b cdb_tag:%b cdb_value:%h dispatch_valid:%b dispatch_dest_reg:%b dispatch_opcode:%b read1:%b read2:%b read_tag1:%b read_tag2:%b retire_valid:%b clear_rob:%b\
                 |Outputs| reg_dest:%b reg_value:%h rob_out_valid:%b rob_tag_out:%b read_value1:%h read_value2:%h rob_full:%b debug:%b",
                 $time, clock, reset, cdb_valid, cdb_tag, cdb_value, dispatch_valid, dispatch_dest_reg, dispatch_opcode, rob_to_rs_read1, rob_to_rs_read2, 
                 rob_read_tag1, rob_read_tag2, retire_valid, clear_rob, reg_dest, reg_value, rob_out_valid, rob_tag_out, rob_to_rs_value1, rob_to_rs_value2, rob_full, rob_debug);

        //Reset 
        clock = 1;
        reset = 1; //Pull reset high
        dispatch_valid = 0;
        retire_valid = 0;
        cdb_valid = 0;
        clear_rob = 0;
        rob_to_rs_read1 = 0;
        rob_to_rs_read2 = 0;

        @(negedge clock);
        reset = 0; //Initialize inputs

        cdb_valid = 0;
        cdb_tag = 5'b00000;
        cdb_value = 32'h0000_0000;

        dispatch_valid = 1;
        dispatch_dest_reg = 5'b00001;
        dispatch_opcode = `RV32_ADD;

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;

        @(negedge clock);
        cdb_valid = 0; //Don't CDB broadcast
        cdb_tag = 5'b00000;
        cdb_value = 32'h0000_0000;

        dispatch_valid = 1; //Dispatch an instruction 
        dispatch_dest_reg = 5'b00010;
        dispatch_opcode = `RV32_ADD;

        rob_to_rs_read1 = 0; 
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;

        @(negedge clock);
        cdb_valid = 0; //Don't CDB broadcast
        cdb_tag = 5'b00000;
        cdb_value = 32'h0000_0000;

        dispatch_valid = 1; //Dispatch another instruction 
        dispatch_dest_reg = 5'b01000;
        dispatch_opcode = `RV32_SUB;

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;

        @(negedge clock);
        cdb_valid = 1; //CDB broadcast with tag 0
        cdb_tag = 5'b00000; 
        cdb_value = 32'hFFFF_FFFF;

        dispatch_valid = 0; //Don't dispatch
        dispatch_dest_reg = 5'b01000; 
        dispatch_opcode = `RV32_SUB;

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;

        @(negedge clock);
        cdb_valid = 0; //Don't CDB broadcast
        cdb_tag = 5'b00000;
        cdb_value = 32'h0000_0000;

        dispatch_valid = 0; //Don't dispatch
        dispatch_dest_reg = 5'b00000;
        dispatch_opcode = 0;

        retire_valid = 1; //Retire instruction! FFFF_FFFF should be stored to register 00001

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;
        @(negedge clock);
        cdb_valid = 0; 
        cdb_tag = 5'b00000;
        cdb_value = 32'h0000_0000;

        dispatch_valid = 0;
        dispatch_dest_reg = 5'b00000;
        dispatch_opcode = 0;

        retire_valid = 1; //Retire instruction again. Nothing should happen since entry and head pointer is not ready!

        rob_to_rs_read1 = 0;
        rob_read_tag1 = 5'b00000;

        rob_to_rs_read2 = 0;
        rob_read_tag2 = 5'b00000;
        @(negedge clock);
        $finish;

    end

endmodule
