`include "verilog/sys_defs.svh"
`timescale 1ns/1ps

module testbench;

    logic clock, reset, read_cdb, load_entry, retire_entry;
    logic [4:0] rs1_addr, rs2_addr, r_dest, tag_in, cdb_tag_in, regfile_rs1_addr, regfile_rs2_addr, retire_addr;
    logic [5:0] rs1_tag, rs2_tag;

    map_table MT(
        .clock(clock),
        .reset(reset),
        .read_cdb(read_cdb),
        .retire_entry(retire_entry),
        .load_entry(load_entry),
        .rs1_addr(rs1_addr),
        .rs2_addr(rs2_addr),
        .r_dest(r_dest),
        .tag_in(tag_in),
        .cdb_tag_in(cdb_tag_in),
        .retire_addr(retire_addr),
        .rs1_tag(rs1_tag),
        .rs2_tag(rs2_tag),
        .regfile_rs1_addr(regfile_rs1_addr),
        .regfile_rs2_addr(regfile_rs2_addr)
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    initial begin
        $monitor("Time:%4.0f clock:%b reset:%b read_cdb:%b load_entry:%b retire_entry: %b retire_addr: %b rs1_addr:%b rs2_addr:%b r_dest:%b tag_in:%b cdb_tag_in:%b rs1_tag:%b rs2_tag:%b",
                 $time, clock, reset, read_cdb, load_entry, retire_entry, retire_addr, rs1_addr, rs2_addr, r_dest, tag_in, cdb_tag_in, rs1_tag, rs2_tag);

        //Reset 
        clock = 1'b0;
        reset = 1'b1; //Pull reset high
        read_cdb = 1'b0; //Do not read from CDB
        load_entry = 1'b0; //Do not load tag from RS
        retire_entry = 1'b0; //Do not retire entry 
        rs1_addr = 5'b0; //Set RS inputs to zero 
        rs2_addr = 5'b0;
        r_dest = 5'b0;
        tag_in = 5'b0; 
        cdb_tag_in = 5'b0; //Set CDB tag input to zero 
        @(negedge clock);
        reset = 1'b0; //Pull reset low 
        read_cdb = 1'b0; //Do not read from CDB
        load_entry = 1'b1; //Load a tag from RS
        retire_entry = 1'b0; //Do not retire entry 
        rs1_addr = 5'b00001; //Get tags for these registers
        rs2_addr = 5'b00010;
        r_dest = 5'b00011; //Set tag at r_dest to 1
        tag_in = 5'b00001; 
        cdb_tag_in = 5'b0; //Set CDB tag input to zero 
        @(negedge clock);
        read_cdb = 1'b0; //Do not read from CDB
        load_entry = 1'b0; //Do not load tag from RS
        retire_entry = 1'b0; //Do not retire entry 
        rs1_addr = 5'b00010; //Get tags for these registers 
        rs2_addr = 5'b00011; //rs2 tag should be the tag we just fed in + 0 for the ready in ROB bit!
        r_dest = 5'b00000; 
        tag_in = 5'b00000; 
        cdb_tag_in = 5'b0; //Set CDB tag input to zero 
        @(negedge clock);
        read_cdb = 1'b1; //Read from CDB
        load_entry = 1'b0; //Do not load tag from RS
        retire_entry = 1'b0; //Do not retire entry 
        rs1_addr = 5'b00010; //Get tags for these registers 
        rs2_addr = 5'b00011; //rs2 tag should be the same tag + 1 for the ready in ROB bit
        r_dest = 5'b00000; 
        tag_in = 5'b00000; 
        cdb_tag_in = 5'b00001; //Set CDB tag input to the tag at rs2
        @(negedge clock);
        read_cdb = 1'b0; //Do not read from CDB
        load_entry = 1'b0; //Do not load tag from RS
        retire_entry = 1'b1; //Retire entry 
        retire_addr = 5'b00011; //Retire entry at rs2 
        rs1_addr = 5'b00010; //Get tags for these registers 
        rs2_addr = 5'b00011; //rs2 tag should be cleared!
        r_dest = 5'b00011; //These should be ignored
        tag_in = 5'b00010; 
        cdb_tag_in = 5'b00001; 
        @(negedge clock);

        $finish;

    end

endmodule
