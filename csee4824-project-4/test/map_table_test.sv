`include "verilog/sys_defs.svh"
`timescale 1ns/1ps

module testbench;

    logic clock, reset, read_cdb, load_entry, retire_entry;
    logic [4:0] rs1_addr, rs2_addr, r_dest, regfile_rs1_addr, regfile_rs2_addr, retire_addr;
    logic [5:0] tag_in, cdb_tag_in, retire_tag;
    logic [6:0] rs1_tag, rs2_tag;
    //logic [7:0] tags_debug[31:0]; //DEBUGGING

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
        .retire_tag(retire_tag),
        .rs1_tag(rs1_tag),
        .rs2_tag(rs2_tag),
        .regfile_rs1_addr(regfile_rs1_addr),
        .regfile_rs2_addr(regfile_rs2_addr)
        //.tags_debug(tags_debug) /DEBUGGING!
    );
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    task print_contents;
        input [7:0] array [31:0];
        integer i;
        begin
            $display("MT contents:");
            for (i = 0; i < 32; i++) begin
                $display("Register:%b Tag:%b Ready_in_ROB:%b Has_tag:%h", i[4:0], array[i][7:2], array[i][1], array[i][0]);
            end
        end
    endtask

    initial begin
        $monitor("Time:%4.0f clock:%b reset:%b read_cdb:%b load_entry:%b retire_entry:%b retire_addr:%b retire_tag:%b rs1_addr:%b rs2_addr:%b r_dest:%b tag_in:%b cdb_tag_in:%b rs1_tag:%b rs2_tag:%b",
                 $time, clock, reset, read_cdb, load_entry, retire_entry, retire_addr, retire_tag, rs1_addr, rs2_addr, r_dest, tag_in, cdb_tag_in, rs1_tag, rs2_tag);

        //Reset 
        clock = 1'b0;
        reset = 1'b1; //Pull reset high
        read_cdb = 1'b0; //Do not read from CDB
        load_entry = 1'b0; //Do not load tag from RS
        retire_entry = 1'b0; //Do not retire entry 
        retire_tag = 6'b0;
        retire_addr = 5'b0;
        rs1_addr = 5'b0; //Set RS inputs to zero 
        rs2_addr = 5'b0;
        r_dest = 5'b0;
        tag_in = 6'b0; 
        cdb_tag_in = 6'b0; //Set CDB tag input to zero 
        @(negedge clock);
        //print_contents(tags_debug); DEBUGGING
        reset = 1'b0; //Pull reset low 
        read_cdb = 1'b0; //Do not read from CDB
        load_entry = 1'b1; //Load a tag from RS
        retire_entry = 1'b0; //Do not retire entry 
        retire_tag = 6'b0;
        rs1_addr = 5'b00001; //Get tags for these registers
        rs2_addr = 5'b00010;
        r_dest = 5'b00001; //Set tag at r_dest to 1
        tag_in = 6'b00001; 
        cdb_tag_in = 6'b0; //Set CDB tag input to zero 
        @(negedge clock);
        //print_contents(tags_debug);
        read_cdb = 1'b0; //Do not read from CDB
        load_entry = 1'b1; //Do not load tag from RS
        retire_entry = 1'b0; //Do not retire entry 
        retire_tag = 6'b0;
        rs1_addr = 5'b00010; //Get tags for these registers 
        rs2_addr = 5'b00001; //rs2 tag should be the tag we just fed in + 0 for the ready in ROB bit!
        r_dest = 5'b00010; 
        tag_in = 6'b00010; 
        cdb_tag_in = 6'b0; //Set CDB tag input to zero 
        @(negedge clock);
        //print_contents(tags_debug);
        read_cdb = 1'b1; //Read from CDB
        load_entry = 1'b0; //Do not load tag from RS
        retire_entry = 1'b0; //Do not retire entry 
        retire_tag = 6'b0;
        rs1_addr = 5'b00010; //Get tags for these registers 
        rs2_addr = 5'b00001; //rs2 tag should be the same tag + 1 for the ready in ROB bit
        r_dest = 5'b00000; 
        tag_in = 6'b00000; 
        cdb_tag_in = 6'b000001; //Set CDB tag input to the tag at rs2
        @(negedge clock);
        //print_contents(tags_debug);
        read_cdb = 1'b0; //Do not read from CDB
        load_entry = 1'b0; //Do not load tag from RS
        retire_entry = 1'b1; //Retire entry 
        retire_tag = 6'b000001;
        retire_addr = 5'b00001; //Retire entry at rs2 
        rs1_addr = 5'b00010; //Get tags for these registers 
        rs2_addr = 5'b00011; //rs2 tag should be cleared!
        r_dest = 5'b00011; //These should be ignored
        tag_in = 6'b000001; 
        cdb_tag_in = 6'b000001; 
        @(negedge clock);
        //print_contents(tags_debug);
        read_cdb = 1'b0; //Do not read from CDB
        load_entry = 1'b1; //Load tag from RS
        retire_entry = 1'b0; 
        retire_addr = 5'b00000;
        rs1_addr = 5'b00001; //Get tags for these registers 
        rs2_addr = 5'b00010; //rs2 tag should be cleared!
        r_dest = 5'b00011; //These should be ignored
        tag_in = 6'b000010; 
        cdb_tag_in = 6'b0; 
        @(negedge clock);
        //print_contents(tags_debug);

        $finish;

    end

endmodule
