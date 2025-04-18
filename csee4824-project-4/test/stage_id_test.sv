`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

`timescale 1ns/1ps

module testbench;
    
    logic clock;
    logic reset;

    //IF packet
    IF_ID_PACKET if_id_packet;
    
    //CDB inputs
    logic cdb_valid;
    logic [4:0] cdb_tag;
    logic [31:0] cdb_value;

    //Control inputs
    logic mt_retire_entry;
    logic rs1_clear;
    logic rs1_issue;
    logic rob_retire_entry;
    logic rob_clear;

    //Ouputs
    logic [31:0] opA;
    logic [31:0] opB;
    logic [4:0] output_tag;

    ALU_OPA_SELECT opa_select;
    ALU_OPB_SELECT opb_select;
    logic has_dest_reg;
    logic [4:0] dest_reg_idx;

    //Debug inputs
    //logic [45:0] rob_debug [31:0];
    logic [11:0] rob_pointers_debug;
    //logic [7:0] mt_tags_debug [31:0];
    //logic [74:0] rs_debug;

    stage_id dispatch(
        .clock(clock),
        .reset(reset),

        .if_id_reg(if_id_packet),

        .cdb_valid(cdb_valid),
        .cdb_tag(cdb_tag),
        .cdb_value(cdb_value),

        .mt_retire_entry(mt_retire_entry),
        .rs1_issue(rs1_issue),
        .rs1_clear(rs1_clear),
        .rob_retire_entry(rob_retire_entry),
        .rob_clear(rob_clear),

        .opA(opA),
        .opB(opB),
        .output_tag(output_tag),
        
        //.rob_debug(rob_debug),
        .rob_pointers_debug(rob_pointers_debug),
        //.mt_tags_debug(mt_tags_debug),
        //.rs_debug(rs_debug),
        .opa_select(opa_select),
        .opb_select(opb_select),
        .has_dest_reg(has_dest_reg),
        .dest_reg_idx(dest_reg_idx)
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    task print_MT;
        input [7:0] array [31:0];
        integer i;
        begin
            $display("mt contents:");
            for (i = 0; i < 32; i++) begin
                $display("register:%b tag:%b ready_in_rob:%b has_tag:%h", i[4:0], array[i][7:2], array[i][1], array[i][0]);
            end
        end
    endtask

    task print_ROB;
        input [45:0] array [31:0];
        input [11:0] pointers;
        integer i;
        begin
            $display("rob contents:");
            for (i = 0; i < 32; i++) begin
                $display("status:%b opcode:%b dest:%b value:%h", array[i][45:44], array[i][43:37], array[i][36:32], array[i][31:0]);
            end
            $display("h/t pointers:");
            $display("head:%b tail:%b", pointers[11:6], pointers[5:0]);
        end
    endtask

    task print_RS;
        input [74:0] entry;
        $display("rs contents:");
        $display("opA:%h opA_valid:%b opB:%h opB_valid:%b dest_tag:%b in_use:%b ready:%b available:%b", entry[74:43], entry[42], entry[41:10], entry[9], entry[8:3], entry[2], entry[1], entry[0]);
        $display("=======================================================================");
    endtask

    initial begin
        $monitor("Time:%4.0f clock:%b |Inst| instruction:%h valid:%b |CDB| cdb_broadcast:%b cdb_tag:%b cdb_value:%h |Control| mt_retire:%b rob_retire:%b rob_clear:%b rs1_issue:%b rs1_clear:%b || Outputs: opA:%h opB:%h output_tag:%b", 
                $time, clock, if_id_packet.inst, if_id_packet.valid, cdb_valid, cdb_tag, cdb_value, mt_retire_entry, rob_retire_entry, rob_clear, rs1_issue, rs1_clear, opA, opB, output_tag);
        //Reset 
        clock = 1;
        reset = 1; //Pull reset high
        //Test instruction: addi 
        if_id_packet.inst.i.imm = 12'b0000_0000_1010;
        if_id_packet.inst.i.rs1 = 5'b00001;
        if_id_packet.inst.i.funct3 = 3'b000;
        if_id_packet.inst.i.rd = 5'b00010;
        if_id_packet.inst.i.opcode = `RV32_OP_IMM;
        //Other packet parameters
        if_id_packet.PC = 32'h0000_0000;
        if_id_packet.NPC = 32'h0000_0004;
        if_id_packet.valid = 1'b0;
        //CDB
        cdb_valid = 1'b0;
        cdb_tag = 6'b0;
        cdb_value = 32'h0;
        //Control signals
        mt_retire_entry = 1'b0;
        rob_retire_entry = 1'b0;
        rob_clear = 1'b0;
        rs1_issue = 1'b0;
        rs1_clear = 1'b0;

        @(negedge clock)
        //print_MT(mt_tags_debug);
        //print_ROB(rob_debug, rob_pointers_debug);
        //print_RS(rs_debug);
        reset = 0; //Pull reset low
        //Test instruction to load: addi r1 r0 123
        if_id_packet.inst.i.imm = 12'h123;
        if_id_packet.inst.i.rs1 = 5'b00000;
        if_id_packet.inst.i.funct3 = 3'b000;
        if_id_packet.inst.i.rd = 5'b00001;
        if_id_packet.inst.i.opcode = `RV32_OP_IMM;
        //Other packet parameters
        if_id_packet.PC = 32'h0000_0000;
        if_id_packet.NPC = 32'h0000_0004;
        if_id_packet.valid = 1'b1;
        //CDB
        cdb_valid = 1'b0;
        cdb_tag = 6'b0;
        cdb_value = 32'h0;
        //Control signals
        mt_retire_entry = 1'b0;
        rob_retire_entry = 1'b0;
        rob_clear = 1'b0;
        rs1_issue = 1'b0;
        rs1_clear = 1'b0;

        @(negedge clock)
        //print_MT(mt_tags_debug);
        //print_ROB(rob_debug, rob_pointers_debug);
        //print_RS(rs_debug);
        if_id_packet.valid = 1'b0; //Stall dispatch

        cdb_valid = 1'b1; //Simulate CDB broadcast
        cdb_tag = 6'b000001;
        cdb_value = 32'h0000_0123;

        mt_retire_entry = 1'b0;
        rob_retire_entry = 1'b0;
        rob_clear = 1'b0;
        rs1_issue = 1'b0;
        rs1_clear = 1'b0;

        @(negedge clock)
        //print_MT(mt_tags_debug);
        //print_ROB(rob_debug, rob_pointers_debug);
        //print_RS(rs_debug);
        if_id_packet.valid = 1'b0; //Stall dispatch

        cdb_valid = 1'b0; //Stop CDB broadcast
        cdb_tag = 6'b000000;
        cdb_value = 32'h0000_0000;

        mt_retire_entry = 1'b1; //Retire instruction
        rob_retire_entry = 1'b1;
        rob_clear = 1'b0;
        rs1_issue = 1'b0;
        rs1_clear = 1'b1; //Clear RS for next inst - In reality, this would happen when inst has proceeded to execute

        @(negedge clock)
        //print_MT(mt_tags_debug);
        //print_ROB(rob_debug, rob_pointers_debug);
        //print_RS(rs_debug);
        //Test instruction to load: addi r2 r0 ABC
        if_id_packet.inst.i.imm = 12'hABC;
        if_id_packet.inst.i.rs1 = 5'b00000;
        if_id_packet.inst.i.funct3 = 3'b000;
        if_id_packet.inst.i.rd = 5'b00010;
        if_id_packet.inst.i.opcode = `RV32_OP_IMM;
        //Other packet parameters
        if_id_packet.PC = 32'h0000_0000;
        if_id_packet.NPC = 32'h0000_0004;
        if_id_packet.valid = 1'b1;
        //CDB
        cdb_valid = 1'b0;
        cdb_tag = 6'b00000;
        cdb_value = 32'h0000_0000;
        //Control signals
        mt_retire_entry = 1'b0;
        rob_retire_entry = 1'b0;
        rob_clear = 1'b0;
        rs1_issue = 1'b0;
        rs1_clear = 1'b0;

        @(negedge clock)
        //print_MT(mt_tags_debug);
        //print_ROB(rob_debug, rob_pointers_debug);
        //print_RS(rs_debug);
        if_id_packet.valid = 1'b0; //Stall dispatch

        cdb_valid = 1'b1; //Simulate CDB broadcast
        cdb_tag = 6'b000010;
        cdb_value = 32'hFFFF_FABC;

        mt_retire_entry = 1'b0;
        rob_retire_entry = 1'b0;
        rob_clear = 1'b0;
        rs1_issue = 1'b0;
        rs1_clear = 1'b0;

        @(negedge clock)
        //print_MT(mt_tags_debug);
        //print_ROB(rob_debug, rob_pointers_debug);
        //print_RS(rs_debug);
        if_id_packet.valid = 1'b0; //Stall dispatch

        cdb_valid = 1'b0; //Stop CDB broadcast
        cdb_tag = 6'b000000;
        cdb_value = 32'h0000_0000;

        mt_retire_entry = 1'b1; //Retire instruction 
        rob_retire_entry = 1'b1;
        rob_clear = 1'b0;
        rs1_issue = 1'b0;
        rs1_clear = 1'b1; //Clear RS for next inst - In reality, this would happen when inst has proceeded to execute

        @(negedge clock)
        //print_MT(mt_tags_debug);
        //print_ROB(rob_debug, rob_pointers_debug);
        //print_RS(rs_debug);
        //Test instruction to load: add r3 r1 r2 <- previously retired values for r1 and r2 should show up in RS operands!
        if_id_packet.inst.r.funct7 = 7'b0;
        if_id_packet.inst.r.rs1 = 5'b00001;
        if_id_packet.inst.r.rs2 = 5'b00010;
        if_id_packet.inst.r.funct3 = 3'b000;
        if_id_packet.inst.r.rd = 5'b00011;
        if_id_packet.inst.r.opcode = `RV32_OP;
        //Other packet parameters
        if_id_packet.PC = 32'h0000_0000;
        if_id_packet.NPC = 32'h0000_0004;
        if_id_packet.valid = 1'b1;
        //CDB
        cdb_valid = 1'b0;
        cdb_tag = 6'b00000;
        cdb_value = 32'h0000_0000;
        //Control signals
        mt_retire_entry = 1'b0;
        rob_retire_entry = 1'b0;
        rob_clear = 1'b0;
        rs1_issue = 1'b0;
        rs1_clear = 1'b0;

        @(negedge clock)
        //print_MT(mt_tags_debug);
        //print_ROB(rob_debug, rob_pointers_debug);
        //print_RS(rs_debug);
        if_id_packet.valid = 1'b0;
        rs1_clear = 1'b1; //clear RS for next test

        @(negedge clock)
        //print_MT(mt_tags_debug);
        //print_ROB(rob_debug, rob_pointers_debug);
        //print_RS(rs_debug);
        //Test instruction to load: add r4 r2 r3 <- previously retired values for r2 should show up in RS operand, r3 should be a tag!
        if_id_packet.inst.r.funct7 = 7'b0;
        if_id_packet.inst.r.rs1 = 5'b00010;
        if_id_packet.inst.r.rs2 = 5'b00011;
        if_id_packet.inst.r.funct3 = 3'b000;
        if_id_packet.inst.r.rd = 5'b00100;
        if_id_packet.inst.r.opcode = `RV32_OP;
        //Other packet parameters
        if_id_packet.PC = 32'h0000_0000;
        if_id_packet.NPC = 32'h0000_0004;
        if_id_packet.valid = 1'b1;
        //CDB
        cdb_valid = 1'b0;
        cdb_tag = 6'b00000;
        cdb_value = 32'h0000_0000;
        //Control signals
        mt_retire_entry = 1'b0;
        rob_retire_entry = 1'b0;
        rob_clear = 1'b0;
        rs1_issue = 1'b0;
        rs1_clear = 1'b0;

        @(negedge clock)
        //print_MT(mt_tags_debug);
        //print_ROB(rob_debug, rob_pointers_debug);
        //print_RS(rs_debug);

        $finish;

    end

endmodule
