`timescale 1ns/1ps

`include "verilog/sys_defs.svh"

module tb_stage_is();

    // Clock and reset
    logic clock;
    logic reset;

    // Reservation Station signals
    logic [`RS_SIZE-1:0] rs_ready_out;
    logic [31:0] rs_opa_out     [`RS_SIZE];
    logic [31:0] rs_opb_out     [`RS_SIZE];
    logic [5:0]  rs_tag_out     [`RS_SIZE];
    ALU_FUNC     rs_alu_func_out[`RS_SIZE];
    logic [31:0] rs_npc_out     [`RS_SIZE];
    logic [31:0] rs_inst_out    [`RS_SIZE];

    // Functional unit ready
    logic fu_ready;

    // Outputs
    logic issue_valid;
    IS_EX_PACKET is_packet;
    logic [`RS_SIZE-1:0] rs_issue_enable;

    // DUT
    stage_is dut (
        .clock(clock),
        .reset(reset),
        .rs_ready_out(rs_ready_out),
        .rs_opa_out(rs_opa_out),
        .rs_opb_out(rs_opb_out),
        .rs_tag_out(rs_tag_out),
        .rs_alu_func_out(rs_alu_func_out),
        .rs_npc_out(rs_npc_out),
        .rs_inst_out(rs_inst_out),
        .fu_ready(fu_ready),
        .issue_valid(issue_valid),
        .is_packet(is_packet),
        .rs_issue_enable(rs_issue_enable)
    );

    // Clock generation
    always #5 clock = ~clock;

    // Test
    initial begin
        $display("Starting stage_is test...");

        // Init
        clock = 0;
        reset = 1;
        fu_ready = 0;

        foreach (rs_ready_out[i]) begin
            rs_ready_out[i] = 0;
            rs_opa_out[i] = 0;
            rs_opb_out[i] = 0;
            rs_tag_out[i] = 0;
            rs_alu_func_out[i] = ALU_ADD;
            rs_npc_out[i] = 0;
            rs_inst_out[i] = 0;
        end

        // Release reset
        #10 reset = 0;

        // Cycle 1: Issue not ready
        #10;

        // Cycle 2: FU ready, one RS entry ready
        rs_ready_out[2] = 1;
        rs_opa_out[2] = 32'hA;
        rs_opb_out[2] = 32'hB;
        rs_tag_out[2] = 6'd12;
        rs_alu_func_out[2] = ALU_SUB;
        rs_npc_out[2] = 32'h1000;
        rs_inst_out[2] = 32'h12345678;
        fu_ready = 1;
        #10;

        $display("Time %0t | Issued? %b | OPA=0x%h OPB=0x%h tag=%0d ALU_FUNC=%0d NPC=0x%h INST=0x%h", 
                  $time, issue_valid, is_packet.OPA, is_packet.OPB, is_packet.rob_tag, is_packet.alu_func, is_packet.NPC, is_packet.inst);

        // Cycle 3: No FU ready
        fu_ready = 0;
        #10;

        // Cycle 4: Another entry ready
        rs_ready_out[1] = 1;
        rs_opa_out[1] = 32'hC;
        rs_opb_out[1] = 32'hD;
        rs_tag_out[1] = 6'd5;
        rs_alu_func_out[1] = ALU_AND;
        rs_npc_out[1] = 32'h2000;
        rs_inst_out[1] = 32'hAABBCCDD;
        fu_ready = 1;
        #10;

        $display("Time %0t | Issued? %b | OPA=0x%h OPB=0x%h tag=%0d ALU_FUNC=%0d NPC=0x%h INST=0x%h", 
                  $time, issue_valid, is_packet.OPA, is_packet.OPB, is_packet.rob_tag, is_packet.alu_func, is_packet.NPC, is_packet.inst);

        $display("Test complete.");
        $finish;
    end

endmodule
