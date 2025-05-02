`timescale 1ns/1ps

`include "verilog/sys_defs.svh"

module tb_stage_is();

    // Clock and reset
    logic clock;
    logic reset;

    // Reservation Station signals
    logic [31:0] rs_opa_out [`RS_SIZE];
    logic [31:0] rs_opb_out [`RS_SIZE];
    logic [4:0]  rs_tag_out [`RS_SIZE];
    ALU_FUNC     rs_alu_func_out [`RS_SIZE];
    INST         rs_inst_out [`RS_SIZE];
    logic [31:0] rs_npc_out [`RS_SIZE];

    logic rd_mem         [`RS_SIZE];
    logic wr_mem         [`RS_SIZE];
    logic cond_branch    [`RS_SIZE];
    logic uncond_branch  [`RS_SIZE];

    // Functional unit ready
    logic fu_ready_alu0, fu_ready_alu1, fu_ready_mult;

    // Outputs
    logic [`RS_SIZE-1:0] rs_issue_enable;
    logic [`RS_SIZE-1:0] rs_ready_out;
    IS_EX_PACKET is_packets[2:0];

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
        .rd_mem(rd_mem),
        .wr_mem(wr_mem),
        .cond_branch(cond_branch),
        .uncond_branch(uncond_branch),
        .fu_ready_alu0(fu_ready_alu0),
        .fu_ready_alu1(fu_ready_alu1),
        .fu_ready_mult(fu_ready_mult),
        .is_packets(is_packets),
        .rs_issue_enable(rs_issue_enable)
    );

    // Clock generation
    always #5 clock = ~clock;

    task display_issued();
        for (int i = 0; i < 3; i++) begin
            if (is_packets[i].issue_valid) begin
                $display("FU[%0d] issued | OPA=0x%h OPB=0x%h tag=%0d ALU_FUNC=%0d NPC=0x%h INST=0x%h", 
                         i, is_packets[i].OPA, is_packets[i].OPB, is_packets[i].rob_tag, 
                         is_packets[i].alu_func, is_packets[i].NPC, is_packets[i].inst);
            end
        end
    endtask

    // Helper: clear all rs_ready signals
    task clear_rs_ready();
        foreach (rs_ready_out[i])
            rs_ready_out[i] = 0;
    endtask

    // Test
    initial begin
        $display("Starting stage_is test...");

        clock = 0;
        reset = 1;
        fu_ready_alu0 = 0;
        fu_ready_alu1 = 0;
        fu_ready_mult = 0;

        foreach (rs_ready_out[i]) begin
            rs_ready_out[i] = 0;
            rs_opa_out[i] = 0;
            rs_opb_out[i] = 0;
            rs_tag_out[i] = 0;
            rs_alu_func_out[i] = ALU_ADD;
            rs_npc_out[i] = 0;
            rs_inst_out[i] = 0;
            rd_mem[i] = 0;
            wr_mem[i] = 0;
            cond_branch[i] = 0;
            uncond_branch[i] = 0;
        end

        // Release reset
        #10 reset = 0;

        // Cycle 1: ALU0 ready, issue from slot 0
        clear_rs_ready();
        rs_ready_out[0] = 1;
        rs_opa_out[0] = 32'h1111;
        rs_opb_out[0] = 32'h2222;
        rs_tag_out[0] = 6'd1;
        rs_alu_func_out[0] = ALU_XOR;
        rs_npc_out[0] = 32'h1000;
        rs_inst_out[0] = 32'hABCD1234;
        fu_ready_alu0 = 1;
        #10 display_issued();

        // Cycle 2: ALU1 ready, issue from slot 1
        clear_rs_ready();
        rs_ready_out[1] = 1;
        rs_opa_out[1] = 32'h3333;
        rs_opb_out[1] = 32'h4444;
        rs_tag_out[1] = 6'd2;
        rs_alu_func_out[1] = ALU_AND;
        rs_npc_out[1] = 32'h2000;
        rs_inst_out[1] = 32'hBEEF5678;
        fu_ready_alu0 = 0;
        fu_ready_alu1 = 1;
        #10 display_issued();

        // Cycle 3: Mult ready, issue from slot 2
        clear_rs_ready();
        rs_ready_out[2] = 1;
        rs_opa_out[2] = 32'h5555;
        rs_opb_out[2] = 32'h6666;
        rs_tag_out[2] = 6'd3;
        rs_alu_func_out[2] = ALU_MUL;
        rs_npc_out[2] = 32'h3000;
        rs_inst_out[2] = 32'hDEADBEEF;
        fu_ready_alu1 = 0;
        fu_ready_mult = 1;
        #10 display_issued();

        $display("Test complete.");
        $finish;
    end

endmodule
