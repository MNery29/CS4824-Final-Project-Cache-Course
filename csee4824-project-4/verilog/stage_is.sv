/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_is.sv                                         //
//                                                                     //
//  Description :  instruction issue (IS) stage of the pipeline;       //
//                 issue instructions from reservation station,        //
//                 send them to FU units via issue                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module stage_is (
    //standard signals
    input logic clock,
    input logic reset,

    // Functional Unit readiness signals
    input logic fu_ready_alu0,
    input logic fu_ready_alu1,
    input logic fu_ready_mult,

    // Reservation Station inputs
    input logic [`RS_SIZE-1:0] rs_ready_out,
    input logic [31:0] rs_opa_out [`RS_SIZE],
    input logic [31:0] rs_opb_out [`RS_SIZE],
    input INST rs_inst_out [`RS_SIZE],
    input ALU_OPA_SELECT rs_opa_select_out [`RS_SIZE],
    input ALU_OPB_SELECT rs_opb_select_out [`RS_SIZE],
    input logic [4:0] rs_tag_out [`RS_SIZE],
    input ALU_FUNC rs_alu_func_out [`RS_SIZE],
    input logic [31:0] rs_npc_out [`RS_SIZE],
    input logic [31:0] rs_pc_out [`RS_SIZE],
    input logic rd_mem [`RS_SIZE],
    input logic wr_mem [`RS_SIZE],
    input logic cond_branch [`RS_SIZE],
    input logic uncond_branch [`RS_SIZE],

    // Outputs
    output IS_EX_PACKET is_packets [2:0],
    output logic [`RS_SIZE-1:0] rs_issue_enable
);

// Local flags to track FU usage
logic issued_alu0;
logic issued_alu1;
logic issued_mult;

// Function to determine if ALU_FUNC is a multiplier op
function logic is_mult(ALU_FUNC funct);
    return (funct inside {ALU_MUL, ALU_MULH, ALU_MULHSU, ALU_MULHU});
endfunction

// Issue logic moved into task to avoid hierarchical reference issues
task automatic issue_instructions();
    for (int i = 0; i < `RS_SIZE; i++) begin
        if (rs_ready_out[i]) begin
            // Copy values from indexed arrays into local temps
            logic [31:0] opa      = rs_opa_out[i];
            logic [31:0] opb      = rs_opb_out[i];
            logic [4:0]  tag      = rs_tag_out[i];
            ALU_FUNC     funct    = rs_alu_func_out[i];
            INST         inst     = rs_inst_out[i];
            ALU_OPA_SELECT opa_sel = rs_opa_select_out[i];
            ALU_OPB_SELECT opb_sel = rs_opb_select_out[i];
            logic [31:0] npc      = rs_npc_out[i];
            logic [31:0] pc       = rs_pc_out[i];
            logic        rd       = rd_mem[i];
            logic        wr       = wr_mem[i];
            logic        cb       = cond_branch[i];
            logic        ucb      = uncond_branch[i];

            // Build the issue packet
            IS_EX_PACKET packet;
            packet.OPA            = opa;
            packet.OPB            = opb;
            packet.opa_select     = opa_sel;
            packet.opb_select     = opb_sel;
            packet.rob_tag        = tag;
            packet.issue_valid    = 1;
            packet.alu_func       = funct;
            packet.NPC            = npc;
            packet.PC             = pc;
            packet.inst           = inst;
            packet.RS_tag         = i;
            packet.rd_mem         = rd;
            packet.wr_mem         = wr;
            packet.cond_branch    = cb;
            packet.uncond_branch  = ucb;

            // Dispatch to available FU
            if (is_mult(funct) && fu_ready_mult && !issued_mult) begin
                packet.fu_selection = 2;
                is_packets[2] = packet;
                rs_issue_enable[i] = 1;
                issued_mult = 1;
            end else if (!is_mult(funct)) begin
                if (fu_ready_alu0 && !issued_alu0) begin
                    packet.fu_selection = 0;
                    is_packets[0] = packet;
                    rs_issue_enable[i] = 1;
                    issued_alu0 = 1;
                end else if (fu_ready_alu1 && !issued_alu1) begin
                    packet.fu_selection = 1;
                    is_packets[1] = packet;
                    rs_issue_enable[i] = 1;
                    issued_alu1 = 1;
                end
            end
        end
    end
endtask

// Main comb logic
always_comb begin
    is_packets = '{default: '0};
    rs_issue_enable = {`RS_SIZE{1'b0}};
    issued_alu0 = 0;
    issued_alu1 = 0;
    issued_mult = 0;

    for (int i = 0; i < `RS_SIZE; i++) begin
        if (rs_ready_out[i]) begin
            IS_EX_PACKET packet;

            packet.OPA            = rs_opa_out[i];
            packet.OPB            = rs_opb_out[i];
            packet.opa_select     = rs_opa_select_out[i];
            packet.opb_select     = rs_opb_select_out[i];
            packet.rob_tag        = rs_tag_out[i];
            packet.issue_valid    = 1;
            packet.alu_func       = rs_alu_func_out[i];
            packet.NPC            = rs_npc_out[i];
            packet.PC             = rs_pc_out[i];
            packet.inst           = rs_inst_out[i];
            packet.RS_tag         = i;
            packet.rd_mem         = rd_mem[i];
            packet.wr_mem         = wr_mem[i];
            packet.cond_branch    = cond_branch[i];
            packet.uncond_branch  = uncond_branch[i];

            if (is_mult(rs_alu_func_out[i]) && fu_ready_mult && !issued_mult) begin
                packet.fu_selection = 2;
                is_packets[2] = packet;
                rs_issue_enable[i] = 1;
                issued_mult = 1;
            end else if (!is_mult(rs_alu_func_out[i])) begin
                if (fu_ready_alu0 && !issued_alu0) begin
                    packet.fu_selection = 0;
                    is_packets[0] = packet;
                    rs_issue_enable[i] = 1;
                    issued_alu0 = 1;
                end else if (fu_ready_alu1 && !issued_alu1) begin
                    packet.fu_selection = 1;
                    is_packets[1] = packet;
                    rs_issue_enable[i] = 1;
                    issued_alu1 = 1;
                end
            end
        end
    end
end


endmodule
