/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_is.sv                                         //
//                                                                     //
//  Description :  instruction issue (IS) stage of the pipeline;       //
//                 issue instructions from reservation station,        //
//                 send them to FU units via issue                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`timescale 1ns/100ps
`include "verilog/sys_defs.svh"

module stage_is (
    //standard signals
    input logic clock,
    input logic reset,


    //data from functional units on availabllity
    input logic fu_ready_alu0,
    input logic fu_ready_alu1,
    input logic fu_ready_mult,

    // Reservation Station signals
    
    input logic [`RS_SIZE-1:0]          rs_ready_out, // valid bits for reservation station
    input logic [`RS_SIZE-1:0][31:0]    rs_opa_out [`RS_SIZE],  // changed from input  logic [31:0]  rs_opa_out   [`RS_SIZE],
    input  logic [`RS_SIZE-1:0][31:0]   rs_opb_out [`RS_SIZE],
    input ALU_OPA_SELECT rs_opa_select_out [`RS_SIZE],
    input ALU_OPB_SELECT rs_opb_select_out[`RS_SIZE],
    input  logic [`RS_SIZE-1:0][4:0]    rs_tag_out [`RS_SIZE],
    input ALU_FUNC [`RS_SIZE-1:0]       rs_alu_func_out [`RS_SIZE],
    input logic [31:0]    rs_npc_out [`RS_SIZE],       // Next PC
    input logic [31:0]    rs_pc_out [`RS_SIZE],       // PC
    input INST rs_inst_out [`RS_SIZE],
    // input logic [`RS_SIZE-1:0][31:0]    rs_inst_out,       // Instruction word
    input logic rd_mem [`RS_SIZE], 
    input logic wr_mem [`RS_SIZE], 
    input logic cond_branch [`RS_SIZE],
    input logic uncond_branch [`RS_SIZE],// read/write memory


    //OUTPUTS:




    output IS_EX_PACKET [`RS_SIZE-1:0] is_packets,      // packets for each FU
    output logic [`RS_SIZE-1:0] rs_issue_enable // one-hot enabling. 
    
);

//function to determine if instruction is mult instruction
function logic is_mult(ALU_FUNC funct);
    return (funct inside {MUL_ALU_MUL, MUL_ALU_MULH, MUL_ALU_MULHSU, MUL_ALU_MULHU});
endfunction

// function to build packet, for each IS_EX packet output 
function IS_EX_PACKET build_packet(input int i);
    build_packet.OPA            = rs_opa_out[i];
    build_packet.OPB            = rs_opb_out[i];
    build_packet.opa_select     = rs_opa_select_out[i];
    build_packet.opb_select     = rs_opb_select_out[i];
    build_packet.rob_tag        = rs_tag_out[i];
    build_packet.issue_valid    = 1;
    build_packet.alu_func       = rs_alu_func_out[i];
    build_packet.NPC            = rs_npc_out[i];
    build_packet.PC             = rs_pc_out[i];
    build_packet.inst           = rs_inst_out[i];
    build_packet.RS_tag         = i;
    build_packet.rd_mem         = rd_mem[i];
    build_packet.wr_mem         = wr_mem[i];
    build_packet.cond_branch    = cond_branch[i];
    build_packet.uncond_branch  = uncond_branch[i];

    return build_packet;
endfunction


always_comb begin 
    is_packets = '{default: '0};
    rs_issue_enable = '0;

    for (int i = 0; i < `RS_SIZE; i++) begin
        if (rs_ready_out[i]) begin
            ALU_FUNC funct = rs_alu_func_out[i];
            IS_EX_PACKET packet = build_packet(i);

            if (is_mult(funct) && !issue_valids[2] && fu_ready_mult) begin
                packet.fu_selection = 2'd2;
                is_packets[2] = packet;
                rs_issue_enable[i] = 1;
            end else if (!is_mult(funct)) begin
                if (!issue_valids[0] && fu_ready_alu0) begin
                    packet.fu_selection = 2'd0;
                    is_packets[0] = packet;
                    rs_issue_enable[i] = 1;
                end else if (!issue_valids[1] && fu_ready_alu1) begin
                    packet.fu_selection = 2'd1;
                    is_packets[1] = packet;
                    rs_issue_enable[i] = 1;
                end
            end
        end
    end
end


endmodule