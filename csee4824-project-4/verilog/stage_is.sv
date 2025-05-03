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
    // Reservation Station signals
    
    input logic [`RS_SIZE-1:0]          rs_ready_out, // valid bits for reservation station
    input logic [`RS_SIZE-1:0][31:0]    rs_opa_out,  // changed from input  logic [31:0]  rs_opa_out   [`RS_SIZE],
    input  logic [`RS_SIZE-1:0][31:0]   rs_opb_out,
    input ALU_OPA_SELECT rs_opa_select_out,
    input ALU_OPB_SELECT rs_opb_select_out ,
    input  logic [`RS_SIZE-1:0][4:0]    rs_tag_out,
    input ALU_FUNC [`RS_SIZE-1:0]       rs_alu_func_out,
    input logic [31:0]    rs_npc_out,       // Next PC
    input logic [31:0]    rs_pc_out,       // PC
    input INST rs_inst_out,
    // input logic [`RS_SIZE-1:0][31:0]    rs_inst_out,       // Instruction word
    input logic rd_mem, wr_mem, cond_branch,uncond_branch,// read/write memory

    //Functional unit ready signal 
    input logic fu_ready,

    //OUTPUTS:
    output logic issue_valid, // valid bit for issue
    output IS_EX_PACKET is_packet, // packet to be sent to functional unit

    //Send to Reservation Station which entry was issued, corresponds to rs_use_enable,  
    output logic [`RS_SIZE-1:0] rs_issue_enable
  
);
logic issued;

always_comb begin 
    issued = 0;
    issue_valid = 0;
    is_packet = '0;
    rs_issue_enable = '0;

    if (fu_ready) begin
        for (int i = 0; i < `RS_SIZE; i++) begin
            if (!issued && rs_ready_out[i]) begin
                issue_valid               = 1;
                is_packet.OPA          = rs_opa_out[i];
                is_packet.OPB          = rs_opb_out[i];
                is_packet.opa_select    = rs_opa_select_out;
                is_packet.opb_select    = rs_opb_select_out;
                is_packet.rob_tag      = rs_tag_out[i][4:0]; // fix to only send the last 4 bits of tag, keep msb internal
                is_packet.issue_valid  = 1;
                is_packet.alu_func     = rs_alu_func_out[i];
                is_packet.NPC          = rs_npc_out;
                is_packet.PC = rs_pc_out;
                is_packet.inst = rs_inst_out;
                // is_packet.inst         = rs_inst_out[i];
                is_packet.RS_tag       = i;
                is_packet.rd_mem       = rd_mem;
                is_packet.wr_mem       = wr_mem;
                is_packet.cond_branch   = cond_branch;
                is_packet.uncond_branch = uncond_branch;

                rs_issue_enable[i]        = 1;
                issued = 1; 
            end
        end
    end
end

endmodule