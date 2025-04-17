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
    input logic [`RS_SIZE-1:0] rs_ready_out, // valid bits for reservation station
    input  logic [31:0]         rs_opa_out   [`RS_SIZE], 
    input  logic [31:0]         rs_opb_out   [`RS_SIZE],
    input  logic [5:0]          rs_tag_out   [`RS_SIZE],
    input ALU_FUNC             rs_alu_func_out[`RS_SIZE],       // ALU operation
    input logic [31:0]         rs_npc_out     [`RS_SIZE],       // Next PC
    input logic [31:0]         rs_inst_out    [`RS_SIZE],       // Instruction word

    //Functional unit ready signal 
    input logic fu_ready,

    //OUTPUTS:
    output logic issue_valid, // valid bit for issue
    output IS_EX_PACKET is_packet, // packet to be sent to functional unit

    //Send to Reservation Station which entry was issued, corresponds to rs_use_enable,  
    output logic [`RS_SIZE-1:0] rs_issue_enable
  
);

always_comb begin 
    logic issued;
    issued = 0;
    issue_valid = 0;
    issue_packet = '0;
    rs_issue_enable = '0;

    if (fu_ready) begin
        for (int i = 0; i < `RS_SIZE; i++) begin
            if (!issued && rs_ready_out[i]) begin
                issue_valid               = 1;
                issue_packet.OPA          = rs_opa_out[i];
                issue_packet.OPB          = rs_opb_out[i];
                issue_packet.rob_tag      = rs_tag_out[i];
                issue_packet.issue_valid  = 1;
                issue_packet.alu_func     = rs_alu_func_out[i];
                issue_packet.NPC          = rs_npc_out[i];
                issue_packet.inst         = rs_inst_out[i];
                issue_packet.RS_tag       = i;

                rs_issue_enable[i]        = 1;
                issued = 1; 
            end
        end
    end
end

