`ifndef SYNTHESIS

//
// This is an automatically generated file from 
// dc_shell Version U-2022.12-SP7 -- Oct 10, 2023
//

// For simulation only. Do not modify.

module stage_is_svsim (
        input logic clock,
    input logic reset,

        
    input logic [4-1:0]          rs_ready_out,     input logic [4-1:0][31:0]    rs_opa_out,      input  logic [4-1:0][31:0]   rs_opb_out,
    input  logic [4-1:0][5:0]    rs_tag_out,
    input ALU_FUNC [4-1:0]       rs_alu_func_out,
    input logic [4-1:0][31:0]    rs_npc_out,           input logic [4-1:0][31:0]    rs_inst_out,           input logic rd_mem, wr_mem, 
        input logic fu_ready,

        output logic issue_valid,     output IS_EX_PACKET is_packet, 
        output logic [4-1:0] rs_issue_enable
  
);


  stage_is stage_is( {>>{ clock }}, {>>{ reset }}, {>>{ rs_ready_out }}, 
        {>>{ rs_opa_out }}, {>>{ rs_opb_out }}, {>>{ rs_tag_out }}, 
        {>>{ rs_alu_func_out }}, {>>{ rs_npc_out }}, {>>{ rs_inst_out }}, 
        {>>{ rd_mem }}, {>>{ wr_mem }}, {>>{ fu_ready }}, {>>{ issue_valid }}, 
        {>>{ is_packet }}, {>>{ rs_issue_enable }} );
endmodule
`endif
