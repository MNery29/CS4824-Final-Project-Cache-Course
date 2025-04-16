`ifndef SYNTHESIS

//
// This is an automatically generated file from 
// dc_shell Version U-2022.12-SP7 -- Oct 10, 2023
//

// For simulation only. Do not modify.

module stage_id_svsim (
    input              clock,               input              reset,               input IF_ID_PACKET if_id_reg,

            input cdb_valid,
    input [5:0] cdb_tag,
    input [31:0] cdb_value,

    input mt_retire_entry,

    input rs1_issue,
    input rs1_clear,

    input rob_retire_entry,
    input rob_clear,
    
    output logic [31:0] opA,
    output logic [31:0] opB,
    output logic [5:0] output_tag,

    output logic [45:0] rob_debug [31:0],
    output logic [11:0] rob_pointers_debug,
    output logic [7:0] mt_tags_debug [31:0],
    output logic [74:0] rs_debug,

    output ALU_OPA_SELECT opa_select,
    output ALU_OPB_SELECT opb_select,
    output logic has_dest_reg,
    output logic [4:0] dest_reg_idx

);
    

  stage_id stage_id( {>>{ clock }}, {>>{ reset }}, {>>{ if_id_reg }}, 
        {>>{ cdb_valid }}, {>>{ cdb_tag }}, {>>{ cdb_value }}, 
        {>>{ mt_retire_entry }}, {>>{ rs1_issue }}, {>>{ rs1_clear }}, 
        {>>{ rob_retire_entry }}, {>>{ rob_clear }}, {>>{ opA }}, {>>{ opB }}, 
        {>>{ output_tag }}, {>>{ rob_debug }}, {>>{ rob_pointers_debug }}, 
        {>>{ mt_tags_debug }}, {>>{ rs_debug }}, {>>{ opa_select }}, 
        {>>{ opb_select }}, {>>{ has_dest_reg }}, {>>{ dest_reg_idx }} );
endmodule
`endif
