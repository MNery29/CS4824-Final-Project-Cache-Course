`ifndef SYNTHESIS

//
// This is an automatically generated file from 
// dc_shell Version U-2022.12-SP7 -- Oct 10, 2023
//

// For simulation only. Do not modify.

module reorder_buffer_svsim(
        input reset,
    input clock,

            input [4:0] dispatch_dest_reg,
    input [6:0] dispatch_opcode,
    input load_entry,
        input rob_to_rs_read1,
    input [5:0] rob_read_tag1,
    input rob_to_rs_read2,
    input [5:0] rob_read_tag2,

        input [5:0] cdb_tag,     input [31:0] cdb_value,
    input cdb_valid,

        input retire_entry,
    input rob_clear,

        output logic [4:0] reg_dest,
    output logic [31:0] reg_value,
    output logic reg_valid,

        output [5:0] rob_tag_out,
    output [5:0] rob_retire_tag_out,
    output [31:0] rob_to_rs_value1,
    output [31:0] rob_to_rs_value2,
    output logic rob_out_valid,
    
        output logic [31:0] mem_addr,
    output logic mem_valid,
        output rob_full,
        output logic [45:0] rob_debug [31:0],
    output logic [11:0] rob_pointers
);

    

  reorder_buffer reorder_buffer( {>>{ reset }}, {>>{ clock }}, 
        {>>{ dispatch_dest_reg }}, {>>{ dispatch_opcode }}, {>>{ load_entry }}, 
        {>>{ rob_to_rs_read1 }}, {>>{ rob_read_tag1 }}, 
        {>>{ rob_to_rs_read2 }}, {>>{ rob_read_tag2 }}, {>>{ cdb_tag }}, 
        {>>{ cdb_value }}, {>>{ cdb_valid }}, {>>{ retire_entry }}, 
        {>>{ rob_clear }}, {>>{ reg_dest }}, {>>{ reg_value }}, 
        {>>{ reg_valid }}, {>>{ rob_tag_out }}, {>>{ rob_retire_tag_out }}, 
        {>>{ rob_to_rs_value1 }}, {>>{ rob_to_rs_value2 }}, 
        {>>{ rob_out_valid }}, {>>{ mem_addr }}, {>>{ mem_valid }}, 
        {>>{ rob_full }}, {>>{ rob_debug }}, {>>{ rob_pointers }} );
endmodule
`endif
