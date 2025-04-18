`ifndef SYNTHESIS

//
// This is an automatically generated file from 
// dc_shell Version U-2022.12-SP7 -- Oct 10, 2023
//

// For simulation only. Do not modify.

module reorder_buffer_svsim(
        input reset,
    input clock,

        input DISPATCH_ROB_PACKET rob_dispatch_in,


        input logic rob_to_rs_read1,
    input logic [5-1:0] rob_read_tag1,
    input logic rob_to_rs_read2,
    input logic [5-1:0] rob_read_tag2,

        input CDB_ROB_PACKET rob_cdb_in,

        input retire_entry,
    input rob_clear, 

        output ROB_DISPATCH_PACKET rob_dispatch_out,


        output ROB_RETIRE_PACKET rob_retire_out,


        output [31:0] rob_to_rs_value1,
    output [31:0] rob_to_rs_value2,
    
        output rob_full,

            output logic [11:0] rob_pointers
);

    

  reorder_buffer reorder_buffer( {>>{ reset }}, {>>{ clock }}, 
        {>>{ rob_dispatch_in }}, {>>{ rob_to_rs_read1 }}, 
        {>>{ rob_read_tag1 }}, {>>{ rob_to_rs_read2 }}, {>>{ rob_read_tag2 }}, 
        {>>{ rob_cdb_in }}, {>>{ retire_entry }}, {>>{ rob_clear }}, 
        {>>{ rob_dispatch_out }}, {>>{ rob_retire_out }}, 
        {>>{ rob_to_rs_value1 }}, {>>{ rob_to_rs_value2 }}, {>>{ rob_full }}, 
        {>>{ rob_pointers }} );
endmodule
`endif
