`ifndef SYNTHESIS

//
// This is an automatically generated file from 
// dc_shell Version U-2022.12-SP7 -- Oct 10, 2023
//

// For simulation only. Do not modify.

module map_table_svsim(
    input reset,
    input clock,
    input [4:0] rs1_addr,     input [4:0] rs2_addr,     input [4:0] r_dest, 
    input [5:0] tag_in,     input load_entry,
    input [5:0] cdb_tag_in,     input read_cdb,     input [4:0] retire_addr,     input [5:0] retire_tag,
    input retire_entry, 
    output logic [6:0] rs1_tag,     output logic [6:0] rs2_tag, 
    output logic [4:0] regfile_rs1_addr,     output logic [4:0] regfile_rs2_addr,
        output logic [7:0] tags_debug[31:0]
);

    

  map_table map_table( {>>{ reset }}, {>>{ clock }}, {>>{ rs1_addr }}, 
        {>>{ rs2_addr }}, {>>{ r_dest }}, {>>{ tag_in }}, {>>{ load_entry }}, 
        {>>{ cdb_tag_in }}, {>>{ read_cdb }}, {>>{ retire_addr }}, 
        {>>{ retire_tag }}, {>>{ retire_entry }}, {>>{ rs1_tag }}, 
        {>>{ rs2_tag }}, {>>{ regfile_rs1_addr }}, {>>{ regfile_rs2_addr }}, 
        {>>{ tags_debug }} );
endmodule
`endif
