`ifndef SYNTHESIS

//
// This is an automatically generated file from 
// dc_shell Version U-2022.12-SP7 -- Oct 10, 2023
//

// For simulation only. Do not modify.

module cdb_svsim (
        
    input logic clock,
    input logic reset,

    
    input logic [4:0] fu_tag,     input logic [63:0] fu_result,     input logic fu_valid, 
    
    output logic [63:0] cdb_data,     output logic [4:0] cdb_tag,     output logic cdb_valid 
);



  cdb cdb( {>>{ clock }}, {>>{ reset }}, {>>{ fu_tag }}, {>>{ fu_result }}, 
        {>>{ fu_valid }}, {>>{ cdb_data }}, {>>{ cdb_tag }}, {>>{ cdb_valid }}
 );
endmodule
`endif
