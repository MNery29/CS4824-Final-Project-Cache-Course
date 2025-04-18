`ifndef SYNTHESIS

//
// This is an automatically generated file from 
// dc_shell Version U-2022.12-SP7 -- Oct 10, 2023
//

// For simulation only. Do not modify.

module cdb_svsim (
        
    input logic clock,
    input logic reset,

            input CDB_PACKET cdb_in,
    
    output logic [31:0] cdb_data,     output logic [4:0] cdb_tag,     output logic cdb_valid 
);



  cdb cdb( {>>{ clock }}, {>>{ reset }}, {>>{ cdb_in }}, {>>{ cdb_data }}, 
        {>>{ cdb_tag }}, {>>{ cdb_valid }} );
endmodule
`endif
