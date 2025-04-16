`ifndef SYNTHESIS

//
// This is an automatically generated file from 
// dc_shell Version U-2022.12-SP7 -- Oct 10, 2023
//

// For simulation only. Do not modify.

module reservation_station_svsim(
    input  [5:0] rs_rob_tag,          input [31:0] rs_cdb_in,           input  [5:0] rs_cdb_tag,          input        rs_cdb_valid,        input [31:0] rs_opa_in,           input [31:0] rs_opb_in,           input        rs_opa_valid,        input        rs_opb_valid,        input        rs_load_in,          input        rs_use_enable,       input        rs_free_in,          input        reset,                input        clock,                output       rs_ready_out,        output [31:0] rs_opa_out,         output [31:0] rs_opb_out,         output [5:0] rs_tag_out,     output       rs_avail_out,        output [74:0] rs_debug
);



  reservation_station reservation_station( {>>{ rs_rob_tag }}, 
        {>>{ rs_cdb_in }}, {>>{ rs_cdb_tag }}, {>>{ rs_cdb_valid }}, 
        {>>{ rs_opa_in }}, {>>{ rs_opb_in }}, {>>{ rs_opa_valid }}, 
        {>>{ rs_opb_valid }}, {>>{ rs_load_in }}, {>>{ rs_use_enable }}, 
        {>>{ rs_free_in }}, {>>{ reset }}, {>>{ clock }}, {>>{ rs_ready_out }}, 
        {>>{ rs_opa_out }}, {>>{ rs_opb_out }}, {>>{ rs_tag_out }}, 
        {>>{ rs_avail_out }}, {>>{ rs_debug }} );
endmodule
`endif
