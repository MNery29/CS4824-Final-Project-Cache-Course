`ifndef SYNTHESIS

//
// This is an automatically generated file from 
// dc_shell Version U-2022.12-SP7 -- Oct 10, 2023
//

// For simulation only. Do not modify.

module stage_if_svsim (
    input             clock,              input             reset,              input             if_valid,           input             take_branch,        input [32-1:0] branch_target,      input [63:0]      Icache_data_out,     input             Icache_valid_out,
    output IF_ID_PACKET      if_packet,
    output logic [32-1:0] proc2Icache_addr,     output logic stall_if );

    

  stage_if stage_if( {>>{ clock }}, {>>{ reset }}, {>>{ if_valid }}, 
        {>>{ take_branch }}, {>>{ branch_target }}, {>>{ Icache_data_out }}, 
        {>>{ Icache_valid_out }}, {>>{ if_packet }}, {>>{ proc2Icache_addr }}, 
        {>>{ stall_if }} );
endmodule
`endif
