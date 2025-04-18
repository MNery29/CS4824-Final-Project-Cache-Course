`ifndef SYNTHESIS

//
// This is an automatically generated file from 
// dc_shell Version U-2022.12-SP7 -- Oct 10, 2023
//

// For simulation only. Do not modify.

module dcache_svsim
#(
    TAG_BITS = 26,
    INDEX_BITS = 6,
    OFFSET_BITS = 0
)
(
    input logic clk,
    input logic reset,

    input logic [32-1:0] proc2Dcache_addr,
    input logic [1:0] proc2Dcache_command, 
    input logic [3:0]  mem2dcache_response,     input logic [63:0] mem2dcache_data,         input logic [3:0]  mem2dcache_tag,       
    output logic [32-1:0] dcache2mem_addr,
    output logic [63:0]      dcache2mem_data,     output logic [1:0]       dcache2mem_command,     output MEM_SIZE    dcache2mem_size,

    output logic [63:0] hit_data,     output logic hit,     output logic [3:0] data_tag,
    output logic [3:0] data_response,

    output logic next_state,     output logic state,     output logic [3:0] number_of_waits,     output logic [3:0] next_number_of_waits     
);
    

  dcache dcache( {>>{ clk }}, {>>{ reset }}, {>>{ proc2Dcache_addr }}, 
        {>>{ proc2Dcache_command }}, {>>{ mem2dcache_response }}, 
        {>>{ mem2dcache_data }}, {>>{ mem2dcache_tag }}, 
        {>>{ dcache2mem_addr }}, {>>{ dcache2mem_data }}, 
        {>>{ dcache2mem_command }}, {>>{ dcache2mem_size }}, {>>{ hit_data }}, 
        {>>{ hit }}, {>>{ data_tag }}, {>>{ data_response }}, 
        {>>{ next_state }}, {>>{ state }}, {>>{ number_of_waits }}, 
        {>>{ next_number_of_waits }} );
endmodule
`endif
