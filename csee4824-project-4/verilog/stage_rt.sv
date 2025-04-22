/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_rt.sv                                         //
//                                                                     //
//  Description :  instruction retire (RT) stage of the pipeline;      //
//                 determine instruction register destination and      //
//                 write result to regfile, prep for next PC           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module stage_rt (

input logic clock, 
input logic reset,

//inputs from ROB
input ROB_RETIRE_PACKET rob_retire_packet, // packet from ROB

input logic rob_ready, // ready bit from ROB
input logic rob_valid, // valid bit from ROB

// Commit control 
input logic branch_mispredict, // branch mispredict bit from ROB

//outputs
output logic [31:0] retire_value, // data to write to register file
output logic [4:0] retire_dest, // destination register to write to
output logic retire_valid_out, // valid bit to register file

//memory outputs
// output logic [63:0] mem_addr, // memory address to write to
output logic [4:0] mem_tag,
output logic mem_valid // memory valid bit


);

always_ff @(posedge clock) begin
    if (reset || branch_mispredict) begin
        //cleare all retire outputs
        retire_value <= 64'b0;
        retire_dest <= 5'b0;
        retire_valid_out <= 1'b0;

        //clear all mem outputs
        // mem_addr <= 64'b0;
        mem_tag <= 4'b0;
        mem_valid <= 1'b0;
    end else begin
        if (rob_ready && rob_valid) begin
            // retiring an instruction: valid entry from ROB
            retire_value <= rob_retire_packet.value;
            retire_dest  <= rob_retire_packet.dest_reg;
            retire_valid_out <= 1'b1;
            // mem_addr     <= rob_retire_packet.mem_addr;
            mem_tag      <= rob_retire_packet.tag[4:0];
            mem_valid    <= rob_retire_packet.mem_valid;
        end else begin
            //nothing to retire - set to default
            retire_value <= 64'b0;
            retire_dest <= 5'b0;
            retire_valid_out <= 1'b0;
        end
    end
end
endmodule