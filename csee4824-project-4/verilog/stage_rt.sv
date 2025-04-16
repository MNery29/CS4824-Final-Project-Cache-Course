/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_rt.sv                                         //
//                                                                     //
//  Description :  instruction retire (RT) stage of the pipeline;      //
//                 determine instruction register destination and      //
//                 write result to regfile, prep for next PC           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

module stage_rt (

input logic clock, 
input logic reset,

//inputs from ROB
input logic [63:0] rob_data, // data from ROB
input logic [4:0] rob_dest, // destination register from ROB
input logic rob_valid, // valid bit from ROB
input logic rob_ready, //ready bit from ROB 

input logic [63:0] rob_mem_addr, // memory address from ROB
input logic rob_mem_valid, // memory valid bit from ROB

// Commit control 

input logic retire_valid, // commit valid bit from ROB
input logic branch_mispredict, // branch mispredict bit from ROB

//outputs
output logic [63:0] retire_value, // data to write to register file
output logic [4:0] retire_dest, // destination register to write to
output logic retire_valid, // valid bit to register file

//memory outputs
output logic [63:0] mem_addr, // memory address to write to
output logic mem_valid // memory valid bit
);

always_ff @(posedge clock) begin
    if (reset || branch_mispredict) begin
        //cleare all retire outputs
        retire_value <= 64'b0;
        retire_dest <= 5'b0;
        retire_valid <= 1'b0;

        //clear all mem outputs
        mem_addr <= 64'b0;
        mem_valid <= 1'b0;
    end else begin
        if (rob_ready && rob_valid) begin
            // if ROB is ready and valid, write data to register file
            retire_value <= rob_data;
            retire_dest <= rob_dest;
            retire_valid <= 1'b1;
            mem_addr <= rob_mem_addr;
            mem_valid <= rob_mem_valid;
        end else begin
            //nothing to retire - set to default
            retire_value <= 64'b0;
            retire_dest <= 5'b0;
            retire_valid <= 1'b0;
        end
    end
end
