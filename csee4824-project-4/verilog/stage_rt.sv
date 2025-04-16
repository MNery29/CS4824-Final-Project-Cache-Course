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
input logic reset;

//inputs from ROB
input logic [31:0] rob_data, // data from ROB
input logic rob_valid // valid bit from ROB
input logic rob_ready //ready bit from ROB 


 



);

