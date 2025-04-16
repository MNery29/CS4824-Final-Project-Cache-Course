/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_cp.sv                                         //
//                                                                     //
//  Description :  instruction Complete (CP) stage of the pipeline;    //
//                 Broadcast results from functional units to ROB,     //
//                 mark ROB as completed and set map table to ready    //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"
`include "verilog/rob.svh"

module stage_cp (

input logic clock,
input logic reset,

//input packet from stage_ex: defined in sys_defs.sv
input EX_CP_PACKET ex_cp_packet,

//output to ROB (CDB): defined in sys_defs.sv
output CDB_PACKET cdb_packet_out,
);


always_ff @(posedge clock) begin
        if (reset) begin
            cdb_packet_out.value <= 64'b0;
            cdb_packet_out.tag   <= 5'b0;
            cdb_packet_out.valid <= 1'b0;
        end else begin
            if (ex_cp_packet.done) begin
                cdb_packet_out.value <= ex_cp_packet.value;
                cdb_packet_out.tag   <= ex_cp_packet.rob_tag;
                cdb_packet_out.valid <= 1'b1;
            end else begin
                cdb_packet_out.valid <= 1'b0;
            end
        end
    end
endmodule
