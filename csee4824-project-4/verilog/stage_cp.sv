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
`include "verilog/reorder_buffer.sv"

module stage_cp (

input logic clock,
input logic reset,

//input packet from stage_ex: defined in sys_defs.sv
input EX_CP_PACKET ex_cp_packet, //diff func units but same info!
input EX_CP_PACKET lsq_cp_packet, 

//output to ROB (CDB): defined in sys_defs.sv
output CDB_PACKET cdb_packet_out,
output ex_rejected // passed to execute and issue stage, says FU is busy!
// lsq WILL ALWAYS have priority, so it will NEVER by rejected
);

assign ex_rejected = ex_cp_packet.valid && !lsq_cp_packet.valid;


always_ff @(posedge clock) begin
        if (reset) begin
            cdb_packet_out.value <= 64'b0;
            cdb_packet_out.tag   <= 5'b0;
            cdb_packet_out.valid <= 1'b0;
        end else begin
            if (lsq_cp_packet.done && lsq_cp_packet.valid) begin

                cdb_packet_out.value <= lsq_cp_packet.value;
                cdb_packet_out.tag   <= lsq_cp_packet.rob_tag;
                cdb_packet_out.valid <= 1'b1;
            end else if (ex_cp_packet.done && ex_cp_packet.valid)begin
                cdb_packet_out.value <= ex_cp_packet.value;
                cdb_packet_out.tag   <= ex_cp_packet.rob_tag;
                cdb_packet_out.valid <= 1'b1;
            end else begin
                cdb_packet_out.value <= 64'b0; 
                cdb_packet_out.tag   <= 5'b0;
                cdb_packet_out.valid <= 1'b0;
            end
        end
    end
endmodule
