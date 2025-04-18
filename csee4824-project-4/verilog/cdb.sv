/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cdb.sv                                              //
//                                                                     //
//  Description :  Common Data Bus, used to store results of F.Units   //
//                 and pass them along to ROB                          //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module cdb (
    //fundamental signals
    
    input logic clock,
    input logic reset,

    //input signals 
    //input of FU signals from complete stage
    input CDB_PACKET cdb_in,
    //output signals

    output logic [31:0] cdb_data, // data to be sent to broadcast
    output logic [4:0] cdb_tag, // tag to be sent to broadcast
    output logic cdb_valid // is the data valid?

);

always_ff @(posedge clock or posedge reset) begin
    if (reset) begin
        cdb_data <= 0;
        cdb_tag <= 0;
        cdb_valid <= 0;
    end else begin
        if (cdb_in.valid) begin //if fu_data is valid, broadcast
            cdb_data <= cdb_in.value;
            cdb_tag <= cdb_in.tag;
            cdb_valid <= cdb_in.valid;
        end else begin
            cdb_valid <= 1'b0; //no broadcasts if no asserted valid data
        end
    end
end

endmodule
