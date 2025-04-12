/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  cdb.sv                                              //
//                                                                     //
//  Description :  Common Data Bus, used to store results of F.Units   //
//                 and pass them along to ROB                          //
//                                                                     //
//                                                                     //
/////////////////////////////////////////////////////////////////////////



module cdb (
    //fundamental signals
    
    input logic clock,
    input logic reset,

    //input signals 

    input logic [4:0] fu_tag, // functional unit tag to be broadcast
    input logic [63:0] fu_result, // data from functional unit
    input logic fu_valid, // is the data valid?

    //output signals

    output logic [63:0] cdb_data, // data to be sent to broadcast
    output logic [4:0] cdb_tag, // tag to be sent to broadcast
    output logic cdb_valid, // is the data valid?

);

always_ff @(posedge clock or posedge reset) begin
    if (reset) begin
        cdb_data <= 0;
        cdb_tag <= 0;
        cdb_valid <= 0;
    end else begin
        if (fu_valid) begin //if fu_data is valid, broadcast
            cdb_data <= fu_result;
            cdb_tag <= fu_tag;
            cdb_valid <= fu_valid
        end else begin
            cdb_valid <= 1'b0; //no broadcasts if no asserted valid data
        end
    end
end