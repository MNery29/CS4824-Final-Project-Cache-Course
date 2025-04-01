/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rs.sv                                               //
//                                                                     //
//  Description :  Reservation Station for P6 architecture, used       //
//                 for register renaming in Tomasulo algorithm for     //
//                 dynamic instruction scheduling                      //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module reservation_station(
    input  [4:0] rs_rob_tag,      // Tag input for instruction (ROB tail pointer)
    input [31:0] rs_cdb_in,       // Data from the CDB - FU operation result 
    input  [4:0] rs_cdb_tag,      // Tag from the CDB - for identifying the corresponding register for the result
    input        rs_cdb_valid,    // CDB data is valid: indicating if CDB is valid and ready for use.
    input [31:0] rs_opa_in,       // Operand A: input from instruction, either register or CDB.
    input [31:0] rs_opb_in,       // Operand B: input from instruction, either register or CDB.
    input        rs_opa_valid,    // Operand A is valid (is immediate data and not tag)
    input        rs_opb_valid,    // Operand B is valid
    input        rs_load_in,      // Load new instruction, indicates whether to intake new instruction
    input        rs_use_enable,   // Issue new instruction, indicates whether to issue new instruction to a functional unit
    input        rs_free_in,      // Free RS entry, indicates whether to free RS entry upon completion
    input        reset,            // Reset signal
    input        clock,            // Clock signal
    output       rs_ready_out,    // RS is ready for execution - indicates if RS is ready to issue instruction to FU
    output [31:0] rs_opa_out,     // Output Operand A
    output [31:0] rs_opb_out,     // Output Operand B
    output [4:0] rs_tag_out, // Output destination tag - where result is written to, used by ROB and Map Table 
    output       rs_avail_out,    // RS is available for a new instruction - indicates if RS is available for new instruction
    output [4:0] rs_debug_status // Debugging info
);

//internal storage: 
logic [31:0] OPa, OPb; // Operand A and B
logic [4:0] DestTag; // Destination register tag, and op tags
logic OpaValid, OpbValid; // Operand A and B valid
logic InUse; // RS entry is in use

// Load from CDB if tags match
logic LoadAFromCDB = (rs_cdb_tag == OPa) && !OpaValid && InUse && rs_cdb_valid;
logic LoadBFromCDB = (rs_cdb_tag == OPb) && !OpbValid && InUse && rs_cdb_valid;

//RS is availabile if not in use: 
assign rs_avail_out = !InUse;

//RS ready if both operands are valid 
assign rs_ready_out = InUse && OpaValid && OpbValid;

//Output operands
//Output values
assign rs_opa_out = (rs_use_enable) ? OPa : 31'b0;
assign rs_opb_out = (rs_use_enable) ? OPb : 31'b0;
assign rs_tag_out = (rs_use_enable) ? DestTag : 5'b0;

//Debugging output to show status bits: 
assign rs_debug_status = {InUse, OpaValid, OpbValid};

always @(posedge clock) begin
    if (reset) begin 
        OPa <= 31'b0;
        OPb <= 31'b0;
        DestTag <= 5'b0;
        OpaValid <= 1'b0;
        OpbValid <= 1'b0;
        InUse <= 1'b0;
    end else begin 
        //Load new instruction into RS
        if (rs_load_in && !InUse) begin
            OPa <= rs_opa_in;
            OPb <= rs_opb_in;
            DestTag <= rs_rob_tag;
            OpaValid <= rs_opa_valid;
            OpbValid <= rs_opb_valid;
            InUse <= 1'b1;
        end else begin 
            //check CDB for operand availability
            if (LoadAFromCDB) begin
                OPa <= rs_cdb_in;
                OpaValid <= 1'b1;
            end
            if (LoadBFromCDB) begin
                OPb <= rs_cdb_in;
                OpbValid <= 1'b1;
            end 
            //Free RS entry when instruction is issued. 
            if (rs_free_in) begin
                InUse <= 1'b0;
            end 
        end
    end
end



endmodule

