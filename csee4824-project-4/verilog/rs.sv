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

module rs(
    input  [4:0] rs1_dest_in,      // Destination register tag from instruction, result location
    input [63:0] rs1_cdb_in,       // Data from the CDB - FU operation result 
    input  [4:0] rs1_cdb_tag,      // Tag from the CDB - for identifying the corresponding register for the result
    input        rs1_cdb_valid,    // CDB data is valid: indicating if CDB is valid and ready for use.
    input [63:0] rs1_opa_in,       // Operand A: input from instruction, either register or CDB.
    input [63:0] rs1_opb_in,       // Operand B: input from instruction, either register or CDB.
    input  [4:0] rs1_opa_tag,      // Operand A tag: source register tag
    input  [4:0] rs1_opb_tag,      // Operand B tag: source register tag
    input        rs1_opa_valid,    // Operand A is valid
    input        rs1_opb_valid,    // Operand B is valid
    input        rs1_load_in,      // Load new instruction, indicates whether to intake new instruction
    input        rs1_use_enable,   // Issue new instruction, indicates whether to issue new instruction to a functional unit
    input        rs1_free_in,      // Free RS entry, indicates whether to free RS entry upon completion
    input        reset,            // Reset signal
    input        clk,            // Clock signal
    input  [1:0] rs1_fu_select,    // Functional unit selection - Selects functional unit for execution (ALU, FPU)
    output       rs1_ready_out,    // RS is ready for execution - indicates if RS is ready to issue instruction to FU
    output [63:0] rs1_opa_out,     // Output Operand A 
    output [63:0] rs1_opb_out,     // Output Operand B
    output [4:0] rs1_dest_tag_out, // Output destination tag - where result is written to, used by ROB and Map Table 
    output       rs1_avail_out,    // RS is available for a new instruction - indicates if RS is available for new instruction
    output [4:0] rs1_debug_status, // Debugging info
    output [1:0] rs1_fu_out        // Functional unit selection output - indicates which functional unit to use for execution
);

//internal storage: 
reg [63:0] OPa, OPb; // Operand A and B
reg [4:0] OPaTag, OpbTag, DestTag; // Destination register tag, and op tags
reg OpaValid, OpbValid; // Operand A and B valid
reg InUse; // RS entry is in use

// Load from CDB if tags match
wire LoadAFromCDB = (rs1_cdb_tag == OPaTag) && !OPaValid && InUse && rs1_cdb_valid;
wire LoadBFromCDB = (rs1_cdb_tag == OPbTag) && !OPbValid && InUse && rs1_cdb_valid;

//RS is availabile if not in use: 
assign rs1_avail_out = !InUse;

//RS ready if both operands are valid 
assign rs1_ready_out = InUse && OpaValid && OpbValid;

//Output operands
// Output values
assign rs1_opa_out = (rs1_use_enable) ? OPa : 64'b0;
assign rs1_opb_out = (rs1_use_enable) ? OPb : 64'b0;
assign rs1_dest_tag_out = (rs1_use_enable) ? DestTag : 5'b0;

//Debugging output to show status bits: 
assign rs1_debug_status = {InUse, OpaValid, OpbValid, rs1_cdb_valid, rs1_load_in};


// Output the selected functional unit (rs1_fu_select) to the functional unit (possibly reduntant?)
assign rs1_fu_out = rs1_fu_select;

always @(posedge clock) begin
    if (reset) begin 
        OPa <= 64'b0;
        OPb <= 64'b0;
        OPaTag <= 5'b0;
        OpbTag <= 5'b0;
        DestTag <= 5'b0;
        OpaValid <= 1'b0;
        OpbValid <= 1'b0;
        InUse <= 1'b0;
    end else begin 
        //Load new instruction into RS
        if (rs1_load_in && !InUse) begin
            OPa <= rs1_opa_in;
            OPb <= rs1_opb_in;
            OPaTag <= rs1_opa_tag;
            OpbTag <= rs1_opb_tag;
            DestTag <= rs1_dest_in;
            OpaValid <= rs1_opa_valid;
            OpbValid <= rs1_opb_valid;
            InUse <= 1'b1;
        end else begin 
            //check CDB for operand availability
            if (LoadAFromCDB) begin
                OPa <= rs1_cdb_in;
                OpaValid <= 1'b1;
            end
            if (LoadBFromCDB) begin
                OPb <= rs1_cdb_in;
                OpbValid <= 1'b1;
            end 
            //Free RS entry when instruction is issued. 
            if (rs1_free_in) begin
                InUse <= 1'b0;
            end 
        end
    end
end



endmodule

