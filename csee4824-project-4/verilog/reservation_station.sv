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
    input [31:0]   rs_npc_in,    // Next PC (from fetch/decode)
    input [31:0]   rs_inst_in,     // Full instruction bits
    input ALU_FUNC rs_alu_func_in, // ALU function input for instruction
    
    input  [5:0] rs_rob_tag,      // Tag input for instruction (ROB tail pointer)
    input [31:0] rs_cdb_in,       // Data from the CDB - FU operation result 
    input  [5:0] rs_cdb_tag,      // Tag from the CDB - for identifying the corresponding register for the result
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


    output       rs_ready_out,            // Ready to issue
    output [31:0] rs_opa_out,             // Out: Operand A
    output [31:0] rs_opb_out,             // Out: Operand B
    output [5:0]  rs_tag_out,             // Out: ROB tag
    output ALU_FUNC rs_alu_func_out,      // Out: ALU func
    output [31:0]  rs_npc_out,            // Out: NPC
    output [31:0]  rs_inst_out,           // Out: instruction bits
    output       rs_avail_out,            // Is this entry available?
    output [74:0] rs_debug
);

//internal storage: 
logic [31:0] OPa, OPb; // Operand A and B
logic [5:0] DestTag; // Destination register tag, and op tags
logic OpaValid, OpbValid; // Operand A and B valid
logic InUse; // RS entry is in use

ALU_FUNC alu_func; //internal ALU track
logic[31:0] NPC; //internal NPC track
logic [31:0] Inst; //internal instruction track

// Load from CDB if needed
assign rs_avail_out = !InUse;
assign rs_ready_out = InUse && OpaValid && OpbValid;
assign rs_opa_out   = (rs_use_enable) ? OPa : 32'b0;
assign rs_opb_out   = (rs_use_enable) ? OPb : 32'b0;
assign rs_tag_out   = (rs_use_enable) ? DestTag : 6'b0;
assign rs_alu_func_out = (rs_use_enable) ? Func : `0;
assign rs_npc_out       = (rs_use_enable) ? NPC : 32'b0;
assign rs_inst_out      = (rs_use_enable) ? Inst : 32'b0;

// Load from CDB if tag matches
wire LoadAFromCDB = (rs_cdb_tag == OPa) && !OpaValid && InUse && rs_cdb_valid;
wire LoadBFromCDB = (rs_cdb_tag == OPb) && !OpbValid && InUse && rs_cdb_valid;

//Debug
assign rs_debug = {OPa, OpaValid, OPb, OpbValid, DestTag, InUse, rs_ready_out, rs_avail_out};

always_ff @(posedge clock) begin
    if (reset) begin
        OPa      <= 32'b0;
        OPb      <= 32'b0;
        DestTag  <= 6'b0;
        OpaValid <= 1'b0;
        OpbValid <= 1'b0;
        InUse    <= 1'b0;
        Func     <= ALU_ADD;
        NPC      <= 32'b0;
        Inst     <= 32'b0;
    end else begin
        // Load new instruction
        if (rs_load_in && !InUse) begin
            OPa      <= rs_opa_in;
            OPb      <= rs_opb_in;
            DestTag  <= rs_rob_tag;
            OpaValid <= rs_opa_valid;
            OpbValid <= rs_opb_valid;
            InUse    <= 1'b1;

            Func     <= rs_alu_func_in;
            NPC      <= rs_npc_in;
            Inst     <= rs_inst_in;

        end else begin
            // CDB broadcasts update operand readiness
            if (LoadAFromCDB) begin
                OPa      <= rs_cdb_in;
                OpaValid <= 1'b1;
            end
            if (LoadBFromCDB) begin
                OPb      <= rs_cdb_in;
                OpbValid <= 1'b1;
            end

            // Free after issue
            if (rs_free_in) begin
                OPa      <= 32'b0;
                OPb      <= 32'b0;
                DestTag  <= 6'b0;
                OpaValid <= 1'b0;
                OpbValid <= 1'b0;
                InUse    <= 1'b0;

                Func     <= ALU_ADD;
                NPC      <= 32'b0;
                Inst     <= 32'b0;
            end
        end
    end
end



endmodule

