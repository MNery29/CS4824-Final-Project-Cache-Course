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

//NOTE: CURRENTLY IS A SINGULAR ENTRY. (Yes. I know. Sad.)
//WE DID BUILD A MULTIPLE ENTRY VERSION BUT DIDN"T HAVE TIME TO INTEGRATE 
//WOULD BE ABLE TO INTEGRATE WITHIN A WEEK EXTRA, MIGHT ADD IN FINAL REPORT 

module reservation_station(
    input [31:0]   rs_npc_in,    // Next PC (from fetch/decode)
    input [31:0] rs_pc_in,
    input ALU_FUNC rs_alu_func_in, // ALU function input for instruction
    input rd_mem, wr_mem, cond_branch,uncond_branch, // read/write memory

    input INST rs_inst,
    
    input [`ROB_TAG_BITS-1:0] rs_rob_tag,      // Tag input for instruction (ROB tail pointer)
    input [31:0] rs_cdb_in,       // Data from the CDB - FU operation result 
    input [`ROB_TAG_BITS-1:0] rs_cdb_tag,      // Tag from the CDB - for identifying the corresponding register for the result
    input        rs_cdb_valid,    // CDB data is valid: indicating if CDB is valid and ready for use.
    
    input [31:0] rs_opa_in,       // Operand A: input from instruction, either register or CDB.
    input [31:0] rs_opb_in,       // Operand B: input from instruction, either register or CDB.
    input ALU_OPA_SELECT rs_opa_select, // Operand A select: indicates if the operand is a register or immediate data
    input ALU_OPB_SELECT rs_opb_select, // Operand B select: indicates if the operand is a register or immediate data
    input        rs_opa_valid,    // Operand A is valid (is immediate data and not tag)
    input        rs_opb_valid,    // Operand B is valid
    
    input        rs_load_in,      // Load new instruction, indicates whether to intake new instruction
    input        fu_busy,   // high if fu_busy
    input        rs_free_in,      // Free RS entry, indicates whether to free RS entry upon completion
    
    input        reset,            // Reset signal
    input        clock,            // Clock signal

    output       rs_ready_out,            // Ready to issue
    output [31:0] rs_opa_out,             // Out: Operand A
    output [31:0] rs_opb_out,             // Out: Operand B
    output INST rs_inst_out,
    output ALU_OPA_SELECT rs_opa_select_out, // Out: Operand A select
    output ALU_OPB_SELECT rs_opb_select_out, // Out: Operand B select
    output [`ROB_TAG_BITS-1:0]  rs_tag_out,             // Out: ROB tag
    output ALU_FUNC rs_alu_func_out,      // Out: ALU func
    output [31:0]  rs_npc_out,            // Out: NPC
    output [31:0] rs_pc_out,             // Out: PC
    output rs_rd_mem_out,
    output rs_wr_mem_out,
    output rs_cond_branch_out,             // Out: is branch
    output rs_uncond_branch_out,             // Out: is branch
    output       rs_avail_out,            // Is this entry available?
    output [73:0] rs_debug
);

//internal storage: 
logic [31:0] OPa, OPb; // Operand A and B
ALU_OPA_SELECT rs_opa_select_internal; // Operand A select
ALU_OPB_SELECT rs_opb_select_internal; // Operand B select
INST internal_inst; // internal instruction storage
logic [`ROB_TAG_BITS-1:0] OPaTag, OPbTag; // Operand A and B tags
logic [`ROB_TAG_BITS-1:0] DestTag;// Destination register tag, and op tags
logic OpaValid, OpbValid; // Operand A and B valid
logic InUse; // RS entry is in use

ALU_FUNC alu_func; //internal ALU track
logic[31:0] NPC; //internal NPC track
logic [31:0] PC; //internal CDB track
logic internal_rd_mem, internal_wr_mem, internal_cond_branch, internal_uncond_branch; //internal memory track

// Outputs
assign rs_avail_out = !InUse;
assign rs_ready_out = InUse && OpaValid && OpbValid && !fu_busy;
assign rs_opa_out   = OPa;
assign rs_opb_out   = OPb;
assign rs_tag_out   = DestTag;
assign rs_alu_func_out = alu_func;
assign rs_npc_out       = NPC;
assign rs_pc_out       = PC;
assign rs_rd_mem_out = internal_rd_mem;
assign rs_wr_mem_out = internal_wr_mem;
assign rs_cond_branch_out = internal_cond_branch;
assign rs_uncond_branch_out = internal_uncond_branch;
assign rs_opa_select_out = rs_opa_select_internal;
assign rs_opb_select_out = rs_opb_select_internal;
assign rs_inst_out = internal_inst;

// Load from CDB if tag matches
wire LoadAFromCDB = (rs_cdb_tag == OPaTag) && !OpaValid && InUse && rs_cdb_valid;
wire LoadBFromCDB = (rs_cdb_tag == OPbTag) && !OpbValid && InUse && rs_cdb_valid;
//Debug
assign rs_debug = {OPa, OpaValid, OPb, OpbValid, DestTag, InUse, rs_ready_out, rs_avail_out};

always_ff @(posedge clock) begin
    if (reset) begin
        OPa      <= 32'b0;
        OPb      <= 32'b0;
        OPaTag   <= 5'b0;
        OPbTag   <= 5'b0;
        DestTag  <= 5'b0;
        OpaValid <= 1'b0;
        OpbValid <= 1'b0;
        InUse    <= 1'b0;
        alu_func     <= ALU_ADD;
        NPC      <= 32'b0;
        PC       <= 32'b0;
        internal_rd_mem <= 1'b0;
        internal_wr_mem <= 1'b0;
        internal_cond_branch <= 1'b0;
        internal_uncond_branch <= 1'b0;
        rs_opa_select_internal <= OPA_IS_RS1;
        rs_opb_select_internal <= OPB_IS_RS2;
    end else begin
        // Load new instruction
        if (rs_load_in && !InUse) begin
            OPa      <= rs_opa_in;
            OPb      <= rs_opb_in;
            DestTag  <= rs_rob_tag;
            OpaValid <= rs_opa_valid;
            OpbValid <= rs_opb_valid;
            OPaTag <= (rs_opa_valid) ? 0 : rs_opa_in[`ROB_TAG_BITS-1:0];
            OPbTag <= (rs_opb_valid) ? 0 : rs_opb_in[`ROB_TAG_BITS-1:0];
            InUse    <= 1'b1;

            alu_func <= rs_alu_func_in;
            NPC      <= rs_npc_in;
            PC       <= rs_pc_in;
            internal_rd_mem <= rd_mem;
            internal_wr_mem <= wr_mem;
            internal_cond_branch <= cond_branch;
            internal_uncond_branch <= uncond_branch;
            rs_opa_select_internal <= rs_opa_select;
            rs_opb_select_internal <= rs_opb_select;
            internal_inst <= rs_inst;
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
                OPa <= 32'b0;
                OPb <= 32'b0;
                OPaTag <= 0;
                OPbTag <= 0;
                DestTag <= 0;
                OpaValid <= 0;
                OpbValid <= 0;
                alu_func <= ALU_ADD;
                NPC <= 32'b0;
                PC <= 32'b0;
                InUse <= 0;
                internal_rd_mem <= 1'b0;
                internal_wr_mem <= 1'b0;
                internal_cond_branch <= 1'b0;
                internal_uncond_branch <= 1'b0;
                rs_opa_select_internal <= OPA_IS_RS1;
                rs_opb_select_internal <= OPB_IS_RS2;

            end
        end
    end
end



endmodule

