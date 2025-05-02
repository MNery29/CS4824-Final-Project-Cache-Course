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

module reservation_station_entry(
    input logic clock,
    input logic reset,

    // Control
    input logic rs_load_in,
    input logic rs_free_in,
    input logic fu_busy,

    // Instruction and metadata
    input INST rs_inst,
    input [31:0] rs_npc_in,
    input [31:0] rs_pc_in,
    input ALU_FUNC rs_alu_func_in,
    input logic rd_mem, wr_mem, cond_branch, uncond_branch,
    input [`ROB_TAG_BITS-1:0] rs_rob_tag,

    // Operand inputs and tags
    input logic [31:0] rs_opa_in, rs_opb_in,
    input logic rs_opa_valid, rs_opb_valid,
    input ALU_OPA_SELECT rs_opa_select,
    input ALU_OPB_SELECT rs_opb_select,

    // Common data bus
    input logic [31:0] rs_cdb_in,
    input [`ROB_TAG_BITS-1:0] rs_cdb_tag,
    input logic rs_cdb_valid,

    // Outputs
    output logic rs_ready_out,
    output logic [31:0] rs_opa_out, rs_opb_out,
    output INST rs_inst_out,
    output ALU_OPA_SELECT rs_opa_select_out,
    output ALU_OPB_SELECT rs_opb_select_out,
    output [`ROB_TAG_BITS-1:0] rs_tag_out,
    output ALU_FUNC rs_alu_func_out,
    output [31:0] rs_npc_out, rs_pc_out,
    output logic rs_rd_mem_out, rs_wr_mem_out,
    output logic rs_cond_branch_out, rs_uncond_branch_out,
    output logic rs_avail_out,
    output logic [73:0] rs_debug
);

    // Internal state
    logic [31:0] OPa, OPb;
    logic [`ROB_TAG_BITS-1:0] OPaTag, OPbTag, DestTag;
    logic OpaValid, OpbValid, InUse;
    logic [31:0] NPC, PC;
    logic rd_internal, wr_internal, cb_internal, ucb_internal;
    ALU_FUNC alu_func_internal;
    ALU_OPA_SELECT opa_sel_internal;
    ALU_OPB_SELECT opb_sel_internal;
    INST inst_internal;

    // Outputs
    assign rs_ready_out          = InUse && OpaValid && OpbValid && !fu_busy;
    assign rs_avail_out          = ~InUse;
    assign rs_opa_out            = OPa;
    assign rs_opb_out            = OPb;
    assign rs_tag_out            = DestTag;
    assign rs_alu_func_out       = alu_func_internal;
    assign rs_npc_out            = NPC;
    assign rs_pc_out             = PC;
    assign rs_rd_mem_out         = rd_internal;
    assign rs_wr_mem_out         = wr_internal;
    assign rs_cond_branch_out    = cb_internal;
    assign rs_uncond_branch_out  = ucb_internal;
    assign rs_opa_select_out     = opa_sel_internal;
    assign rs_opb_select_out     = opb_sel_internal;
    assign rs_inst_out           = inst_internal;

    assign rs_debug = {OPa, OpaValid, OPb, OpbValid, DestTag, InUse, rs_ready_out, rs_avail_out};

    wire LoadAFromCDB = (rs_cdb_tag == OPaTag) && !OpaValid && InUse && rs_cdb_valid;
    wire LoadBFromCDB = (rs_cdb_tag == OPbTag) && !OpbValid && InUse && rs_cdb_valid;

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            OPa <= 0; OPb <= 0;
            OPaTag <= 0; OPbTag <= 0; DestTag <= 0;
            OpaValid <= 0; OpbValid <= 0;
            InUse <= 0;
            NPC <= 0; PC <= 0;
            rd_internal <= 0; wr_internal <= 0;
            cb_internal <= 0; ucb_internal <= 0;
            alu_func_internal <= ALU_ADD;
            opa_sel_internal <= OPA_IS_RS1;
            opb_sel_internal <= OPB_IS_RS2;
            inst_internal <= '0;
        end else begin
            if (rs_load_in && !InUse) begin
                OPa      <= rs_opa_in;
                OPb      <= rs_opb_in;
                OPaTag   <= rs_opa_valid ? '0 : rs_opa_in[`ROB_TAG_BITS-1:0];
                OPbTag   <= rs_opb_valid ? '0 : rs_opb_in[`ROB_TAG_BITS-1:0];
                OpaValid <= rs_opa_valid;
                OpbValid <= rs_opb_valid;
                DestTag  <= rs_rob_tag;
                InUse    <= 1;
                NPC      <= rs_npc_in;
                PC       <= rs_pc_in;
                rd_internal  <= rd_mem;
                wr_internal  <= wr_mem;
                cb_internal  <= cond_branch;
                ucb_internal <= uncond_branch;
                alu_func_internal <= rs_alu_func_in;
                opa_sel_internal <= rs_opa_select;
                opb_sel_internal <= rs_opb_select;
                inst_internal    <= rs_inst;
            end

            if (LoadAFromCDB) begin
                OPa      <= rs_cdb_in;
                OpaValid <= 1;
            end
            if (LoadBFromCDB) begin
                OPb      <= rs_cdb_in;
                OpbValid <= 1;
            end

            if (rs_free_in) begin
                OPa <= 0; OPb <= 0;
                OPaTag <= 0; OPbTag <= 0; DestTag <= 0;
                OpaValid <= 0; OpbValid <= 0;
                InUse <= 0;
                NPC <= 0; PC <= 0;
                rd_internal <= 0; wr_internal <= 0;
                cb_internal <= 0; ucb_internal <= 0;
                alu_func_internal <= ALU_ADD;
                opa_sel_internal <= OPA_IS_RS1;
                opb_sel_internal <= OPB_IS_RS2;
                inst_internal <= '0;
            end
        end
    end

endmodule


module reservation_station (
    input clock,
    input reset,

    // Control signals
    input [1:0] rs_fu_select_in,
    input       rs_load_in,
    input [`RS_SIZE-1:0] rs_free_in,
    input [`RS_SIZE-1:0] fu_busy,

    // Instruction info
    input INST rs_inst,
    input [31:0] rs_npc_in,
    input [31:0] rs_pc_in,
    input ALU_FUNC rs_alu_func_in,
    input rd_mem, wr_mem, cond_branch, uncond_branch,
    input [`ROB_TAG_BITS-1:0] rs_rob_tag,

    // Operand info
    input [31:0] rs_cdb_in,
    input [`ROB_TAG_BITS-1:0] rs_cdb_tag,
    input rs_cdb_valid,
    input [31:0] rs_opa_in, rs_opb_in,
    input ALU_OPA_SELECT rs_opa_select,
    input ALU_OPB_SELECT rs_opb_select,
    input rs_opa_valid, rs_opb_valid,

    // Outputs
    output logic [`RS_SIZE-1:0] rs_ready_out,
    output logic [31:0] rs_opa_out [`RS_SIZE],
    output logic [31:0] rs_opb_out [`RS_SIZE],
    output INST rs_inst_out [`RS_SIZE],
    output ALU_OPA_SELECT rs_opa_select_out [`RS_SIZE],
    output ALU_OPB_SELECT rs_opb_select_out [`RS_SIZE],
    output [`ROB_TAG_BITS-1:0] rs_tag_out [`RS_SIZE],
    output ALU_FUNC rs_alu_func_out [`RS_SIZE],
    output [31:0] rs_npc_out [`RS_SIZE],
    output [31:0] rs_pc_out [`RS_SIZE],
    output logic rs_rd_mem_out [`RS_SIZE],
    output logic rs_wr_mem_out [`RS_SIZE],
    output logic rs_cond_branch_out [`RS_SIZE],
    output logic rs_uncond_branch_out [`RS_SIZE],
    output logic rs_avail_out [`RS_SIZE],
    output logic [73:0] rs_debug [`RS_SIZE]
);

genvar i;
generate
    for (i = 0; i < `RS_SIZE; i++) begin : rs_entries
        reservation_station_entry rs_entry (
            .clock(clock),
            .reset(reset),

            .rs_npc_in(rs_npc_in),
            .rs_pc_in(rs_pc_in),
            .rs_alu_func_in(rs_alu_func_in),
            .rd_mem(rd_mem),
            .wr_mem(wr_mem),
            .cond_branch(cond_branch),
            .uncond_branch(uncond_branch),

            .rs_inst(rs_inst),
            .rs_rob_tag(rs_rob_tag),
            .rs_cdb_in(rs_cdb_in),
            .rs_cdb_tag(rs_cdb_tag),
            .rs_cdb_valid(rs_cdb_valid),

            .rs_opa_in(rs_opa_in),
            .rs_opb_in(rs_opb_in),
            .rs_opa_select(rs_opa_select),
            .rs_opb_select(rs_opb_select),
            .rs_opa_valid(rs_opa_valid),
            .rs_opb_valid(rs_opb_valid),

            .rs_load_in(rs_load_in && (rs_fu_select_in == i[1:0])),  // Only allow one entry to load
            .fu_busy(fu_busy[i]),
            .rs_free_in(rs_free_in[i]),

            .rs_ready_out(rs_ready_out[i]),
            .rs_opa_out(rs_opa_out[i]),
            .rs_opb_out(rs_opb_out[i]),
            .rs_inst_out(rs_inst_out[i]),
            .rs_opa_select_out(rs_opa_select_out[i]),
            .rs_opb_select_out(rs_opb_select_out[i]),
            .rs_tag_out(rs_tag_out[i]),
            .rs_alu_func_out(rs_alu_func_out[i]),
            .rs_npc_out(rs_npc_out[i]),
            .rs_pc_out(rs_pc_out[i]),
            .rs_rd_mem_out(rs_rd_mem_out[i]),
            .rs_wr_mem_out(rs_wr_mem_out[i]),
            .rs_cond_branch_out(rs_cond_branch_out[i]),
            .rs_uncond_branch_out(rs_uncond_branch_out[i]),
            .rs_avail_out(rs_avail_out[i]),
            .rs_debug(rs_debug[i])

        );
    end
endgenerate


endmodule