/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_id.sv                                         //
//                                                                     //
//  Description :  instruction decode (ID) stage of the pipeline;      //
//                 decode the instruction fetch register operands, and //
//                 compute immediate operand (if applicable)           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"
//`include "verilog/map_table.sv" //TEMPORARILY COMMENTED OUT FOR INTEGRATION TEST
`include "verilog/reservation_station.sv"
//`include "verilog/reorder_buffer.sv" //TEMPORARILY COMMENTED OUT FOR INTEGRATION TEST
`include "verilog/regfile.sv"

// Decode an instruction: generate useful datapath control signals by matching the RISC-V ISA
// This module is purely combinational
module decoder (
    input INST  inst,
    input logic valid, // when low, ignore inst. Output will look like a NOP

    output ALU_OPA_SELECT opa_select,
    output ALU_OPB_SELECT opb_select,
    output logic          has_dest, // if there is a destination register
    output ALU_FUNC       alu_func,
    output logic          rd_mem, wr_mem, cond_branch, uncond_branch,
    output logic          csr_op, // used for CSR operations, we only use this as a cheap way to get the return code out
    output logic          halt,   // non-zero on a halt
    output logic          illegal // non-zero on an illegal instruction

);

    // Note: I recommend using an IDE's code folding feature on this block
    always_comb begin
        // Default control values (looks like a NOP)
        // See sys_defs.svh for the constants used here
        opa_select    = OPA_IS_RS1;
        opb_select    = OPB_IS_RS2;
        alu_func      = ALU_ADD;
        has_dest      = `FALSE;
        csr_op        = `FALSE;
        rd_mem        = `FALSE;
        wr_mem        = `FALSE;
        cond_branch   = `FALSE;
        uncond_branch = `FALSE;
        halt          = `FALSE;
        illegal       = `FALSE;

        if (valid) begin
            casez (inst)
                `RV32_LUI: begin
                    has_dest   = `TRUE;
                    opa_select = OPA_IS_ZERO;
                    opb_select = OPB_IS_U_IMM;
                end
                `RV32_AUIPC: begin
                    has_dest   = `TRUE;
                    opa_select = OPA_IS_PC;
                    opb_select = OPB_IS_U_IMM;
                end
                `RV32_JAL: begin
                    has_dest      = `TRUE;
                    opa_select    = OPA_IS_PC;
                    opb_select    = OPB_IS_J_IMM;
                    uncond_branch = `TRUE;
                end
                `RV32_JALR: begin
                    has_dest      = `TRUE;
                    opa_select    = OPA_IS_RS1;
                    opb_select    = OPB_IS_I_IMM;
                    uncond_branch = `TRUE;
                end
                `RV32_BEQ, `RV32_BNE, `RV32_BLT, `RV32_BGE,
                `RV32_BLTU, `RV32_BGEU: begin
                    opa_select  = OPA_IS_PC;
                    opb_select  = OPB_IS_B_IMM;
                    cond_branch = `TRUE;
                end
                `RV32_LB, `RV32_LH, `RV32_LW,
                `RV32_LBU, `RV32_LHU: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    rd_mem     = `TRUE;
                end
                `RV32_SB, `RV32_SH, `RV32_SW: begin
                    opb_select = OPB_IS_S_IMM;
                    wr_mem     = `TRUE;
                end
                `RV32_ADDI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                end
                `RV32_SLTI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_SLT;
                end
                `RV32_SLTIU: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_SLTU;
                end
                `RV32_ANDI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_AND;
                end
                `RV32_ORI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_OR;
                end
                `RV32_XORI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_XOR;
                end
                `RV32_SLLI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_SLL;
                end
                `RV32_SRLI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_SRL;
                end
                `RV32_SRAI: begin
                    has_dest   = `TRUE;
                    opb_select = OPB_IS_I_IMM;
                    alu_func   = ALU_SRA;
                end
                `RV32_ADD: begin
                    has_dest   = `TRUE;
                end
                `RV32_SUB: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SUB;
                end
                `RV32_SLT: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SLT;
                end
                `RV32_SLTU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SLTU;
                end
                `RV32_AND: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_AND;
                end
                `RV32_OR: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_OR;
                end
                `RV32_XOR: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_XOR;
                end
                `RV32_SLL: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SLL;
                end
                `RV32_SRL: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SRL;
                end
                `RV32_SRA: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_SRA;
                end
                `RV32_MUL: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MUL;
                end
                `RV32_MULH: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MULH;
                end
                `RV32_MULHSU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MULHSU;
                end
                `RV32_MULHU: begin
                    has_dest   = `TRUE;
                    alu_func   = ALU_MULHU;
                end
                `RV32_CSRRW, `RV32_CSRRS, `RV32_CSRRC: begin
                    csr_op = `TRUE;
                end
                `WFI: begin
                    halt = `TRUE;
                end
                default: begin
                    illegal = `TRUE;
                end
        endcase // casez (inst)
        end // if (valid)
    end // always

endmodule // decoder

module stage_id (
    input              clock,           // system clock
    input              reset,           // system reset
    input IF_ID_PACKET if_id_reg,

    input if_stall, //for when the inst has to stall bc of perhaps load or storing is happening

    //New I/O
    //CDB 
    input cdb_valid,
    input [`ROB_TAG_BITS-1:0] cdb_tag,
    input [31:0] cdb_value,
    input cdb_take_branch,

    input fu_busy,
    input rs1_clear,
    input rob_retire_entry,


    //this is signal from LSQ to let ROB know that a store command is ready
    input store_retire,
    input [4:0] store_tag,

    //data from retire stage
    input [4:0] rob_dest_reg,
    input [31:0] rob_to_regfile_value,
    input retire_entry,
    input [4:0] retire_tag,
    // input retire_entry,


    // input rob_regfile_valid,

    input lsq_free,




    
    output [31:0] opA,
    output [31:0] opB,
    output INST inst_out,
    output ALU_OPA_SELECT opa_select_out,
    output ALU_OPB_SELECT opb_select_out,
    output [`ROB_TAG_BITS-1:0] output_tag,
    output rs1_ready,
    output [31:0] rs1_npc_out,
    output [31:0] rs1_pc_out,

    output logic [45:0] rob_debug [`ROB_SZ-1:0],
    output [11:0] rob_pointers_debug,
    //output logic [7:0] mt_tags_debug [31:0],
    //output logic [74:0] rs_debug,
    //debugging 
    output ALU_OPA_SELECT opa_select,
    output ALU_OPB_SELECT opb_select,
    output logic has_dest_reg,
    output logic [4:0] dest_reg_idx,
    output ALU_FUNC alu_func_out,
    output ROB_RETIRE_PACKET rob_retire_out, // matches port type exactly

    output logic rd_mem_out, wr_mem_out, cond_branch_out, uncond_branch_out,
    output logic rob_valid, rob_ready, // ready bit from ROB

    // information to send to LSQ about current instruction
    output LSQ_PACKET lsq_packet,
    //debugging signals BELOW
    output logic rob_full, // ROB full signal debugging
    output logic rs1_available, // RS available signal debugging
    output logic dispatch_ok, // Dispatch OK signal debugging
    output logic [73:0] rs_debug, // RS debug signal debugging
    
    output logic [5:0] mt_to_rs_tag1, mt_to_rs_tag2,

    output logic [31:0] rs1_value, rs2_value,
    output logic [31:0] rob_to_rs_value1, rob_to_rs_value2,

    output logic [31:1] [`XLEN-1:0] debug_reg,
    output logic [4:0] mt_to_regfile_rs1, mt_to_regfile_rs2,

    output logic [31:0] rs1_opa_in, rs1_opb_in



);
    logic rd_mem, wr_mem, is_branch;
    logic halt, illegal, csr_op;
    logic cond_branch, uncond_branch;
    ALU_FUNC alu_func;
    logic [6:0] opcode;
    assign opcode = if_id_reg.inst[6:0];

    assign dest_reg_idx = (has_dest_reg) ? if_id_reg.inst.r.rd : `ZERO_REG;

    // Dispatch control signals
    // logic dispatch_ok;
    // logic rob_full;
    // logic rs1_available;

    assign dispatch_ok = (!rob_full) && (rs1_available) && (lsq_free) && (!if_stall);

    logic mt_load_entry, rob_load_entry, rs1_load_entry;
    assign mt_load_entry  = dispatch_ok && if_id_reg.valid;
    assign rob_load_entry = dispatch_ok && if_id_reg.valid;
    assign rs1_load_entry = dispatch_ok && if_id_reg.valid;

    CDB_ROB_PACKET rob_cdb_packet;
    assign rob_cdb_packet.tag = cdb_tag;
    assign rob_cdb_packet.value = cdb_value;
    assign rob_cdb_packet.valid = cdb_valid;
    assign rob_cdb_packet.take_branch = cdb_take_branch;

    // Outputs from map table
    // logic [5:0] mt_to_rs_tag1, mt_to_rs_tag2;
    // logic [4:0] mt_to_regfile_rs1, mt_to_regfile_rs2;

    // Regfile read values
    // logic [31:0] rs1_value, rs2_value;

    // RS operand handling
    // logic [31:0] rs1_opa_in, rs1_opb_in;
    logic rs1_opa_valid, rs1_opb_valid;

    // ROB to RS read signals
    logic rob_to_rs_read1;
    logic [`ROB_TAG_BITS-1:0] rob_read_tag1;
    // logic [31:0] rob_to_rs_value1;
    logic rob_to_rs_read2;
    logic [`ROB_TAG_BITS-1:0] rob_read_tag2;
    // logic [31:0] rob_to_rs_value2;

    // ROB dispatch and retire signals
    logic [`ROB_TAG_BITS-1:0] rob_tag_out;
    logic [`ROB_TAG_BITS-1:0] rob_retire_tag_out;

    logic is_call; //for branch predictor
    logic is_return; //for branch predictor

    assign is_call   = (opcode == 7'b1101111); // opcode for JAL
    assign is_return = (opcode == 7'b1100111) && (if_id_reg.inst.r.rd == 5'd0); // opcode for JALR + destination register x0 packets
    DISPATCH_ROB_PACKET rob_dispatch_packet;
    ROB_DISPATCH_PACKET rob_dispatch_out;

    assign rob_dispatch_packet.dest_reg = dest_reg_idx;
    assign rob_dispatch_packet.opcode   = opcode;
    assign rob_dispatch_packet.valid    = rob_load_entry;
    assign rob_dispatch_packet.is_branch = cond_branch || uncond_branch;
    assign rob_dispatch_packet.halt = halt;
    assign rob_dispatch_packet.illegal = illegal;
    assign rob_dispatch_packet.csr_op = csr_op;
    assign rob_dispatch_packet.npc = if_id_reg.NPC;
    assign rob_dispatch_packet.is_call = is_call;
    assign rob_dispatch_packet.is_return = is_return;


    //operand select (OPA)
    
    always_comb begin
        rs1_opa_in = 32'b0;
        rs1_opa_valid = 0;
        rob_to_rs_read1 = 1;
        rob_read_tag1 = 0;
        if (mt_to_rs_tag1[5:1] == 5'b0) begin
            rs1_opa_in = rs1_value;
            rs1_opa_valid = 1;
        end else if (!mt_to_rs_tag1[0]) begin
            rs1_opa_in = (cdb_valid && cdb_tag == mt_to_rs_tag1[5:1]) ? cdb_value :  {28'b0, mt_to_rs_tag1[5:1]};
            rs1_opa_valid = (cdb_valid && cdb_tag == mt_to_rs_tag1[5:1]) ? 1 : 0;
        end else begin
            rob_to_rs_read1 = 1;
            rob_read_tag1 = mt_to_rs_tag1[5:1];
            rs1_opa_in = rob_to_rs_value1;
            rs1_opa_valid = 1;
        end



        // case (opa_select)
        //     OPA_IS_NPC  : rs1_opa_in = if_id_reg.NPC;
        //     OPA_IS_PC   : rs1_opa_in = if_id_reg.PC;
        //     OPA_IS_ZERO : rs1_opa_in = 32'b0;
        //     OPA_IS_RS1  : begin
        //          if (mt_to_rs_tag1[5:1] == 5'b0) begin
        //             rs1_opa_in = rs1_value;
        //             rs1_opa_valid = 1;
        //         end else if (!mt_to_rs_tag1[0]) begin
        //             rs1_opa_in = {28'b0, mt_to_rs_tag1[5:1]};
        //             rs1_opa_valid = 0;
        //         end else begin
        //             rob_to_rs_read1 = 1;
        //             rob_read_tag1 = mt_to_rs_tag1[5:1];
        //             rs1_opa_in = rob_to_rs_value1;
        //             rs1_opa_valid = 1;
        //         end
        //     end
        // endcase

        // if (opa_select != OPA_IS_RS1 && !cond_branch)
        //     rs1_opa_valid = 1;
    end

    //operand select (OPB)
    always_comb begin
        rs1_opb_in = 32'b0;
        rs1_opb_valid = 0;
        rob_to_rs_read2 = 1;
        rob_read_tag2 = 0;
        
        if (mt_to_rs_tag2[5:1] == 5'b0) begin
            rs1_opb_in = rs2_value;
            rs1_opb_valid = 1;
        end else if (!mt_to_rs_tag2[0]) begin
            rs1_opb_in = (cdb_valid && cdb_tag == mt_to_rs_tag2[5:1]) ? cdb_value : {28'b0, mt_to_rs_tag2[5:1]};
            rs1_opb_valid = (cdb_valid && cdb_tag == mt_to_rs_tag2[5:1]) ? 1 : 0;
        end else begin
            rob_to_rs_read2 = 1;
            rob_read_tag2 = mt_to_rs_tag2[5:1];
            rs1_opb_in = rob_to_rs_value2;
            rs1_opb_valid = 1;
        end

        // case (opb_select)
        //     OPB_IS_I_IMM : rs1_opb_in = `RV32_signext_Iimm(if_id_reg.inst);
        //     OPB_IS_S_IMM : rs1_opb_in = `RV32_signext_Simm(if_id_reg.inst);
        //     OPB_IS_B_IMM : rs1_opb_in = `RV32_signext_Bimm(if_id_reg.inst);
        //     OPB_IS_U_IMM : rs1_opb_in = `RV32_signext_Uimm(if_id_reg.inst);
        //     OPB_IS_J_IMM : rs1_opb_in = `RV32_signext_Jimm(if_id_reg.inst);
        //     OPB_IS_RS2   : begin
        //             if (mt_to_rs_tag2[5:1] == 5'b0) begin
        //                 rs1_opb_in = rs2_value;
        //                 rs1_opb_valid = 1;
        //             end else if (!mt_to_rs_tag2[0]) begin
        //                 rs1_opb_in = {28'b0, mt_to_rs_tag2[5:1]};
        //                 rs1_opb_valid = 0;
        //             end else begin
        //                 rob_to_rs_read2 = 1;
        //                 rob_read_tag2 = mt_to_rs_tag2[5:1];
        //                 rs1_opb_in = rob_to_rs_value2;
        //                 rs1_opb_valid = 1;
        //             end
        //     end
        // endcase

        // if (opb_select != OPB_IS_RS2 && !cond_branch)
        //     rs1_opb_valid = 1;
    end
    logic mt_reset;
    assign mt_reset = reset;
    
    // Map Table
    map_table map_table_0 (
        .reset(mt_reset),
        .clock(clock),
        .rs1_addr(if_id_reg.inst.r.rs1),
        .rs2_addr(if_id_reg.inst.r.rs2),
        .r_dest(dest_reg_idx),
        .tag_in(rob_tag_out),
        .load_entry(mt_load_entry),
        .cdb_tag_in(cdb_tag),
        .read_cdb(cdb_valid),
        .retire_addr(rob_dest_reg),
        .retire_entry(retire_entry),
        .retire_tag(retire_tag),
        .rs1_tag(mt_to_rs_tag1),
        .rs2_tag(mt_to_rs_tag2),
        .regfile_rs1_addr(mt_to_regfile_rs1),
        .regfile_rs2_addr(mt_to_regfile_rs2)
        //.tags_debug(mt_tags_debug)
    );
    logic rs_reset;
    assign rs_reset = reset;

    // Reservation Station
    reservation_station reservation_station_1 (
        .reset(rs_reset),
        .clock(clock),
        .rs_npc_in(if_id_reg.NPC),
        .rs_pc_in(if_id_reg.PC),
        .rs_inst(if_id_reg.inst),
        .rs_rob_tag(rob_tag_out),
        .rs_cdb_in(cdb_value),
        .rs_cdb_tag(cdb_tag),

        .rs_cdb_valid(cdb_valid),
        .rs_opa_in(rs1_opa_in),
        .rs_opb_in(rs1_opb_in),
        .rs_opa_select(opa_select),
        .rs_opb_select(opb_select),
        .rs_opa_valid(rs1_opa_valid),
        .rs_opb_valid(rs1_opb_valid),
        .rs_alu_func_in(alu_func),


        .rd_mem(rd_mem),
        .wr_mem(wr_mem),
        .cond_branch(cond_branch),
        .uncond_branch(uncond_branch),
        .rs_load_in(rs1_load_entry),
        .rs_inst_out(inst_out),
        .fu_busy(fu_busy),
        .rs_free_in(rs1_clear),
        .rs_alu_func_out(alu_func_out),
        .rs_npc_out(rs1_npc_out),
        .rs_pc_out(rs1_pc_out),
        .rs_rd_mem_out(rd_mem_out),
        .rs_wr_mem_out(wr_mem_out),
        .rs_cond_branch_out(cond_branch_out),
        .rs_uncond_branch_out(uncond_branch_out),
        .rs_ready_out(rs1_ready),
        .rs_opa_out(opA),
        .rs_opb_out(opB),
        .rs_opa_select_out(opa_select_out),
        .rs_opb_select_out(opb_select_out),
        .rs_tag_out(output_tag),
        .rs_avail_out(rs1_available),
        .rs_debug(rs_debug)
    );

    // Reorder Buffer
    reorder_buffer reorder_buffer_0 (
        .reset(reset),
        .clock(clock),
        .rob_dispatch_in(rob_dispatch_packet),
        .rob_dispatch_out(rob_dispatch_out),
        .rob_to_rs_read1(rob_to_rs_read1),
        .rob_read_tag1(rob_read_tag1),
        .rob_to_rs_read2(rob_to_rs_read2),
        .rob_read_tag2(rob_read_tag2),
        //.rob_cdb_in('{tag: cdb_tag, value: cdb_value, valid: cdb_valid}), Synthesis Issues, Replacing with, with other instantiations above:
        .rob_cdb_in(rob_cdb_packet),
        .retire_entry(retire_entry),
        .retire_tag(retire_tag),
        .rob_clear(1'b0),
        .store_retire(store_retire),
        .store_tag(store_tag),
        .rob_retire_out(rob_retire_out),
        .rob_to_rs_value1(rob_to_rs_value1),
        .rob_to_rs_value2(rob_to_rs_value2),
        .rob_full(rob_full),
        .rob_ready(rob_ready),
        .rob_valid(rob_valid),
        .rob_debug(rob_debug),
        .rob_pointers(rob_pointers_debug)
    );

    // Register File
    regfile regfile_0 (
        .clock(clock),
        .read_idx_1(mt_to_regfile_rs1),
        .read_idx_2(mt_to_regfile_rs2),
        .write_en(retire_entry),
        .write_idx(rob_dest_reg),
        .write_data(rob_to_regfile_value),
        .read_out_1(rs1_value),
        .read_out_2(rs2_value),

        .debug_reg(debug_reg)
    );

    // Decoder
    decoder decoder_0 (
        .inst(if_id_reg.inst),
        .valid(if_id_reg.valid),
        .opa_select(opa_select),
        .opb_select(opb_select),
        .alu_func(alu_func),
        .rd_mem(rd_mem),
        .wr_mem(wr_mem),
        .cond_branch(cond_branch),
        .uncond_branch(uncond_branch),
        .halt(halt),
        .illegal(illegal),
        .csr_op(csr_op),
        
        .has_dest(has_dest_reg)
    );
    assign rob_tag_out = rob_dispatch_out.tag;

    assign is_branch = cond_branch || uncond_branch;
    assign lsq_packet.valid = if_id_reg.valid && dispatch_ok;
    assign lsq_packet.rd_mem = rd_mem;
    assign lsq_packet.wr_mem = wr_mem;
    assign lsq_packet.store_data = rs1_opb_valid ? rs1_opb_in : 32'b0;
    assign lsq_packet.store_data_valid = rs1_opb_valid;
    assign lsq_packet.store_data_tag = rs1_opb_valid ? 5'b0 : rs1_opb_in[4:0]; //omit MSB
    assign lsq_packet.rob_tag = rob_tag_out;
    assign lsq_packet.rd_unsigned = if_id_reg.inst.r.funct3[2];
    assign lsq_packet.mem_size = MEM_SIZE'(if_id_reg.inst.r.funct3[1:0]);

endmodule 
