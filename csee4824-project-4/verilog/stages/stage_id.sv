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
    input              wb_regfile_en,   // Reg write enable from WB Stage
    input [4:0]        wb_regfile_idx,  // Reg write index from WB Stage
    input [`XLEN-1:0]  wb_regfile_data, // Reg write data from WB Stage

    //New I/O
    input cdb_broadcast,
    input [4:0] cdb_tag,
    input [31:0] cdb_value,

    output ID_EX_PACKET id_packet
);

    assign id_packet.inst = if_id_reg.inst;
    assign id_packet.PC   = if_id_reg.PC;
    assign id_packet.NPC  = if_id_reg.NPC;

    // For counting valid instructions executed
    // and making the fetch stage die on halts/keeping track of when
    // to allow the next instruction out of fetch
    // 0 for HALT and illegal instructions (end processor on halt)
    assign id_packet.valid = if_id_reg.valid & ~id_packet.illegal;

    logic has_dest_reg;
    assign id_packet.dest_reg_idx = (has_dest_reg) ? if_id_reg.inst.r.rd : `ZERO_REG;

    logic [4:0] rs1_to_mt_tag;
    logic [4:0] rob_tail; 
    logic [4:0] mt_to_rs1_tag1, mt_to_rs1_tag2;
    logic [4:0] mt_to_regfile_rs1, mt_to_regfile_rs2, mt_to_regfile_rd;

    logic dispatch_valid;
    logic retire_valid;

    //Instantiate the map table
    map_table map_table_0 (
        .reset (reset),
        .clock (clock),
        .rs1_addr (if_id_reg.inst.r.rs1),
        .rs2_addr (if_id_reg.inst.r.rs2),
        .r_dest (if_id_reg.inst.r.rd),
        .tag_in (rob_tail),
        .dispatch_valid (dispatch_valid),
        .cdb_tag_in (cdb_tag),
        .read_cdb (cdb_broadcast),
        .rs1_tag (mt_to_rs_tag1),
        .rs2_tag (mt_to_rs_tag2),
        .retire_addr (),
        .retire_valid (retire_valid),
        //.rs1_tag_valid(),
        //.rs2_tag_valid(),
        .regfile_rs1_addr (mt_to_regfile_rs1),
        .regfile_rs2_addr (mt_to_regfile_rs2),
        .reg_write (mt_to_regfile_rd),

    );

    logic [31:0] rs1_opa_in;
    logic [31:0] rs1_opb_in;
    logic rs1_opa_valid;
    logic rs1_opb_valid;

    logic load_rs; //RS control signals
    logic issue_rs;
    logic clear_rs;

    logic rs_ready; //RS status outputs
    logic rs_available;

    logic [31:0] opA;
    logic [31:0] opB;
    logic output_tag;

    logic rob_to_rs_read1;
    logic rob_read_tag1;
    logic rob_to_rs_value1;
    logic rob_to_rs_read2;
    logic rob_read_tag2;
    logic rob_to_rs_value2;

    //Reservation station operand muxes
    always_comb begin
        case (opa_select)
            OPA_IS_RS1 : rs1_opa_in = {27'b0, mt_to_rs_tag1[5:1]}; //Sign extended tag for rs1
            OPA_IS_NPC : rs1_opa_in = if_id_reg.NPC; //Immediates
            OPA_IS_PC : rs1_opa_in = if_id_reg.PC;
            OPA_IS_ZERO : rs1_opa_in = 32'b0; 
        endcase
        if (opa_select == OPA_IS_RS1) begin //If inst is R type
            if (mt_to_rs_tag1[4:0]) begin
                
            end if (!mt_to_rs_tag1[0]) begin //Rs1 is not ready in ROB
                rs1_opa_valid = 0;
            end else begin
                rs1_opa_valid = 1;
                rob_to_rs_read1 = 1;
                rob_read_tag1 = mt_to_rs_tag1[5:1];
                rs1_opa_in = rob_to_rs_value1; 
            end
        end else begin
            rs1_opa_valid = 1;
        end 
    end

    always_comb begin
        case (opb_select)
            OPB_IS_RS2 : rs_opb_in = {27'b0, mt_to_rs_tag2[5:1]}; //Sign extended tag for rs2
            OPB_IS_I_IMM  : rs1_opb_in = `RV32_signext_Iimm(id_ex_reg.inst);
            OPB_IS_S_IMM  : rs1_opb_in = `RV32_signext_Simm(id_ex_reg.inst);
            OPB_IS_B_IMM  : rs1_opb_in = `RV32_signext_Bimm(id_ex_reg.inst);
            OPB_IS_U_IMM  : rs1_opb_in = `RV32_signext_Uimm(id_ex_reg.inst);
            OPB_IS_J_IMM  : rs1_opb_in = `RV32_signext_Jimm(id_ex_reg.inst);
        endcase
        if (opa_select == OPA_IS_RS2) begin //If inst is R type
            if (!mt_to_rs_tag2[0]) begin //Rs1 is not ready in ROB
                rs1_opb_valid = 0;
            end else begin //ADD READ FROM REGFILE LOGIC!
                rs1_opb_valid = 1;
                rob_to_rs_read2 = 1;
                rob_read_tag2 = mt_to_rs_tag2[5:1];
                rs1_opb_in = rob_to_rs_value2; 
            end
        end else begin
            rs1_opb_valid = 1;
        end 
    end

    //Instantiate the reservation station - Currently only supports one FU!
    reservation_station reservation_station_1 (
        .reset (reset),
        .clock (clock),
        .rs_dest_in (rob_tail),
        .rs_cdb_in (cdb_value),
        .rs_cdb_tag (cdb_tag),
        .rs_cdb_valid (cdb_broadcast),
        .rs_opa_in (rs1_opa_in),
        .rs_opb_in (rs1_opb_in),
        .rs_opa_valid (rs1_opa_valid),
        .rs_opb_valid (rs1_opb_valid),
        .rs_load_in (load_rs),
        .rs_use_enable (issue_rs),
        .rs_free_in (clear_rs),
 
        .rs_ready_out (rs_ready),
        .rs_opa_out (opA),
        .rs_opb_out (opB),
        .rs_dest_tag_out (output_tag),
        .rs_avail_out (rs_available)
        
    );

    logic clear_rob;
    logic [4:0] rob_to_regfile_addr;
    logic [4:0] rob_to_regfile_value;
    logic rob_full;

    reorder_buffer reorder_buffer_0 (
        .reset (reset),
        .clock (clock),
        .dispatch_dest_reg (if_id_reg.inst.r.rd),
        .dispatch_opcode (if_id_reg.inst.opcode),
        .dispatch_valid (dispatch_valid),
        .rob_to_rs_read1 (rob_to_rs_read1),
        .rob_read_tag1 (rob_read_tag1),
        .rob_to_rs_read2 (rob_to_rs_read2),
        .rob_read_tag2 (rob_read_tag2),

    //input signals from execute stage 
        .cdb_tag (cdb_tag), //cdb - commnon data bus
        .cdb_value (cdb_value),
        .cdb_valid (cdb_broadcast),

    //input signals from retire stage
        .retire_valid (retire_valid),
        .branch_mispredict (clear_rob),

    //output signals to Regfile
        .reg_dest(rob_to_regfile_addr),
        .reg_value(rob_to_regfile_value),

    // output for map table: Tag for latest dispatch 
        .rob_tag_out(rob_tail),
        .rob_to_rs_value1(rob_to_rs_value1)
        .rob_to_rs_value2(rob_to_rs_value2)
        //.rob_out_valid(),
    //output signals to memory
        //.mem_addr(), //Will incorporate later with LSQ structures 
        //.mem_valid(),
    //rob full signal, for stalling/hazards 
        .rob_full(rob_full)
    );

    // Instantiate the register file
    regfile regfile_0 (
        .clock  (clock),
        .read_idx_1 (if_id_reg.inst.r.rs1),
        .read_idx_2 (if_id_reg.inst.r.rs2),
        .write_en   (wb_regfile_en),
        .write_idx  (wb_regfile_idx),
        .write_data (wb_regfile_data),

        .read_out_1 (id_packet.rs1_value),
        .read_out_2 (id_packet.rs2_value)
    );

    // Instantiate the instruction decoder
    decoder decoder_0 (
        // Inputs
        .inst  (if_id_reg.inst),
        .valid (if_id_reg.valid),

        // Outputs
        .opa_select    (id_packet.opa_select),
        .opb_select    (id_packet.opb_select),
        .alu_func      (id_packet.alu_func),
        .has_dest      (has_dest_reg),
        .rd_mem        (id_packet.rd_mem),
        .wr_mem        (id_packet.wr_mem),
        .cond_branch   (id_packet.cond_branch),
        .uncond_branch (id_packet.uncond_branch),
        .csr_op        (id_packet.csr_op),
        .halt          (id_packet.halt),
        .illegal       (id_packet.illegal)
    );


endmodule // stage_id