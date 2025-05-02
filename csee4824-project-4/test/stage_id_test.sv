`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

`timescale 1ns/1ps

module testbench;

  logic clock, reset;

  IF_ID_PACKET if_id_reg;
  logic cdb_valid;
  logic [`ROB_TAG_BITS-1:0] cdb_tag;
  logic [31:0] cdb_value;
  logic cdb_take_branch;

  logic [2:0] fu_busy;
  logic [`RS_SIZE-1:0] rs_clear_vec;

  logic store_retire;
  logic [4:0] store_tag;
  logic [4:0] rob_dest_reg;
  logic [31:0] rob_to_regfile_value;
  logic retire_entry;
  logic [4:0] retire_tag;
  logic lsq_free;
  logic if_stall;

  logic [`RS_SIZE-1:0] rs_ready_out;
  logic [31:0] rs_opa_out [`RS_SIZE];
  logic [31:0] rs_opb_out [`RS_SIZE];
  ALU_OPA_SELECT rs_opa_select_out [`RS_SIZE];
  ALU_OPB_SELECT rs_opb_select_out [`RS_SIZE];
  INST rs_inst_out [`RS_SIZE];
  logic [4:0] rs_tag_out [`RS_SIZE];
  logic [31:0] rs_npc_out [`RS_SIZE];
  logic [31:0] rs_pc_out [`RS_SIZE];
  ALU_FUNC rs_alu_func_out [`RS_SIZE];
  logic rs_rd_mem_out [`RS_SIZE];
  logic rs_wr_mem_out [`RS_SIZE];
  logic rs_cond_branch_out [`RS_SIZE];
  logic rs_uncond_branch_out [`RS_SIZE];
  logic rs_avail_out [`RS_SIZE];

  logic [45:0] rob_debug [`ROB_SZ-1:0];
  logic [11:0] rob_pointers_debug;
  logic [73:0] rs_debug [`RS_SIZE];

  ALU_OPA_SELECT opa_select;
  ALU_OPB_SELECT opb_select;
  logic has_dest_reg;
  logic [4:0] dest_reg_idx;
  ALU_FUNC alu_func_out;
  ROB_RETIRE_PACKET rob_retire_out;
  logic rd_mem_out, wr_mem_out, cond_branch_out, uncond_branch_out;
  logic rob_valid, rob_ready;
  LSQ_PACKET lsq_packet;
  logic rob_full, rs1_available, dispatch_ok;
  logic [5:0] mt_to_rs_tag1, mt_to_rs_tag2;
  logic [31:0] rs1_value, rs2_value;
  logic [31:0] rob_to_rs_value1, rob_to_rs_value2;
  logic [31:1][`XLEN-1:0] debug_reg;
  logic [4:0] mt_to_regfile_rs1, mt_to_regfile_rs2;
  logic [31:0] rs1_opa_in, rs1_opb_in;
  logic [1:0] fu_select;

  stage_id dut (
    .clock(clock),
    .reset(reset),
    .if_id_reg(if_id_reg),
    .if_stall(if_stall),
    .cdb_valid(cdb_valid),
    .cdb_tag(cdb_tag),
    .cdb_value(cdb_value),
    .cdb_take_branch(cdb_take_branch),
    .fu_busy(fu_busy),
    .rs_clear_vec(rs_clear_vec),
    .store_retire(store_retire),
    .store_tag(store_tag),
    .rob_dest_reg(rob_dest_reg),
    .rob_to_regfile_value(rob_to_regfile_value),
    .retire_entry(retire_entry),
    .retire_tag(retire_tag),
    .lsq_free(lsq_free),
    .rs_ready_out(rs_ready_out),
    .rs_opa_out(rs_opa_out),
    .rs_opb_out(rs_opb_out),
    .rs_opa_select_out(rs_opa_select_out),
    .rs_opb_select_out(rs_opb_select_out),
    .rs_inst_out(rs_inst_out),
    .rs_tag_out(rs_tag_out),
    .rs_npc_out(rs_npc_out),
    .rs_pc_out(rs_pc_out),
    .rs_alu_func_out(rs_alu_func_out),
    .rs_rd_mem_out(rs_rd_mem_out),
    .rs_wr_mem_out(rs_wr_mem_out),
    .rs_cond_branch_out(rs_cond_branch_out),
    .rs_uncond_branch_out(rs_uncond_branch_out),
    .rs_avail_out(rs_avail_out),
    .rob_debug(rob_debug),
    .rob_pointers_debug(rob_pointers_debug),
    .opa_select(opa_select),
    .opb_select(opb_select),
    .has_dest_reg(has_dest_reg),
    .dest_reg_idx(dest_reg_idx),
    .alu_func_out(alu_func_out),
    .rob_retire_out(rob_retire_out),
    .rd_mem_out(rd_mem_out),
    .wr_mem_out(wr_mem_out),
    .cond_branch_out(cond_branch_out),
    .uncond_branch_out(uncond_branch_out),
    .rob_valid(rob_valid),
    .rob_ready(rob_ready),
    .lsq_packet(lsq_packet),
    .rob_full(rob_full),
    .rs1_available(rs1_available),
    .dispatch_ok(dispatch_ok),
    .rs_debug(rs_debug),
    .mt_to_rs_tag1(mt_to_rs_tag1),
    .mt_to_rs_tag2(mt_to_rs_tag2),
    .rs1_value(rs1_value),
    .rs2_value(rs2_value),
    .rob_to_rs_value1(rob_to_rs_value1),
    .rob_to_rs_value2(rob_to_rs_value2),
    .debug_reg(debug_reg),
    .mt_to_regfile_rs1(mt_to_regfile_rs1),
    .mt_to_regfile_rs2(mt_to_regfile_rs2),
    .rs1_opa_in(rs1_opa_in),
    .rs1_opb_in(rs1_opb_in),
    .fu_select(fu_select)
  );

  always begin
    #(`CLOCK_PERIOD/2.0);
    clock = ~clock;
  end

  initial begin
    $display("Starting stage_id unit test...");

    // Reset
    clock = 0;
    reset = 1;
    cdb_valid = 0;
    rs_clear_vec = 0;
    cdb_tag = 0;
    cdb_value = 0;
    cdb_take_branch = 0;
    fu_busy = 0;
    store_retire = 0;
    store_tag = 0;
    rob_dest_reg = 0;
    rob_to_regfile_value = 0;
    retire_entry = 0;
    retire_tag = 0;
    lsq_free = 1;
    if_stall = 0;

    @(negedge clock);
    reset = 0;

    // ------------------------
    // Test Case 1: ADD x5, x1, x2
    //   --> opcode = 0110011
    //   --> funct7 = 0000000, rs2 = 2, rs1 = 1, funct3 = 000, rd = 5
    //   --> binary: 0000000 00010 00001 000 00101 0110011
    //   --> hex:    0x002081B3
    // ------------------------
    if_id_reg.inst = 32'h002081B3;
    if_id_reg.valid = 1;
    if_id_reg.PC = 32'h100;
    if_id_reg.NPC = 32'h104;

    @(negedge clock);
    $display("[ADD] rs1 = x1, rs2 = x2 -> rd = x5");
    $display("Dest reg: %d, Has dest: %b", dest_reg_idx, has_dest_reg);

    // ------------------------
    // Test Case 2: LW x6, 8(x3)
    //   --> opcode = 0000011
    //   --> imm = 000000001000, rs1 = 3, funct3 = 010, rd = 6
    //   --> binary: 000000001000 00011 010 00110 0000011
    //   --> hex:    0x00831283
    // ------------------------
    if_id_reg.inst = 32'h00831283;
    if_id_reg.valid = 1;
    if_id_reg.PC = 32'h104;
    if_id_reg.NPC = 32'h108;

    @(negedge clock);
    $display("[LW] rs1 = x3, offset = 8 -> rd = x6");
    $display("Dest reg: %d, Has dest: %b, rd_mem: %b", dest_reg_idx, has_dest_reg, rd_mem_out);

    // ------------------------
    // Test Case 3: BEQ x1, x2, offset = 16
    //   --> opcode = 1100011
    //   --> imm[12|10:5] = 000001, rs2 = 2, rs1 = 1, funct3 = 000,
    //       imm[4:1|11] = 0000|0 --> final imm = 16
    //   --> binary: 0000000 00010 00001 000 00000 1100011
    //   --> encoded: 000000000010 00001 000 00010 1100011 (beq x1,x2,16)
    //   --> hex:     0x00208263
    // ------------------------
    if_id_reg.inst = 32'h00208263;
    if_id_reg.valid = 1;
    if_id_reg.PC = 32'h108;
    if_id_reg.NPC = 32'h10C;

    @(negedge clock);
    $display("[BEQ] rs1 = x1, rs2 = x2 -> branch if equal");
    $display("Cond branch: %b, Uncond branch: %b", cond_branch_out, uncond_branch_out);

    // ------------------------
    // Wrap up
    // ------------------------
    repeat (2) @(negedge clock);
    $display("stage_id test complete.");
    $finish;
end

endmodule
