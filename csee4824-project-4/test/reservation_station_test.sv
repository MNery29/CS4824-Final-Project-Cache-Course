`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module rs_testbench;

  logic clock, reset;
  logic [1:0] rs_fu_select_in;
  logic rs_load_in;
  logic [`RS_SIZE-1:0] rs_free_in;
  logic [`RS_SIZE-1:0] fu_busy;

  logic [31:0] rs_opa_in, rs_opb_in;
  logic rs_opa_valid, rs_opb_valid;
  ALU_OPA_SELECT rs_opa_select;
  ALU_OPB_SELECT rs_opb_select;
  logic [31:0] rs_npc_in, rs_pc_in;
  ALU_FUNC rs_alu_func_in;
  logic rd_mem, wr_mem, cond_branch, uncond_branch;
  logic [`ROB_TAG_BITS-1:0] rs_rob_tag;
  INST rs_inst;

  logic [31:0] rs_cdb_in;
  logic [`ROB_TAG_BITS-1:0] rs_cdb_tag;
  logic rs_cdb_valid;

  logic [`RS_SIZE-1:0] rs_ready_out;
  logic [31:0] rs_opa_out [`RS_SIZE];
  logic [31:0] rs_opb_out [`RS_SIZE];
  logic rs_rd_mem_out [`RS_SIZE];
  logic rs_wr_mem_out [`RS_SIZE];
  logic rs_cond_branch_out [`RS_SIZE];
  logic rs_uncond_branch_out [`RS_SIZE];
  logic [73:0] rs_debug [`RS_SIZE];
  logic rs_avail_out [`RS_SIZE];
  logic [31:0] rs_npc_out [`RS_SIZE];
  logic [31:0] rs_pc_out [`RS_SIZE];
  INST rs_inst_out [`RS_SIZE];
  ALU_FUNC rs_alu_func_out [`RS_SIZE];
  ALU_OPA_SELECT rs_opa_select_out [`RS_SIZE];
  ALU_OPB_SELECT rs_opb_select_out [`RS_SIZE];
  logic [`ROB_TAG_BITS-1:0] rs_tag_out [`RS_SIZE];

  reservation_station uut (
    .clock(clock),
    .reset(reset),
    .rs_fu_select_in(rs_fu_select_in),
    .rs_load_in(rs_load_in),
    .rs_free_in(rs_free_in),
    .fu_busy(fu_busy),
    .rs_inst(rs_inst),
    .rs_npc_in(rs_npc_in),
    .rs_pc_in(rs_pc_in),
    .rs_alu_func_in(rs_alu_func_in),
    .rd_mem(rd_mem),
    .wr_mem(wr_mem),
    .cond_branch(cond_branch),
    .uncond_branch(uncond_branch),
    .rs_rob_tag(rs_rob_tag),
    .rs_opa_in(rs_opa_in),
    .rs_opb_in(rs_opb_in),
    .rs_opa_valid(rs_opa_valid),
    .rs_opb_valid(rs_opb_valid),
    .rs_opa_select(rs_opa_select),
    .rs_opb_select(rs_opb_select),
    .rs_cdb_in(rs_cdb_in),
    .rs_cdb_tag(rs_cdb_tag),
    .rs_cdb_valid(rs_cdb_valid),
    .rs_ready_out(rs_ready_out),
    .rs_opa_out(rs_opa_out),
    .rs_opb_out(rs_opb_out),
    .rs_inst_out(rs_inst_out),
    .rs_opa_select_out(rs_opa_select_out),
    .rs_opb_select_out(rs_opb_select_out),
    .rs_tag_out(rs_tag_out),
    .rs_alu_func_out(rs_alu_func_out),
    .rs_npc_out(rs_npc_out),
    .rs_pc_out(rs_pc_out),
    .rs_rd_mem_out(rs_rd_mem_out),
    .rs_wr_mem_out(rs_wr_mem_out),
    .rs_cond_branch_out(rs_cond_branch_out),
    .rs_uncond_branch_out(rs_uncond_branch_out),
    .rs_avail_out(rs_avail_out),
    .rs_debug(rs_debug)
  );

  always #5 clock = ~clock;

  task print_status;
    for (int i = 0; i < `RS_SIZE; i++) begin
      $display("RS[%0d] avail=%b ready=%b opa=%h opb=%h", i, rs_avail_out[i], rs_ready_out[i], rs_opa_out[i], rs_opb_out[i]);
    end
    $display("---");
  endtask

  initial begin
    clock = 0;
    reset = 1;
    rs_load_in = 0;
    rs_free_in = '0;
    fu_busy = '0;
    rs_opa_valid = 0;
    rs_opb_valid = 0;
    rs_cdb_valid = 0;
    rs_cdb_in = 32'hAABBCCDD;
    rs_cdb_tag = 6'd2;
    rs_opa_in = 32'h00000002;
    rs_opb_in = 32'h00000003;
    rs_opa_select = OPA_IS_RS1;
    rs_opb_select = OPB_IS_RS2;
    rs_npc_in = 32'h00000010;
    rs_pc_in = 32'h00000004;
    rs_inst = '0;
    rs_alu_func_in = ALU_ADD;
    rd_mem = 0; wr_mem = 0; cond_branch = 0; uncond_branch = 0;
    rs_rob_tag = 6'd5;

    @(negedge clock);
    reset = 0;

    // Load RS[1]
    rs_fu_select_in = 2'd1;
    rs_load_in = 1;
    @(negedge clock);
    rs_load_in = 0;

    // Make OPA ready via CDB
    rs_cdb_valid = 1;
    rs_cdb_tag = 6'd2;
    @(negedge clock);
    rs_cdb_valid = 0;

    // Make OPB ready via CDB
    rs_cdb_valid = 1;
    rs_cdb_tag = 6'd3;
    rs_cdb_in = 32'hDEADBEEF;
    @(negedge clock);
    rs_cdb_valid = 0;

    // Entry should now be ready
    print_status();

    // Free only RS[1]
    rs_free_in = 3'b010;
    @(negedge clock);

    // Check RS[1] is free
    print_status();

    $finish;
  end
endmodule