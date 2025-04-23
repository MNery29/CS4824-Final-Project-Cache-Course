`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

// to run:
// vcs -sverilog -timescale=1ns/1ps -full64 -debug_access+all \
//    verilog/stage_id.sv verilog/map_table.sv verilog/reorder_buffer.sv \
//    test/id_rob_mp_test.sv \
//    -o simv_id_map_rob


module testbench;

  logic clock, reset;

  // Inputs to stage_id
  IF_ID_PACKET if_id_reg;
  logic cdb_valid;
  logic [`ROB_TAG_BITS-1:0] cdb_tag;
  logic [31:0] cdb_value;
  logic mt_retire_entry;
  logic rs1_issue, rs1_clear;
  logic rob_retire_entry, rob_clear;
  logic [4:0] rob_dest_reg;
  logic [31:0] rob_to_regfile_value;
  logic rob_regfile_valid;
  logic lsq_free;

  // Outputs from stage_id
  logic [31:0] opA, opB;
  logic [`ROB_TAG_BITS-1:0] output_tag;
  logic [11:0] rob_pointers_debug;
  ALU_OPA_SELECT opa_select;
  ALU_OPB_SELECT opb_select;
  logic has_dest_reg;
  logic [4:0] dest_reg_idx;
  ALU_FUNC alu_func_out;
  ROB_RETIRE_PACKET rob_retire_out;
  logic rd_mem_out, wr_mem_out;
  LSQ_PACKET lsq_packet;

  // Internal wires for integration
  logic [4:0] rs1_addr, rs2_addr, dest_reg_idx_id;
  logic [6:0] rs1_tag, rs2_tag;
  logic [4:0] regfile_rs1_addr, regfile_rs2_addr;
  logic [4:0] tag_in, retire_addr, retire_tag;
  logic load_entry, read_cdb, retire_entry;

  DISPATCH_ROB_PACKET rob_dispatch_out;
  ROB_RETIRE_PACKET rob_retire_packet;
  logic [31:0] rob_to_rs_value1, rob_to_rs_value2;
  logic rob_full, rob_ready, rob_valid;

  // Clock generator
  always #5 clock = ~clock;

  // Instantiate DUTs
  stage_id u_stage_id (
    .clock(clock), .reset(reset), .if_id_reg(if_id_reg),
    .cdb_valid(cdb_valid), .cdb_tag(cdb_tag), .cdb_value(cdb_value),
    .mt_retire_entry(mt_retire_entry),
    .rs1_issue(rs1_issue), .rs1_clear(rs1_clear),
    .rob_retire_entry(rob_retire_entry), .rob_clear(rob_clear),
    .rob_dest_reg(rob_dest_reg), .rob_to_regfile_value(rob_to_regfile_value), .rob_regfile_valid(rob_regfile_valid),
    .lsq_free(lsq_free),
    .opA(opA), .opB(opB), .output_tag(output_tag),
    //.rob_pointers_debug(rob_pointers_debug),
    .opa_select(opa_select), .opb_select(opb_select),
    .has_dest_reg(has_dest_reg), .dest_reg_idx(dest_reg_idx),
    .alu_func_out(alu_func_out), .rob_retire_out(rob_retire_out),
    .rd_mem_out(rd_mem_out), .wr_mem_out(wr_mem_out), .lsq_packet(lsq_packet)
  );

  map_table u_map_table (
    .reset(reset), .clock(clock),
    .rs1_addr(rs1_addr), .rs2_addr(rs2_addr), .r_dest(dest_reg_idx_id),
    .tag_in(tag_in), .load_entry(load_entry), .cdb_tag_in(cdb_tag), .read_cdb(read_cdb),
    .retire_addr(retire_addr), .retire_tag(retire_tag), .retire_entry(retire_entry),
    .rs1_tag(rs1_tag), .rs2_tag(rs2_tag),
    .regfile_rs1_addr(regfile_rs1_addr), .regfile_rs2_addr(regfile_rs2_addr)
  );

  reorder_buffer u_rob (
    .reset(reset), .clock(clock),
    .rob_dispatch_in('{default:0}),
    .rob_to_rs_read1(1'b0), .rob_read_tag1(5'b0),
    .rob_to_rs_read2(1'b0), .rob_read_tag2(5'b0),
    .rob_cdb_in('{default:0}), .retire_entry(1'b0), .rob_clear(1'b0),
    .store_retire(1'b0), .store_tag(1'b0),
    .rob_dispatch_out(rob_dispatch_out), .rob_retire_out(rob_retire_packet),
    .rob_to_rs_value1(rob_to_rs_value1), .rob_to_rs_value2(rob_to_rs_value2),
    .rob_full(rob_full), .rob_ready(rob_ready), .rob_valid(rob_valid), .rob_pointers(rob_pointers_debug)
  );

  // Test logic
  initial begin
    clock = 0;
    reset = 1;
    @(posedge clock);
    @(posedge clock);
    reset = 0;

    // Test 1: Register Rename
    if_id_reg.inst = 32'h00a00513; // addi a0, zero, 10 (opcode = 0x13)
    if_id_reg.valid = 1;
    if_id_reg.PC = 32'h0;
    if_id_reg.NPC = 32'h4;
    rs1_addr = 5'd0;
    rs2_addr = 5'd0;
    dest_reg_idx_id = 5'd10; // a0
    load_entry = 1;
    @(posedge clock);
    load_entry = 0;
    if_id_reg.valid = 0;

    // Check tag assignment and availability
    if (rs1_tag[4:0] == 5'd0 || !has_dest_reg) begin
      $display("@@@ Register rename failed");
      $finish;
    end

    // Test 2: Rollback on mispredict
    rob_clear = 1;
    @(posedge clock);
    rob_clear = 0;
    // Tag should be reset for dest_reg_idx_id
    retire_addr = 5'd10;
    retire_tag = 0;
    retire_entry = 1;
    @(posedge clock);
    retire_entry = 0;

    // Test 3: Map Table Read-after-Write Consistency
    if_id_reg.inst = 32'h00a00513; // addi a0, zero, 10 again
    if_id_reg.valid = 1;
    load_entry = 1;
    @(posedge clock);
    if_id_reg.valid = 0;
    load_entry = 0;

    if (rs1_tag == 0 || regfile_rs1_addr != 5'd0) begin
      $display("@@@ Map Table RAW consistency failed");
      $finish;
    end

    $display("@@@ Passed");
    $finish;
  end

endmodule
