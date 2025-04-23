`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module rs_cdb_fu_integration_test;
// Clock and reset
logic clock, reset;

// Reservation Station signals
logic [31:0] rs_npc_in, rs_opa_in, rs_opb_in;
logic [`ROB_TAG_BITS-1:0] rs_rob_tag;
logic rs_opa_valid, rs_opb_valid, rs_load_in, rs_use_enable, rs_free_in;
logic rs_rd_mem, rs_wr_mem;
logic [31:0] rs_rd_mem_out, rs_wr_mem_out, rs_npc_out;
ALU_FUNC rs_alu_func_in;  // Added missing signal

// CDB signals 
logic [31:0] cdb_value, rs_cdb_value;  // Added rs_cdb_value
logic [`ROB_TAG_BITS-1:0] cdb_tag, rs_cdb_tag;  // Added rs_cdb_tag
logic cdb_valid, rs_cdb_valid;  // Added rs_cdb_valid

  // Functional unit signals
    logic [31:0] fu_result;
    logic fu_done;

    // RS outputs
    logic rs_ready_out, rs_avail_out;
    logic [31:0] rs_opa_out, rs_opb_out;
    logic [`ROB_TAG_BITS-1:0] rs_tag_out; 
    ALU_FUNC rs_alu_func_out;

  // Instantiate stage_ex (functional unit)
  stage_ex u_stage_ex (
    .clk(clock),
    .rst(reset),
    .id_ex_reg('{
        opa_select: OPA_IS_RS1,
        opb_select: OPB_IS_RS2,
        inst: 32'h0,
        alu_func: rs_alu_func_out,
        rs1_value: rs_opa_out,
        rs2_value: rs_opb_out,
        valid: 1'b1
    }),
    .is_ex_reg('{
        OPA: rs_opa_out,
        OPB: rs_opb_out, 
        rob_tag: rs_tag_out,
        alu_func: rs_alu_func_out,
        issue_valid: rs_ready_out,
        rd_mem: rs_rd_mem_out,
        wr_mem: rs_wr_mem_out,
        NPC: rs_npc_out,
        inst: 32'h0
    }),
    .ex_cp_packet('{
        value: fu_result,
        rob_tag: rs_tag_out,
        valid: fu_done,
        done: fu_done
    })
  );

  // Instantiate CDB
  cdb u_cdb (
    .clock(clock),
    .reset(reset),
    .cdb_in('{
        valid: fu_done,
        value: fu_result,
        tag: rs_tag_out
    }),
    .cdb_data(cdb_value),
    .cdb_tag(cdb_tag),
    .cdb_valid(cdb_valid)
  );

  // Instantiate Reservation Station
  reservation_station u_rs (
    .clock(clock),
    .reset(reset),
    .rs_npc_in(rs_npc_in),
    .rs_alu_func_in(rs_alu_func_out),
    .rd_mem(rs_rd_mem),
    .wr_mem(rs_wr_mem),
    .rs_rob_tag(rs_rob_tag),
    .rs_cdb_in(cdb_value),
    .rs_cdb_tag(cdb_tag),
    .rs_cdb_valid(cdb_valid),
    .rs_opa_in(rs_opa_in),
    .rs_opb_in(rs_opb_in),
    .rs_opa_valid(rs_opa_valid),
    .rs_opb_valid(rs_opb_valid),
    .rs_load_in(rs_load_in),
    .rs_use_enable(rs_use_enable),
    .rs_free_in(rs_free_in),
    .rs_ready_out(rs_ready_out),
    .rs_opa_out(rs_opa_out),
    .rs_opb_out(rs_opb_out),
    .rs_tag_out(rs_tag_out),
    .rs_alu_func_out(rs_alu_func_out),
    .rs_avail_out(rs_avail_out)
  );

  // Clock generator
  always #5 clock = ~clock;

  // Test scenarios
  initial begin
    // Initialize signals
    clock = 0;
    reset = 1;
    rs_load_in = 0;
    rs_use_enable = 0;
    rs_free_in = 0;
    rs_opa_valid = 0;
    rs_opb_valid = 0;
    rs_rd_mem = 0;
    rs_wr_mem = 0;

    @(posedge clock);
    reset = 0;

    // Test Case 1: Basic ALU Operation
    // Load ADD instruction with ready operands
    rs_load_in = 1;
    rs_opa_in = 32'd10;
    rs_opb_in = 32'd20;
    rs_rob_tag = 6'd1;
    rs_alu_func_in = ALU_ADD;
    rs_opa_valid = 1;
    rs_opb_valid = 1;
    @(posedge clock);
    rs_load_in = 0;

    // Issue instruction
    rs_use_enable = 1;
    @(posedge clock);
    rs_use_enable = 0;

    // Wait for result and verify
    @(posedge clock);
    if (!cdb_valid || cdb_value != 32'd30) begin
      $display("Test 1 Failed: ADD result incorrect");
      $finish;
    end

    // Test Case 2: CDB Broadcast and RS Update
    // Load MUL instruction with one pending operand
    rs_load_in = 1;
    rs_opa_in = 32'd5; 
    rs_opb_in = 32'd0;  // Tag will be updated by CDB
    rs_rob_tag = 6'd2;
    rs_alu_func_in = ALU_MUL;
    rs_opa_valid = 1;
    rs_opb_valid = 0;  // Waiting for operand
    @(posedge clock);

    // Simulate CDB broadcast of pending operand
    rs_cdb_tag = 6'd2;
    rs_cdb_valid = 1;
    rs_cdb_value = 32'd6;
    @(posedge clock);

    // Check if RS captured the broadcast value
    if (!rs_ready_out) begin
      $display("Test 2 Failed: RS did not capture CDB value");
      $finish;
    end

    $display("All tests passed!");
    $finish;
  end

endmodule