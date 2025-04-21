// failing tests:
// -- ALU OR
// -- Branch BEQ Taken

//STILL IN PROGRESS!!!!!

`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module testbench;

  IS_EX_PACKET is_ex_reg;
  ID_EX_PACKET id_ex_reg;

  EX_MEM_PACKET ex_packet;
  CDB_PACKET cdb_out;

  stage_ex dut (
    .is_ex_reg(is_ex_reg),
    .id_ex_reg(id_ex_reg),
    .ex_packet(ex_packet),
    .cdb_out(cdb_out)
  );

  task check_outputs(input [31:0] expected_result, input [4:0] expected_tag, input string testname);
    begin
      if (ex_packet.alu_result !== expected_result || cdb_out.value !== expected_result || cdb_out.tag !== expected_tag) begin
        $display("@@@FAILED: %s", testname);
        $display("    Expected Result: 0x%h, Got: 0x%h", expected_result, ex_packet.alu_result);
        $display("    Expected Tag: %0d, Got: %0d", expected_tag, cdb_out.tag);
      end else begin
        $display("@@@PASSED: %s", testname);
      end
    end
  endtask

  task run_basic_test(
    input logic [31:0] opa, opb,
    input ALU_FUNC func,
    input [31:0] expected,
    input string testname,
    input logic [4:0] rob_tag = 5'd1
  );
    begin
      is_ex_reg = '0;
      id_ex_reg = '0;

      is_ex_reg.OPA = opa;
      is_ex_reg.OPB = opb;
      is_ex_reg.rob_tag = rob_tag;
      is_ex_reg.alu_func = func;
      is_ex_reg.issue_valid = 1'b1;
      is_ex_reg.inst = '{default:0};

      id_ex_reg.opa_select = OPA_IS_RS1;
      id_ex_reg.opb_select = OPB_IS_RS2;
      id_ex_reg.valid = 1'b1;

      #1;
      check_outputs(expected, rob_tag, testname);
    end
  endtask

  initial begin
    $display("==== stage_ex Full Test Suite ====");

    // --- ALU and logical ops
    run_basic_test(10, 5, ALU_ADD, 15, "ALU ADD");
    run_basic_test(10, 5, ALU_SUB, 5, "ALU SUB");
    run_basic_test(32'hFF00, 32'h0F0F, ALU_AND, 32'h0F00, "ALU AND");
    run_basic_test(32'hFF00, 32'h0F0F, ALU_OR,  32'hFFFF, "ALU OR");

    // --- Signed and unsigned comparison
    run_basic_test(-1, 1, ALU_SLT, 1, "ALU SLT (signed)");
    run_basic_test(32'hFFFFFFFF, 1, ALU_SLTU, 0, "ALU SLTU (unsigned)");

    // --- Shift ops
    run_basic_test(32'h80000000, 2, ALU_SRA, 32'hE0000000, "ALU SRA");
    run_basic_test(32'h80000000, 2, ALU_SRL, 32'h20000000, "ALU SRL");
    run_basic_test(1, 4, ALU_SLL, 16, "ALU SLL");

    // --- Memory flag propagation + unsigned bit
    is_ex_reg = '0;
    id_ex_reg = '0;
    is_ex_reg.OPA = 10;
    is_ex_reg.OPB = 20;
    is_ex_reg.alu_func = ALU_ADD;
    is_ex_reg.issue_valid = 1;
    is_ex_reg.rd_mem = 1;
    is_ex_reg.inst.r.funct3 = 3'b100;
    is_ex_reg.rob_tag = 5'd4;

    id_ex_reg.opa_select = OPA_IS_RS1;
    id_ex_reg.opb_select = OPB_IS_RS2;
    id_ex_reg.valid = 1;

    #1;
    check_outputs(30, 5'd4, "Memory Access Flags Test");
    if (ex_packet.rd_mem !== 1 || ex_packet.rd_unsigned !== 1) $display("@@@FAILED: mem flag/unsigned");

    // --- Immediate operand (U-type)
    is_ex_reg = '0;
    id_ex_reg = '0;
    id_ex_reg.opb_select = OPB_IS_U_IMM;
    id_ex_reg.opa_select = OPA_IS_RS1;
    id_ex_reg.inst.u.imm = 20'hABCD0;
    is_ex_reg.OPA = 32'd0;
    is_ex_reg.alu_func = ALU_ADD;
    is_ex_reg.issue_valid = 1;
    is_ex_reg.rob_tag = 5'd7;
    is_ex_reg.inst = id_ex_reg.inst;
    id_ex_reg.valid = 1;

    #1;
    check_outputs(32'hABCD0000, 5'd7, "Immediate mux U-type");

    // --- Conditional branch evaluation: BEQ
    is_ex_reg = '0;
    id_ex_reg = '0;
    is_ex_reg.OPA = 5;
    is_ex_reg.OPB = 5;
    is_ex_reg.inst.b.funct3 = 3'b000;  // BEQ
    is_ex_reg.issue_valid = 1;
    is_ex_reg.rob_tag = 5'd8;
    is_ex_reg.alu_func = ALU_ADD;

    id_ex_reg.inst = is_ex_reg.inst;
    id_ex_reg.opa_select = OPA_IS_RS1;
    id_ex_reg.opb_select = OPB_IS_RS2;
    id_ex_reg.cond_branch = 1;
    id_ex_reg.valid = 1;

    #1;
    if (ex_packet.take_branch !== 1) $display("@@@FAILED: Branch BEQ taken");
    else $display("@@@PASSED: Branch BEQ taken");

    // --- Multiplier test: 2 * 3
    is_ex_reg = '0;
    id_ex_reg = '0;
    is_ex_reg.OPA = 32'd2;
    is_ex_reg.OPB = 32'd3;
    is_ex_reg.rob_tag = 5'd9;
    is_ex_reg.alu_func = ALU_MUL;
    is_ex_reg.issue_valid = 1;
    id_ex_reg.opa_select = OPA_IS_RS1;
    id_ex_reg.opb_select = OPB_IS_RS2;
    id_ex_reg.valid = 1;

    // simulate multiple cycles to allow multiplier to finish
    repeat (6) #1;
    if (cdb_out.value !== 6) $display("@@@FAILED: Multiplier 2*3");
    else $display("@@@PASSED: Multiplier 2*3");

    // --- NOP / Invalid Instruction
    is_ex_reg = '0;
    id_ex_reg = '0;
    is_ex_reg.issue_valid = 0;
    id_ex_reg.valid = 0;
    #1;
    if (ex_packet.valid !== 0 && cdb_out.valid !== 0) $display("@@@FAILED: NOP handling");
    else $display("@@@PASSED: NOP handling");

    // --- Edge case: negative numbers
    run_basic_test(-32, 4, ALU_ADD, -28, "Edge Case: Negative + Positive");
    run_basic_test(0, 0, ALU_ADD, 0, "Edge Case: Zero inputs");

    $display("==== All Tests Complete ====");
    $finish;
  end

endmodule
