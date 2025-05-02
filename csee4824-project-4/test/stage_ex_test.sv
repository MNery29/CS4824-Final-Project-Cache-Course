`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module testbench;

    logic clk = 0, rst = 1;
    parameter real CLK_PERIOD = 10.0;
    always #(CLK_PERIOD/2.0) clk = ~clk;

    IS_EX_PACKET is_ex_reg[2:0];
    logic cdb_busy = 1'b0;

    EX_CP_PACKET ex_cp_packet;
    priv_addr_packet priv_addr_out;
    logic [2:0] fu_busy_signals;
    logic take_conditional, mult_done;

    stage_ex dut (
        .clk(clk),
        .rst(rst),
        .is_ex_reg(is_ex_reg),
        .cdb_packet_busy(cdb_busy),
        .take_conditional(take_conditional),
        .ex_cp_packet(ex_cp_packet),
        .priv_addr_out(priv_addr_out),
        .fu_busy_signals(fu_busy_signals),
        .mult_done(mult_done)
    );

    task automatic check_outputs(
        input [31:0] exp_value,
        input [4:0] exp_tag,
        input string name
    );
        if (ex_cp_packet.valid !== 1 || ex_cp_packet.value !== exp_value || ex_cp_packet.rob_tag !== exp_tag) begin
            $display("@@@ FAILED: %s", name);
            $display("      value exp/got = 0x%h / 0x%h", exp_value, ex_cp_packet.value);
            $display("      tag   exp/got = %0d / %0d", exp_tag, ex_cp_packet.rob_tag);
        end else begin
            $display("@@@ PASSED: %s", name);
        end
    endtask

    task automatic run_basic_test(
        input int fu_idx,
        input [31:0] opa, opb,
        input ALU_FUNC func,
        input [31:0] exp_value,
        input string name,
        input [4:0] tag = 5'd1
    );
        @(negedge clk);
        foreach (is_ex_reg[i]) is_ex_reg[i] = '0;

        is_ex_reg[fu_idx].OPA = opa;
        is_ex_reg[fu_idx].OPB = opb;
        is_ex_reg[fu_idx].alu_func = func;
        is_ex_reg[fu_idx].issue_valid = 1;
        is_ex_reg[fu_idx].rob_tag = tag;
        is_ex_reg[fu_idx].opa_select = OPA_IS_RS1;
        is_ex_reg[fu_idx].opb_select = OPB_IS_RS2;

        @(posedge clk);

        // Wait for result if it's the multiplier
        if (fu_idx == 2) begin
            int timeout = 20;
            while (!ex_cp_packet.valid && timeout > 0) begin
                @(posedge clk);
                timeout--;
            end
        end

        check_outputs(exp_value, tag, name);
    endtask

    always @(posedge clk) begin
        if (!rst) begin
            $display("Time: %0t | mult_done=%b | valid=%b | result=0x%h | tag=%0d",
                    $time, mult_done, ex_cp_packet.valid, ex_cp_packet.value, ex_cp_packet.rob_tag);

            if (is_ex_reg[2].issue_valid) begin
                $display("    MULT STAGE ACTIVE:");
                $display("      OPA  = 0x%h", is_ex_reg[2].OPA);
                $display("      OPB  = 0x%h", is_ex_reg[2].OPB);
                $display("      FUNC = %0d", is_ex_reg[2].alu_func);
            end
        end
    end

    initial begin
        $display("==== stage_ex test suite ====");
        #(CLK_PERIOD);
        rst = 0;
        @(posedge clk);

        // ALU0 basic ops
        run_basic_test(0, 10, 5, ALU_ADD, 15, "ALU0 ADD");
        run_basic_test(0, 10, 5, ALU_SUB, 5, "ALU0 SUB");
        run_basic_test(0, 32'hFF00, 32'h0F0F, ALU_AND, 32'h0F00, "ALU0 AND");

        // ALU1 test
        run_basic_test(1, 32'h8000_0000, 2, ALU_SRA, 32'hE000_0000, "ALU1 SRA");

        // Multiplier test
        run_basic_test(2, 2, 3, ALU_MUL, 6, "MULT 2*3", 5'd9);

        // Edge case
        run_basic_test(0, -32, 4, ALU_ADD, -28, "Edge: negative + positive");

        $display("==== All tests complete ====");
        $finish;
    end

endmodule
