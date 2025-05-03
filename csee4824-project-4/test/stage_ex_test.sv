// stage_ex self-test
`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module testbench;

    logic clk = 0;
    logic rst = 1;

    parameter real CLK_PERIOD = 10.0;
    always #(CLK_PERIOD/2.0) clk = ~clk;

    IS_EX_PACKET      is_ex_reg;
    logic             cdb_busy;

    logic             alu_busy;
    EX_CP_PACKET      ex_cp_packet;
    priv_addr_packet  priv_addr_out;
    assign cdb_busy = 1'b0;

    //DUT
    stage_ex dut (
        .clk              (clk),
        .rst              (rst),
        .is_ex_reg        (is_ex_reg),
        .cdb_packet_busy  (cdb_busy),

        .alu_busy         (alu_busy),
        .ex_cp_packet     (ex_cp_packet),
        .priv_addr_out    (priv_addr_out)
    );

    //  Helper tasks
    task automatic check_outputs (
        input [31:0] exp_value,
        input  [4:0] exp_tag,
        input  string name
    );
        if (   ex_cp_packet.valid !== 1
            || ex_cp_packet.value !== exp_value
            || ex_cp_packet.rob_tag !== exp_tag )
        begin
            $display("@@@ FAILED: %s", name);
            $display("      value exp/got = 0x%h / 0x%h",
                     exp_value, ex_cp_packet.value);
            $display("      tag   exp/got = %0d / %0d",
                     exp_tag,   ex_cp_packet.rob_tag);
        end
        else
            $display("@@@ PASSED: %s", name);
    endtask


    task automatic run_basic_test (
        input  [31:0] opa, opb,
        input  ALU_FUNC func,
        input  [31:0] exp_value,
        input  string  name,
        input  [4:0]   tag = 5'd1
    );
        // drive operands on falling edge,
        // sample on next rising edge
        @(negedge clk);
            is_ex_reg            = '0;
            is_ex_reg.OPA        = opa;
            is_ex_reg.OPB        = opb;
            is_ex_reg.alu_func   = func;
            is_ex_reg.rob_tag    = tag;
            is_ex_reg.issue_valid= 1'b1;
        @(posedge clk);
            check_outputs(exp_value, tag, name);
    endtask

    initial begin
        $display("==== stage_ex test suite ====");

        // global reset
        #(CLK_PERIOD);      // allow some time with rst=1
        rst = 0;
        @(posedge clk);
-
        run_basic_test(10, 5, ALU_ADD , 15        , "ALU ADD");
        run_basic_test(10, 5, ALU_SUB , 5         , "ALU SUB");
        run_basic_test(32'hFF00, 32'h0F0F, ALU_AND, 32'h0F00  , "ALU AND");
        run_basic_test(32'hFF00, 32'h0F0F, ALU_OR , 32'hFF0F  , "ALU OR");  // FIXED

        run_basic_test(-1 , 1, ALU_SLT , 1 , "ALU SLT (signed)"  );
        run_basic_test(32'hFFFF_FFFF, 1, ALU_SLTU, 0, "ALU SLTU (unsigned)");

        run_basic_test(32'h8000_0000, 2, ALU_SRA, 32'hE000_0000, "ALU SRA");
        run_basic_test(32'h8000_0000, 2, ALU_SRL, 32'h2000_0000, "ALU SRL");
        run_basic_test(1, 4, ALU_SLL, 16, "ALU SLL");
        @(negedge clk);
            is_ex_reg = '0;
            is_ex_reg.OPA         = 0;
            is_ex_reg.OPB         = 0;      
            is_ex_reg.alu_func    = ALU_ADD;
            is_ex_reg.issue_valid = 1;
            is_ex_reg.rob_tag     = 5'd7;
            // insert U-type imm into raw instr
            is_ex_reg.inst.u.imm  = 20'hABCD0;
        @(posedge clk);
            check_outputs(32'hABCD_0000, 5'd7, "Immediate mux U-type");
        @(negedge clk);
            is_ex_reg = '0;
            is_ex_reg.OPA           = 5;
            is_ex_reg.OPB           = 5;
            is_ex_reg.inst.b.funct3 = 3'b000;   // BEQ
            is_ex_reg.alu_func      = ALU_ADD;  // ALU still adds for next-PC
            is_ex_reg.issue_valid   = 1;
            is_ex_reg.rob_tag       = 5'd8;
        @(posedge clk);
            // When BEQ is taken, ex_cp_packet should be valid
            // and carry the ALU result (5+5 = 10)
            if ( ex_cp_packet.valid && ex_cp_packet.value == 32'd10 )
                $display("@@@ PASSED: Branch BEQ taken");
            else
                $display("@@@ FAILED: Branch BEQ taken");
        run_basic_test(2, 3, ALU_MUL, 6, "Multiplier 2*3", 5'd9);

        // NOP handling 
        @(negedge clk);
            is_ex_reg = '0;          // no valid issue
        @(posedge clk);
            if (ex_cp_packet.valid)
                $display("@@@ FAILED: NOP handling (packet emitted)");
            else
                $display("@@@ PASSED: NOP handling");

        // Edge cases 
        run_basic_test(-32, 4, ALU_ADD, -28, "Edge: negative + positive");
        run_basic_test(0   , 0, ALU_ADD, 0  , "Edge: zero inputs");

        $display("==== All tests complete ====");
        $finish;
    end

endmodule
