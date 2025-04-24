`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

// TO RUN: vcs -sverilog -full64     +incdir+./verilog     -debug_acc+all+dmptf     -debug_region+cell+encrypt     -timescale=1ns/1ps     +v2k     ./verilog/sys_defs.svh     ./verilog/ISA.svh     ./verilog/stage_ex.sv     ./verilog/cdb.sv     ./verilog/reservation_station.sv    ./test/rs_cdb_fu_integration_test.sv

module rs_cdb_fu_integration_test;
    // Clock and reset
    logic clock, reset;

    // Reservation Station signals
    logic [31:0] rs_npc_in;
    logic [31:0] rs_opa_in, rs_opb_in;
    logic [`ROB_TAG_BITS-1:0] rs_rob_tag;
    logic rs_opa_valid, rs_opb_valid;
    logic rs_load_in, rs_free_in;
    logic fu_busy;
    logic rd_mem, wr_mem;

    // RS outputs
    logic rs_ready_out;
    logic [31:0] rs_opa_out, rs_opb_out;
    logic [`ROB_TAG_BITS-1:0] rs_tag_out;
    ALU_FUNC rs_alu_func_in, rs_alu_func_out;
    logic [31:0] rs_npc_out;
    logic rs_rd_mem_out, rs_wr_mem_out;
    logic rs_avail_out;
    logic [74:0] rs_debug;

    // CDB signals
    logic [31:0] cdb_data;
    logic [4:0] cdb_tag;
    logic cdb_valid;
    CDB_PACKET cdb_packet;

    // Stage_EX signals
    logic cdb_packet_busy;
    logic alu_busy;
    IS_EX_PACKET is_ex_packet;
    ID_EX_PACKET id_ex_packet;
    EX_CP_PACKET ex_cp_packet;
    priv_addr_packet priv_addr_out;

    // Stage_EX instantiation
    stage_ex u_stage_ex (
        .clk(clock),
        .rst(reset),
        .is_ex_reg(is_ex_packet),
        .cdb_packet_busy(cdb_packet_busy),
        .alu_busy(alu_busy),
        .ex_cp_packet(ex_cp_packet),
        .priv_addr_out(priv_addr_out)
    );

    // CDB instantiation
    cdb u_cdb (
        .clock(clock),
        .reset(reset),
        .cdb_in(cdb_packet),
        .cdb_data(cdb_data),
        .cdb_tag(cdb_tag),
        .cdb_valid(cdb_valid)
    );

    // Reservation Station instantiation
    reservation_station u_rs (
        .clock(clock),
        .reset(reset),
        .rs_npc_in(rs_npc_in),
        .rs_alu_func_in(rs_alu_func_in),
        .rd_mem(rd_mem),
        .wr_mem(wr_mem),
        .rs_rob_tag(rs_rob_tag),
        .rs_cdb_in(cdb_data),
        .rs_cdb_tag(cdb_tag),
        .rs_cdb_valid(cdb_valid),
        .rs_opa_in(rs_opa_in),
        .rs_opb_in(rs_opb_in),
        .rs_opa_valid(rs_opa_valid),
        .rs_opb_valid(rs_opb_valid),
        .rs_load_in(rs_load_in),
        .fu_busy(fu_busy),
        .rs_free_in(rs_free_in),
        .rs_ready_out(rs_ready_out),
        .rs_opa_out(rs_opa_out),
        .rs_opb_out(rs_opb_out),
        .rs_tag_out(rs_tag_out),
        .rs_alu_func_out(rs_alu_func_out),
        .rs_npc_out(rs_npc_out),
        .rs_rd_mem_out(rs_rd_mem_out),
        .rs_wr_mem_out(rs_wr_mem_out),
        .rs_avail_out(rs_avail_out),
        .rs_debug(rs_debug)
    );

    // Fix ID_EX_PACKET assignment
    assign id_ex_packet = '{
        inst: 32'h0,
        PC: 32'h0,              // Add missing field
        NPC: rs_npc_out,        // Add missing field
        rs1_value: rs_opa_out,
        rs2_value: rs_opb_out,
        opa_select: OPA_IS_RS1,
        opb_select: OPB_IS_RS2,
        dest_reg_idx: 5'h0,     // Add missing field
        alu_func: rs_alu_func_out,
        rd_mem: rs_rd_mem_out,
        wr_mem: rs_wr_mem_out,
        cond_branch: 1'b0,      // Add missing field
        uncond_branch: 1'b0,    // Add missing field
        halt: 1'b0,             // Add missing field
        illegal: 1'b0,          // Add missing field
        csr_op: 1'b0,           // Add missing field
        valid: rs_ready_out
    };

    // Fix IS_EX_PACKET assignment
    assign is_ex_packet = '{
        OPA: rs_opa_out,
        OPB: rs_opb_out,
        rob_tag: rs_tag_out,
        alu_func: rs_alu_func_out,  // Make sure this propagates
        issue_valid: rs_ready_out && !fu_busy,  // Only valid if RS ready and FU not busy
        rd_mem: rs_rd_mem_out,
        wr_mem: rs_wr_mem_out,
        NPC: rs_npc_out,
        inst: 32'h0,
        default: '0
    };

    assign cdb_packet = '{
        valid: ex_cp_packet.done,
        value: ex_cp_packet.value,
        tag: rs_tag_out
    };

    // Clock generator
    always #5 clock = ~clock;

    initial begin
        // Initialize signals
        clock = 0;
        reset = 1;
        rs_load_in = 0;
        rs_free_in = 0;
        rs_opa_valid = 0;
        rs_opb_valid = 0;
        rd_mem = 0;
        wr_mem = 0;
        rs_npc_in = 32'h0;
        rs_alu_func_in = ALU_ADD;
        rs_rob_tag = 0;
        fu_busy = 0;
        cdb_packet_busy = 0;
        
        // Reset sequence with verification
        repeat(5) @(posedge clock);
        reset = 0;
        repeat(3) @(posedge clock);  // Add more cycles after reset
        
        // Verify RS is in clean state
        $display("Post-reset state: rs_avail=%b, rs_ready=%b, rs_debug=%h",
                 rs_avail_out, rs_ready_out, rs_debug);
        
        if (!rs_avail_out) begin
            $display("ERROR: RS not available after reset");
            $finish;
        end

        // Ensure all modules are ready
        @(posedge clock);
        if (fu_busy || cdb_packet_busy) begin
            $display("ERROR: FU or CDB busy after reset");
            $finish;
        end
        // Test basic ALU operations
        
        // Test ADD
        test_alu_op(ALU_ADD, 32'h5, 32'h3, 32'h8, "ADD");
        
        // Test SUB
        test_alu_op(ALU_SUB, 32'h8, 32'h3, 32'h5, "SUB");
        
        // Test AND
        test_alu_op(ALU_AND, 32'hF0, 32'h0F, 32'h00, "AND");
        
        // Test OR
        test_alu_op(ALU_OR, 32'hF0, 32'h0F, 32'hFF, "OR");
        
        // Test XOR
        test_alu_op(ALU_XOR, 32'hFF, 32'h0F, 32'hF0, "XOR");
        
        // Test SLL
        test_alu_op(ALU_SLL, 32'h1, 32'h4, 32'h10, "SLL");
        
        // Test SRL
        test_alu_op(ALU_SRL, 32'h10, 32'h4, 32'h1, "SRL");

        // Test data dependencies
        test_data_dependency();

        // Test multiple operations
        test_multiple_ops();
    end

    task test_alu_op(
        input ALU_FUNC alu_func,
        input logic [31:0] opa,
        input logic [31:0] opb,
        input logic [31:0] expected_result,
        input string op_name
    );
        int timeout;
        
        // Set up the operation
        @(posedge clock);
        rs_load_in = 1;
        rs_opa_in = opa;
        rs_opb_in = opb;
        rs_rob_tag = rs_rob_tag + 1;
        rs_alu_func_in = alu_func;  // Make sure ALU function is set correctly
        rs_opa_valid = 1;
        rs_opb_valid = 1;
        fu_busy = 0;
        cdb_packet_busy = 0;
        
        // Debug print initial values
        $display("\nStarting %s operation test", op_name);
        $display("DEBUG: Setting opa=%h, opb=%h, alu_func=%h", opa, opb, alu_func);
        
        @(posedge clock);
        rs_load_in = 0;
        
        // Wait for one cycle to let RS latch the values
        @(posedge clock);
        
        // Verify inputs were latched
        if (rs_alu_func_out !== alu_func) begin
            $display("ERROR: ALU function not latched. Expected %h, got %h", 
                     alu_func, rs_alu_func_out);
            $finish;
        end

        // Wait for reservation station to be ready
        timeout = 0;
        while (!rs_ready_out && timeout < 10) begin
            @(posedge clock);
            $display("DEBUG: Cycle %0d:", timeout);
            $display("  rs_ready=%b, rs_alu_func=%h", rs_ready_out, rs_alu_func_out);
            $display("  is_ex_packet.issue_valid=%b", is_ex_packet.issue_valid);
            $display("  is_ex_packet.alu_func=%h", is_ex_packet.alu_func);
            $display("  fu_busy=%b, cdb_packet_busy=%b", fu_busy, cdb_packet_busy);
            timeout++;
        end
        
        if (timeout >= 10) begin
            $display("Test Failed: %s operation - Timeout waiting for rs_ready_out", op_name);
            $display("DEBUG: rs_ready_out=%b, rs_opa_valid=%b, rs_opb_valid=%b", 
                     rs_ready_out, rs_opa_valid, rs_opb_valid);
            $display("DEBUG: fu_busy=%b, rs_debug=%h", fu_busy, rs_debug);
            $finish;
        end
        
        // Debug prints
        $display("DEBUG: opa=%h, opb=%h, alu_func=%d", opa, opb, alu_func);
        $display("DEBUG: rs_opa_out=%h, rs_opb_out=%h", rs_opa_out, rs_opb_out);
        $display("DEBUG: cdb_valid=%b, cdb_data=%h", cdb_valid, cdb_data);
        
        if (!cdb_valid || cdb_data !== expected_result) begin
            $display("Test Failed: %s operation", op_name);
            $display("Expected: %0h, Got: %0h", expected_result, cdb_data);
            $finish;
        end
        
        $display("Test Passed: %s operation", op_name);
        repeat(2) @(posedge clock);
    endtask

    // Task for testing data dependencies
    task test_data_dependency();
        // First instruction
        rs_load_in = 1;
        rs_opa_in = 32'd10;
        rs_opb_in = 32'd20;
        rs_rob_tag = 6'd1;
        rs_alu_func_in = ALU_ADD;
        rs_opa_valid = 1;
        rs_opb_valid = 1;
        @(posedge clock);

        // Second instruction (dependent on first)
        rs_rob_tag = 6'd2;
        rs_opa_valid = 0;
        rs_opb_in = 32'd5;
        rs_opb_valid = 1;
        @(posedge clock);
        rs_load_in = 0;

        // Wait for first result
        repeat(3) @(posedge clock);
        
        if (!rs_ready_out) begin
            $display("Test Failed: Data dependency not resolved");
            $finish;
        end
        $display("Test Passed: Data dependency resolved");
    endtask

    // Task for testing multiple operations
    task test_multiple_ops();
        for (int i = 0; i < 4; i++) begin
            rs_load_in = 1;
            rs_opa_in = 32'd1 << i;
            rs_opb_in = 32'd1 << (i + 1);
            rs_rob_tag = i + 10;
            rs_alu_func_in = ALU_ADD;
            rs_opa_valid = 1;
            rs_opb_valid = 1;
            @(posedge clock);
        end
        rs_load_in = 0;

        repeat(8) @(posedge clock);
        
        if (!rs_avail_out) begin
            $display("Test Failed: Multiple operations not completed");
            $finish;
        end
        $display("Test Passed: Multiple operations completed");
    endtask

endmodule