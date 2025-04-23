`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"


module testbench_rob_rs;

    logic clock;
    logic reset;

    // Dispatch inputs
    DISPATCH_ROB_PACKET rob_dispatch_in;
    
    // CDB inputs
    CDB_ROB_PACKET rob_cdb_in;
    
    // Retire inputs
    logic retire_entry;
    logic rob_clear;

    // Reservation Station inputs
    logic [31:0] rs_npc_in;
    logic [31:0] rs_inst_in;
    ALU_FUNC rs_alu_func_in;
    logic [`ROB_TAG_BITS-1:0] rs_rob_tag;
    logic [31:0] rs_cdb_in;
    logic [`ROB_TAG_BITS-1:0] rs_cdb_tag;
    logic rs_cdb_valid;
    logic [31:0] rs_opa_in, rs_opb_in;
    logic rs_opa_valid, rs_opb_valid;
    logic rs_load_in;
    logic rs_use_enable;
    logic rs_free_in;

    // Wires
    ROB_DISPATCH_PACKET rob_dispatch_out;
    ROB_RETIRE_PACKET rob_retire_out;
    logic [31:0] rob_to_rs_value1, rob_to_rs_value2;
    logic rob_full;
    logic rob_ready;
    logic rob_valid;
    logic [11:0] rob_pointers;

    logic rs_ready_out;
    logic [31:0] rs_opa_out, rs_opb_out;
    logic [`ROB_TAG_BITS-1:0] rs_tag_out;
    ALU_FUNC rs_alu_func_out;
    logic [31:0] rs_npc_out, rs_inst_out;
    logic rs_avail_out;
    logic [74:0] rs_debug;

    // Instantiate the modules
    reorder_buffer u_rob (
        .reset(reset),
        .clock(clock),
        .rob_dispatch_in(rob_dispatch_in),
        .rob_to_rs_read1(1'b0),
        .rob_read_tag1(5'b0),
        .rob_to_rs_read2(1'b0),
        .rob_read_tag2(5'b0),
        .rob_cdb_in(rob_cdb_in),
        .retire_entry(retire_entry),
        .rob_clear(rob_clear),
        .rob_dispatch_out(rob_dispatch_out),
        .rob_retire_out(rob_retire_out),
        .rob_to_rs_value1(rob_to_rs_value1),
        .rob_to_rs_value2(rob_to_rs_value2),
        .rob_full(rob_full),
        .rob_ready(rob_ready),
        .rob_valid(rob_valid),
        .rob_pointers(rob_pointers)
    );

    reservation_station u_rs (
        .rs_npc_in(rs_npc_in),
        .rs_inst_in(rs_inst_in),
        .rs_alu_func_in(rs_alu_func_in),
        .rs_rob_tag(rs_rob_tag),
        .rs_cdb_in(rs_cdb_in),
        .rs_cdb_tag(rs_cdb_tag),
        .rs_cdb_valid(rs_cdb_valid),
        .rs_opa_in(rs_opa_in),
        .rs_opb_in(rs_opb_in),
        .rs_opa_valid(rs_opa_valid),
        .rs_opb_valid(rs_opb_valid),
        .rs_load_in(rs_load_in),
        .rs_use_enable(rs_use_enable),
        .rs_free_in(rs_free_in),
        .reset(reset),
        .clock(clock),
        .rs_ready_out(rs_ready_out),
        .rs_opa_out(rs_opa_out),
        .rs_opb_out(rs_opb_out),
        .rs_tag_out(rs_tag_out),
        .rs_alu_func_out(rs_alu_func_out),
        .rs_npc_out(rs_npc_out),
        .rs_inst_out(rs_inst_out),
        .rs_avail_out(rs_avail_out),
        .rs_debug(rs_debug)
    );

    // Clock generator
    always #5 clock = ~clock;

    // Testbench logic
    logic expected_rs_ready;
    logic expected_rs_avail;
    logic [31:0] expected_rs_opa, expected_rs_opb;
    logic [`ROB_TAG_BITS-1:0] expected_rs_tag;
    logic broadcast_seen;

    logic prev_rs_free; //to account for non-instant changes in stations

    always @(posedge clock) begin 
        prev_rs_free <= rs_free_in; //to sync up timing check.
    end

   logic [1:0] rs_free_shift;

    always @(posedge clock) begin
        rs_free_shift <= {rs_free_shift[0], rs_free_in}; // Shift register

        if (reset) begin
            expected_rs_ready = 0;
            expected_rs_avail = 1;
            expected_rs_opa = 0;
            expected_rs_opb = 0;
            expected_rs_tag = 0;
            broadcast_seen = 0;
            rs_free_shift = 2'b00;
        end else begin
            if (rs_load_in) begin
                expected_rs_avail = 0;
                expected_rs_opa = rs_opa_valid ? rs_opa_in : 32'b0;
                expected_rs_opb = rs_opb_valid ? rs_opb_in : 32'b0;
                expected_rs_tag = rs_rob_tag;
                broadcast_seen = 0;
            end

            if (rs_cdb_valid && !broadcast_seen) begin
                expected_rs_opb = rs_cdb_in;
                broadcast_seen = 1;
            end

            // Only clear after the RS had a full cycle to reset
            if (rs_free_shift == 2'b10) begin
                expected_rs_avail = 1;
                expected_rs_ready = 0;
                expected_rs_opa = 0;
                expected_rs_opb = 0;
            end

            // Set ready flag based on RS state
            expected_rs_ready = u_rs.rs_debug[5] &&  // InUse
                                u_rs.rs_debug[43] && // opa_valid
                                u_rs.rs_debug[11];   // opb_valid

            expected_rs_avail = u_rs.rs_avail_out;
        end
    end

    //reset toggle check 
    logic after_reset; // to check asserts don't start until after reset

    // Track if we've seen a broadcast yet
    logic broadcast_seen;
    //assert checks

    always @(posedge clock) begin
        if (after_reset) begin
            // RS availability
            if (u_rs.rs_avail_out !== expected_rs_avail) begin
            $fatal("T=%0t | RS avail mismatch: got %b, expected %b", $time, u_rs.rs_avail_out, expected_rs_avail);
            end

            // RS readiness
            if (u_rs.rs_ready_out !== expected_rs_ready) begin
            $fatal("T=%0t | RS ready mismatch: got %b, expected %b", $time, u_rs.rs_ready_out, expected_rs_ready);
            end

            // RS tag
            if (!expected_rs_avail && u_rs.rs_tag_out !== expected_rs_tag) begin
            $fatal("T=%0t | RS tag mismatch: got %0d, expected %0d", $time, u_rs.rs_tag_out, expected_rs_tag);
            end

            // Operand A value
            if (!expected_rs_avail && expected_rs_ready &&
                (u_rs.rs_opa_out !== expected_rs_opa)) begin
            $fatal("T=%0t | RS OPA mismatch: got 0x%h, expected 0x%h", $time, u_rs.rs_opa_out, expected_rs_opa);
            end

            // Operand B value
            if (!expected_rs_avail && expected_rs_ready &&
                (u_rs.rs_opb_out !== expected_rs_opb)) begin
            $fatal("T=%0t | RS OPB mismatch: got 0x%h, expected 0x%h", $time, u_rs.rs_opb_out, expected_rs_opb);
            end

            if (!expected_rs_avail) begin
                if (u_rs.rs_tag_out !== expected_rs_tag) begin
                    $fatal("T=%0t | RS tag mismatch: got %0d, expected %0d", 
                        $time, u_rs.rs_tag_out, expected_rs_tag);
                end
            end
        end

        // Optional display for cycle summary
        $display("T=%0t | rs_ready=%b rs_avail=%b | opa=0x%h opb=0x%h | tag=%0d",
                $time, u_rs.rs_ready_out, u_rs.rs_avail_out, 
                u_rs.rs_opa_out, u_rs.rs_opb_out, u_rs.rs_tag_out);
    end


      // Stimulus
    initial begin
        $display("Starting ROB + RS integration test...");
        clock = 0;

        // Sync up stimulus
        reset = 1;
        after_reset = 0;
        retire_entry = 0;
        rob_clear = 0;
        rs_load_in = 0;
        rs_use_enable = 0;
        rs_free_in = 0;
        rs_cdb_valid = 0;
        rob_dispatch_in = '{default:0};
        rob_cdb_in = '{default:0};

        // Wait for a few clock cycles
        repeat (2) @(posedge clock);
        reset = 0;
        after_reset = 1;

        // Cycle 1: Dispatch to ROB
        @(posedge clock);
        rob_dispatch_in.valid = 1;
        rob_dispatch_in.dest_reg = 5'd1;
        rob_dispatch_in.opcode = `RV32_OP_IMM;

        @(posedge clock);
        rob_dispatch_in.valid = 0;

        // Cycle 2: Load RS
        @(posedge clock);
        rs_npc_in = 32'h100;
        rs_inst_in = 32'hDEADBEEF;
        rs_alu_func_in = ALU_ADD;
        rs_rob_tag = rob_dispatch_out.tag;
        rs_opa_in = 32'hAAAA0000;
        rs_opa_valid = 1;
        rs_opb_in = {{(32-`ROB_TAG_BITS){1'b0}}, rob_dispatch_out.tag}; // Waiting on CDB
        rs_opb_valid = 0;
        rs_load_in = 1;

        @(posedge clock);
        rs_load_in = 0;

        // Wait 1 cycle
        @(posedge clock);

        // Cycle 4: Broadcast via CDB
        rob_cdb_in.valid = 1;
        rob_cdb_in.tag = rob_dispatch_out.tag;
        rob_cdb_in.value = 32'h12345678;
        rs_cdb_in = 32'h12345678;
        rs_cdb_tag = rob_dispatch_out.tag;
        rs_cdb_valid = 1;

        @(posedge clock);
        rob_cdb_in.valid = 0;
        rs_cdb_valid = 0;

        // Wait one more cycle for RS to update
        @(posedge clock);

        // Free RS
        rs_free_in = 1;
        @(posedge clock);
        rs_free_in = 0;

        @(posedge clock);
        $display("Test complete.");
        $display("@@@ Passed");
        $finish;
    end


endmodule