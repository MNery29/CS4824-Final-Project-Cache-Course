`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

`timescale 1ns/1ps

module testbench;

    logic clock, reset;
    logic rob_to_rs_read1, rob_to_rs_read2;
    logic [4:0] rob_read_tag1, rob_read_tag2;
    logic retire_entry, rob_clear;
    logic rob_full;
    logic [31:0] rob_to_rs_value1, rob_to_rs_value2;

    // DEBUG signals:
    logic [11:0] rob_pointers; 
    //logic [45:0] rob_debug[31:0];

    //INPUTS FROM DISPATCH TO ROB 
    // logic        dispatch_to_rob.dispatch_valid;
    // logic [4:0]  dispatch_to_rob.dispatch_dest;
    // logic [6:0]  dispatch_to_rob.dispatch_opcode;
    DISPATCH_ROB_PACKET dispatch_to_rob;


    // INPUTS FROM CDB TO ROB
    // logic        cdb_to_rob.cdb_valid;
    // logic [5:0]  cdb_to_rob.cdb_tag;
    // logic [31:0] cdb_to_rob.cdb_value;
    CDB_ROB_PACKET cdb_to_rob;

    // DUT output
    ROB_DISPATCH_PACKET rob_dispatch_out;
    ROB_RETIRE_PACKET   rob_retire_out;

    //outputs if the ROB has something to retire
    logic rob_ready;
    logic rob_valid;

    // Used to hold the ROB dispatch input for testbench 
    DISPATCH_ROB_PACKET rob_dispatch_in_temp;

    // Used to hold the CDB input for testbench
    CDB_ROB_PACKET rob_cdb_in_temp;

    // Clock Generation
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    // DUT
    reorder_buffer rob (
        // Standard control signals
        .reset(reset),
        .clock(clock),

        // RS reads
        .rob_to_rs_read1(rob_to_rs_read1),
        .rob_read_tag1(rob_read_tag1),
        .rob_to_rs_read2(rob_to_rs_read2),
        .rob_read_tag2(rob_read_tag2),

        // Outputs to RS
        .rob_to_rs_value1(rob_to_rs_value1),
        .rob_to_rs_value2(rob_to_rs_value2),

        // Retire and flush
        .retire_entry(retire_entry),
        .rob_clear(rob_clear),

        // Dispatch input
        .rob_dispatch_in(dispatch_to_rob),

        // CDB input
        .rob_cdb_in(cdb_to_rob),

        // Store retire control (note: make sure these types are declared in your testbench)
        .store_retire(store_retire),
        .store_tag(store_tag),

        // Output to dispatch
        .rob_dispatch_out(rob_dispatch_out),

        // Output to retire/complete
        .rob_retire_out(rob_retire_out),

        // ROB full output
        .rob_full(rob_full),

        // Ready/valid signals
        .rob_ready(rob_ready),
        .rob_valid(rob_valid),

        // Debug
        .rob_pointers(rob_pointers)
        //.rob_debug(rob_debug)
    );

    // Struct Assembly
    always_comb begin
        rob_dispatch_in_temp = '{valid: dispatch_to_rob.dispatch_valid, dest_reg: dispatch_to_rob.dispatch_dest, opcode: dispatch_to_rob.dispatch_opcode};
        rob_cdb_in_temp       = '{valid: cdb_to_rob.cdb_valid, tag: cdb_to_rob.cdb_tag, value: cdb_to_rob.cdb_value};
    end

    // Debug
    task print_contents;
        input [45:0] array [31:0];
        input [11:0] pointers;
        integer i;
        begin
            $display("ROB contents:");
            for (i = 0; i < 32; i++) begin
                $display("Status:%b Opcode:%b Dest:%b Value:%h", array[i][45:44], array[i][43:37], array[i][36:32], array[i][31:0]);
            end
            $display("H/T pointers: Head:%b Tail:%b", pointers[11:6], pointers[5:0]);
        end
    endtask

    // Output
    initial begin
        $monitor("Time:%4.0f | clk:%b rst:%b | Dispatch: valid=%b dest=%b opcode=%b | CDB: valid=%b tag=%b value=%h | Retire: tag=%b dest=%b value=%h reg_valid=%b mem_valid=%b | rob_full=%b",
            $time, clock, reset,
            dispatch_to_rob.dispatch_valid, dispatch_to_rob.dispatch_dest, dispatch_to_rob.dispatch_opcode,
            cdb_to_rob.cdb_valid, cdb_to_rob.cdb_tag, cdb_to_rob.cdb_value,
            rob_retire_out.tag, rob_retire_out.dest_reg, rob_retire_out.value,
            rob_retire_out.reg_valid, rob_retire_out.mem_valid,
            rob_full);
    end

    initial begin
        // reset
        clock = 1;
        reset = 1;
        dispatch_to_rob.dispatch_valid = 0;
        dispatch_to_rob.dispatch_dest  = 0;
        dispatch_to_rob.dispatch_opcode = 0;
        cdb_to_rob.cdb_valid = 0;
        cdb_to_rob.cdb_tag = 0;
        cdb_to_rob.cdb_value = 0;
        retire_entry = 0;
        rob_clear = 0;
        rob_to_rs_read1 = 0;
        rob_to_rs_read2 = 0;

        @(negedge clock);
        reset = 0;

        // Cycle 1: Instruction 1
        dispatch_to_rob.dispatch_valid = 1;
        dispatch_to_rob.dispatch_dest = 5'd1;
        dispatch_to_rob.dispatch_opcode = `RV32_ADD;

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Cycle 2: Instruction 2 
        dispatch_to_rob.dispatch_dest = 5'd2;

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Cycle 3: Instruction 3
        dispatch_to_rob.dispatch_dest = 5'd8;
        dispatch_to_rob.dispatch_opcode = `RV32_SUB;

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Cycle 4: Broadcast CDB tag 1
        dispatch_to_rob.dispatch_valid = 0;
        cdb_to_rob.cdb_valid = 1;
        cdb_to_rob.cdb_tag = 6'd1;
        cdb_to_rob.cdb_value = 32'hFFFF_FFFF;

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Cycle 5: Retire instruction 
        cdb_to_rob.cdb_valid = 0;
        retire_entry = 1;

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Cycle 6: Attempt another retire (may stall)
        retire_entry = 1;

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        // Cycle 7: Instruction 4
        retire_entry = 0;
        dispatch_to_rob.dispatch_valid = 1;
        dispatch_to_rob.dispatch_dest = 5'd31;
        dispatch_to_rob.dispatch_opcode = `RV32_SUB;

        @(negedge clock);
        //print_contents(rob_debug, rob_pointers);

        $finish;
    end

endmodule