/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob.sv                                              //
//                                                                     //
//  Description :  Re-order buffer for P6 architecture, ensures        //
//                 in-order appearance is maintained                   //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

//Temp ROB for integration with dispatch stage

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module reorder_buffer(
    //standard input signals for module.
    input reset,
    input clock,

    //input signals from dispatch stage 
    input DISPATCH_ROB_PACKET rob_dispatch_in,


    //If RS needs to read value ready in ROB
    input logic rob_to_rs_read1,
    input logic [`ROB_TAG_BITS-1:0] rob_read_tag1,
    input logic rob_to_rs_read2,
    input logic [`ROB_TAG_BITS-1:0] rob_read_tag2,

    //input signals from CDB (execute) stage) 
    input CDB_ROB_PACKET rob_cdb_in,

    //input signals from retire stage
    input retire_entry,
    input rob_clear, //flush all entries 


    //output to send to Dispatch stage
    output ROB_DISPATCH_PACKET rob_dispatch_out,


    //output signals to retire and complete stage
    output ROB_RETIRE_PACKET rob_retire_out,


    //outputs to reservation station
    output [31:0] rob_to_rs_value1,
    output [31:0] rob_to_rs_value2,
    
    //rob full signal, for stalling/hazards   
    output rob_full,

    //debug
    // output logic [45:0] rob_debug [31:0],
    output logic [11:0] rob_pointers
);

    typedef enum logic[1:0] {EMPTY, BUSY, READY} rob_state; //used to track status of an instruction.
    //internal signals
    //rob entries - 5
    rob_state rob_status[`ROB_SZ-1:0]; //status of each instruction in ROB
    logic [31:0] rob_values[`ROB_SZ-1:0];
    logic [6:0] rob_opcode[`ROB_SZ-1:0];
    logic [4:0] rob_dest[`ROB_SZ-1:0];

    //head and tail pointers for queue FIFO structure
    logic [`ROB_TAG_BITS:0] head, tail;

    //check if ROB is full
    assign rob_full = ((head[`ROB_TAG_BITS-1:0] == tail[`ROB_TAG_BITS-1:0]) && (head[`ROB_TAG_BITS] != tail[`ROB_TAG_BITS]));

    //Reservation station value checks
    assign rob_to_rs_value1 = rob_to_rs_read1 ? rob_values[rob_read_tag1] : 32'b0;
    assign rob_to_rs_value2 = rob_to_rs_read2 ? rob_values[rob_read_tag2] : 32'b0;


    //DISPATCH OUTPUT
    assign rob_dispatch_out.tag   = tail[`ROB_TAG_BITS-1:0];
    assign rob_dispatch_out.valid = (rob_dispatch_in.valid && !rob_full);

    // RETIRE PACKET OUTPUT
    always_comb begin
        if ((retire_entry && rob_status[head[`ROB_TAG_BITS-1:0]] == READY) && (!rob_clear)) begin
            rob_retire_out.tag       = head[`ROB_TAG_BITS-1:0];
            rob_retire_out.dest_reg  = regfile_retire(rob_opcode[head[`ROB_TAG_BITS-1:0]]) ? rob_dest[head[`ROB_TAG_BITS-1:0]] : `ZERO_REG;
            rob_retire_out.value     = rob_values[head[`ROB_TAG_BITS-1:0]];
            rob_retire_out.reg_valid = regfile_retire(rob_opcode[head[`ROB_TAG_BITS-1:0]]);
            rob_retire_out.mem_valid = memory_retire(rob_opcode[head[`ROB_TAG_BITS-1:0]]);
            rob_retire_out.mem_addr  = memory_retire(rob_opcode[head[`ROB_TAG_BITS-1:0]]) ? rob_values[head[`ROB_TAG_BITS-1:0]] : 32'b0;
        end else begin
            rob_retire_out = '{default: 0};
        end
    end

    //HELPER FUNCTIONS  

    function logic memory_retire(input [6:0] opcode);
        return (opcode == `RV32_STORE);
    endfunction

    function logic regfile_retire(input [6:0] opcode);
        return (
            (opcode == `RV32_LOAD)    ||
            (opcode == `RV32_OP_IMM)  ||
            (opcode == `RV32_OP)      ||
            (opcode == `RV32_JALR_OP) ||
            (opcode == `RV32_JAL_OP)  ||
            (opcode == `RV32_LUI_OP)  ||
            (opcode == `RV32_AUIPC_OP)
        );
    endfunction


    //DEBUGGING OUTPUTS
    assign rob_pointers = {head, tail};

    //always_comb begin
    //    for (int i = 0; i < `ROB_SZ; i++) begin
    //        rob_debug[i] = {rob_status[i], rob_opcode[i], rob_dest[i], rob_values[i]};
    //    end
    //end

    // ROB LOGIC 
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < `ROB_SZ; i++) begin
                rob_values[i] <= 32'b0;
                rob_dest[i]   <= 5'b0;
                rob_opcode[i] <= 7'b0;
                rob_status[i] <= EMPTY;
            end
            head <= 6'b0;
            tail <= 6'b0;
        end else begin
            // Dispatch new instruction if valid
            if (rob_dispatch_in.valid && !rob_full) begin
                rob_values[tail[`ROB_TAG_BITS-1:0]] <= 32'b0;
                rob_dest[tail[`ROB_TAG_BITS-1:0]]   <= rob_dispatch_in.dest_reg;
                rob_opcode[tail[`ROB_TAG_BITS-1:0]] <= rob_dispatch_in.opcode;
                rob_status[tail[`ROB_TAG_BITS-1:0]] <= BUSY;
                tail <= tail + 1;
            end

            // CDB writeback
            if (rob_cdb_in.valid) begin
                rob_values[rob_cdb_in.tag] <= rob_cdb_in.value;
                rob_status[rob_cdb_in.tag] <= READY;
            end

            // Retire stage
            if (retire_entry && rob_status[head[`ROB_TAG_BITS-1:0]] == READY) begin
                if (rob_clear) begin
                    for (int i = 0; i < `ROB_SZ; i++) begin
                        rob_values[i] <= 32'b0;
                        rob_dest[i]   <= 5'b0;
                        rob_opcode[i] <= 7'b0;
                        rob_status[i] <= EMPTY;
                    end
                    head <= tail;
                end else begin
                    rob_values[head[`ROB_TAG_BITS-1:0]] <= 32'b0;
                    rob_dest[head[`ROB_TAG_BITS-1:0]]   <= 5'b0;
                    rob_opcode[head[`ROB_TAG_BITS-1:0]] <= 7'b0;
                    rob_status[head[`ROB_TAG_BITS-1:0]] <= EMPTY;
                    head <= head + 1;
                end
            end
        end
    end
endmodule

// valid bit needed for ROB

// set the head and tail to one more, store in memory for 

//keep valid bit in memory. 

//also save valid bit as part of memory for ROB, when memory is implement.

//also store valid bits