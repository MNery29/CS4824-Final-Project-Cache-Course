/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob.sv                                              //
//                                                                     //
//  Description :  Re-order buffer for P6 architecture, ensures        //
//                 in-order appearance is maintained                   //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

//ROB INTEGRATED WITH STAGE 
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
    input [4:0] retire_tag,
    input rob_clear, //flush all entries 

    input store_retire, //store retire signal from LSQ
    input [4:0] store_tag, //tag from LSQ


    //output to send to Dispatch stage
    output ROB_DISPATCH_PACKET rob_dispatch_out,


    //output signals to retire and complete stage
    output ROB_RETIRE_PACKET rob_retire_out,


    //outputs to reservation station
    output [31:0] rob_to_rs_value1,
    output [31:0] rob_to_rs_value2,
    
    //rob full signal, for stalling/hazards   
    output rob_full,

    //outputs for if the ROB has something ready to retire
    output logic rob_ready,
    output logic rob_valid,

    //debug
    output logic [45:0] rob_debug [31:0],
    output logic [11:0] rob_pointers
);

    typedef enum logic[1:0] {EMPTY, BUSY, READY} rob_state; //used to track status of an instruction.
    //internal signals
    //rob entries - 5
    rob_state rob_status[`ROB_SZ-1:0]; //status of each instruction in ROB
    logic [31:0] rob_values[`ROB_SZ-1:0];
    logic [6:0] rob_opcode[`ROB_SZ-1:0];
    logic [4:0] rob_dest[`ROB_SZ-1:0];
    logic rob_is_branch[`ROB_SZ-1:0]; //used to track if instruction is a branch or not
    logic rob_branch_taken[`ROB_SZ-1:0]; //used to track if instruction is a branch or not
    logic rob_illegal[`ROB_SZ-1:0]; 
    logic rob_halt[`ROB_SZ-1:0]; 
    logic rob_csr_op[`ROB_SZ-1:0]; 
    logic [31:0] npc_rob[`ROB_SZ-1:0]; //used to track the next program counter for branch instructions
    logic rob_is_store[`ROB_SZ-1:0]; //used to track if instruction is a store or not

    //head and tail pointers for queue FIFO structure
    logic [`ROB_TAG_BITS:0] head, tail;
    logic [`ROB_TAG_BITS:0] next_head, next_tail;
    assign next_head[`ROB_TAG_BITS] = head[`ROB_TAG_BITS];
    assign next_tail[`ROB_TAG_BITS] = tail[`ROB_TAG_BITS];
    assign next_head[`ROB_TAG_BITS-1:0] = head[`ROB_TAG_BITS-1:0] == 5'b11111 ? 1 : head[`ROB_TAG_BITS-1:0] + 1;
    assign next_tail[`ROB_TAG_BITS-1:0] = tail[`ROB_TAG_BITS-1:0] == 5'b11111 ? 1 : tail[`ROB_TAG_BITS-1:0] + 1;


    //check if ROB is full
    assign rob_full = ((head[`ROB_TAG_BITS-1:0] == tail[`ROB_TAG_BITS-1:0]) && (head[`ROB_TAG_BITS] != tail[`ROB_TAG_BITS]));

    //Reservation station value checks
    assign rob_to_rs_value1 = rob_to_rs_read1 ? rob_values[rob_read_tag1] : 32'b0;
    assign rob_to_rs_value2 = rob_to_rs_read2 ? rob_values[rob_read_tag2] : 32'b0;


    //check if ROB has something ready to retire
    assign rob_ready = (rob_status[head[`ROB_TAG_BITS-1:0]] == READY);
    assign rob_valid = (rob_status[head[`ROB_TAG_BITS-1:0]] != EMPTY);
    
    //DISPATCH OUTPUT
    assign rob_dispatch_out.tag   = tail[`ROB_TAG_BITS-1:0];
    assign rob_dispatch_out.valid = (rob_dispatch_in.valid && !rob_full);


    // RETIRE PACKET OUTPUT
    always_comb begin
        if ((rob_status[head[`ROB_TAG_BITS-1:0]] == READY) && (!rob_clear)) begin
            rob_retire_out.tag       = head[`ROB_TAG_BITS-1:0];
            rob_retire_out.dest_reg  = regfile_retire(rob_opcode[head[`ROB_TAG_BITS-1:0]]) ? rob_dest[head[`ROB_TAG_BITS-1:0]] : `ZERO_REG;
            rob_retire_out.value     = rob_values[head[`ROB_TAG_BITS-1:0]];
            rob_retire_out.reg_valid = regfile_retire(rob_opcode[head[`ROB_TAG_BITS-1:0]]);
            rob_retire_out.mem_valid = memory_retire(rob_opcode[head[`ROB_TAG_BITS-1:0]]);
            rob_retire_out.mem_addr  = memory_retire(rob_opcode[head[`ROB_TAG_BITS-1:0]]) ? rob_values[head[`ROB_TAG_BITS-1:0]] : 32'b0;
            rob_retire_out.is_branch = rob_is_branch[head[`ROB_TAG_BITS-1:0]];
            rob_retire_out.take_branch = rob_branch_taken[head[`ROB_TAG_BITS-1:0]];
            rob_retire_out.halt      = rob_halt[head[`ROB_TAG_BITS-1:0]];
            rob_retire_out.illegal   = rob_illegal[head[`ROB_TAG_BITS-1:0]];
            rob_retire_out.csr_op    = rob_csr_op[head[`ROB_TAG_BITS-1:0]];
            rob_retire_out.npc        = npc_rob[head[`ROB_TAG_BITS-1:0]]; // this will only be used if we do TAKE BRANCH (RT will handle this)
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

    always_comb begin
       for (int i = 0; i < `ROB_SZ; i++) begin
           rob_debug[i] = {rob_status[i], rob_opcode[i], rob_dest[i], rob_values[i]};
       end
    end

    // ROB LOGIC 
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < `ROB_SZ; i++) begin
                rob_values[i] <= 32'b0;
                rob_dest[i]   <= 5'b0;
                rob_opcode[i] <= 7'b0;
                rob_status[i] <= EMPTY;
                rob_is_branch[i] <= 1'b0;
                rob_halt[i] <= 1'b0;
                rob_illegal[i] <= 1'b0;
                rob_csr_op[i] <= 1'b0;
                npc_rob[i] <= 32'b0;
                rob_is_store[i] <= 1'b0;
            end
            head <= 1;
            tail <= 1;
        end else begin
            // Dispatch new instruction if valid
            if (rob_dispatch_in.valid && !rob_full) begin
                rob_values[tail[`ROB_TAG_BITS-1:0]] <= 32'b0;
                rob_dest[tail[`ROB_TAG_BITS-1:0]]   <= rob_dispatch_in.dest_reg;
                rob_opcode[tail[`ROB_TAG_BITS-1:0]] <= rob_dispatch_in.opcode;
                rob_status[tail[`ROB_TAG_BITS-1:0]] <= BUSY;
                rob_is_branch[tail[`ROB_TAG_BITS-1:0]] <= rob_dispatch_in.is_branch;
                rob_branch_taken[tail[`ROB_TAG_BITS-1:0]] <= 1'b0;
                rob_halt[tail[`ROB_TAG_BITS-1:0]] <= rob_dispatch_in.halt;
                rob_illegal[tail[`ROB_TAG_BITS-1:0]] <= rob_dispatch_in.illegal;
                rob_csr_op[tail[`ROB_TAG_BITS-1:0]] <= rob_dispatch_in.csr_op;
                npc_rob[tail[`ROB_TAG_BITS-1:0]] <= rob_dispatch_in.npc;
                rob_is_store[tail[`ROB_TAG_BITS-1:0]] <= 0;
                tail <= next_tail;
            end

            // CDB writeback
            if (rob_cdb_in.valid) begin
                // so i guess for branches, what if we assume that a NON zero value means its taken?
                // and we ALSO assume we always predict not taken
                rob_values[rob_cdb_in.tag] <= rob_cdb_in.value;
                rob_status[rob_cdb_in.tag] <= READY;
                rob_branch_taken[rob_cdb_in.tag] <= rob_cdb_in.take_branch;
            end

            if (store_retire && rob_status[store_tag] == BUSY) begin
                rob_values[store_tag] <= 32'b0;
                rob_status[store_tag] <= READY;
                rob_is_store[store_tag] <= 1'b1;
            end

            // Retire stage
            if ( (retire_entry && rob_status[head[`ROB_TAG_BITS-1:0]] == READY && retire_tag == head[`ROB_TAG_BITS-1:0] )||
                rob_is_store[head[`ROB_TAG_BITS-1:0]] ) begin
                if (rob_clear) begin
                    for (int i = 0; i < `ROB_SZ; i++) begin
                        rob_values[i] <= 32'b0;
                        rob_dest[i]   <= 5'b0;
                        rob_opcode[i] <= 7'b0;
                        rob_status[i] <= EMPTY;
                        rob_is_branch[i] <= 1'b0;
                        rob_branch_taken[i] <= 1'b0;
                        rob_halt[i] <= 1'b0;
                        rob_illegal[i] <= 1'b0;
                        rob_csr_op[i] <= 1'b0;
                        npc_rob[i] <= 32'b0;
                        rob_is_store[i] <= 1'b0;
                    end
                    head <= tail;
                end else begin
                    rob_values[head[`ROB_TAG_BITS-1:0]] <= 32'b0;
                    rob_dest[head[`ROB_TAG_BITS-1:0]]   <= 5'b0;
                    rob_opcode[head[`ROB_TAG_BITS-1:0]] <= 7'b0;
                    rob_status[head[`ROB_TAG_BITS-1:0]] <= EMPTY;
                    rob_is_branch[head[`ROB_TAG_BITS-1:0]] <= 1'b0;
                    rob_branch_taken[head[`ROB_TAG_BITS-1:0]] <= 1'b0;
                    rob_halt[head[`ROB_TAG_BITS-1:0]] <= 1'b0;
                    rob_illegal[head[`ROB_TAG_BITS-1:0]] <= 1'b0;
                    rob_csr_op[head[`ROB_TAG_BITS-1:0]] <= 1'b0;
                    npc_rob[head[`ROB_TAG_BITS-1:0]] <= 32'b0;
                    rob_is_store[head[`ROB_TAG_BITS-1:0]] <= 1'b0;
                    head <= next_head;
                end
            end
        end
    end
endmodule


//BELOW IS ALL OLD NOTES WHEN DEBUGGING 


// valid bit needed for ROB

// set the head and tail to one more, store in memory for 

//keep valid bit in memory. 

//also save valid bit as part of memory for ROB, when memory is implement.

//also store valid bits