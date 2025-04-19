`include "verilog/sys_defs.svh"
// //packet from IS (issue stage) to EX (execute stage)
// typedef struct packed {
//     logic [31:0] OPA;         // Operand A
//     logic [31:0] OPB;         // Operand B
//     logic [4:0]  rob_tag;     // ROB tag for destination
//     logic [5:0]  RS_tag;      // Optional: ID of issuing RS
//     ALU_FUNC alu_func;    // ALU operation selector
//     logic [31:0] NPC;         // Next PC (for branch evaluation)
//     logic [31:0] inst;        // Raw instruction bits
//     logic        issue_valid; // This packet is valid to execute
//     logic rd_mem;
//     logic wr_mem;
// } IS_EX_PACKET;

// //ROB data to send to dispatch: 
// typedef struct packed {
//     logic [5:0] tag;     // ROB tag for the newly allocated entry
//     logic       valid;   // Whether the output is valid (i.e., we dispatched)
// } ROB_DISPATCH_PACKET;

// //ROB data to send to complete and retire

// typedef struct packed {
//     logic [5:0] tag;     // ROB tag for the newly allocated entry
//     logic [4:0]   dest_reg; // Destination register for the instruction
//     logic [31:0]  value;  // Value to write back to the register file
//     logic        reg_valid; // Whether the output is valid (i.e., we dispatched)
//     logic        mem_valid; 
//     logic [31:0] mem_addr; // Memory address to write back to the register file
// } ROB_RETIRE_PACKET;

// //CDB packet: to be sent to CDB
// typedef struct packed {
//     logic [4:0]  tag;     // ROB tag
//     logic [31:0] value;   // Result value
//     logic        valid;   // Valid signal
// } CDB_PACKET;

// ASSUMPTIONS MADE BY MY LSQ
// stores and loads are stored in one queue (together)

module lsq_adder#(
)(
    input logic [31:0] a,
    input logic [31:0] b
);
    assign sum = a + b;
endmodule

module lsq#(
    parameter LSQ_SIZE = 8,
    parameter LSQ_SIZE_W = 3, // log2(LSQ_SIZE)
)
(
    input logic clk,
    input logic reset,

    input logic [63:0] dcache_data_out, // data coming back from cache
    input logic [3:0] dcache_tag, // high when valid
    input logic [3:0] dcache_response, // 0 = can't accept, other=tag of transaction

    input ROB_RETIRE_PACKET rob_retire_in, //so stores know when they are allowed to write
    input ROB_DISPATCH_PACKET rob_dispatch_in, // so loads know when they are allowed to read / tags of instructions
    input IS_EX_PACKET is_ex_in, // packet from issue stage
    input CDB_PACKET cdb_in, // packet from CDB stage

    output CDB_PACKET cdb_out, // broadcast load data
    output logic [1:0] dcache_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE
    output logic [`XLEN-1:0] dcache_addr, // sending address to dcache

    output logic store_ready, // let ROB know that store ready to write
    output logic [4:0] store_ready_tag, // tag of store ready to write
    output logic stall_dispatch, // stall dispatch if lsq is full
);
    typedef struct packed {
        logic              valid;      // if entry is occupied
        logic              is_store;   // distinguish load vs store
        logic [4:0]        address_tag;    // tag from the IS_EX_PACKET
        logic [31:0]       address;    // memory address (OPA + OPB)
        logic         address_valid; // if we have the address from CDB
        logic [31:0]       store_data; // data to store (if store)
        logic [4:0]        store_data_tag;    // tag from the ROB
        logic store_data_valid; // if we have the data from CDB
        logic              retired;    // store can only write if retired
    } lsq_entry_t;

    lsq_entry_t lsq [LSQ_SIZE-1:0];
    logic [LSQ_IDX_W:0] head_ptr; //points to OLDEST entry
    logic [LSQ_IDX_W:0] tail_ptr; //points to next free entry

    // track whether we have an outstanding D‐cache request
    // now right now, we can only track ONE
    logic cache_in_flight;
    logic [3:0] cache_tag_in_flight;
    logic [LSQ_IDX_W:0] cache_index_in_flight;

    
    logic is_mem_op;
    logic is_store_op;
    logic is_load_op;

    // basically determines whether it is store or read
    always_comb begin
        is_store_op = is_ex_in.issue_valid && is_ex_in.wr_mem;
        is_load_op  = is_ex_in.issue_valid && is_ex_in.rd_mem;
        is_mem_op   = is_store_op || is_load_op;
    end

    logic [LSQ_SIZE_W:0] next_head_ptr, next_tail_ptr;
    assign next_head_ptr = head_ptr + 1;
    assign next_tail_ptr = tail_ptr + 1;

    logic empty, full;
    // we determine it is empty if the head pointer is the same as tail pointer
    assign empty = (head_ptr == tail_ptr);
    // we determine it is full if the next pointer is the same as head pointer
    assign full  = (next_tail_ptr == head_ptr);

    // assign stall if it is full
    assign stall_dispatch = full;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            head_ptr        <= '0;
            tail_ptr        <= '0;
            cache_in_flight <= 1'b0;
            cache_tag_in_flight <= 4'b0;

            for (int i=0; i<LSQ_SIZE; i++) begin
                lsq[i].valid   <= 1'b0;
                lsq[i].retired <= 1'b0;
            end
        end 
        else begin
            // enqueue if we have a mem op and there's space
            if (is_mem_op && !full) begin
                lsq[tail_ptr].valid      <= 1'b1;
                lsq[tail_ptr].is_store   <= is_store_op;
                lsq[tail_ptr].retired    <= 1'b0;

                // so now we have to wait for the address tag to arrive through the CDB
                lsq[tail_ptr].address_valid <= 1'b0;
                lsq[tail_ptr].address_tag   <= is_ex_in.rob_tag; 
                lsq[tail_ptr].address       <= 32'hDEAD_BEEF; // placeholder

                lsq[tail_ptr].store_data_valid <= 1'b0;
                lsq[tail_ptr].store_data_tag   <= is_ex_in.rob_tag;
                lsq[tail_ptr].store_data       <= 32'hDEAD_BEEF; // placeholder

                // bump tail
                tail_ptr <= next_tail_ptr;
            end
        // stores become "retired" when the ROB indicates so
        // if the ROB says this tag is done, mark that LSQ entry as retired
        if (rob_retire_in.mem_valid) begin
            for (int j=0; j<LSQ_SIZE; j++) begin
                if (lsq[j].valid && lsq[j].is_store 
                    && lsq[j].rob_tag == rob_retire_in.tag[4:0]) begin
                    lsq[j].retired <= 1'b1;
                end
            end
        end
        end
    end
    //listen to cdb for incoming data
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // all cleared in the previous block
        end else begin
            if (cdb_in.valid) begin
                for (int j=0; j<LSQ_SIZE; j++) begin
                    // If we're waiting for the address
                    if ( lsq[j].valid 
                        && !lsq[j].address_valid 
                        && (lsq[j].address_tag == cdb_in.tag) ) begin
                        lsq[j].address       <= cdb_in.value; 
                        lsq[j].address_valid <= 1'b1;
                    end

                    // If we're waiting for the store data
                    if ( lsq[j].valid 
                        && lsq[j].is_store 
                        && !lsq[j].store_data_valid 
                        && (lsq[j].store_data_tag == cdb_in.tag) ) begin
                        lsq[j].store_data       <= cdb_in.value; 
                        lsq[j].store_data_valid <= 1'b1;
                    end
                end
            end
        end
    end

    assign dcache_command = BUS_NONE;
    assign dcache_addr    = 32'b0;

    logic [1:0]  dcache_cmd_next;
    logic [31:0] dcache_addr_next;

    // info about the head
    lsq_entry_t head_entry = lsq[head_ptr];

    //this determines whether we are ready to send a request to the cache
    // the requirements are: if entry is load, then if we have address from CDB, we are ready to go
    // if entry is store, then we need to have the address AND
    // permission from our replay buffer (retired)
    logic head_ready_for_mem = head_entry.valid
                            && head_entry.address_valid
                            && (head_entry.is_store || head_entry.store_data_valid)
                            && (!head_entry.is_store || head_entry.retired);

    assign store_ready = ( head_entry.valid
                         && head_entry.is_store
                         && head_entry.retired
                        && head_entry.store_data_valid
                         && head_entry.address_valid
                         && !cache_in_flight );
    
    assign store_ready_tag = head_entry.rob_tag;

    always_comb begin
    dcache_cmd_next  = BUS_NONE;
    dcache_addr_next = 32'b0;
    //if cache in flight we can wait for now, even though the cache is non-blocking 
    // maybe in the future we can change this
    if (!cache_in_flight && head_ready_for_mem) begin
        if (head_entry.is_store) begin
            dcache_cmd_next  = BUS_STORE;
            dcache_addr_next = head_entry.address;
            end 
            else begin
            // load
            dcache_cmd_next  = BUS_LOAD;
            dcache_addr_next = head_entry.address;
        end
    end
    end

    assign dcache_command = dcache_cmd_next;
    assign dcache_addr    = dcache_addr_next;

    //TODO:
    // in flight cache tracking, we need to track the memories reqeust that are inflight

    // we need to broadcast the cdb for loads
    logic load_completed;
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            load_completed <= 1'b0;
        end 
        else begin
            load_completed <= 1'b0;
            if ( cache_in_flight 
                && (dcache_response == cache_tag_in_flight)
                && (dcache_response != 4'b0) ) begin
                    // if that was a load (not a store)
                    if (!lsq[cache_index_in_flight].is_store)
                    load_completed <= 1'b1;
            end
        end
    end

    // we broadcast the load data with the same ROB tag 
    assign cdb_out.valid = load_completed;
    assign cdb_out.tag   = load_completed 
                            ? lsq[cache_index_in_flight].rob_tag
                            : 5'b0;
    // return 32 bits from D-cache
    assign cdb_out.value = load_completed
                            ? dcache_data_out[31:0]
                            : 32'b0;



endmodule