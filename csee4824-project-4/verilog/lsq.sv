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

//this private addr packet will only be READ by LSQ's and only be written to by functional units
// this is to prevent other modules from mistaking the tag as register values
// typedef struct packed {
//     logic [`XLEN-1:0]       addr;      // data coming back from cache
//     logic [3:0]        tag;        // tag of the transaction
//     logic              valid;      // if entry is occupied
// } priv_addr_packet;

module lsq#(
    parameter LSQ_SIZE = 8,
    parameter LSQ_SIZE_W = 2, // log2(LSQ_SIZE) = 3 but 3 -1 =2
    parameter NONBLOCKING = 0 // 0 = blocking, 1 = non-blocking
)
(
    input logic clk,
    input logic reset,

    input logic [63:0] dcache_data_out, // data coming back from cache
    input logic [3:0] dcache_tag, // high when valid
    input logic [3:0] dcache_response, // 0 = can't accept, other=tag of transaction
    input logic dcache_hit, // 1 if hit, 0 if miss

    input [4:0] mem_tag, // from rt stage
    input mem_valid, // from rt stage
    input LSQ_PACKET lsq_packet, // packet from issue stage
    input CDB_PACKET cdb_in, // packet from CDB stage
    input priv_addr_packet priv_addr_in, // packet from execute stage (giving us the address)

    // input cdb_busy, // this is high if the previous CDB message we sent was not recieved, which means we should send again

    output EX_CP_PACKET cdb_out, // broadcast load data
    output logic [1:0] dcache_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE
    output logic [`XLEN-1:0] dcache_addr, // sending address to dcache
    output logic [63:0] dcache_data, // data for current command (if store)
    output logic [1:0] dcache_size, // size of the data (BYTE, HALF, WORD or DOUBLE)

    output logic store_ready, // let ROB know that store ready to write
    output logic [4:0] store_ready_tag, // tag of store ready to write
    output logic lsq_free,// if lsq has empty entry

    output logic cache_in_flight, //debugging
    output logic head_ready_for_mem, // debugging

    output logic [LSQ_SIZE_W:0] head_ptr, //points to OLDEST entry debugging
    output logic [LSQ_SIZE_W:0] tail_ptr, //points to next free entry debugging

    output lsq_entry_t lsq_out [LSQ_SIZE-1:0] // debugging
);

    lsq_entry_t lsq [LSQ_SIZE-1:0];
    assign lsq_out = lsq;
    // logic [LSQ_SIZE_W:0] head_ptr; //points to OLDEST entry
    // logic [LSQ_SIZE_W:0] tail_ptr; //points to next free entry

    // track whether we have an outstanding D‐cache request (or multiple since we have a nonblocking cache)
    logic [4:0] cache_tag_in_flight [3:0]; //indexed by dcache_tag (3 bits)
    logic cache_in_flight_valid [3:0]; //indexed by dcache_tag (3 bits)
    logic cache_offset_in_flight [3:0]; //indexed by dcache_tag (3 bits) gets us whether it is top half of cache line or bottom half    
    logic cache_in_flight_rd_unsigned [3:0]; //indexed by dcache_tag (3 bits) gets us whether it is unsigned or signed
    MEM_SIZE cache_in_flight_mem_size [3:0]; //indexed by dcache_tag (3 bits) gets us whether it is 8, 16, or 32 bit



    
    logic is_mem_op;
    logic is_store_op;
    logic is_load_op;

    // basically determines whether it is store or read
    always_comb begin
        is_store_op = lsq_packet.valid && lsq_packet.wr_mem;
        is_load_op  = lsq_packet.valid && lsq_packet.rd_mem;
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
    assign lsq_free = !full;

     // assign dcache_command = BUS_NONE;
    // assign dcache_addr    = 32'b0;
    // info about the head
    lsq_entry_t head_entry; 
    assign head_entry = lsq[head_ptr];

    //this determines whether we are ready to send a request to the cache
    // the requirements are: if entry is load, then if we have address from CDB, we are ready to go
    // if entry is store, then we need to have the address AND
    // permission from our replay buffer (retired)
    // logic head_ready_for_mem;
    assign head_ready_for_mem = (head_entry.valid
                                 && head_entry.address_valid
                                 && ( !head_entry.is_store
                                      || (head_entry.store_data_valid && head_entry.retired) ));


    assign store_ready = ( head_entry.valid
                           && head_entry.is_store
                           && head_entry.address_valid
                           && head_entry.store_data_valid
                           && !cache_in_flight );
    assign store_ready_tag = head_entry.rob_tag;

    logic [1:0]  dcache_cmd_next;
    logic [31:0] dcache_addr_next;
    logic [63:0] dcache_data_next;

    logic [4:0] tag_to_broadcast;
    always_comb begin
        dcache_cmd_next  = BUS_NONE;
        dcache_addr_next = 32'b0;
        dcache_data_next = 64'b0;
        dcache_size = 0;

        if (head_ready_for_mem) begin
            if (head_entry.is_store) begin
                dcache_cmd_next  = BUS_STORE;
                dcache_addr_next = head_entry.address;
                dcache_data_next[31:0] = head_entry.store_data;
                dcache_size = head_entry.mem_size;
            end
            else begin
                if ((NONBLOCKING || !cache_in_flight)) begin
                    dcache_cmd_next  = BUS_LOAD;
                    dcache_addr_next = head_entry.address;
                    dcache_size = head_entry.mem_size;
                end
                else begin
                end
                // data_next = 0 for loads
            end
        end
    end

    assign dcache_command = dcache_cmd_next;
    assign dcache_addr    = dcache_addr_next;
    assign dcache_data    = dcache_data_next;

    
    logic load_completed;

    logic [31:0] data_to_broadcast;
    logic [31:0] next_data_to_broadcast;
    //right now, my lsq is allowing duplicates in, so we will use this to track duplicates
    // logic [4:0] last_tag;

    // for indexing mem of cache data (from returns of tag)
    always_comb begin
        if (dcache_tag != 0 && !dcache_hit) begin
            next_data_to_broadcast = cache_offset_in_flight[dcache_tag] ? dcache_data_out[63:32] : dcache_data_out[31:0];
            if (cache_in_flight_rd_unsigned[dcache_tag]) begin
                // unsigned: zero-extend the data
                if (cache_in_flight_mem_size[dcache_tag] == BYTE) begin
                    next_data_to_broadcast[`XLEN-1:8] = 0;
                end else if (cache_in_flight_mem_size[dcache_tag] == HALF) begin
                    next_data_to_broadcast[`XLEN-1:16] = 0;
                end
            end else begin
                // signed: sign-extend the data
                if (cache_in_flight_mem_size[dcache_tag][1:0] == BYTE) begin
                    next_data_to_broadcast[`XLEN-1:8] = {(`XLEN-8){next_data_to_broadcast[7]}}; //replicates the next_data[7] however many times
                end else if (cache_in_flight_mem_size[dcache_tag] == HALF) begin
                    next_data_to_broadcast[`XLEN-1:16] = {(`XLEN-16){next_data_to_broadcast[15]}};
                end
            end
        end
        else if (dcache_hit) begin
            next_data_to_broadcast = head_entry.address[2] ? dcache_data_out[63:32] : dcache_data_out[31:0];
            if (head_entry.rd_unsigned) begin
                // unsigned: zero-extend the data
                if (head_entry.mem_size == BYTE) begin
                    next_data_to_broadcast[`XLEN-1:8] = 0;
                end else if (head_entry.mem_size == HALF) begin
                    next_data_to_broadcast[`XLEN-1:16] = 0;
                end
            end else begin
                // signed: sign-extend the data
                if (head_entry.mem_size == BYTE) begin
                    next_data_to_broadcast[`XLEN-1:8] = {(`XLEN-8){next_data_to_broadcast[7]}};
                end else if (head_entry.mem_size == HALF) begin
                    next_data_to_broadcast[`XLEN-1:16] = {(`XLEN-16){next_data_to_broadcast[15]}};
                end
            end
        end
        else begin
            next_data_to_broadcast = 32'b0;
        end
    end

    

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            tail_ptr        <= '0;
            head_ptr        <= '0;
            load_completed          <= 1'b0;
            data_to_broadcast       <= 31'b0;
            cache_in_flight       <= 1'b0;
            // last_tag             <= 5'b0;

            for (int i = 0; i < LSQ_SIZE; i++) begin
                lsq[i].valid             <= 1'b0;
                lsq[i].retired           <= 1'b0;
                lsq[i].address_valid     <= 1'b0;
                lsq[i].store_data_valid  <= 1'b0;
                cache_in_flight_valid[i] <= 1'b0;
            end
        end 
        else begin
            //this is for checking if there is a request in flight (we should wait for this to finish beofre)
            //blocking method
            // no load completes this cycle
            load_completed <= 1'b0;

            // ff we just sent a request and the cache accepted it (dcache_tag != 0),
            // then record it as "in flight"
            // we want to handle stores seperately
            if (head_ready_for_mem && !lsq[next_head_ptr-1].is_store) begin
                // this happens when we do testing
                if (dcache_response != 0 && !dcache_hit) begin
                    if (NONBLOCKING) begin
                        lsq[next_head_ptr-1].valid <= 1'b0;
                    end

                    //means a miss
                    cache_in_flight       <= 1'b1; // this is rly only used for blocking case
                    cache_tag_in_flight[dcache_response] <= lsq[next_head_ptr-1].rob_tag;
                    cache_in_flight_valid[dcache_response] <= 1'b1;
                    cache_offset_in_flight[dcache_response] <= lsq[next_head_ptr-1].address[2]; // 0 for lower half, 1 for upper half
                    cache_in_flight_rd_unsigned[dcache_response] <= lsq[next_head_ptr-1].rd_unsigned;
                    cache_in_flight_mem_size[dcache_response] <= lsq[next_head_ptr-1].mem_size;


                    // but we still want the pointer to advance
                    // so we can send the next request
                    if (NONBLOCKING) begin
                        head_ptr <= next_head_ptr;
                    end
                end

                if (dcache_hit) begin
                    lsq[next_head_ptr-1].valid <= 1'b0;

                    //means a hit
                    // we dont have to track the cache_in_flight, data is already there
                    load_completed <= 1'b1;
                    data_to_broadcast <= next_data_to_broadcast;
                    tag_to_broadcast <= lsq[next_head_ptr-1].rob_tag;

                    // advance pointer
                    head_ptr <= next_head_ptr;

                end
                
            end
            if (head_ready_for_mem && (dcache_response != 0) && lsq[head_ptr].is_store) begin
                // we dont have to track request
                // but we do need to wait for dcache_response != 0 as a handshake
                lsq[next_head_ptr-1].valid <= 1'b0;

                head_ptr <= next_head_ptr;
            end



            // ff a response arrives that matches our in-flight tag, the transaction completes
            if ( (dcache_tag != 0) && cache_in_flight_valid[dcache_tag])
            begin
                if (!NONBLOCKING) begin
                    lsq[next_head_ptr-1].valid <= 1'b0;
                    cache_in_flight <= 1'b0;
                    head_ptr <= next_head_ptr;
                end
                // ff it's a load, broadcast the data for exactly one cycle
                load_completed <= 1'b1;


                data_to_broadcast <= next_data_to_broadcast;
                tag_to_broadcast <= cache_tag_in_flight[dcache_tag];
                cache_in_flight_valid[dcache_tag] <= 1'b0;

            end
            else begin
            end
            // enqueue if we have a mem op and there's space
            if (is_mem_op && !full) begin
                lsq[tail_ptr].valid      <= 1'b1;
                lsq[tail_ptr].is_store   <= is_store_op;
                lsq[tail_ptr].retired    <= 1'b0;

                lsq[tail_ptr].rd_unsigned <= lsq_packet.rd_unsigned;

                // Store the 5-bit ROB tag for retirement & final broadcasting
                lsq[tail_ptr].rob_tag    <= lsq_packet.rob_tag;

                // We'll store a 4-bit address_tag from the lower bits of rob_tag (or some other scheme).
                // Must match how `priv_addr_in.tag` will be produced by EX.
                lsq[tail_ptr].address_tag <= lsq_packet.rob_tag;
                lsq[tail_ptr].address_valid <= 1'b0;
                lsq[tail_ptr].address       <= 32'hDEAD_BEEF;  // placeholder

                // For store data, we also track its tag. The CDB has a 5-bit tag:
                lsq[tail_ptr].store_data_tag   <= lsq_packet.store_data_tag;  
                lsq[tail_ptr].store_data_valid <= lsq_packet.store_data_valid;
                lsq[tail_ptr].store_data       <= lsq_packet.store_data; 

                lsq[tail_ptr].mem_size <= lsq_packet.mem_size;

                // last_tag <= lsq_packet.rob_tag;

                // Bump tail
                tail_ptr <= next_tail_ptr;
            end
            // stores become "retired" when the rt stage indicates so
            // if the ROB says this tag is done, mark that LSQ entry as retired
            if (mem_valid) begin
                for (int j = 0; j < LSQ_SIZE; j++) begin
                    if (lsq[j].valid && lsq[j].is_store &&
                        (lsq[j].rob_tag == mem_tag)) 
                    begin
                        lsq[j].retired <= 1'b1;
                    end
                end
            end

            if (priv_addr_in.valid) begin
                for (int j = 0; j < LSQ_SIZE; j++) begin
                    if (lsq[j].valid && !lsq[j].address_valid &&
                        (lsq[j].address_tag == priv_addr_in.tag)) 
                    begin
                        lsq[j].address       <= priv_addr_in.addr; 
                        lsq[j].address_valid <= 1'b1;
                    end
                end
            end

            if (cdb_in.valid) begin
                for (int j = 0; j < LSQ_SIZE; j++) begin
                    if (lsq[j].valid && lsq[j].is_store && 
                        !lsq[j].store_data_valid &&
                        (lsq[j].store_data_tag == cdb_in.tag)) 
                    begin
                        // store only 32 bits for now (lower half)
                        lsq[j].store_data[31:0]  <= cdb_in.value;
                        lsq[j].store_data[63:32] <= 32'b0;
                        lsq[j].store_data_valid  <= 1'b1;
                    end
                end
            end
            
        end
    end

    assign cdb_out.valid = load_completed;
    assign cdb_out.done =  load_completed;
    // -1 is a hacky way to get the load that was just completed (this is very hacky)
    // only works if it is indeed blocking
    assign cdb_out.rob_tag   = tag_to_broadcast;
    // return 32 bits from the 64-bit D-cache line
    assign cdb_out.value = data_to_broadcast;
    assign cdb_out.take_branch = 1'b0; // not used in LSQ



endmodule