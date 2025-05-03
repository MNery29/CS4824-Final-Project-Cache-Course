/*
Main work on Dcache by William - Mateo
dcache specifications:
    512 bytes
    directly mapped
    non-blocking
    write-through? <-- not yet implemented
    cache line size of 8 bytes, 64 lines
    8 bytes / 8 bytes = 1 = 0 offset length
    64 lines = 6 index bits
    for a 32 bit address length: { Tag(26 bits) | Index(6 bits) | Offset(0 bits) }
*/
`include "verilog/sys_defs.svh"

`define IDLE 1'b0
`define MISS 1'b1
`define NONE 3'b000

module dcache
#(
    TAG_BITS = 23,
    INDEX_BITS = 6,
    OFFSET_BITS = 3
)
(
    input logic clk,
    input logic reset,

    input logic [`XLEN-1:0] proc2Dcache_addr,
    input logic [63:0] proc2Dcache_data, // datafor current command (if store)
    input logic [1:0] proc2Dcache_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE
    input logic [1:0] proc2Dcache_size, // `MEM_BYTE, `MEM_HALF, `MEM_WORD, `MEM_DOUBLE

    input logic [3:0]  mem2dcache_response, // 0 = can't accept, other=tag of transaction
    input logic [63:0] mem2dcache_data,     // data resulting from a load
    input logic [3:0]  mem2dcache_tag,       // 0 = no value, other=tag of transaction

    input logic halt,

    output logic [`XLEN-1:0] dcache2mem_addr,
    output logic [63:0]      dcache2mem_data, // address for current command
    output logic [1:0]       dcache2mem_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE

    output logic [63:0] hit_data, // data resulting from a load
    output logic hit, // 1 if hit, 0 if miss
    output logic [3:0] data_tag, 
    output logic [3:0] data_response, //im actaully going to increase the bits from my cache, because i want to be able to assign my own tags to certain requests

    output logic [31:0] cur_addr,
    output logic [1:0] cur_command, // this is the command that we are currently executing
    output logic [63:0] cur_data,

    output logic halt_confirm, 

    output logic next_state, //for debugging 
    output logic state, //for debugging 


    output logic [`XLEN-1:0] tag_to_addr [15:0],
    output logic tag_to_addr_valid [15:0],
    output logic [1:0] tag_to_memsize [15:0],
    output logic [63:0] tag_to_memdata [15:0], // this is for stores exclusively
    output logic tag_to_is_store [15:0],

    output logic [63:0] cache_data [0:63], // 64 lines of 8 bytes
    output logic [TAG_BITS-1:0] cache_tag [0:63], // 64 lines of tag bits
    output logic cache_valid [0:63], // 64 lines of valid bits
    output logic cache_dirty [0:63], // 64 lines of dirty bits

    output logic wb_eviction, // this will be high if we need to evict a DIRTY line from the cache
    output logic next_wb_eviction // this will be high if we need to evict a DIRTY line from the cache
    // output logic [3:0] number_of_waits, //for debugging
    // output logic [3:0] next_number_of_waits //for debugging
    /* 
    so the way this module works is that,
    when a miss occurs, we will send you a data_response,
    you have to wait for this data response to be non-zero,
    this data_response is the tag of the transaction. Once this module 
    has fetched the data from the memory module, it will
    send the data back thru the hit_data line, with the data_tag
    populated with the tag of the transaction.

    NOTE: if you send a request to this module, and it is a miss,
    and the data_response is 0, you have to KEEP HOLDING that same value to 
    the input, until you do recieve a data_response that is non-zero. 

    THe data_response being 0 means that this module is preoccupied with something, and 
    cannot handle your request this timestep (for example, returning previously requested data)

    FOR STORES: 
    we will first load the cache line (evict anything that is in its way), then edit the cache line and send it for a store request
    */
);
    logic [TAG_BITS-1:0] addr_tag;
    logic [INDEX_BITS-1:0] addr_index;
    logic [OFFSET_BITS-1:0] addr_offset;
    EXAMPLE_CACHE_BLOCK c;

    //extract tag, index, and offset from address
    always_comb begin
        addr_tag    = proc2Dcache_addr[`XLEN-1 : INDEX_BITS + OFFSET_BITS];
        addr_index  = proc2Dcache_addr[INDEX_BITS + OFFSET_BITS - 1 : OFFSET_BITS];
        addr_offset = proc2Dcache_addr[OFFSET_BITS-1 : 0];
    end

    // cache information
    // logic [63:0] cache_data [0:63]; // 64 lines of 8 bytes
    // logic [TAG_BITS-1:0] cache_tag [0:63]; // 64 lines of tag bits
    // logic cache_valid [0:63]; // 64 lines of valid bits
    // logic cache_dirty [0:63]; // 64 lines of dirty bits

    logic store_load_queue; // this will be high if we had a store request that was a miss, and now we need to queue a load request
    logic [2:0] waiting;
    logic [2:0] next_waiting;
    // request information (if we have requests in flight)
    // logic [`XLEN-1:0] tag_to_addr [15:0];
    // logic tag_to_addr_valid [15:0];
    // logic [1:0] tag_to_memsize [15:0];
    // logic [63:0] tag_to_memdata [15:0]; // this is for stores exclusively
    // logic tag_to_is_store [15:0];

    logic [63:0] next_data_to_write; // this will be for when a load comes back, and we need to write it for store

    logic in_flight; //this will check if the incoming request is already inflight in a previous request

    // hit detection
    logic next_hit;
    logic [63:0] next_data;
    logic [3:0] next_data_tag;
    logic [3:0] next_data_response;

    logic [63:0] next_store_write;
    logic [31:0] next_addr;
    // logic [1:0] proc_command;
    logic [1:0] next_cur_command;
    logic [63:0] next_cur_data;

    logic next_halt_confirm;
    logic [3:0] tag_in_flight; // this is for storing back


    // state machine
    // logic [1:0] state; // this will track whether we are in a MISS state or IDLE state
    // logic [1:0] next_state; // this will track whether we are in a MISS state or IDLE state

    //eviction stuff
    // logic wb_eviction; // this will be high if we need to evict a DIRTY line from the cache
    // logic next_wb_eviction; // this will be high if we need to evict a DIRTY line from the cache
    logic [`XLEN-1:0] evict_addr;
    logic [63:0] evict_data;
    logic [`XLEN-1:0] next_evict_addr;
    logic [63:0] next_evict_data;

    //helpers
    logic [INDEX_BITS-1:0] current_tag_index;
    logic new_eviction;
    assign current_tag_index = tag_to_addr[mem2dcache_tag][`XLEN-1-TAG_BITS:OFFSET_BITS]; // this means nothing if tag_to_addr_valid is low
    logic early_eviction;
    always_comb begin

        // this eviction i can nly hold for ONE cycle, so i am introducing early eviction
        // next_wb_eviction = ((mem2dcache_tag != 0 && tag_to_addr_valid[mem2dcache_tag] == 1) &&
        //                     (cache_dirty[current_tag_index] == 1) && 
        //                     (cache_valid[current_tag_index] == 1));
        // next_evict_addr = {cache_tag[current_tag_index], current_tag_index, 3'b000};
        // next_evict_data = cache_data[current_tag_index];
        next_wb_eviction = 0;

        next_evict_addr = 0;
        next_evict_data = 0;
       
        in_flight = 0;
        next_halt_confirm = 0;
        for (int i = 0; i < 16; i++) begin
            if (tag_to_addr_valid[i] && tag_to_addr[i][`XLEN-1:3] == proc2Dcache_addr[`XLEN-1:3]) begin
                in_flight =1;
            end
        end
        if (in_flight == 0 && cache_valid[addr_index] && (cache_tag[addr_index] != addr_tag) && cache_dirty[addr_index]) begin
            next_evict_addr = {cache_tag[addr_index], addr_index, 3'b000};
            next_evict_data = cache_data[addr_index];
            next_wb_eviction = 1;
        end

        next_cur_command = proc2Dcache_command;
        next_cur_data = proc2Dcache_data;

        // hit only true if the cache is valid and theres no returning data
        next_hit = cache_valid[addr_index] && (cache_tag[addr_index] == addr_tag) 
                    && !(mem2dcache_tag != 0 && tag_to_addr_valid[mem2dcache_tag] == 1) 
                    && (proc2Dcache_command == BUS_LOAD || proc2Dcache_command == BUS_STORE);

        next_data = next_wb_eviction ? 0 :  (mem2dcache_tag != 0 && tag_to_addr_valid[mem2dcache_tag] == 1) ? 
                            mem2dcache_data
                            : cache_data[addr_index];
        next_data_response = next_wb_eviction ? 0 : (proc2Dcache_command == BUS_STORE & next_hit) ? next_hit : 
                            (state == `MISS) ? mem2dcache_response : 0;
        next_data_tag = (mem2dcache_tag != 0 && tag_to_addr_valid[mem2dcache_tag] == 1 && tag_to_is_store[mem2dcache_tag]) ? 
                            mem2dcache_tag
                            : 0;
        dcache2mem_command = BUS_NONE;
        dcache2mem_addr = 0;
        dcache2mem_data = 0;
        next_store_write = next_data;
        next_addr = proc2Dcache_addr;
        next_waiting = waiting-1;
        // we are not checking whether the tags of cache lines are the same, because we have other
        // precautionary features to prevent a LD request from happening if the addr is alr in flight
       
        tag_in_flight = 0;


        if (halt) begin
            next_halt_confirm =1;
            for (int i =0; i < 64; i++) begin
                if (cache_valid[i] && cache_dirty[i] == 1) begin
                    next_evict_addr = {cache_tag[i], i[INDEX_BITS-1:0], 3'b000};
                    next_evict_data = cache_data[i];
                    next_wb_eviction = 1;
                    next_halt_confirm = 0;
                end
            end
            for (int i =0; i < 16; i++) begin
                if (tag_to_addr_valid[i] && tag_to_is_store[i] == 1) begin
                    next_evict_addr = {tag_to_addr[i][`XLEN-1:3], 3'b000};
                    next_evict_data = tag_to_memdata[i];
                    next_wb_eviction = 1;
                    next_halt_confirm = 0;
                    tag_in_flight = i[3:0];
                end
            end
        end

        if (wb_eviction) begin
            dcache2mem_addr = evict_addr;
            dcache2mem_command = BUS_STORE;
            dcache2mem_data = evict_data;
            if (mem2dcache_response != 0) begin
                next_wb_eviction = 0;
            end
        end


        if (proc2Dcache_command == BUS_STORE && next_hit) begin
            c.byte_level = cache_data[addr_index];
            c.half_level = cache_data[addr_index];
            c.word_level = cache_data[addr_index];
            case (proc2Dcache_size) 
                    BYTE: begin
                        c.byte_level[proc2Dcache_addr[2:0]] = proc2Dcache_data[7:0];
                        next_store_write = c.byte_level;
                    end
                    HALF: begin
                        assert(proc2Dcache_addr[0] == 0);
                        c.half_level[proc2Dcache_addr[2:1]] = proc2Dcache_data[15:0];
                        next_store_write = c.half_level;
                    end
                    WORD: begin
                        assert(proc2Dcache_addr[1:0] == 0);
                        c.word_level[proc2Dcache_addr[2]] = proc2Dcache_data[31:0];
                        next_store_write = c.word_level;
                    end
                    default: begin
                        assert(proc2Dcache_addr[1:0] == 0);
                        c.byte_level[proc2Dcache_addr[2]] = proc2Dcache_data[31:0];
                        next_store_write = c.word_level;
                    end
                endcase
        end
        else if ((mem2dcache_tag != 0 && tag_to_addr_valid[mem2dcache_tag] == 1) && 
                tag_to_is_store[mem2dcache_tag] == 1) begin
                c.byte_level = mem2dcache_data;
                c.half_level = mem2dcache_data;
                c.word_level = mem2dcache_data;
                case (tag_to_memsize[mem2dcache_tag]) 
                    BYTE: begin
                        c.byte_level[tag_to_addr[mem2dcache_tag][2:0]] = tag_to_memdata[mem2dcache_tag][7:0];
                        next_data_to_write = c.byte_level;
                    end
                    HALF: begin
                        assert(tag_to_addr[mem2dcache_tag][0] == 0);
                        c.half_level[tag_to_addr[mem2dcache_tag][2:1]] = tag_to_memdata[mem2dcache_tag][15:0];
                        next_data_to_write = c.half_level;
                    end
                    WORD: begin
                        assert(tag_to_addr[mem2dcache_tag][1:0] == 0);
                        c.word_level[tag_to_addr[mem2dcache_tag][2]] = tag_to_memdata[mem2dcache_tag][31:0];
                        next_data_to_write = c.word_level;
                    end
                    default: begin
                        assert(tag_to_addr[mem2dcache_tag][1:0] == 0);
                        c.byte_level[tag_to_addr[mem2dcache_tag][2]] = tag_to_memdata[mem2dcache_tag][31:0];
                        next_data_to_write = c.word_level;
                    end
                endcase
                end
        else begin
        end
        case (state)
            `IDLE: begin
                if (next_hit || proc2Dcache_command == BUS_NONE) begin
                    next_state = `IDLE;
                end
                else if (in_flight || wb_eviction || next_wb_eviction) begin
                    next_state = `IDLE;
                end
                else begin
                    next_state = `MISS;
                end
            end
            `MISS: begin
                if (!wb_eviction && !next_wb_eviction) begin
                    dcache2mem_addr = {addr_tag, addr_index, 3'b000};
                    dcache2mem_command = BUS_LOAD;
                    dcache2mem_data = 0;
                    if (mem2dcache_response != 0) begin
                        next_state = `IDLE;
                    end
                    else begin
                        next_state = `MISS;
                    end
                end
                else begin
                    next_state = `MISS;
                end
            end
            default: begin
                next_state = `IDLE;
            end
        endcase
        

    end

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < 64; i++) begin
                cache_valid[i] <= 0;
                cache_dirty[i] <= 0;
                cache_tag[i] <= 0;
                cache_data[i] <= 0;
            end
            for (int i =0; i < 16; i++)begin
                tag_to_addr[i] <= 0;
                tag_to_addr_valid[i] <= 0;
                tag_to_memsize[i] <= 0;
                tag_to_memdata[i] <= 0;
                tag_to_is_store[i] <= 0;
            end
            state <= `IDLE;
            hit <= 0;
            hit_data <= 0;
            data_tag <= 0;
            data_response <= 0;
            wb_eviction <= 0;
            cur_addr <= 0;
        end
        else begin
            state <= next_state;
            hit <= next_hit;
            hit_data <= next_data;
            data_tag <= next_data_tag;
            data_response <= next_data_response;
            cur_addr <= next_addr;
            cur_command <= next_cur_command;
            cur_data <= next_cur_data;
            evict_addr <= next_evict_addr;
            evict_data <= next_evict_data;
            wb_eviction <= next_wb_eviction;

            halt_confirm <= next_halt_confirm;

            if (state == `MISS && !halt) begin
                if (mem2dcache_response != 0 && !wb_eviction) begin
                    tag_to_addr_valid[mem2dcache_response] <= 1;
                    tag_to_addr[mem2dcache_response] <= proc2Dcache_addr;
                    tag_to_memsize[mem2dcache_response] <= proc2Dcache_size;
                    tag_to_memdata[mem2dcache_response] <= proc2Dcache_data;
                    tag_to_is_store[mem2dcache_response] <= (proc2Dcache_command == BUS_STORE);
                end
            end
            if (proc2Dcache_command == BUS_STORE && !wb_eviction && !halt) begin
                if (next_hit) begin
                    cache_data[addr_index] <= next_store_write;
                    cache_dirty[addr_index] <= 1;
                end
                else begin
                    store_load_queue <= 1; // tells the dcache that we need to load a store queue
                end
            end

            if (wb_eviction && mem2dcache_response != 0) begin
                // if (waiting == 0) begin
                    if (tag_in_flight != 0 && halt) begin
                        tag_to_addr_valid[tag_in_flight] <= 0;
                        tag_to_is_store[tag_in_flight] <= 0;
                        // waiting <= 1;
                    end
                    else begin
                        cache_dirty[next_evict_addr[`XLEN - 1 - TAG_BITS:OFFSET_BITS]] <= 0;
                        // waiting <= 1;
                    end
                // end
                // else begin
                //     waiting <= next_waiting;
                // end
            end
        

            if (mem2dcache_tag != 0 && tag_to_addr_valid[mem2dcache_tag] == 1 && !halt) begin
                if (tag_to_is_store[mem2dcache_tag]==1)begin
                    cache_data[current_tag_index] <= next_data_to_write;
                    cache_dirty[current_tag_index] <= 1;
                end
                else begin
                    cache_data[current_tag_index] <= mem2dcache_data;
                    cache_dirty[current_tag_index] <= 0;

                    hit_data <= mem2dcache_data;
                    data_tag <= mem2dcache_tag;
                end
                cache_tag[current_tag_index] <= tag_to_addr[mem2dcache_tag][`XLEN-1:OFFSET_BITS+INDEX_BITS];
                cache_valid[current_tag_index] <= 1;
                tag_to_addr_valid[mem2dcache_tag] <= 0;

                tag_to_addr[mem2dcache_tag] <= 0;
                tag_to_memsize[mem2dcache_tag] <= 0;
                tag_to_memdata[mem2dcache_tag] <= 0;
                tag_to_is_store[mem2dcache_tag] <= 0;

            end
        end
    end


endmodule;