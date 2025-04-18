/*
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
    TAG_BITS = 26,
    INDEX_BITS = 6,
    OFFSET_BITS = 0
)
(
    input logic clk,
    input logic reset,

    input logic [`XLEN-1:0] proc2Dcache_addr,
    input logic [63:0] proc2Dcache_data, // datafor current command (if store)
    input logic [1:0] proc2Dcache_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE

    input logic [3:0]  mem2dcache_response, // 0 = can't accept, other=tag of transaction
    input logic [63:0] mem2dcache_data,     // data resulting from a load
    input logic [3:0]  mem2dcache_tag,       // 0 = no value, other=tag of transaction

    output logic [`XLEN-1:0] dcache2mem_addr,
    output logic [63:0]      dcache2mem_data, // address for current command
    output logic [1:0]       dcache2mem_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE
    output MEM_SIZE    dcache2mem_size,

    output logic [63:0] hit_data, // data resulting from a load
    output logic hit, // 1 if hit, 0 if miss
    output logic [3:0] data_tag,
    output logic [3:0] data_response,

    output logic next_state, //for debugging 
    output logic state, //for debugging 
    output logic [3:0] number_of_waits, //for debugging
    output logic [3:0] next_number_of_waits //for debugging
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
    */
);
    logic [TAG_BITS-1:0] addr_tag;
    logic [INDEX_BITS-1:0] addr_index;
    // logic [OFFSET_BITS-1:0] addr_offset;

    //extract tag, index, and offset from address
    always_comb begin
        addr_tag    = proc2Dcache_addr[`XLEN-1 : INDEX_BITS + OFFSET_BITS];
        addr_index  = proc2Dcache_addr[INDEX_BITS + OFFSET_BITS - 1 : OFFSET_BITS];
        //addr_offset = proc2Dcache_addr[OFFSET_BITS-1 : 0];
    end

    logic [63:0] cache_data [0:63]; // 64 lines of 8 bytes
    logic [TAG_BITS-1:0] cache_tag [0:63]; // 64 lines of tag bits
    logic cache_valid [0:63]; // 64 lines of valid bits
    // this is for storing the tag -> addr from mem module
    logic [`XLEN-1:0] tag_addr [3:0]; 
    logic tag_addr_valid [3:0]; // 4 lines of tag bits

    logic [3:0] prev_tag; //we dont need this if we can guarentee the mem module's tag is only HIGH for ONE CYCLE

    // logic state;
    // logic next_state;

    // // this is so we can gauge how many addresses we are waiting for a response with
    // logic [3:0] number_of_waits;
    // logic [3:0] next_number_of_waits;
    logic [`XLEN-1:0] saved_addr; //used for misses, saves the address of the missed lookup

    logic next_hit;
    logic [63:0] next_hit_data;
    logic [`XLEN-1:0] next_tag;
    
    always_comb begin
        next_hit_data = cache_data[addr_index]; //hit_data only valid if hit is high
        next_hit = (cache_tag[addr_index] == addr_tag) && cache_valid[addr_index] && 
                    (proc2Dcache_command == BUS_LOAD || proc2Dcache_command == BUS_STORE);
        dcache2mem_command = BUS_NONE;
        dcache2mem_addr = 32'b0;
        dcache2mem_data = 64'b0;
        dcache2mem_size = 1'b1;
        next_number_of_waits = number_of_waits;
        next_state = state;
        next_tag = 0;
        //ok so the reason i have an idle and miss state, is because when we send a request to memory,
        // im assuming we will get the tag ONLY 

        if (next_hit && proc2Dcache_command == BUS_STORE) begin
            // update the word in the cache line (entire 64‑bit for now)
            next_hit_data = proc2Dcache_data;
            // drive the bus
            dcache2mem_command = BUS_STORE;
            dcache2mem_addr = {proc2Dcache_addr[`XLEN-1:OFFSET_BITS]};
            dcache2mem_data = proc2Dcache_data;
            // keep size = 1 (8‑B line) for now
        end
        else begin
            dcache2mem_command = BUS_STORE;
            dcache2mem_addr = {proc2Dcache_addr[`XLEN-1:OFFSET_BITS]};
            dcache2mem_data = proc2Dcache_data;
        end
        case (state)
            `IDLE: begin
                // so i am not doing some standard operation of load and storing,
                // i am just sending the store request directly to memory, and saving the store request in local cache
                if (!next_hit && (proc2Dcache_command == BUS_LOAD) ) begin
                    next_state = `MISS;
                    dcache2mem_command = BUS_LOAD;
                    dcache2mem_addr = {proc2Dcache_addr[`XLEN-1:OFFSET_BITS]};
                end
                else begin
                end
            end
            `MISS: begin
                //keep driving the bus until we get a response
                if (mem2dcache_response == 0) begin
                    dcache2mem_command = BUS_LOAD;
                    dcache2mem_addr    = {saved_addr[`XLEN-1:OFFSET_BITS]};
                end 
                else begin
                    tag_addr[mem2dcache_response] = saved_addr;
                    next_tag = mem2dcache_response;
                    next_state = `IDLE;
                    next_number_of_waits = number_of_waits + 1;
                    
                end
            end
        endcase

        case(number_of_waits) 
            `NONE: begin
            end
            default begin
                if (mem2dcache_tag != 0) begin
                    // this if statement is to check if the tag we are waiting for is the same as the one we are getting
                    // this is caused if the mem module is sending us a tag for more than one cycle
                    if (mem2dcache_tag != prev_tag && tag_addr_valid[mem2dcache_tag]) begin
                        next_number_of_waits = number_of_waits - 1;
                    end
                end
                else begin 
                end
            end
        endcase


    end


    always_ff @(posedge clk) begin
        if (reset) begin
            //is there a better way to do this?
            integer i;
            for (i=0; i<64; i++) begin
                cache_data[i]  <= 64'b0;
                cache_tag[i]   <= {TAG_BITS{1'b0}};
                cache_valid[i] <= 1'b0;
            end
            state       <= `IDLE;
            saved_addr  <= 32'b0;
            number_of_waits <= `NONE;
            prev_tag <= 0;
            for (i = 0; i < 4; i++) tag_addr_valid[i] <= 1'b0;

        end
        else begin
            state <= next_state;
            number_of_waits <= next_number_of_waits;
            data_tag <= 0;
            data_response <= next_tag;
            hit_data <= next_hit_data;
            hit <= next_hit;
            if (state == `MISS) begin 
                // so the states where this loop will execute is:
                // when the user sends a request to the dcache, and it is a miss,
                // therefore, this is returned. 
                saved_addr <= proc2Dcache_addr;
                tag_addr_valid[next_tag] <= 1'b1;
                // okay we need to make sure to only set this saved_addr once!
            end
            else begin 
            end
            // this can happen regardless of miss or hit, so we will check for it here
            if (mem2dcache_tag != 0 && number_of_waits != `NONE && tag_addr_valid[mem2dcache_tag]) begin
                prev_tag <= mem2dcache_tag;
                cache_data[tag_addr[mem2dcache_tag][INDEX_BITS + OFFSET_BITS - 1 : OFFSET_BITS]]  <= mem2dcache_data;
                cache_tag[tag_addr[mem2dcache_tag][INDEX_BITS + OFFSET_BITS - 1 : OFFSET_BITS]]   <= tag_addr[mem2dcache_tag][`XLEN-1 : INDEX_BITS + OFFSET_BITS];
                cache_valid[tag_addr[mem2dcache_tag][INDEX_BITS + OFFSET_BITS - 1 : OFFSET_BITS]] <= 1'b1;
                //tag is not valid no more
                tag_addr_valid[mem2dcache_tag] <= 1'b0;

                data_tag <= mem2dcache_tag;
                hit_data <= mem2dcache_data;
                data_response <= 0;
                // because we are returning other data so, hit is not going to be high, regardless of what it is before.
                hit <= 0;
            end
            else begin 
            end
            if (proc2Dcache_command == BUS_STORE) begin
                // are we storing to a line that is a miss?
                
                // next_hit just tells us this address line was originally here too
                if (next_hit) begin
                    cache_valid[addr_index] <= 1'b1;
                    cache_tag[addr_index] <= addr_tag;
                    cache_data[addr_index] <= proc2Dcache_data;
                end
                else begin
                    //do we wanna store for misses too?
                    cache_valid[addr_index] <= 1'b1;
                    cache_tag[addr_index] <= addr_tag;
                    cache_data[addr_index] <= proc2Dcache_data;
                end
                
            end
            else begin
            end
        end
    end


endmodule;