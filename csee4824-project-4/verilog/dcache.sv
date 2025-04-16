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

    input logic [3:0]  mem2proc_response, // 0 = can't accept, other=tag of transaction
    input logic [63:0] mem2proc_data,     // data resulting from a load
    input logic [3:0]  mem2proc_tag,       // 0 = no value, other=tag of transaction

    output logic [`XLEN-1:0] proc2mem_addr,
    output logic [63:0]      proc2mem_data, // address for current command
    output logic [1:0]       proc2mem_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE
    output MEM_SIZE    proc2mem_size,

    output logic [63:0] hit_data, // data resulting from a load
    output logic hit, // 1 if hit, 0 if miss
    output logic [3:0] data_tag,
    output logic [3:0] data_response
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

    logic state;
    logic [1:0] next_state;

    // this is so we can gauge how many addresses we are waiting for a response with
    logic [3:0] number_of_waits;
    logic [3:0] next_number_of_waits;
    logic [`XLEN-1:0] saved_addr; //used for misses, saves the address of the missed lookup

    logic next_hit;
    logic [63:0] next_hit_data;
    logic [`XLEN-1:0] next_tag;
    
    always_comb begin
        next_hit_data = cache_data[addr_index]; //hit_data only valid if hit is high
        next_hit = (cache_tag[addr_index] == addr_tag) && cache_valid[addr_index];
        proc2mem_command = BUS_NONE;
        proc2mem_addr = 32'b0;
        proc2mem_data = 64'b0;
        proc2mem_size = 1'b1;
        next_number_of_waits = number_of_waits;
        next_state = state;
        //ok so the reason i have an idle and miss state, is because when we send a request to memory,
        // im assuming we will get the tag ONLY 
        case (state)
            `IDLE: if (!next_hit) begin
                next_state = `MISS;
                proc2mem_command = BUS_LOAD;
                proc2mem_addr = {proc2Dcache_addr[`XLEN-1:OFFSET_BITS]};
                
            end
            `MISS: begin
                if (mem2proc_response != 0) begin
                    tag_addr[mem2proc_response] = saved_addr;
                    next_tag = mem2proc_response;
                    next_state = `IDLE;
                    next_number_of_waits = number_of_waits + 1;
                    
                end
                else begin
                    next_hit = 0;
                end
            end
        endcase

        case(number_of_waits) 
            `NONE: begin
            end
            default begin
                if (mem2proc_tag != 0) begin
                    next_number_of_waits = number_of_waits - 1;
                    proc2mem_command = BUS_NONE;
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
                // okay we need to make sure to only set this saved_addr once!
                data_response <= 0;
                hit <= 0;
            end
            // this can happen regardless of miss or hit, so we will check for it here
            if (mem2proc_tag != 0 && number_of_waits != `NONE) begin
                cache_data[tag_addr[mem2proc_tag][INDEX_BITS + OFFSET_BITS - 1 : OFFSET_BITS]]  <= mem2proc_data;
                cache_tag[tag_addr[mem2proc_tag][INDEX_BITS + OFFSET_BITS - 1 : OFFSET_BITS]]   <= tag_addr[mem2proc_tag][`XLEN-1 : INDEX_BITS + OFFSET_BITS];
                cache_valid[tag_addr[mem2proc_tag][INDEX_BITS + OFFSET_BITS - 1 : OFFSET_BITS]] <= 1'b1;

                data_tag <= mem2proc_tag;
                hit_data <= mem2proc_data;
                data_response <= 0;
                // because we are returning other data so, hit is not going to be high, regardless of what it is before.
                hit <= 0;
            end
            else begin 
            end
        end
    end


endmodule;