/*
dcache specifications:
    512 bytes
    directly mapped
    non-blocking
    write-through
    cache line size of 8 bytes, 32 lines
    16 bytes / 8 bytes = 2 = 1 offset length
    32 lines = 5 index bits
    for a 32 bit address length: { Tag(26 bits) | Index(5 bits) | Offset(1 bits) }
*/
`define IDLE 1'b0
`define MISS 1'b1
`define NONE 5'b00000
module dcache
#(
    TAG_BITS = 26,
    INDEX_BITS = 6,
    OFFSET_BITS = 0
)
(
    input clk;
    input reset;

    input [`XLEN-1:0] proc2Dcache_addr,

    input [3:0]  mem2proc_response, // 0 = can't accept, other=tag of transaction
    input [63:0] mem2proc_data,     // data resulting from a load
    input [3:0]  mem2proc_tag,       // 0 = no value, other=tag of transaction

    output [`XLEN-1:0] proc2mem_addr,
    output [63:0]      proc2mem_data, // address for current command
    output [1:0]       proc2mem_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE
    output MEM_SIZE    proc2mem_size,

    output logic [63:0] hit_data, // data resulting from a load
    output logic hit, // 1 if hit, 0 if miss
    output logic [3:0] data_tag,
    output logic data_response,
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

    logic [63:0] cache_data [0:63]; // 32 lines of 16 bytes
    logic [TAG_BITS-1:0] cache_tag [0:63]; // 32 lines of tag bits
    logic cache_valid [0:63]; // 32 lines of valid bits
    // this is for storing the tag -> addr from mem module
    logic [`XLEN-1:0] tag_addr [3:0]; 

    logic state;
    // this is so we can gauge how many addresses we are waiting for a response with
    logic [3:0] number_of_waits;
    logic [3:0] next_number_of_waits;
    logic [1:0] next_state;
    logic [`XLEN-1:0] saved_addr; //used for misses, saves the address of the missed lookup

    logic next_hit;
    logic [63:0] next_hit_data;
    logic [`XLEN-1:0] next_tag;
    
    always_comb begin
        next_hit_data = cache_data[addr_index]; //hit_data only valid if hit is high
        next_hit = (cache_tag[addr_index] == addr_tag) && cache_valid[addr_index];
        proc2mem_command = `BUS_NONE;
        proc2mem_addr = 32'b0;
        proc2mem_data = 64'b0;
        proc2mem_size = 1'b1;
        next_state = state;
        case (state)
            `IDLE: if (!next_hit) begin
                next_state = `MISS;
                proc2mem_command = `BUS_LOAD;
                proc2mem_addr = {proc2Dcache_addr[31:OFFSET_BITS]};
            end
            `MISS: if (mem2proc_response != 0) begin
                tag_addr[mem2proc_response] = saved_addr;
                next_tag = mem2proc_response;
                next_state = `IDLE;
                next_number_of_waits = number_of_waits + 1;
            end
        endcase

        case(number_of_waits) 
            `NONE: begin
            end
            default begin
                if (mem2proc_tag != 0) begin
                    next_number_of_waits = number_of_waits - 1;
                    proc2mem_command = `BUS_NONE;
                end
            end
        endcase

    end


    always_ff @(posedge clk) begin
        if (reset) begin
            //is there a better way to do this?
            integer i;
            for (i=0; i<32; i++) begin
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
            if (next_state == `MISS && !next_hit) begin 
                saved_addr <= proc2Dcache_addr;
            end
            if (next_state == `IDLE && mem2proc_tag != 0) begin
                cache_data[saved_addr[INDEX_BITS+OFFSET_BITS-1 : OFFSET_BITS]]  <= mem2proc_data;
                cache_tag[saved_addr[INDEX_BITS+OFFSET_BITS-1 : OFFSET_BITS]]   <= saved_addr[31:INDEX_BITS+OFFSET_BITS];
                cache_valid[saved_addr[INDEX_BITS+OFFSET_BITS-1 : OFFSET_BITS]] <= 1'b1;

                data_tag <= mem2proc_tag;
                hit_data <= mem2proc_data;
                data_response <= 0;
                // because we are returning other data so, hit is not going to be high, regardless of what it is before.
                hit <= 0;
            end
        end
    end


endmodule;