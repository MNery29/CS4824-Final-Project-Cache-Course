/*
dcache specifications:
    2kbytes
    8 ways, 32 sets
    non-blocking
    write-back
    set-associative
    z-cache design -- https://people.csail.mit.edu/sanchez/papers/2010.zcache.micro.pdf
    h3 hash functions
*/

//NOTE: This was a planned advanced feature that didn't have time to finish

typedef union packed {
    logic [63:0] double;
    logic [1:0][31:0] words;
    logic [3:0][15:0] halves;
    logic [7:0][7:0] bytes;
} CACHE_BLOCK;

module zcache #(
    parameter integer WAYS  = 8,
    parameter integer SETS_WIDTH = 5, // 32 so 2^5
    parameter integer SETS = 32
)
(
    input clk,
    input reset,
    
    input [`XLEN-1:0] proc2Dcache_addr,

    input [3:0]  mem2proc_response, // 0 = can't accept, other=tag of transaction
    input [63:0] mem2proc_data,     // data resulting from a load
    input [3:0]  mem2proc_tag,       // 0 = no value, other=tag of transaction

    output [`XLEN-1:0] proc2mem_addr,
    output [63:0]      proc2mem_data, // address for current command
    output [1:0]       proc2mem_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE
    output MEM_SIZE    proc2mem_size

);
    logic [WAYS*SETS_WIDTH-1:0] hashed_indices;
    //hash func gives us the different hashed indices for the different ways
    h3hash hash_func(
        .index_in (proc2Dcache_addr),
        .hashed_indices (hashed_indices)
    );

    logic [64 * WAYS * SETS - 1:0] cache_data;
    logic[WAYS * SETS-1:0] cache_data_valid;
    logic[`XLEN * WAYS * SETS-1:0] cache_data_tag;

    logic [WAYS*SETS-1:0] lru; // keep track of least recently used to EVICT

    logic hit;
    logic[`XLEN-1:0] hit_data;

    genvar i;
    //checks if the data is in the cache
    //hit will be high if it is
    generate
        always_comb begin
            hit = 1'b0;
            hit_data = 32'b0;
            for (i=0; i < WAYS; i = i+1) begin
                
                    if (cache_data_valid[i*SETS + hashed_indices[(i+1)*SETS_WIDTH-1:i*SETS_WIDTH]] &
                        (proc2Dcache_addr == 
                        cache_data_tag[`XLEN*i*SETS + (hashed_indices[(i+1)*SETS_WIDTH-1 : i*SETS_WIDTH]+1)*`XLEN -1:
                        `XLEN*i*SETS + (hashed_indices[(i+1)*SETS_WIDTH-1 : i*SETS_WIDTH])*`XLEN])) begin
                        hit = 1;
                        hit_data = 
                            cache_data
                            [64*i*SETS + (hashed_indices[(i+1)*SETS_WIDTH-1 : i*SETS_WIDTH]+1)*64 -1:
                            64*i*SETS + (hashed_indices[(i+1)*SETS_WIDTH-1 : i*SETS_WIDTH])*64];
                        lru[i*SETS + hashed_indices[(i+1)*SETS_WIDTH-1:i*SETS_WIDTH]] 
                            = lru[i*SETS + hashed_indices[(i+1)*SETS_WIDTH-1:i*SETS_WIDTH]] +1;
                end
            end
        end
    endgenerate
    



    


endmodule