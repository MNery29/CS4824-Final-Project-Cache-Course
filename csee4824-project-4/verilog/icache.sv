/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  icache.sv                                           //
//                                                                     //
//  Description :  The instruction cache module that reroutes memory   //
//                 accesses to decrease misses.                        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

// Internal macros, no other file should need these
`define CACHE_LINES 32
`define CACHE_LINE_BITS $clog2(`CACHE_LINES)

typedef struct packed {
    logic [63:0]                  data;
    // (13 bits) since only need 16 bits to access all memory and 3 are the offset
    logic [12-`CACHE_LINE_BITS:0] tags;
    logic                         valid;
} ICACHE_ENTRY;

/**
 * A quick overview of the cache and memory:
 *
 * We've increased the memory latency from 1 cycle to 100ns. which will be
 * multiple cycles for any reasonable processor. Thus, memory can have multiple
 * transactions pending and coordinates them via memory tags (different meaning
 * than cache tags) which represent a transaction it's working on. Memory tags
 * are 4 bits long since 15 mem accesses can be live at one time, and only one
 * access happens per cycle.
 *
 * On a request, memory *responds* with the tag it will use for that request
 * then ceiling(100ns/clock period) cycles later, it will return the data with
 * the corresponding tag. The 0 tag is a sentinel value and unused. It would be
 * very difficult to push your clock period past 100ns/15=6.66ns, so 15 tags is
 * sufficient.
 *
 * This cache coordinates those memory tags to speed up fetching reused data.
 *
 * Note that this cache is blocking, and will wait on one memory request before
 * sending another (unless the input address changes, in which case it abandons
 * that request). Implementing a non-blocking cache can count towards simple
 * feature points, but will require careful management of memory tags.
 */

module icache (
    input clock,
    input reset,

    // From memory
    input [3:0]  Imem2proc_response, // Should be zero unless there is a response
    input [63:0] Imem2proc_data,
    input [3:0]  Imem2proc_tag,

    // From fetch stage
    input [`XLEN-1:0] proc2Icache_addr,

    // To memory
    output logic [1:0]       proc2Imem_command,
    output logic [`XLEN-1:0] proc2Imem_addr,

    // To fetch stage
    output logic [63:0] Icache_data_out, // Data is mem[proc2Icache_addr]
    output logic        Icache_valid_out // When valid is high
);

    // ---- Cache data ---- //

    ICACHE_ENTRY [`CACHE_LINES-1:0] icache_data;

    // ---- Addresses and final outputs ---- //

    // Note: cache tags, not memory tags
    logic [12-`CACHE_LINE_BITS:0] current_tag, last_tag;
    logic [`CACHE_LINE_BITS - 1:0] current_index, last_index;

    assign {current_tag, current_index} = proc2Icache_addr[15:3];

    assign Icache_data_out = icache_data[current_index].data;
    assign Icache_valid_out = icache_data[current_index].valid &&
                              (icache_data[current_index].tags == current_tag);

    // ---- Main cache logic ---- //

    logic [3:0] current_mem_tag; // The current memory tag we might be waiting on
    logic miss_outstanding; // Whether a miss has received its response tag to wait on

    wire got_mem_data = (current_mem_tag == Imem2proc_tag) && (current_mem_tag != 0);

    wire changed_addr = (current_index != last_index) || (current_tag != last_tag);

    // Set mem tag to zero if we changed_addr, and keep resetting while there is
    // a miss_outstanding. Then set to zero when we got_mem_data.
    // (this relies on Imem2proc_response being zero when there is no request)
    wire update_mem_tag = changed_addr || miss_outstanding || got_mem_data;

    // If we have a new miss or still waiting for the response tag, we might
    // need to wait for the response tag because dcache has priority over icache
    wire unanswered_miss = changed_addr ? !Icache_valid_out
                                        : miss_outstanding && (Imem2proc_response == 0);

    // Keep sending memory requests until we receive a response tag or change addresses
    assign proc2Imem_command = (miss_outstanding && !changed_addr) ? BUS_LOAD : BUS_NONE;
    assign proc2Imem_addr    = {proc2Icache_addr[31:3],3'b0};

    // ---- Cache state registers ---- //

    always_ff @(posedge clock) begin
        if (reset) begin
            last_index       <= -1; // These are -1 to get ball rolling when
            last_tag         <= -1; // reset goes low because addr "changes"
            current_mem_tag  <= 0;
            miss_outstanding <= 0;
            icache_data      <= 0; // Set all cache data to 0 (including valid bits)
        end else begin
            last_index       <= current_index;
            last_tag         <= current_tag;
            miss_outstanding <= unanswered_miss;
            if (update_mem_tag) begin
                current_mem_tag <= Imem2proc_response;
            end
            if (got_mem_data) begin // If data came from memory, meaning tag matches
                icache_data[current_index].data  <= Imem2proc_data;
                icache_data[current_index].tags  <= current_tag;
                icache_data[current_index].valid <= 1;
            end
        end
    end

endmodule // icache
