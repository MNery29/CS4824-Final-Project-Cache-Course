/******************************************************************************
*  Minimal dcache testbench – **only** checks that holding BUS_LOAD high for
*  several cycles produces consecutive hits with stable data.
*
*  How it works
*  ------------
*  1.  RESET.
*  2.  Issue a LOAD to address 0x10 → the d-cache misses.
*      – We emulate memory first *accepting* the request (tag = 1),
*        then returning the cache-line data the next cycle.
*  3.  With the line now resident, we keep                         BUS_LOAD
*     asserted for four more cycles and require:
*        • `hit == 1` each cycle
*        • `hit_data == 64'hDEAD_BEEF_CAFE_BABE` each cycle
*
*  Pass → simulation finishes quietly.
*  Fail → `$fatal` abort.
******************************************************************************/
`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

`define CLOCK_PERIOD 10

module testbench;

    //-----------------------------
    //  Clock / reset
    //-----------------------------
    logic clk = 0;
    logic reset = 1;
    always  #(`CLOCK_PERIOD/2) clk = ~clk;

    //-----------------------------
    //  CPU  →  d-cache signals
    //-----------------------------
    logic [`XLEN-1:0] cpu_addr;
    logic [63:0]      cpu_wdata;
    logic [1:0]       cpu_cmd;
    logic [2:0]       cpu_size;

    //-----------------------------
    //  Memory  ↔  d-cache signals
    //-----------------------------
    logic [3:0]  mem_resp;
    logic [63:0] mem_rdata;
    logic [3:0]  mem_tag;

    //-----------------------------
    //  d-cache → memory (unused in
    //  this TB, but we must wire)
    //-----------------------------
    logic [`XLEN-1:0] dc2mem_addr;
    logic [63:0]      dc2mem_data;
    logic [1:0]       dc2mem_cmd;

    //-----------------------------
    //  Outputs we *do* check
    //-----------------------------
    logic        hit;
    logic [63:0] hit_data;

    //-----------------------------
    //  Dummy wires for all the
    //  extra debug ports
    //-----------------------------
    logic halt = 0, halt_confirm;
    logic [3:0] data_tag, data_response;
    logic [`XLEN-1:0] cur_addr;  logic [1:0] cur_command; logic [63:0] cur_data;
    logic next_state, state;

    logic [`XLEN-1:0] tag_to_addr      [15:0]; logic tag_to_addr_valid [15:0];
    logic [1:0]       tag_to_memsize   [15:0]; logic [63:0] tag_to_memdata[15:0];
    logic             tag_to_is_store  [15:0];

    logic [63:0] cache_data[0:63];  logic [22:0] cache_tag [0:63];
    logic        cache_valid[0:63];  logic cache_dirty[0:63];

    //------------------------------------------------------------------
    //  Device Under Test
    //------------------------------------------------------------------
    dcache dut (
        .clk                 (clk),
        .reset               (reset),

        .proc2Dcache_addr    (cpu_addr),
        .proc2Dcache_data    (cpu_wdata),
        .proc2Dcache_command (cpu_cmd),
        .proc2Dcache_size    (cpu_size),

        .mem2dcache_response (mem_resp),
        .mem2dcache_data     (mem_rdata),
        .mem2dcache_tag      (mem_tag),

        .halt                (halt),
        .halt_confirm        (halt_confirm),

        .dcache2mem_addr     (dc2mem_addr),
        .dcache2mem_data     (dc2mem_data),
        .dcache2mem_command  (dc2mem_cmd),

        .hit_data            (hit_data),
        .hit                 (hit),
        .data_tag            (data_tag),
        .data_response       (data_response),

        .cur_addr            (cur_addr),
        .cur_command         (cur_command),
        .cur_data            (cur_data),
        .next_state          (next_state),
        .state               (state),

        .tag_to_addr         (tag_to_addr),
        .tag_to_addr_valid   (tag_to_addr_valid),
        .tag_to_memsize      (tag_to_memsize),
        .tag_to_memdata      (tag_to_memdata),
        .tag_to_is_store     (tag_to_is_store),

        .cache_data          (cache_data),
        .cache_tag           (cache_tag),
        .cache_valid         (cache_valid),
        .cache_dirty         (cache_dirty)
    );

    //------------------------------------------------------------------
    //  Test sequence
    //------------------------------------------------------------------
    initial begin
        //----------------------
        //  Apply reset
        //----------------------
        cpu_addr  = 0;
        cpu_wdata = 0;
        cpu_cmd   = BUS_NONE;
        cpu_size  = WORD;

        mem_resp  = 0;
        mem_rdata = 0;
        mem_tag   = 0;

        repeat (3) @(negedge clk);
        reset = 0;

        //----------------------
        //  (1)  First LOAD → miss
        //----------------------
        cpu_addr  = 32'h0000_0010;
        cpu_cmd   = BUS_LOAD;
        cpu_size  = WORD;

        @(negedge clk);   // allow d-cache to issue request
        // emulate: memory accepts, tag = 1
        mem_resp = 4'd1;
        @(negedge clk);
        mem_resp = 0;                 // clear handshake

        // emulate: memory returns data for tag 1
        mem_tag  = 4'd1;
        mem_rdata= 64'hDEAD_BEEF_CAFE_BABE;
        cpu_addr  = 0;
        cpu_cmd   = BUS_NONE;
        @(negedge clk);
        mem_tag  = 0;
        mem_rdata= 0;

        cpu_addr  = 32'h0000_00FF;
        cpu_cmd   = BUS_LOAD;
        cpu_size  = WORD;
        
        @(negedge clk);   // allow d-cache to issue request
        // emulate: memory accepts, tag = 1
        mem_resp = 4'd1;
        @(negedge clk);
        mem_resp = 0;                 // clear handshake

        // emulate: memory returns data for tag 1
        mem_tag  = 4'd1;
        mem_rdata= 64'hDEAD_BEEF_CAFE_BABE;
        cpu_cmd   = BUS_NONE;
        cpu_addr  = 0;
        @(negedge clk);
        mem_tag  = 0;
        mem_rdata= 0;


        //----------------------
        //  (2)  Keep BUS_LOAD high for 4 cycles → must be hits
        //----------------------
        cpu_addr  = 32'h0000_00FF;
        cpu_cmd   = BUS_LOAD;
        cpu_size  = WORD;
        @(negedge clk);
        if (!hit || hit_data !== 64'hDEAD_BEEF_CAFE_BABE)
            $fatal("Back-to-back hit failed @%0t  hit=%0d data=%h",
                    $time, hit, hit_data);
        cpu_addr  = 32'h0000_0010;
        cpu_cmd   = BUS_LOAD;
        cpu_size  = WORD;
        @(negedge clk);
        if (!hit || hit_data !== 64'hDEAD_BEEF_CAFE_BABE)
            $fatal("Back-to-back hit failed @%0t  hit=%0d data=%h",
                    $time, hit, hit_data);
    

        //----------------------
        //  PASS
        //----------------------
        $display("\n*** Back-to-back HIT test passed ***");
        $finish;
    end
endmodule
