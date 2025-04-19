`include "verilog/sys_defs.svh"
`timescale 1ns/1ps

module testbench;

    logic clk;
    logic reset;
    logic[`XLEN-1:0] proc2Dcache_addr;
    logic [1:0] proc2Dcache_command;
    logic [3:0] mem2proc_response; // 0 = can't accept, other=tag of transaction
    logic [63:0] proc2Dcache_data; // data for current command (if store)
    logic [63:0] mem2proc_data;     // data resulting from a load
    logic [3:0] mem2proc_tag;       // 0 = no value, other=tag of transaction

    logic [`XLEN-1:0] proc2mem_addr;
    logic [63:0] proc2mem_data; // address for current command
    logic [1:0] proc2mem_command; // `BUS_NONE `BUS_LOAD or `BUS_STORE

    MEM_SIZE proc2mem_size;
    logic [63:0] hit_data; // data resulting from a load
    logic hit;
    logic [3:0] data_tag;
    logic [3:0] data_response;
    logic [3:0] number_of_waits;
    logic [3:0] next_number_of_waits;
    logic next_state;
    logic state;


    dcache data_cache(
        .clk(clk),
        .reset(reset),

        .proc2Dcache_addr(proc2Dcache_addr),
        .proc2Dcache_data(proc2Dcache_data),
        .proc2Dcache_command(proc2Dcache_command),

        .mem2dcache_response(mem2proc_response), // matching module port name
        .mem2dcache_data(mem2proc_data),
        .mem2dcache_tag(mem2proc_tag),

        .dcache2mem_addr(proc2mem_addr), // matching module port name
        .dcache2mem_data(proc2mem_data),
        .dcache2mem_command(proc2mem_command),
        .dcache2mem_size(proc2mem_size),

        .hit_data(hit_data),
        .hit(hit),
        .data_tag(data_tag),
        .data_response(data_response),
        .next_state(next_state),
        .state(state),
        .number_of_waits(number_of_waits),
        .next_number_of_waits(next_number_of_waits)
);

    // clk_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clk = ~clk;
    end

    task show_signals;
        $display("[%0t ns] \n clk=%b \n reset=%b | proc2Dcache_addr=0x%h | " ,$time, clk, reset, proc2Dcache_addr);
        $display("           mem2proc_response=%b mem2proc_data=0x%h mem2proc_tag=%b", 
                mem2proc_response, mem2proc_data, mem2proc_tag);
        $display("           proc2mem_addr=0x%h proc2mem_data=0x%h proc2mem_command=%b proc2mem_size=%b",
                proc2mem_addr, proc2mem_data, proc2mem_command, proc2mem_size);
        $display("           hit=%b hit_data=0x%h data_tag=%b data_response=%b",
                hit, hit_data, data_tag, data_response);
        $display("           next_state=%b state=%b number_of_waits=%b next_number_of_waits=%b",
                next_state, state, number_of_waits, next_number_of_waits);
        $display("--------------------------------------------------------------------");
    endtask

    initial begin
        $monitor("Time:%4.0f\n clock:%b\n reset:%b\n proc2Dcache_addr:%h\n proc2Dcache_data:%h\n mem2proc_response:%b\n mem2proc_data:%h\n mem2proc_tag:%b\n proc2mem_addr:%h\n proc2mem_data:%h\n proc2mem_command:%b\n proc2mem_size:%b\n hit_data:%h\n hit:%b\n data_tag:%b\n data_response:%b\n next_state:%b\n state:%b\n number_of_waits:%b\n next_number_of_waits:%b\n",
            $time, clk, reset,
            proc2Dcache_addr, proc2Dcache_data,
            mem2proc_response, mem2proc_data, mem2proc_tag,
            proc2mem_addr, proc2mem_data, proc2mem_command, proc2mem_size,
            hit_data, hit, data_tag, data_response, next_state, state,
            number_of_waits, next_number_of_waits
            );


        //Reset 
        clk               = 0;
        reset             = 1;
        proc2Dcache_addr  = 32'h00000000;
        mem2proc_response = 4'b0;
        mem2proc_data     = 64'h0;
        mem2proc_tag      = 4'b0;

        //wait a couple cycles
        repeat (2) @(negedge clk);
        show_signals();

        reset = 0;
        @(negedge clk);
        $display("---- Trying to access arbritry request ----");
        proc2Dcache_addr = 32'h0000_0010;  // some arbitrary address
        proc2Dcache_command = BUS_LOAD; // load command
        @(negedge clk);
        show_signals();

        // Wait a couple cycles to see the MISS state
        repeat (2) @(negedge clk);
        proc2Dcache_command = BUS_NONE;
        show_signals();

        // Now we emulate memory returning a valid response:
        // The dcache design expects mem2proc_response != 0 to accept the request
        // Then a cycle or two later, mem2proc_tag != 0 + mem2proc_data
        // will cause the cache to store it.
        //------------------------------------------------------
        // 1. LOAD miss (addr 0x0010) -> memory fetch -> HIT check
        //------------------------------------------------------
        $display("\n-- 1. LOAD miss addr=0x10 --");
        proc2Dcache_addr    = 32'h0000_0010;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        // expect MISS => dcache should issue BUS_LOAD to memory
        assert(proc2mem_command == BUS_LOAD)
            else $fatal("Expected BUS_LOAD to memory on miss");

        // memory accepts with tag=1
        mem2proc_response = 4'd1;
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);
        mem2proc_response = '0; // clear
        @(negedge clk);

        // memory returns data (tag 1)
        mem2proc_tag  = 4'd1;
        mem2proc_data = 64'hDEAD_BEEF_0000_0010;
        @(negedge clk);
        mem2proc_tag  = '0;
        mem2proc_data = '0;
        @(negedge clk);

        // now the line should be in cache; LOAD again => HIT
        proc2Dcache_addr    = 32'h0000_0010;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        assert(hit && hit_data == 64'hDEAD_BEEF_0000_0010)
            else $fatal("Expected load hit with correct data after fill");
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);

        //------------------------------------------------------
        // 2. STORE hit (overwrite same line) + verify via LOAD
        //------------------------------------------------------
        $display("\n-- 2. STORE hit addr=0x10 --");
        proc2Dcache_addr    = 32'h0000_0010;
        proc2Dcache_data   = 64'hCAFE_BABE_CAFE_BABE;
        proc2Dcache_command = BUS_STORE;
        @(negedge clk);
        // Expect HIT (write‑through => immediate mem access)
        assert(hit) else $fatal("Expected store hit");
        // cache may mark block dirty but should not raise BUS_STORE immediately
        assert(proc2mem_command == BUS_STORE)
            else $fatal("Unexpected memory transaction on store hit");
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);

        // LOAD back – should return the new data we just wrote
        proc2Dcache_addr    = 32'h0000_0010;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        assert(hit && hit_data == 64'hCAFE_BABE_CAFE_BABE)
            else $fatal("Store data not returned on subsequent load");
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);

        //------------------------------------------------------
        // 3. STORE miss (addr 0x0020) – tests write‑allocate path
        //------------------------------------------------------
        $display("\n-- 3. STORE miss addr=0x20 --");
        proc2Dcache_addr    = 32'h0000_0020;
        proc2Dcache_data   = 64'hFEED_FACE_FEED_FACE;
        proc2Dcache_command = BUS_STORE;
        @(negedge clk);
        // On miss the cache should allocate line: expect BUS_STORE to memory
        assert(proc2mem_command == BUS_STORE)
            else $fatal("Expected cache to fetch line on store miss (write‑allocate)");
        // Memory accepts tag=2
        mem2proc_response = 4'd2;
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);
        mem2proc_response = '0;
        @(negedge clk);
        // Memory returns data so cache can merge write
        mem2proc_tag  = 4'd2;
        mem2proc_data = 64'hDEAD_BEEF_0000_0020;
        @(negedge clk);
        mem2proc_tag  = '0;
        mem2proc_data = '0;
        @(negedge clk);
        // After allocation + write, subsequent LOAD should hit new value
        proc2Dcache_addr    = 32'h0000_0020;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        assert(hit && hit_data == 64'hFEED_FACE_FEED_FACE)
            else $fatal("Store miss data not observed on later load");
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);


        //------------------------------------
        // Done
        //------------------------------------
        $display("---- Test complete ----");
        $finish;

        $finish;

    end

endmodule
