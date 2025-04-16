`include "verilog/sys_defs.svh"
`timescale 1ns/1ps


module testbench;

    logic clk;
    logic reset;
    logic[`XLEN-1:0] proc2Dcache_addr;
    logic [3:0] mem2proc_response; // 0 = can't accept, other=tag of transaction
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


    dcache data_cache(
        .clk(clk),
        .reset(reset),
        .proc2Dcache_addr(proc2Dcache_addr),
        .mem2proc_response(mem2proc_response), // 0 = can't accept, other=tag of transaction
        .mem2proc_data(mem2proc_data),     // data resulting from a load
        .mem2proc_tag(mem2proc_tag),       // 0 = no value, other=tag of transaction
        .proc2mem_addr(proc2mem_addr),
        .proc2mem_data(proc2mem_data), // address for current command
        .proc2mem_command(proc2mem_command), // `BUS_NONE `BUS_LOAD or `BUS_STORE
        .proc2mem_size(proc2mem_size),
        .hit_data(hit_data), // data resulting from a load
        .hit(hit), // 1 if hit, 0 if miss
        .data_tag(data_tag),
        .data_response(data_response)
       
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
        $display("--------------------------------------------------------------------");
    endtask

    initial begin
        $monitor("Time:%4.0f\n clock:%b\n reset:%b\n proc2Dcache_addr:%h\n mem2proc_response:%b\n mem2proc_data:%h\n mem2proc_tag:%b\n proc2mem_addr:%h\n proc2mem_data:%h\n proc2mem_command:%b\n proc2mem_size:%b\n hit_data:%h\n hit:%b\n data_tag:%b\n data_response:%b",
            $time, clk, reset,
            proc2Dcache_addr,
            mem2proc_response, mem2proc_data, mem2proc_tag,
            proc2mem_addr, proc2mem_data, proc2mem_command, proc2mem_size,
            hit_data, hit, data_tag, data_response
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
        @(negedge clk);
        show_signals();

        // Wait a couple cycles to see the MISS state
        repeat (2) @(negedge clk);
        show_signals();

        // Now we emulate memory returning a valid response:
        // The dcache design expects mem2proc_response != 0 to accept the request
        // Then a cycle or two later, mem2proc_tag != 0 + mem2proc_data
        // will cause the cache to store it.
        $display("---- Memory responds with 'response=1' for the load request ----");
        mem2proc_response = 4'b0001;  // means memory has accepted and tagged the request
        @(negedge clk);
        show_signals();

        // Then we can set mem2proc_response back to 0, and later set mem2proc_tag
        // to the same value (1) with the data. This mimics the memory sending data back.
        mem2proc_response = 4'b0000;
        @(negedge clk);
        show_signals();


        // Now memory says: "Here is your data for tag=1"
        $display("---- Memory is returning data for tag=1 ----");
        mem2proc_tag  = 4'b0001;
        mem2proc_data = 64'hDEAD_BEEF_0000_0010; // sample data
        @(negedge clk);
        show_signals();


        // After 1 cycle, clear the mem2proc_tag to indicate no more data
        mem2proc_tag  = 4'b0000;
        mem2proc_data = 64'h0;
        @(negedge clk);
        show_signals();

        //----------------------------------------------------------------
        // Example 2: Read the same address => expect a HIT now
        //----------------------------------------------------------------
        $display("---- Reading the same address => should be a hit ----");
        proc2Dcache_addr = 32'h0000_0010;
        @(negedge clk);
        show_signals();

        // Wait a cycle or two
        repeat (2) @(negedge clk);
        show_signals();

        //----------------------------------------------------------------
        // Example 3: Different address => new MISS
        //----------------------------------------------------------------
        $display("---- Reading a different address => expect another MISS ----");
        proc2Dcache_addr = 32'h0000_0020;
        @(negedge clk);
        show_signals();

        repeat(2) @(negedge clk);
        show_signals();

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

        //------------------------------------
        // 2) Issue FIRST MISS (address A)
        //------------------------------------
        $display("---- Issue FIRST MISS (addr=0x1000) ----");
        proc2Dcache_addr = 32'h0000_1000; 
        @(negedge clk);
        show_signals();

        // Memory says: "I accept that request with tag=1"
        $display("---- Memory ACCEPTS first request: response=1 ----");
        mem2proc_response = 4'b0001; 
        @(negedge clk);
        show_signals();

        // Now we clear response back to 0 
        $display("---- Memory done accepting (response=0) ----");
        mem2proc_response = 4'b0000;
        @(negedge clk);
        show_signals();

        // *** Notice we do NOT provide data for tag=1 yet. ***
        // That means the first miss is still 'in flight.'
        // The cache is presumably waiting for mem2proc_tag=1 + data.

        //------------------------------------
        // 3) Issue SECOND MISS (address B)
        //    while the first is still waiting
        //------------------------------------
        $display("---- Issue SECOND MISS (addr=0x2000) ----");
        proc2Dcache_addr = 32'h0000_2000;
        @(negedge clk);
        show_signals();

        // Memory says: "I accept that request with tag=2"
        $display("---- Memory ACCEPTS second request: response=2 ----");
        mem2proc_response = 4'b0010;
        @(negedge clk);
        show_signals();

        // Clear the response again
        mem2proc_response = 4'b0000;
        @(negedge clk);
        show_signals();

        //------------------------------------
        // 4) Return DATA for FIRST MISS (tag=1)
        //------------------------------------
        $display("---- Returning data for FIRST MISS (tag=1) ----");
        mem2proc_tag  = 4'b0001;
        mem2proc_data = 64'hDEAD_BEEF_0000_1000;
        @(negedge clk);
        show_signals();

        // Clear mem2proc_tag
        mem2proc_tag  = 4'b0000;
        mem2proc_data = 64'h0;
        @(negedge clk);
        show_signals();

        //------------------------------------
        // 5) Return DATA for SECOND MISS (tag=2)
        //------------------------------------
        $display("---- Returning data for SECOND MISS (tag=2) ----");
        mem2proc_tag  = 4'b0010;
        mem2proc_data = 64'hFEED_FACE_0000_2000;
        @(negedge clk);
        show_signals();

        // Clear again
        mem2proc_tag  = 4'b0000;
        mem2proc_data = 64'h0;
        @(negedge clk);
        show_signals();

        //------------------------------------
        // 6) Check that both addresses now HIT
        //------------------------------------
        $display("---- Reading address 0x1000 => expect HIT ----");
        proc2Dcache_addr = 32'h0000_1000;
        @(negedge clk);
        show_signals();

        $display("---- Reading address 0x2000 => expect HIT ----");
        proc2Dcache_addr = 32'h0000_2000;
        @(negedge clk);
        show_signals();

        //------------------------------------
        // Done
        //------------------------------------
        $display("---- Test complete ----");
        $finish;

        $finish;

    end

endmodule
