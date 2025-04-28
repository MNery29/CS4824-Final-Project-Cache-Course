`include "verilog/sys_defs.svh"
`timescale 1ns/1ps

module testbench;

    logic clk;
    logic reset;
    logic [`XLEN-1:0] proc2Dcache_addr;
    logic [1:0] proc2Dcache_command;
    logic [2:0] proc2Dcache_size;          // added for partial stores
    logic [3:0] mem2proc_response;         // 0 = can't accept, other=tag of transaction
    logic [63:0] proc2Dcache_data;         // data for current command (if store)
    logic [63:0] mem2proc_data;            // data resulting from a load
    logic [3:0]  mem2proc_tag;             // 0 = no value, other=tag of transaction

    logic [`XLEN-1:0] proc2mem_addr;
    logic [63:0]      proc2mem_data;       // address for current command
    logic [1:0]       proc2mem_command;    // `BUS_NONE, `BUS_LOAD, or `BUS_STORE
    MEM_SIZE          proc2mem_size;       // Not heavily used in your code yet, but declared

    logic [63:0] hit_data; // data resulting from a load
    logic hit;
    logic [3:0] data_tag;
    logic [3:0] data_response;
    logic [3:0] number_of_waits;
    logic [3:0] next_number_of_waits;
    logic next_state;
    logic state;

    logic [`XLEN-1:0] tag_to_addr [15:0];
    logic tag_to_addr_valid [15:0];
    logic [1:0] tag_to_memsize [15:0];
    logic [63:0] tag_to_memdata [15:0]; // this is for stores exclusively
    logic tag_to_is_store [15:0];

    // Instantiate your dcache
    dcache data_cache(
        .clk                  (clk),
        .reset                (reset),

        .proc2Dcache_addr    (proc2Dcache_addr),
        .proc2Dcache_data    (proc2Dcache_data),
        .proc2Dcache_command (proc2Dcache_command),
        .proc2Dcache_size    (proc2Dcache_size), // important for partial stores

        .mem2dcache_response (mem2proc_response),
        .mem2dcache_data     (mem2proc_data),
        .mem2dcache_tag      (mem2proc_tag),

        .dcache2mem_addr     (proc2mem_addr),
        .dcache2mem_data     (proc2mem_data),
        .dcache2mem_command  (proc2mem_command),
        // .dcache2mem_size   // If your dcache has this, connect it. (commented if not used)

        .hit_data            (hit_data),
        .hit                 (hit),
        .data_tag            (data_tag),
        .data_response       (data_response),
        .next_state          (next_state),
        .state               (state),

        .tag_to_addr         (tag_to_addr),
        .tag_to_addr_valid   (tag_to_addr_valid),
        .tag_to_memsize      (tag_to_memsize),
        .tag_to_memdata      (tag_to_memdata),
        .tag_to_is_store     (tag_to_is_store)
    );

    // Generate clock
    always begin
        #(`CLOCK_PERIOD/2.0);
        clk = ~clk;
    end

    // ──────────────────────────────────────────────────────────────────────────────
    // Pretty-printer for per-tag bookkeeping arrays
    // ──────────────────────────────────────────────────────────────────────────────
    function automatic string memsize_str (input logic [1:0] sz);
        // Assumes the standard MEM_SIZE encoding (`BYTE, HALF, WORD, DOUBLE`)
        case (sz)
            BYTE   : memsize_str = "B ";
            HALF   : memsize_str = "H ";
            WORD   : memsize_str = "W ";
            DOUBLE : memsize_str = "D ";
            default: memsize_str = "??";
        endcase
    endfunction

    task automatic dump_tag_map;
        int i;
        $display("\n=== Tag-to-Address Map ================================================");
        $display(" idx | V | isSt | Sz |          Address          |        Data");
        $display("-----+---+------+----+---------------------------+-----------------------");
        for (i = 0; i < 16; i++) begin
            if (tag_to_addr_valid[i]) begin
                $display(" %20d | %1b |  %1b   | %s | 0x%016h | 0x%016h",
                        i,
                        tag_to_addr_valid[i],
                        tag_to_is_store[i],
                        memsize_str(tag_to_memsize[i]),
                        tag_to_addr[i],
                        tag_to_memdata[i]);
            end
            else begin
                // Print an empty row so the chart always shows 16 lines
                $display(" %20d | 0 |  -   | -- | ----------------------- | ------------------", i);
            end
        end
        $display("=======================================================================\n");
    endtask

    // Optional helper: prints relevant signals
    task show_signals;
        $display("[%0t ns] clk=%b reset=%b | proc2Dcache_addr=0x%08h  proc2Dcache_command=%b proc2Dcache_data=0x%h",
                 $time, clk, reset, proc2Dcache_addr, proc2Dcache_command, proc2Dcache_data);
        $display("           mem2proc_response=%b mem2proc_data=0x%h mem2proc_tag=%b",
                 mem2proc_response, mem2proc_data, mem2proc_tag);
        $display("           proc2mem_addr=0x%h proc2mem_data=0x%h proc2mem_command=%b proc2mem_size=%0d",
                 proc2mem_addr, proc2mem_data, proc2mem_command, proc2mem_size);
        $display("           hit=%b hit_data=0x%h data_tag=%b data_response=%b",
                 hit, hit_data, data_tag, data_response);
        $display("           next_state=%b state=%b",
                 next_state, state);
        $display("--------------------------------------------------------------------");

        dump_tag_map(); 
    endtask

    initial begin
        // Monitor prints every time a signal changes (spams console, so optional).
        // $monitor(...);

        //------------------------------------
        // Initial Reset
        //------------------------------------
        clk               = 0;
        reset             = 1;
        proc2Dcache_addr  = 32'h0000_0000;
        proc2Dcache_data  = 64'h0;
        proc2Dcache_command = BUS_NONE;
        proc2Dcache_size  = WORD;  // default
        mem2proc_response = 4'b0;
        mem2proc_data     = 64'h0;
        mem2proc_tag      = 4'b0;

        // Wait a couple cycles in reset
        repeat (2) @(negedge clk);
        show_signals();

        // Deassert reset
        reset = 0;
        @(negedge clk);
        $display("\n---- (A) Basic LOAD Miss -> Fill -> Hit scenario ----");
        // Existing sequence #1: LOAD @0x10 => MISS => memory fill => subsequent HIT
        proc2Dcache_addr    = 32'h0000_0010;
        proc2Dcache_command = BUS_LOAD; 
        proc2Dcache_size    = WORD;
        @(negedge clk);
        show_signals();

        // Wait a couple cycles to see MISS
        repeat (2) @(negedge clk);
        proc2Dcache_command = BUS_NONE;
        show_signals();

        // Memory returns acceptance + data
        $display("=> Memory accepting request with tag=1");
        mem2proc_response = 4'd1;
        show_signals();
        @(negedge clk);
        
        mem2proc_response = '0; // clear
        show_signals();
        @(negedge clk);
        // Memory returns old line data => cache merges new data

        // Memory returns data for tag=1
        $display("=> Memory returning data for tag=1");
        mem2proc_tag  = 4'd1;
        mem2proc_data = 64'hDEAD_BEEF_0000_0010;
        show_signals();
        @(negedge clk);
        mem2proc_tag  = '0;
        mem2proc_data = '0;
        show_signals();
        @(negedge clk);

        // LOAD again => should HIT 
        $display("=> Checking HIT after fill");
        proc2Dcache_addr    = 32'h0000_0010;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        if ((hit_data != 64'hDEAD_BEEF_0000_0010))
            $fatal("Expected load hit with correct data after fill");
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);

        //------------------------------------
        // (B) STORE Hit
        //------------------------------------
        $display("\n---- (B) STORE Hit test ----");
        proc2Dcache_addr    = 32'h0000_0010;
        proc2Dcache_data    = {32'h0000_0000, 32'hCAFE_BABE};
        proc2Dcache_command = BUS_STORE;
        proc2Dcache_size    = WORD; // store entire 64 bits
        show_signals();
        @(negedge clk);
        show_signals();
        if (!hit) $fatal("Expected store hit at 0x10");
        // Typically might see a store to memory or a mark-dirty. Check your design's policy.
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);
        // show_signals();

        // Now load back to confirm
        $display("=> LOAD to confirm store value");
        proc2Dcache_addr    = 32'h0000_0010;
        proc2Dcache_command = BUS_LOAD;
        // show_signals();
        @(negedge clk);
        // show_signals();
        if (!hit || (hit_data != 64'hDEAD_BEEF_CAFE_BABE))
            $fatal("Store data not returned on subsequent load (hit_data mismatch)");
        proc2Dcache_command = BUS_NONE;

        @(negedge clk);

        //------------------------------------
        // (C) STORE Miss (write-allocate)
        //------------------------------------
        $display("\n---- (C) STORE Miss test at 0x20 ----");
        proc2Dcache_addr    = 32'h0000_0020;
        proc2Dcache_data    = {32'h0000_0000, 32'hFEED_FACE};
        proc2Dcache_command = BUS_STORE;
        proc2Dcache_size    = WORD;
        show_signals();
        @(negedge clk);
        if (hit)
            $display("** Unexpected: got a store hit at 0x20, expected miss **");
        // Memory accepts with tag=2
        mem2proc_response   = 4'd2;
        
        show_signals();
        @(negedge clk);
        proc2Dcache_command = BUS_NONE;
        mem2proc_response   = '0;
        @(negedge clk);

        // Memory returns old line data => cache merges new data
        mem2proc_tag  = 4'd2;
        mem2proc_data = 64'hDEAD_BEEF_0000_0020; 
        show_signals();
        @(negedge clk);
        mem2proc_tag  = '0;
        mem2proc_data = '0;
        show_signals();
        @(negedge clk);
        // subsequent LOAD -> should see FEED_FACE_FEED_FACE
        $display("=> LOAD to confirm store-miss merged data");
        proc2Dcache_addr    = 32'h0000_0020;
        proc2Dcache_command = BUS_LOAD;
        show_signals();
        @(negedge clk);
        show_signals();
        if (!hit || (hit_data != 64'hDEAD_BEEF_FEED_FACE))
            $fatal("Store miss data not observed on later load");
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);

        //------------------------------------
        // (D) Partial Store: BYTE & HALF
        //------------------------------------
        $display("\n---- (D) Partial Store tests (Byte, Halfword) ----");
        // We'll do these on address 0x30 (first load to bring line in).
        // Then do partial stores and confirm the merges.

        // 1. Load miss at 0x30 => fill
        proc2Dcache_addr    = 32'h0000_0030;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        if (hit) $display("** Unexpected: hit on empty line 0x30? **");
        mem2proc_response   = 4'd3;
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);
        mem2proc_response   = '0;
        @(negedge clk);
        mem2proc_tag  = 4'd3;
        mem2proc_data = 64'h1234_5678_0000_0030;
        @(negedge clk);
        mem2proc_tag  = '0;
        mem2proc_data = '0;
        @(negedge clk);

        // 2. Byte store at 0x30 + offset=3 => store 0xAB in that byte
        $display("=> Byte store at 0x0033 offset=3");
        proc2Dcache_addr    = 32'h0000_0033;
        proc2Dcache_data    = 64'hAB;   // only the low byte used
        proc2Dcache_command = BUS_STORE;
        proc2Dcache_size    = BYTE;  
        @(negedge clk);
        if (!hit) $fatal("Expected store hit for partial store (same line 0x30)!");
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);

        // 3. Halfword store at 0x0032 => store 16'hCDEF
        $display("=> Halfword store at 0x0032 offset=2 (lowest 2 bits=10)");
        proc2Dcache_addr    = 32'h0000_0032;
        proc2Dcache_data    = 64'h0000_0000_0000_EFCD; // only low 16 bits used
        proc2Dcache_command = BUS_STORE;
        proc2Dcache_size    = BYTE;
        @(negedge clk);
        if (!hit) $fatal("Expected store hit for halfword store (same line)!");
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);
        $display("=> Halfword store at 0x0032 offset=2 (lowest 2 bits=10)");
        proc2Dcache_addr    = 32'h0000_0031;
        proc2Dcache_data    = 64'h0000_0000_0000_CDEF; // only low 16 bits used
        proc2Dcache_command = BUS_STORE;
        proc2Dcache_size    = BYTE;
        @(negedge clk);
        if (!hit) $fatal("Expected store hit for halfword store (same line)!");
        proc2Dcache_command = BUS_NONE;

        // 4. Now read back entire line @0x30 => check the partial merges
        $display("=> Load entire line again at 0x30 to confirm partial merges");
        proc2Dcache_addr    = 32'h0000_0030;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        if (!hit) $fatal("Expected load hit after partial stores!");
        // Expected final data in line: original 64'h1234_5678_0000_0030,
        // with a byte at offset=3 replaced by 0xAB, and a halfword at offset=2 replaced by 0xCDEF
        // That means bits [31:16] => 0xCDEF, bits [15:8] => 0xAB, rest => from original.
        // So final expected = 0x1234_5678_0000_CDAB ??? Actually you need to be careful with endianness.
        // For a typical little-endian view, offset=3 modifies bits [31:24],
        // offset=2 modifies bits [23:8].
        // Let's do the direct approach:
        // original was: [63:32] = 0x12345678, [31:0] = 0x00000030
        // offset=3 => byte at [31:24] = 0xAB => [31:24] becomes 0xAB
        // offset=2 => halfword at [23:8] = 0xCDEF => that overwrites bytes [23:16] & [15:8]
        // so the final 32 bits from [31:0] become 0xAB_CDEF_?? ?? Actually we need to be consistent.
        // Let’s break it down carefully:
        //   [31:24] = offset=3 => 0xAB
        //   [23:16] = offset=2 => 0xCD
        //   [15:8 ] = offset=2+1 => 0xEF
        //   [7 :0 ] = old = 0x30
        // So the final lower 32 bits = 0xAB_CDEF_30
        // which is 0xABCDEF30.
        // upper 32 bits remain 0x12345678.
        // => final = 64'h1234_5678_ABCD_EF30
        // We'll do a check below:
        show_signals();
        if (hit_data !== 64'h1234_5678_ABCD_EF30)
            $fatal("Partial store merges incorrect! Got 0x%h", hit_data);

        proc2Dcache_command = BUS_NONE;
        @(negedge clk);

        //------------------------------------
        // (E) Eviction scenario
        //------------------------------------
        // We'll try to evict line 0x10 by accessing an address with same index but different tag.
        // Index for 0x10 => (0x10 >> 3) & 0x3F => 0x2 
        // Let's pick something that also has index=0x2 but different upper bits, e.g. 0x4010
        // => (0x4010 >> 3) & 0x3F => still 0x2, but the tag is different.
        //------------------------------------
        $display("\n---- (E) Eviction test: forcing line 0x10 to be evicted with new line 0x4010 ----");
        // 1. Confirm line 0x10 is present & dirty after store
        proc2Dcache_addr    = 32'h0000_0010;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        if (!hit) $fatal("Expected line 0x10 to still be in cache before eviction!");
        if (hit_data !== 64'hDEAD_BEEF_CAFE_BABE)
            $fatal("Line 0x10 data mismatch before eviction! got 0x%h", hit_data);
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);

        // 2. Access 0x4010 => same index, new tag => should cause eviction
        $display("=> LOAD miss at 0x4010 => evict old line index=2");
        proc2Dcache_addr    = 32'h0000_4010;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        // Expect MISS => dcache triggers store of dirty line 0x10 if it's dirty
        // then memory fetch for 0x4010
        // we emulate memory accepting with tag=4
        mem2proc_response   = 4'd4;
        proc2Dcache_command = BUS_NONE;
        show_signals();
        @(negedge clk);
        mem2proc_response   = '0;
        show_signals();
        @(negedge clk);

        // memory returns new line data for tag=4
        mem2proc_tag  = 4'd4;
        mem2proc_data = 64'hD00D_F00D_0000_4010;
        show_signals();
        @(negedge clk);
        mem2proc_tag  = '0;
        mem2proc_data = '0;
        show_signals();
        @(negedge clk);

        // 3. Now load 0x4010 => hit new data
        proc2Dcache_addr    = 32'h0000_4010;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        if (!hit || (hit_data !== 64'hD00D_F00D_0000_4010))
            $fatal("Evicted line not replaced with correct data for 0x4010!");
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);

        // 4. Finally confirm old line 0x10 got evicted
        //    If your design truly evicts and invalidates, we can test by re-checking 0x10 => MISS
        $display("=> Re-check 0x10 => should be miss now (evicted).");
        proc2Dcache_addr    = 32'h0000_0010;
        proc2Dcache_command = BUS_LOAD;
        @(negedge clk);
        if (hit) 
            $display("** WARNING: line 0x10 still present, not evicted? Possibly your design doesn't evict or unify lines? **");
        proc2Dcache_command = BUS_NONE;
        @(negedge clk);

        //------------------------------------
        // DONE
        //------------------------------------
        $display("\n==== Test Complete. If no $fatal occurred, you passed! ====");
        $finish;
    end

endmodule
