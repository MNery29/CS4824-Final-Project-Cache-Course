`include "verilog/sys_defs.svh"
`timescale 1ns/1ps

module testbench_lsq;

    //-------------------------------------------------------------------------
    // Testbench signals
    //-------------------------------------------------------------------------

    logic                       clk;
    logic                       reset;

    // LSQ -> dcache signals
    logic [1:0]                 dcache_command;
    logic [`XLEN-1:0]           dcache_addr;
    logic [63:0]                dcache_data;
    logic                       dcache_hit;

    // dcache -> LSQ signals
    logic [63:0]                dcache_data_out;  
    logic [3:0]                 dcache_tag;       
    logic [3:0]                 dcache_response;  

    // LSQ inputs for retirement & packet
    logic [4:0]                 mem_tag;
    logic                       mem_valid;
    LSQ_PACKET                  lsq_packet_in;

    // CDB and private address inputs
    CDB_PACKET                  cdb_in;
    priv_addr_packet            priv_addr_in;
    logic                       cdb_busy;  // For simplicity, keep it 0 in this test

    // LSQ outputs
    CDB_PACKET                  cdb_out;
    logic                       store_ready;
    logic [4:0]                 store_ready_tag;
    logic                       lsq_free;  // Whether LSQ has space

    logic                       cache_in_flight;
    logic                       head_ready_for_mem;

    // Debug pointers
    logic [2:0]                 head_ptr;
    logic [2:0]                 tail_ptr;

    //-------------------------------------------------------------------------
    // Instantiate LSQ
    //-------------------------------------------------------------------------
    lsq #(
        .LSQ_SIZE      (8),
        .LSQ_SIZE_W    (2),
        .NONBLOCKING   (0)
    ) uut_lsq (
        // Global
        .clk                 (clk),
        .reset               (reset),

        // D-cache interface
        .dcache_data_out     (dcache_data_out),
        .dcache_tag          (dcache_tag),
        .dcache_response     (dcache_response),
        .dcache_hit          (dcache_hit),

        // Retirement signals
        .mem_tag             (mem_tag),
        .mem_valid           (mem_valid),

        // LSQ input packet from pipeline/issue
        .lsq_packet       (lsq_packet_in),

        // CDB input
        .cdb_in              (cdb_in),

        // Private addr channel
        .priv_addr_in        (priv_addr_in),

        // If CDB is busy, we have to re-try
        .cdb_busy            (cdb_busy),

        // Outputs
        .cdb_out             (cdb_out),
        .dcache_command      (dcache_command),
        .dcache_addr         (dcache_addr),
        .dcache_data         (dcache_data),

        .store_ready         (store_ready),
        .store_ready_tag     (store_ready_tag),
        .lsq_free            (lsq_free),

        .cache_in_flight     (cache_in_flight),
        .head_ready_for_mem  (head_ready_for_mem),

        .head_ptr            (head_ptr),
        .tail_ptr            (tail_ptr)
    );

    //-------------------------------------------------------------------------
    // Clock Generation
    //-------------------------------------------------------------------------
    always begin
        #(`CLOCK_PERIOD/2.0) clk = ~clk;
    end

    //-------------------------------------------------------------------------
    // A small task to display signals
    //-------------------------------------------------------------------------
    task show_signals;
        $display($time, "ns :: clk=%b reset=%b", clk, reset);
        $display("  DCACHE: cmd=%b addr=0x%08h data=0x%h | tag_in=%b resp_in=%b hit=%b data_out=0x%h",
                 dcache_command, dcache_addr, dcache_data, dcache_tag, dcache_response, dcache_hit, dcache_data_out);

        $display("  LSQ Packet In: valid=%b rd=%b wr=%b rob_tag=%0d store_data_valid=%b store_data=0x%h store_data_tag=%0d",
                 lsq_packet_in.valid,
                 lsq_packet_in.rd_mem,
                 lsq_packet_in.wr_mem,
                 lsq_packet_in.rob_tag,
                 lsq_packet_in.store_data_valid,
                 lsq_packet_in.store_data,
                 lsq_packet_in.store_data_tag
                );

        $display("  priv_addr_in: valid=%b tag=%0d addr=0x%08h",
                 priv_addr_in.valid, priv_addr_in.tag, priv_addr_in.addr);

        $display("  cdb_in: valid=%b tag=%0d value=0x%08h",
                 cdb_in.valid, cdb_in.tag, cdb_in.value);

        $display("  mem_valid=%b mem_tag=%0d | cdb_busy=%b",
                 mem_valid, mem_tag, cdb_busy);

        $display("  cdb_out: valid=%b tag=%0d value=0x%08h",
                 cdb_out.valid, cdb_out.tag, cdb_out.value);

        $display("  store_ready=%b (tag=%0d), lsq_free=%b",
                 store_ready, store_ready_tag, lsq_free);

        $display("  cache_in_flight=%b head_ready_for_mem=%b head_ptr=%0d tail_ptr=%0d",
                 cache_in_flight, head_ready_for_mem, head_ptr, tail_ptr);
        $display("-----------------------------------------------------------------------------");
    endtask

    //-------------------------------------------------------------------------
    // Test stimulus
    //-------------------------------------------------------------------------
    initial begin
        // Monitor key signals
        $monitor($time,
            " :: clk=%b reset=%b | DCACHE cmd=%b addr=0x%h data=0x%h | cdb_out.valid=%b tag=%0d val=0x%h",
            clk, reset, dcache_command, dcache_addr, dcache_data,
            cdb_out.valid, cdb_out.tag, cdb_out.value);

        // Initialize
        clk            = 0;
        reset          = 1;
        dcache_tag     = 4'b0;
        dcache_response= 4'b0;
        dcache_data_out= 64'h0;
        dcache_hit     = 0;

        cdb_busy       = 0;   // Not modeling CDB backpressure in this test

        // Clear LSQ inputs
        mem_tag        = '0;
        mem_valid      = 0;

        lsq_packet_in  = '{valid:0, wr_mem:0, rd_mem:0, rob_tag:'0,
                           store_data:'0, store_data_tag:'0, store_data_valid:0};

        cdb_in         = '{valid:0, tag:'0, value:'0};
        priv_addr_in   = '{valid:0, tag:'0, addr:'0};

        // Wait a couple of cycles in reset
        repeat (2) @(posedge clk);
        show_signals();

        // Deassert reset
        reset = 0;
        @(posedge clk);
        show_signals();

        //----------------------------------------------------------------------
        // 1) Enqueue a LOAD (rob_tag=5)
        //----------------------------------------------------------------------
        $display("\n--- Test 1: Enqueue a LOAD with rob_tag=5 ---");
        lsq_packet_in.valid          = 1;
        lsq_packet_in.rd_mem         = 1;
        lsq_packet_in.wr_mem         = 0;
        lsq_packet_in.rob_tag        = 5;
        lsq_packet_in.store_data     = 64'h0;
        lsq_packet_in.store_data_tag = 5'd0;
        lsq_packet_in.store_data_valid = 0;
        @(posedge clk);

        // Turn off the packet
        lsq_packet_in.valid  = 0;
        lsq_packet_in.rd_mem = 0;
        @(posedge clk);
        show_signals();

        // Provide the address via priv_addr_in (tag=5 => matches LSQ entry)
        $display("--- Provide address (0x1000) via priv_addr_in.tag=5 ---");
        priv_addr_in.valid = 1;
        priv_addr_in.tag   = 4'd5;
        priv_addr_in.addr  = 32'h0000_1000;
        @(posedge clk);
        priv_addr_in.valid = 0;
        @(posedge clk);
        show_signals();

        // LSQ should now send a LOAD to the dcache
        // We'll emulate the cache accepting it by setting dcache_tag=1 if command != BUS_NONE
        if (dcache_command != BUS_NONE) begin
            $display("DCACHE sees LOAD => setting dcache_tag=1 (ACCEPT)...");
            dcache_tag <= 4'd1;
        end
        @(posedge clk);

        // Turn off acceptance
        dcache_tag <= 4'd0;
        show_signals();

        // Now LSQ is waiting for data => Provide a hit & data
        $display("--- Emulate a dcache hit => data=0xDEAD_BEEF_1234_0000 ---");
        dcache_hit       <= 1;
        dcache_data_out  <= 64'hDEAD_BEEF_1234_0000;
        @(posedge clk);

        // Turn off response
        dcache_hit       <= 0;
        dcache_data_out  <= 64'h0;
        @(posedge clk);
        show_signals();

        // Expect cdb_out.valid=1 for one cycle with tag=5 and value=0x1234_0000 (lower 32 bits)
        if (cdb_out.valid && (cdb_out.tag == 5) && (cdb_out.value == 32'h1234_0000))
            $display("LOAD completion broadcast: OK");
        else
            $display("WARNING: Did not see expected LOAD completion on cdb_out");

        //----------------------------------------------------------------------
        // 2) Enqueue a STORE (rob_tag=6)
        //    We'll supply the store data from cdb_in.tag=6, then retire it
        //----------------------------------------------------------------------
        $display("\n--- Test 2: Enqueue a STORE, then retire it and supply data ---");
        lsq_packet_in.valid          = 1;
        lsq_packet_in.rd_mem         = 0;
        lsq_packet_in.wr_mem         = 1;
        lsq_packet_in.rob_tag        = 4;
        lsq_packet_in.store_data     = 32'h0;
        lsq_packet_in.store_data_tag = 6;
        lsq_packet_in.store_data_valid = 0;
        @(posedge clk);

        lsq_packet_in.valid = 0;
        lsq_packet_in.wr_mem = 0;
        @(posedge clk);

        // Provide address via priv_addr_in.tag=6 => 0x2000
        priv_addr_in.valid = 1;
        priv_addr_in.tag   = 4;
        priv_addr_in.addr  = 32'h0000_2000;
        @(posedge clk);
        priv_addr_in.valid = 0;
        @(posedge clk);
        show_signals();

        // Provide store data from cdb_in (tag=6), e.g. 32'hABCD_1234
        $display("--- Provide store data cdb_in.tag=6 => 0xABCD_1234 in lower half ---");
        cdb_in.valid = 1;
        cdb_in.tag   = 6;
        cdb_in.value = 32'hABCD_1234;
        @(posedge clk);

        cdb_in.valid = 0;
        @(posedge clk);
        show_signals();

        // Mark this store as retired => LSQ can now push it to the cache
        $display("--- Mark store as retired => mem_valid=1, mem_tag=6 ---");
        mem_valid = 1;
        mem_tag   = 4;
        @(posedge clk);

        mem_valid = 0;
        @(posedge clk);
        show_signals();

        // LSQ should now attempt a STORE => emulate acceptance with dcache_tag=2
        if (dcache_command == BUS_STORE) begin
            $display("DCACHE store command => set dcache_tag=2 (accepted)");
            dcache_response <= 4'd2;
        end
        @(posedge clk);

        dcache_tag <= 4'd0;
        @(posedge clk);
        show_signals();

        // Complete the store => dcache_response=2
        dcache_response <= 4'd2;
        @(posedge clk);

        dcache_response <= 4'd0;
        @(posedge clk);
        show_signals();

        // No cdb_out expected for a store
        if (cdb_out.valid)
            $display("WARNING: Store incorrectly broadcast a result?");

        //----------------------------------------------------------------------
        // Done
        //----------------------------------------------------------------------
        $display("\nAll tests complete. Ending simulation.\n");
        $finish;
    end

endmodule
