`include "verilog/sys_defs.svh"
`timescale 1ns/1ps

module testbench_lsq;

    logic                       clk;
    logic                       reset;

    // LSQ -> dcache signals
    logic [1:0]                 dcache_command;
    logic [`XLEN-1:0]           dcache_addr;
    logic [63:0]                dcache_data;
    logic dcache_hit;

    // dcache -> LSQ signals
    logic [63:0]                dcache_data_out;  
    logic [3:0]                 dcache_tag;       
    logic [3:0]                 dcache_response;  

    // Pipeline/ROB -> LSQ
    IS_EX_PACKET                is_ex_in;
    ROB_RETIRE_PACKET           rob_retire_in;
    ROB_DISPATCH_PACKET         rob_dispatch_in;
    CDB_PACKET                  cdb_in;
    priv_addr_packet            priv_addr_in;

    // LSQ -> pipeline
    CDB_PACKET                  cdb_out;
    logic                       store_ready;
    logic [4:0]                 store_ready_tag;
    logic                       stall_dispatch;

    logic cache_in_flight;
    logic head_ready_for_mem;

    logic [2:0] head_ptr;
    logic [2:0] tail_ptr;

    // -----------------------------
    // Instantiate LSQ
    // -----------------------------
    lsq uut_lsq (
        .clk               (clk),
        .reset             (reset),

        .dcache_data_out   (dcache_data_out),
        .dcache_tag        (dcache_tag),
        .dcache_response   (dcache_response),
        .dcache_hit        (dcache_hit),

        .rob_retire_in     (rob_retire_in),
        .rob_dispatch_in   (rob_dispatch_in),
        .is_ex_in          (is_ex_in),
        .cdb_in            (cdb_in),
        .priv_addr_in      (priv_addr_in),

        .cdb_out           (cdb_out),
        .dcache_command    (dcache_command),
        .dcache_addr       (dcache_addr),
        .dcache_data       (dcache_data),

        .store_ready       (store_ready),
        .store_ready_tag   (store_ready_tag),
        .stall_dispatch    (stall_dispatch),

        .cache_in_flight   (cache_in_flight),
        .head_ready_for_mem(head_ready_for_mem),

        .head_ptr(head_ptr),
        .tail_ptr(tail_ptr)
    );


    // clk_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clk = ~clk;
    end

    // -----------------------------
    // Display/Monitor
    // -----------------------------
    task show_signals;
        $display($time, "ns :: clk=%b reset=%b | dcache_cmd=%b dcache_addr=0x%h dcache_data=0x%h",
                 clk, reset, dcache_command, dcache_addr, dcache_data);
        $display("    >> dcache_tag_in=%b dcache_resp_in=%b dcache_data_out=0x%h",
                 dcache_tag, dcache_response, dcache_data_out);
        $display("    >> is_ex_in(issue=%b rd=%b wr=%b rob_tag=%0d), stall_dispatch=%b",
                 is_ex_in.issue_valid, is_ex_in.rd_mem, is_ex_in.wr_mem, is_ex_in.rob_tag, stall_dispatch);
        $display("    >> priv_addr_in(valid=%b, tag=%0d, addr=0x%08h)",
                 priv_addr_in.valid, priv_addr_in.tag, priv_addr_in.addr);
        $display("    >> cdb_in(valid=%b, tag=%0d, value=0x%08h)",
                 cdb_in.valid, cdb_in.tag, cdb_in.value);
        $display("    >> rob_retire_in(mem_valid=%b, tag=%0d)", 
                 rob_retire_in.mem_valid, rob_retire_in.tag);
        $display("    -- cdb_out(valid=%b, tag=%0d, value=0x%08h)",
                 cdb_out.valid, cdb_out.tag, cdb_out.value);
        $display("    -- store_ready=%b (tag=%0d)", store_ready, store_ready_tag);
        $display("    -- cache_in_flight=%b head_ready_for_mem=%b head_ptr=%h tail_ptr=%h", 
                 cache_in_flight, head_ready_for_mem, head_ptr, tail_ptr);
        $display("-------------------------------------------------------------------------------");
    endtask

    // -----------------------------
    // Initial Test Sequence
    // -----------------------------
    initial begin
        // Monitor these signals continuously
        $monitor($time, 
            " :: clk=%b reset=%b | dcache_cmd=%b dcache_addr=0x%h dcache_data=0x%h \n -- cache_in_flight=%b head_ready_for_mem=%b stall_dispatch=%b head_ptr=%h tail_ptr=%h dcache_hit=%b dcache_data_out=0x%h \n -- cdb_out(valid=%b, tag=%0d, value=0x%08h)",
            clk, reset, dcache_command, dcache_addr, dcache_data, cache_in_flight, head_ready_for_mem, stall_dispatch, head_ptr, tail_ptr, dcache_hit, dcache_data_out, cdb_out.valid, cdb_out.tag, cdb_out.value);

        // Initialize
        clk               = 0;
        reset           = 1;
        dcache_tag      = 4'b0;
        dcache_response = 4'b0;
        dcache_data_out = 64'h0;

        // Clear inputs to LSQ
        rob_retire_in    = '{tag:'0, dest_reg:'0, value:'0, reg_valid:0, mem_valid:0, mem_addr:'0};
        rob_dispatch_in  = '{tag:'0, valid:0};
        is_ex_in         = '{OPA:'0, OPB:'0, rob_tag:'0, RS_tag:'0, alu_func:'0, NPC:'0, inst:'0, 
                            issue_valid:0, rd_mem:0, wr_mem:0};
        cdb_in           = '{tag:'0, value:'0, valid:0};
        priv_addr_in     = '{addr:'0, tag:'0, valid:0};

        // Wait a couple of cycles in reset
        repeat (2) @(posedge clk);
        show_signals();

        // Deassert reset
        reset = 0;
        @(posedge clk);
        show_signals();

        // --------------------------------------------------
        // 1) Enqueue a LOAD (issue_valid=1, rd_mem=1)
        //    rob_tag=5 => (lowest nibble= 5 => address_tag=5)
        // --------------------------------------------------
        $display("\n--- Test 1: Enqueue a LOAD with rob_tag=5 ---");
        is_ex_in.issue_valid = 1;
        is_ex_in.rd_mem      = 1;
        is_ex_in.wr_mem      = 0;
        is_ex_in.rob_tag     = 5;   // LSQ will store address_tag=5
        @(posedge clk);

        // Turn off issue_valid
        is_ex_in.issue_valid = 0;
        is_ex_in.rd_mem      = 0;
        @(posedge clk);
        show_signals();

        // Provide the address via priv_addr_in (tag=5 => matches LSQ entry)
        $display("--- Provide address (0x1000) via priv_addr_in.tag=5 ---");
        priv_addr_in.valid = 1;
        priv_addr_in.tag   = 4'd5;  
        priv_addr_in.addr  = 32'h0000_1000;
        @(posedge clk);
        // Turn off
        priv_addr_in.valid = 0;
        @(posedge clk);
        show_signals();

        // LSQ should now attempt to send a LOAD to the dcache if no store_data is needed
        // We'll emulate the cache accepting it with dcache_tag=1
        if (dcache_command != BUS_NONE) begin
            $display("DCACHE command = %b, sending dcache_tag=1 (ACCEPT)", dcache_command);
            dcache_tag <= 4'd1;
        end
        @(posedge clk);

        // Turn off acceptance
        dcache_tag <= 4'b0;
        show_signals();

        // Now the LSQ is waiting for completion => we eventually provide dcache_response=1
        $display("--- Provide dcache_response=1, with data=0xDEAD_BEEF_1234_0000 ---");
        dcache_hit <= 1; // Emulate a hit
        dcache_data_out <= 64'hDEAD_BEEF_1234_0000;
        @(posedge clk);

        // Turn off response
        dcache_hit <= 0;
        dcache_data_out <= 64'h0;
        @(posedge clk);
        show_signals();

        // We expect cdb_out.valid=1 for one cycle with tag=5 and value=0x1234_0000
        // (the lower 32 bits). 
        if (cdb_out.valid && (cdb_out.tag == 5) && (cdb_out.value == 32'h1234_0000))
            $display("LOAD completion broadcast: OK");
        else
            $display("WARNING: Did not see expected LOAD completion on cdb_out");

        // --------------------------------------------------
        // 2) Enqueue a STORE
        //    We'll do: rob_tag=6 => address_tag=6
        //    We'll supply store data from cdb_in.tag=6
        //    Then retire it => LSQ issues to the cache
        // --------------------------------------------------
        $display("\n--- Test 2: Enqueue a STORE, then retire it and supply store data ---");
        is_ex_in.issue_valid = 1;
        is_ex_in.rd_mem      = 0;
        is_ex_in.wr_mem      = 1;
        is_ex_in.rob_tag     = 6;
        @(posedge clk);
        // Turn off
        is_ex_in.issue_valid = 0;
        is_ex_in.wr_mem      = 0;
        @(posedge clk);

        // Provide address via priv_addr_in.tag=6 => 0x2000
        priv_addr_in.valid = 1;
        priv_addr_in.tag   = 4'd6;
        priv_addr_in.addr  = 32'h0000_2000;
        @(posedge clk);
        priv_addr_in.valid = 0;
        @(posedge clk);
        show_signals();

        // Provide store data from cdb_in (tag=6), e.g. 32'hABCD_1234
        $display("--- Provide store data cdb_in.tag=6 => 0xABCD_1234 in lower half ---");
        cdb_in.valid = 1;
        cdb_in.tag   = 5'd6;
        cdb_in.value = 32'hABCD_1234;
        @(posedge clk);
        cdb_in.valid = 0;
        @(posedge clk);
        show_signals();

        // Mark store as retired => rob_retire_in(mem_valid=1, tag=6)
        $display("--- Mark store as retired => LSQ can now send store to dcache ---");
        rob_retire_in.mem_valid = 1;
        rob_retire_in.tag       = 6;
        @(posedge clk);
        rob_retire_in.mem_valid = 0;
        @(posedge clk);
        show_signals();

        // LSQ should now attempt a STORE => we emulate acceptance with dcache_tag=2
        if (dcache_command == BUS_STORE) begin
            $display("DCACHE store command seen => set dcache_tag=2 (accepted)");
            dcache_tag <= 4'd2;
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

        // No cdb_out expected for store completion
        if (cdb_out.valid)
            $display("WARNING: Store incorrectly broadcast a result?");

        // --------------------------------------------------
        // Done
        // --------------------------------------------------
        $display("\nAll tests complete. Ending simulation.\n");
        $finish;
    end

endmodule
