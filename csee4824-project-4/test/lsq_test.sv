//WILLS LSQ TESTBENCh
`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

module testbench;

   localparam int CLOCK_PERIOD = 10;   // 100 MHz
   logic clk = 0;
   always #(CLOCK_PERIOD/2) clk = ~clk;

   logic reset;

   // 2. DUT I/O wires
   logic [63:0] dcache_data_out;
   logic [3:0]  dcache_tag;
   logic [3:0]  dcache_response;
   logic        dcache_hit;
   logic [31:0] dcache_cur_addr;
   logic [1:0]  dcache_cur_command;
   logic [63:0] dcache_cur_data;

   //issue/RT/CDB side 
   LSQ_PACKET        lsq_packet;
   priv_addr_packet  priv_addr_in;
   CDB_PACKET        cdb_in;

   logic       mem_valid;
   logic [4:0] mem_tag;

   //  LSQ 
   EX_CP_PACKET cdb_out;

   logic [1:0]  dcache_command;
   logic [31:0] dcache_addr;
   logic [63:0] dcache_data;
   logic [1:0]  dcache_size;

   logic        store_ready;
   logic [4:0]  store_ready_tag;
   logic        lsq_free;

   logic        lsq_op_in_progress;

   logic        cache_in_flight;
   logic        head_ready_for_mem;

   logic [2:0]  head_ptr;
   logic [2:0]  tail_ptr;

   lsq_entry_t  lsq_out [7:0];

   logic [4:0]  cache_tag_in_flight       [15:0];
   logic        cache_in_flight_valid     [15:0];
   logic        cache_offset_in_flight    [15:0];
   logic        cache_in_flight_rd_unsigned[15:0];
   MEM_SIZE     cache_in_flight_mem_size  [15:0];

   lsq #(
        .LSQ_SIZE    (8),
        .LSQ_SIZE_W  (3),   // log2(8)
        .NONBLOCKING (0)
   ) dut (
        .clk                    (clk),
        .reset                  (reset),
        // D-cache interface 
        .dcache_data_out        (dcache_data_out),
        .dcache_tag             (dcache_tag),
        .dcache_response        (dcache_response),
        .dcache_hit             (dcache_hit),
        .dcache_cur_addr        (dcache_cur_addr),
        .dcache_cur_command     (dcache_cur_command),
        .dcache_cur_data        (dcache_cur_data),

        // pipeline side 
        .mem_tag                (mem_tag),
        .mem_valid              (mem_valid),
        .lsq_packet             (lsq_packet),
        .cdb_in                 (cdb_in),
        .priv_addr_in           (priv_addr_in),

        //  outputs
        .cdb_out                (cdb_out),
        .dcache_command         (dcache_command),
        .dcache_addr            (dcache_addr),
        .dcache_data            (dcache_data),
        .dcache_size            (dcache_size),
        .store_ready            (store_ready),
        .store_ready_tag        (store_ready_tag),
        .lsq_free               (lsq_free),
        .lsq_op_in_progress     (lsq_op_in_progress),
        .cache_in_flight        (cache_in_flight),
        .head_ready_for_mem     (head_ready_for_mem),
        .head_ptr               (head_ptr),
        .tail_ptr               (tail_ptr),
        .lsq_out                (lsq_out),
        .cache_tag_in_flight    (cache_tag_in_flight),
        .cache_in_flight_valid  (cache_in_flight_valid),
        .cache_offset_in_flight (cache_offset_in_flight),
        .cache_in_flight_rd_unsigned(cache_in_flight_rd_unsigned),
        .cache_in_flight_mem_size(cache_in_flight_mem_size)
   );

   localparam logic [1:0] BUS_NONE  = 2'b00,
                          BUS_LOAD  = 2'b01,
                          BUS_STORE = 2'b10;

   localparam MEM_SIZE WORD = MEM_SIZE'(2'b10);

   task automatic dump_state();
      $display("\n[%0t] ------------------------------------------------", $time);
      $display(" head=%0d  tail=%0d  lsq_free=%b  op_in_prog=%b", 
               head_ptr, tail_ptr, lsq_free, lsq_op_in_progress);
      for (int i = 0; i < 8; i++) begin
         $display("  LSQ[%0d]: v=%b  store=%b  addr_v=%b  rob=%0d  addr=0x%08h",
                  i,
                  lsq_out[i].valid,
                  lsq_out[i].is_store,
                  lsq_out[i].address_valid,
                  lsq_out[i].rob_tag,
                  lsq_out[i].address);
      end
      $display(" dcache_cmd=%b  dcache_addr=0x%08h  hit=%b", 
               dcache_command, dcache_addr, dcache_hit);
      $display(" cdb_out.valid=%b  rob_tag=%0d  value=0x%08h",
               cdb_out.valid, cdb_out.rob_tag, cdb_out.value);
   endtask

   // Dump every posedge so we can watch the pointers advance.
   always_ff @(posedge clk) dump_state();
   // Stimulus helpers
   // “enqueue” a LOAD into the LSQ for ROB tag = <rob>, unsigned = 0.
   task automatic enqueue_load(input logic [4:0] rob);
      lsq_packet.valid            = 1;
      lsq_packet.rd_mem           = 1;
      lsq_packet.wr_mem           = 0;
      lsq_packet.rob_tag          = rob;
      lsq_packet.rd_unsigned      = 1'b0;
      lsq_packet.store_data_tag   = 5'h0;
      lsq_packet.store_data_valid = 1'b0;
      lsq_packet.store_data       = 64'h0;
      lsq_packet.mem_size         = WORD;
      @(negedge clk);                       // keep for one cycle
      lsq_packet.valid = 0;                 // drop .valid afterwards
   endtask

   // Send the address that EX computed 
   task automatic send_priv_addr(input logic [4:0] tag,
                                 input logic [31:0] addr);
      priv_addr_in.valid = 1;
      priv_addr_in.tag   = tag;
      priv_addr_in.addr  = addr;
      @(negedge clk);
      priv_addr_in.valid = 0;
   endtask
   // Wait until the LSQ issues a LOAD, then provide two-cycle D-cache hit
   // with the chosen data word.
   task automatic simulate_two_cycle_hit(input logic [31:0] addr,
                                         input logic [63:0] data);
      // Wait for LSQ to assert BUS_LOAD with the correct address
      @(posedge clk);
      wait(dcache_command == BUS_LOAD && dcache_addr == addr);

      // Two consecutive cycles of “hit”
      repeat (2) @(negedge clk) begin
         dcache_hit         = 1;
         dcache_cur_addr    = addr;
         dcache_cur_command = BUS_LOAD;
         dcache_data_out    = data;
         dcache_response    = 0;
         dcache_tag         = 0;
      end

      // De-assert afterwards
      @(negedge clk);
      dcache_hit         = 0;
      dcache_cur_command = BUS_NONE;
      dcache_data_out    = 64'h0;
   endtask

   initial begin
      reset               = 1;
      lsq_packet          = '0;
      priv_addr_in        = '0;
      cdb_in              = '0;
      mem_valid           = 0;
      dcache_data_out     = 64'h0;
      dcache_tag          = 4'h0;
      dcache_response     = 4'h0;
      dcache_hit          = 0;
      dcache_cur_addr     = 32'h0;
      dcache_cur_command  = BUS_NONE;
      dcache_cur_data     = 64'h0;

      // Hold reset for two cycles
      repeat (2) @(negedge clk);
      reset = 0;
      enqueue_load(5'd1);
      send_priv_addr(5'd1, 32'h0000_0010);
      enqueue_load(5'd2);
      send_priv_addr(5'd2, 32'h0000_0020);
      simulate_two_cycle_hit(32'h0000_0010, 64'h0123_4567_89AB_CDEF);
      simulate_two_cycle_hit(32'h0000_0020, 64'hFACE_B00C_DEAD_BEEF);
      // allow a few cycles for CDB broadcast, then quit
      repeat (4) @(posedge clk);
      $display("\n*** TEST COMPLETED – watch waveform or console output ***\n");
      $finish;
   end

endmodule
