`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

module zcache_test;

  logic clk = 0;
  logic reset;
  logic [`XLEN-1:0] proc2Dcache_addr;
  logic [3:0] mem2proc_response, mem2proc_tag;
  logic [63:0] mem2proc_data;
  logic [`XLEN-1:0] proc2mem_addr;
  logic [63:0] proc2mem_data;
  logic [1:0] proc2mem_command;
  MEM_SIZE proc2mem_size;

  // Clock generation
  always #5 clk = ~clk;

  zcache dut (
    .clk(clk),
    .reset(reset),
    .proc2Dcache_addr(proc2Dcache_addr),
    .mem2proc_response(mem2proc_response),
    .mem2proc_data(mem2proc_data),
    .mem2proc_tag(mem2proc_tag),
    .proc2mem_addr(proc2mem_addr),
    .proc2mem_data(proc2mem_data),
    .proc2mem_command(proc2mem_command),
    .proc2mem_size(proc2mem_size)
  );

  // Task to preload cache manually
  task preload_cache_line(input int way, input [`XLEN-1:0] addr, input [63:0] data);
    int idx;
    begin
      idx = dut.hashed_indices[(way+1)*5-1 : way*5]; // SETS_WIDTH = 5
      dut.cache_data_valid[way*32 + idx] = 1'b1;      // SETS = 32
      dut.cache_data_tag[`XLEN*way*32 + (idx+1)*`XLEN -1 -: `XLEN] = addr;
      dut.cache_data[64*way*32 + (idx+1)*64 -1 -: 64] = data;
    end
  endtask

  reg [4:0] lru_idx; // SETS_WIDTH = 5

  initial begin
    $display("=== ZCACHE UNIT TEST START ===");
    reset = 1;
    proc2Dcache_addr = 0;
    mem2proc_response = 0;
    mem2proc_data = 0;
    mem2proc_tag = 0;

    @(posedge clk); reset = 0;

    // Test 1: Cold Miss
    proc2Dcache_addr = 32'h0000_1234;
    @(posedge clk);
    @(posedge clk);
    assert(dut.hit == 0) else $fatal("FAIL: Expected miss on cold cache");

    // Test 2: Preload way 0
    preload_cache_line(0, 32'h0000_1234, 64'hDEADBEEFCAFEBABE);
    @(posedge clk);
    assert(dut.hit == 1) else $fatal("FAIL: Expected hit after preload");
    assert(dut.hit_data == 64'hDEADBEEFCAFEBABE) else $fatal("FAIL: Data mismatch");

    // Test 3: Miss on different address
    proc2Dcache_addr = 32'h0000_5678;
    @(posedge clk);
    @(posedge clk);
    assert(dut.hit == 0) else $fatal("FAIL: Expected miss on new address");

    // Test 4: Preload into way 3
    preload_cache_line(3, 32'h0000_5678, 64'hBADDCAFE12345678);
    @(posedge clk);
    assert(dut.hit == 1) else $fatal("FAIL: Expected hit on way 3");
    assert(dut.hit_data == 64'hBADDCAFE12345678) else $fatal("FAIL: Data mismatch");

    // Test 5: LRU update
    lru_idx = dut.hashed_indices[(3+1)*5-1 : 3*5];
    @(posedge clk);
    assert(dut.lru[3*32 + lru_idx] != 0) else $fatal("FAIL: LRU not updated");

    $display("@@@ Passed");
    $finish;
  end

endmodule
