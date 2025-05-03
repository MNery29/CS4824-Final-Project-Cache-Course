`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

module testbench();

  // Clock and reset
  logic clock, reset;

  // Memory interface
  logic [3:0]  Imem2proc_response;
  logic [63:0] Imem2proc_data;
  logic [3:0]  Imem2proc_tag;

  // Processor interface
  logic [`XLEN-1:0] proc2Icache_addr;
  logic [1:0]       proc2Imem_command;
  logic [`XLEN-1:0] proc2Imem_addr;

  // Cache output
  logic [63:0] Icache_data_out;
  logic        Icache_valid_out;

  // DUT
  icache dut (
    .clock(clock),
    .reset(reset),
    .Imem2proc_response(Imem2proc_response),
    .Imem2proc_data(Imem2proc_data),
    .Imem2proc_tag(Imem2proc_tag),
    .proc2Icache_addr(proc2Icache_addr),
    .proc2Imem_command(proc2Imem_command),
    .proc2Imem_addr(proc2Imem_addr),
    .Icache_data_out(Icache_data_out),
    .Icache_valid_out(Icache_valid_out)
  );

  // Clock generation
  always #5 clock = ~clock;

  // Task to simulate a memory return
  task memory_respond(input [3:0] tag, input [63:0] data);
    begin
      @(posedge clock);
      Imem2proc_response <= tag;
      Imem2proc_data <= data;
      Imem2proc_tag <= tag;
      @(posedge clock);
      Imem2proc_response <= 0;
      Imem2proc_data <= 0;
      Imem2proc_tag <= 0;
    end
  endtask

  initial begin
    $display(" ICACHE TEST START ");

    clock = 0;
    reset = 1;
    Imem2proc_response = 0;
    Imem2proc_data = 0;
    Imem2proc_tag = 0;
    proc2Icache_addr = 0;

    @(posedge clock);
    reset = 0;

    // MISS 
    $display("\n Test 1: Cold Miss");
    proc2Icache_addr = 32'h0000_1000;
    repeat (2) @(posedge clock);
    assert(Icache_valid_out == 0) else $fatal("FAIL: Expected cache miss");
    // MEM Fills Cache
    $display("\n Test 2: Memory Response");
    memory_respond(4'd1, 64'hDEADBEEFCAFEBABE); // respond with data
    repeat (2) @(posedge clock);
    assert(Icache_valid_out == 1 && Icache_data_out == 64'hDEADBEEFCAFEBABE)
      else $fatal("FAIL: Expected cache hit with correct data");
    // Cache Hit
    $display("\n Test 3: Cache Hit");
    repeat (2) @(posedge clock); // stay at same address
    assert(Icache_valid_out == 1) else $fatal("FAIL: Expected cache hit");
    //Address Change -> Miss 
    $display("\n Test 4: Address Change Miss");
    proc2Icache_addr = 32'h0000_2000; // different index
    repeat (2) @(posedge clock);
    assert(Icache_valid_out == 0) else $fatal("FAIL: Expected miss on new address");
    // Fill second line
    memory_respond(4'd2, 64'h1122334455667788);
    repeat (2) @(posedge clock);
    assert(Icache_valid_out == 1 && Icache_data_out == 64'h1122334455667788)
      else $fatal("FAIL: Expected cache hit with new data");
    // Reset clears cache
    $display("\n Test 5: Reset Invalidation");
    reset = 1;
    @(posedge clock);
    reset = 0;
    proc2Icache_addr = 32'h0000_1000; // previous hit address
    repeat (2) @(posedge clock);
    assert(Icache_valid_out == 0) else $fatal("FAIL: Cache should be invalid after reset");
    $display("\n ICACHE TEST PASSED");
    $finish;
  end

endmodule
