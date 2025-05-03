`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

module testbench;
  logic clock = 0;
  logic reset;

  // Inputs
  EX_CP_PACKET ex_cp_packet;

  // Outputs
  CDB_PACKET cdb_packet_out;


  always #5 clock = ~clock;

  // DUT
  stage_cp dut (
    .clock(clock),
    .reset(reset),
    .ex_cp_packet(ex_cp_packet),
    .cdb_packet_out(cdb_packet_out)
  );

  initial begin
    $display(" STAGE_CP TEST START ");
    reset = 1;
    ex_cp_packet = '0;

    @(posedge clock);
    reset = 0;

    // Test 1: Reset clears outputs
    @(posedge clock);
    assert(cdb_packet_out.valid == 0) else $fatal("FAIL: valid should be 0 after reset");
    assert(cdb_packet_out.value == 0) else $fatal("FAIL: value should be 0 after reset");
    assert(cdb_packet_out.tag == 0) else $fatal("FAIL: tag should be 0 after reset");

    // Test 2: Packet with done = 1
    ex_cp_packet.done    = 1;
    ex_cp_packet.value   = 64'hDEADBEEFCAFEBABE;
    ex_cp_packet.rob_tag = 5'd12;

    @(posedge clock);
    assert(cdb_packet_out.valid == 1) else $fatal("FAIL: valid should be 1 when done=1");
    assert(cdb_packet_out.value == 64'hDEADBEEFCAFEBABE) else $fatal("FAIL: incorrect value");
    assert(cdb_packet_out.tag == 5'd12) else $fatal("FAIL: incorrect tag");

    // Test 3: Packet with done = 0 
    ex_cp_packet.done = 0;

    @(posedge clock);
    assert(cdb_packet_out.valid == 0) else $fatal("FAIL: valid should be 0 when done=0");
    assert(cdb_packet_out.value == 0) else $fatal("FAIL: value should be 0 when done=0");
    assert(cdb_packet_out.tag == 0) else $fatal("FAIL: tag should be 0 when done=0");

    $display("@@@ Passed");
    $finish;
  end

endmodule