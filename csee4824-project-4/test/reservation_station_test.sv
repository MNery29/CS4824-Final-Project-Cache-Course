`include "verilog/sys_defs.svh"
`timescale 1ns/1ps

module testbench;

    logic clock, reset, rs_cdb_valid, rs_opa_valid, rs_opb_valid, rs_load_in, rs_use_enable, rs_free_in, rs_ready_out, rs_avail_out;
    logic [4:0] rs_rob_tag, rs_cdb_tag, rs_tag_out;
    logic [2:0] rs_debug_status;
    logic [31:0] rs_cdb_in, rs_opa_in, rs_opb_in, rs_opa_out, rs_opb_out;

    reservation_station rs(
        .clock(clock), 
        .reset(reset),
        .rs_cdb_valid(rs_cdb_valid),
        .rs_opa_valid(rs_opa_valid),
        .rs_opb_valid(rs_opb_valid),
        .rs_load_in(rs_load_in),
        .rs_use_enable(rs_use_enable),
        .rs_free_in(rs_free_in),
        .rs_ready_out(rs_ready_out),
        .rs_avail_out(rs_avail_out),
        .rs_rob_tag(rs_rob_tag),
        .rs_cdb_tag(rs_cdb_tag),
        .rs_tag_out(rs_tag_out),
        .rs_debug_status(rs_debug_status),
        .rs_cdb_in(rs_cdb_in),
        .rs_opa_in(rs_opa_in),
        .rs_opb_in(rs_opb_in),
        .rs_opa_out(rs_opa_out),
        .rs_opb_out(rs_opb_out)
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    initial begin
        $monitor("Time:%4.0f clock:%b reset:%b |Inputs:| cdb_valid:%b cdb_tag:%b cdb_in:%h rob_tag:%b opa_valid:%b opa_in:%h opb_valid:%b opb_in:%h load_in:%b use_en:%b free_in:%b\
                 |Outputs:| ready_out:%b avail_out:%b opa_out:%h opb_out:%h tag_out:%b debug_status:%b",
                 $time, clock, reset, rs_cdb_valid, rs_cdb_tag, rs_cdb_in, rs_rob_tag, rs_opa_valid, rs_opa_in, rs_opb_valid, rs_opb_in, rs_load_in, rs_use_enable, rs_free_in,
                 rs_ready_out, rs_avail_out, rs_opa_out, rs_opb_out, rs_tag_out, rs_debug_status);

        //Reset 
        clock = 1;
        reset = 1; //Pull reset high

        rs_cdb_valid = 0;
        rs_cdb_tag = 0;
        rs_cdb_in = 0;

        rs_rob_tag = 0;
        rs_opa_valid = 0;
        rs_opa_in = 0;
        rs_opb_valid = 0;
        rs_opb_in = 0;

        rs_load_in = 0;
        rs_use_enable = 0;
        rs_free_in = 0;
        @(negedge clock);
        reset = 0;
        @(negedge clock);
        rs_opa_in = 32'h0000_0002; //Simulate loading in tags
        rs_opa_valid = 0; //opa is a tag
        rs_opb_in = 32'h0000_0003;
        rs_opb_valid = 0; //opb is a tag
        rs_rob_tag = 5'b00001;
        rs_load_in = 1; //Load RS entry
        rs_use_enable = 1; //Output values for debugging
        @(negedge clock);
        rs_load_in = 0; 
        rs_cdb_valid = 1; //Load CDB broadcast
        rs_cdb_tag = 32'h0000_0002; //opa tag should be cleared
        rs_cdb_in = 32'h0000_FFFF; //this value should be loaded into opa, which should now be valid
        @(negedge clock);
        rs_load_in = 0; 
        rs_cdb_valid = 1; //Load CDB broadcast
        rs_cdb_tag = 32'h0000_0003; //opb tag should be cleared
        rs_cdb_in = 32'hFFFF_0000; //this value should be loaded into opb, which should now be valid
        @(negedge clock);
        rs_cdb_valid = 0; 
        rs_free_in = 1; //RS entry should be freed and made available again
        @(negedge clock);

        $finish;

    end

endmodule
