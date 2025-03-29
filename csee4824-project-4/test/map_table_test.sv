`include "verilog/sys_defs.svh"
`timescale 1ns/1ps

module testbench;

    logic clock, reset, read_cdb;
    logic [4:0] rs1_addr, rs2_addr, r_dest, rs_tag_in, cdb_tag_in, rs1_tag, rs2_tag, regfile_rs1_addr, regfile_rs2_addr, reg_write;

    map_table MT(
        .clock(clock),
        .reset(reset),
        .read_cdb(read_cdb),
        .rs1_addr(rs1_addr),
        .rs2_addr(rs2_addr),
        .r_dest(r_dest),
        .rs_tag_in(rs_tag_in),
        .cdb_tag_in(cdb_tag_in),
        .rs1_tag(rs1_tag),
        .rs2_tag(rs2_tag),
        .regfile_rs1_addr(regfile_rs1_addr),
        .regfile_rs2_addr(regfile_rs2_addr),
        .reg_write(reg_write)
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    initial begin
        $monitor("Time:%4.0f clock:%b reset:%b read_cdb:%b rs1_addr:%b rs2_addr:%b r_dest:%b rs_tag_in:%b cdb_tag_in:%b rs1_tag:%b rs2_tag:%b regfile_rs1:%b regfile_rs2:%b reg_write:%b",
                 $time, clock, reset, read_cdb, rs1_addr, rs2_addr, r_dest, rs_tag_in, cdb_tag_in, rs1_tag, rs2_tag, regfile_rs1_addr, regfile_rs2_addr, reg_write);

        clock = 1'b0;
        reset = 1'b1;
        read_cdb = 1'b0;
        rs1_addr = 5'b0;
        rs2_addr = 5'b0;
        r_dest = 5'b0;
        rs_tag_in = 5'b0;
        cdb_tag_in = 5'b0;
        @(negedge clock);
        @(negedge clock);
        reset = 1'b0;

        @(negedge clock);
        read_cdb = 1'b0;
        rs1_addr = 5'b0;
        rs2_addr = 5'b0;
        r_dest = 5'b00001;
        rs_tag_in = 5'b00001;

        @(negedge clock);
        read_cdb = 1'b0;
        rs1_addr = 5'b0;
        rs2_addr = 5'b0;
        r_dest = 5'b00001;
        rs_tag_in = 5'b00001;

        @(negedge clock);
        read_cdb = 1'b0;
        rs1_addr = 5'b1;
        rs2_addr = 5'b0;
        r_dest = 5'b00010;
        rs_tag_in = 5'b00010;

        @(negedge clock);
        read_cdb = 1'b1;
        cdb_tag_in = 5'b00001;

        @(negedge clock);
        read_cdb = 1'b0;
        rs1_addr = 5'b1;
        rs2_addr = 5'b0;
        r_dest = 5'b00010;
        rs_tag_in = 5'b00010;
        @(negedge clock);
        @(negedge clock);

        $finish;

    end

endmodule
