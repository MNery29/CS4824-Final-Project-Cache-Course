/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  ras.sv                                              //
//                                                                     //
//  Description :                                                      //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

module ras(
    input  logic         clock,
    input  logic         reset,

    input  logic         push,
    input  logic         pop,
    input  logic [`XLEN-1:0] push_address,

    output logic [`XLEN-1:0] predict_return
);

    localparam RAS_DEPTH = 16;

    logic [`XLEN-1:0] stack [RAS_DEPTH];
    logic [$clog2(RAS_DEPTH)-1:0] sp; // stack pointer

    assign predict_return = stack[sp];

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            sp <= 0;
        end else begin
            if (push) begin
                stack[sp] <= push_address;
                sp <= sp + 1;
            end else if (pop) begin
                if (sp > 0)
                    sp <= sp - 1;
            end
        end
    end

endmodule