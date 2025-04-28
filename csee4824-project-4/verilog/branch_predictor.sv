/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  branch_predictor.sv                                 //
//                                                                     //
//  Description :  wraps together the BTB, predictor and RAS           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/btb.sv"
`include "verilog/predictor_table.sv"
`include "verilog/ras.sv"

module branch_predictor(
    input  logic         clock,
    input  logic         reset,

    // Fetch interface
    input  logic [`XLEN-1:0] fetch_pc,         // from IF
    output logic             predict_taken,   // to IF
    output logic [`XLEN-1:0] predict_target,   // to IF

    // Resolve interface (from Retire)
    input  logic             resolve_valid,
    input  logic [`XLEN-1:0] resolve_pc,
    input  logic             resolve_taken,
    input  logic [`XLEN-1:0] resolve_target,
    input  logic             resolve_is_return,
    input  logic             resolve_is_call
);

    // Internal wires
    logic               btb_hit;
    logic [`XLEN-1:0]   btb_target;
    logic               direction_predict_taken;
    logic [`XLEN-1:0]   ras_target;

    btb btb_0( //branch target buffer
        .clock(clock),
        .reset(reset),
        .lookup_pc(fetch_pc),
        .hit(btb_hit),
        .target_out(btb_target),
        .update(resolve_valid & resolve_taken),
        .update_pc(resolve_pc),
        .update_target(resolve_target)
    );

    predictor_table predictor_table_0( //branch predictor
        .clock(clock),
        .reset(reset),
        .lookup_pc(fetch_pc),
        .predict_taken(direction_predict_taken),
        .update(resolve_valid),
        .update_pc(resolve_pc),
        .actual_taken(resolve_taken)
    );

    ras ras_0( //return address stack
        .clock(clock),
        .reset(reset),
        .push(resolve_is_call & resolve_valid),
        .pop(resolve_is_return & resolve_valid),
        .push_address(resolve_pc + 4),
        .predict_return(ras_target)
    );

    // Fetch Prediction Logic
    always_comb begin
        if (btb_hit && direction_predict_taken) begin
            predict_taken = 1'b1;
            predict_target = btb_target;
        end else begin
            predict_taken = 1'b0;
            predict_target = fetch_pc + 4;
        end
    end

endmodule
