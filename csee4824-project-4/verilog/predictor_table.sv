/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  predictor_table.sv                                  //
//                                                                     //
//  Description :                                                      //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

module predictor_table(
    input  logic         clock,
    input  logic         reset,

    input  logic [`XLEN-1:0] lookup_pc,
    output logic             predict_taken,

    input  logic             update,
    input  logic [`XLEN-1:0] update_pc,
    input  logic             actual_taken
);

    localparam PREDICTOR_ENTRIES = 128;
    localparam INDEX_BITS = 7;

    logic [1:0] counters [PREDICTOR_ENTRIES];

    assign predict_taken = (counters[lookup_pc[8:2]] >= 2);

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < PREDICTOR_ENTRIES; i++) begin
                counters[i] <= 2'b01; // Weakly Not Taken
            end
        end else if (update) begin
            if (actual_taken) begin
                if (counters[update_pc[8:2]] != 2'b11)
                    counters[update_pc[8:2]] <= counters[update_pc[8:2]] + 1;
            end else begin
                if (counters[update_pc[8:2]] != 2'b00)
                    counters[update_pc[8:2]] <= counters[update_pc[8:2]] - 1;
            end
        end
    end

endmodule