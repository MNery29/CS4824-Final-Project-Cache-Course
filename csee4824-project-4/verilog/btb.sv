/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  btb.sv                                              //
//                                                                     //
//  Description :                                                      //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`include "verilog/sys_defs.svh"

module btb(
    input  logic         clock,
    input  logic         reset,

    input  logic [`XLEN-1:0] lookup_pc,
    output logic             hit,
    output logic [`XLEN-1:0] target_out,

    input  logic             update,
    input  logic [`XLEN-1:0] update_pc,
    input  logic [`XLEN-1:0] update_target
);

    // Parameters
    localparam BTB_ENTRIES = 128;
    localparam INDEX_BITS  = 7;  // log2(128)

    // Separate arrays for each field
    logic              valid_array [BTB_ENTRIES];
    logic [19:0]       tag_array   [BTB_ENTRIES];
    logic [`XLEN-1:0]  target_array[BTB_ENTRIES];

    // Index and tag extraction
    logic [INDEX_BITS-1:0] lookup_index;
    logic [19:0]           lookup_tag;
    logic [INDEX_BITS-1:0] update_index;
    logic [19:0]           update_tag;

    assign lookup_index = lookup_pc[8:2];
    assign lookup_tag   = lookup_pc[31:12];

    assign update_index = update_pc[8:2];
    assign update_tag   = update_pc[31:12];

    // Lookup (pure combinational)
    always_comb begin
        if (valid_array[lookup_index] && (tag_array[lookup_index] == lookup_tag)) begin
            hit = 1'b1;
            target_out = target_array[lookup_index];
        end else begin
            hit = 1'b0;
            target_out = 32'b0;
        end
    end

    // Update (only on branch resolution)
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < BTB_ENTRIES; i++) begin
                valid_array[i] <= 1'b0;
                tag_array[i]   <= 20'b0;
                target_array[i] <= 32'b0;
            end
        end else if (update) begin
            valid_array[update_index] <= 1'b1;
            tag_array[update_index]   <= update_tag;
            target_array[update_index] <= update_target;
        end
    end

endmodule