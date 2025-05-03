
// This is one stage of an 8 stage pipelined multiplier that multiplies
// two 64-bit integers and returns the low 64 bits of the result.
// This is not an ideal multiplier but is sufficient to allow a faster clock
// period than straight multiplication.

`include "verilog/sys_defs.svh"

module mult_stage (
    input logic clock, reset, start,
    input logic [127:0] prev_sum,
    input logic [127:0] mplier, mcand,

    output logic [127:0] product_sum,
    output logic [127:0] next_mplier, next_mcand,
    output logic done
);

    parameter SHIFT = 128/`MULT_STAGES; // Note: inputs are now 128 bits

    logic [SHIFT-1:0] mplier_slice;
    logic [127:0] shifted_mplier, shifted_mcand;
    logic [127:0] partial_product;

    assign mplier_slice = mplier[SHIFT-1:0];

    // Simple unsigned partial product (operands already extended)
    assign partial_product = mplier_slice * mcand;

    // Shift mplier and mcand
    assign shifted_mplier = { {SHIFT{1'b0}}, mplier[127:SHIFT] };
    assign shifted_mcand  = { mcand[127-SHIFT:0], {SHIFT{1'b0}} };

    always_ff @(posedge clock) begin
        if (reset) begin
            product_sum <= 128'b0;
            next_mplier <= 128'b0;
            next_mcand  <= 128'b0;
            done        <= 1'b0;
        end else begin
            product_sum <= prev_sum + partial_product;
            next_mplier <= shifted_mplier;
            next_mcand  <= shifted_mcand;
            done        <= start;
        end
    end

endmodule
