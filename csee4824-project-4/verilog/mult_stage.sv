
// This is one stage of an 8 stage pipelined multiplier that multiplies
// two 64-bit integers and returns the low 64 bits of the result.
// This is not an ideal multiplier but is sufficient to allow a faster clock
// period than straight multiplication.

`include "verilog/sys_defs.svh"

module mult_stage (
    input logic clock, reset, start,
    input logic [127:0] prev_sum, //use full 128 bits for the sum
    input logic [63:0] mplier, mcand,      // inputs still 64 bits

    output logic [127:0] product_sum, //full 128 bits for the sum
    output logic [63:0] next_mplier, next_mcand,
    output logic done
);

    parameter SHIFT = 64/`MULT_STAGES;

    logic [127:0] partial_product;
    logic [63:0] shifted_mplier, shifted_mcand;

    assign partial_product = mplier[SHIFT-1:0] * mcand;

    assign shifted_mplier = {SHIFT'('b0), mplier[63:SHIFT]};
    assign shifted_mcand = {mcand[63-SHIFT:0], SHIFT'('b0)};

    always_ff @(posedge clock) begin
        if (reset) begin
            product_sum <= 128'b0;
            next_mplier <= 64'b0;
            next_mcand  <= 64'b0;
            done        <= 1'b0;
        end else begin
            product_sum <= prev_sum + partial_product;
            next_mplier <= shifted_mplier;
            next_mcand  <= shifted_mcand;
            done        <= start;
        end
    end

endmodule
