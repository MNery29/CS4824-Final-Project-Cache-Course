
// This is a pipelined multiplier that multiplies two 64-bit integers and
// returns the low 64 bits of the result.
// This is not an ideal multiplier but is sufficient to allow a faster clock
// period than straight multiplication.

`include "verilog/sys_defs.svh"

// P4 TODO: You must implement the different types of multiplication here and
//          in mult_stage. See the original ALU and it's different behavior for
//          different multiply functions.

// 3 types of multiplication support
// where 
// logic signed [`XLEN-1:0]   signed_opa, signed_opb;
// logic signed [2*`XLEN-1:0] signed_mul, mixed_mul;
// logic        [2*`XLEN-1:0] unsigned_mul;
// assign signed_mul   = signed_opa * signed_opb;
// assign unsigned_mul = opa * opb;
// assign mixed_mul    = signed_opa * opb;

module booth_mult (
    input clock, reset,
    input [63:0] mcand, mplier,
    input start,

    output [63:0] product,
    output done
);

    logic [`MULT_STAGES-2:0] internal_dones, internal_phantoms;
    logic [(64*(`MULT_STAGES-1))-1:0] internal_mcands;
    logic [(128*(`MULT_STAGES-1))-1:0] internal_product_sums;
    logic [127:0] full_product;
    logic [63:0] mcand_out, mplier_out; // unused, just for wiring
    logic [127:0] initial_product_sum; // this is the initial product sum, {AC, QR} = {0, mplier}
    logic initial_phantom_bit; // this is the initial phantom bit, starts off as 0
    logic out_phantom_bit; // this is the output phantom bit, used for the next stage

    assign initial_product_sum = {64'h0, mplier};
    assign initial_phantom_bit = 1'b0;

    // instantiate an array of mult_stage modules
    // this uses concatenation syntax for internal wiring, see lab 2 slides
    booth_mult_stage mstage [`MULT_STAGES-1:0] (
        .clock (clock),
        .reset (reset),
        .start       ({internal_dones,        start}), // forward prev done as next start
        .prev_sum    ({internal_product_sums, initial_product_sum}), // start the sum at 0
        .mcand       ({internal_mcands,       mcand}),
        .phantom_bit ({internal_phantoms,      initial_phantom_bit}), // start the phantom bit at 0
        .product_sum ({full_product,    internal_product_sums}),
        .out_phantom_bit ({out_phantom_bit, internal_phantoms}), // phantom bit for next stage
        .next_mcand  ({mcand_out,  internal_mcands}),
        .done        ({done,       internal_dones}) // done when the final stage is done
    );
    assign product = full_product[63:0]; // this is the final product

endmodule
