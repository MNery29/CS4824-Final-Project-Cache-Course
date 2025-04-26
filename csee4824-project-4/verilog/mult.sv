
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


//Used to determine which type of multiplication to perform 
typedef enum logic [1:0] {
    MUL_ALU_MUL    = 2'b00, // result = low 64 bits of signed_mul
    MUL_ALU_MULH   = 2'b01, // result = high 64 bits of signed_mul
    MUL_ALU_MULHSU = 2'b10, // result = high 64 bits of mixed_mul
    MUL_ALU_MULHU  = 2'b11  // result = high 64 bits of unsigned_mul
} mul_type_t;


module mult (
    input logic clock, reset,
    input logic [63:0] mcand, mplier,
    input mul_type_t mul_type, // which kind of multiplication to compute 
    input logic start,

    output logic [63:0] product,
    output logic done
);

    logic [(128*(`MULT_STAGES-1))-1:0] internal_product_sums;
    logic [(64*(`MULT_STAGES-1))-1:0] internal_mpliers;
    logic [(64*(`MULT_STAGES-1))-1:0] internal_mcands;
    logic [`MULT_STAGES-2:0] internal_dones;

    logic [127:0] full_product_sum;
    logic [63:0]  mplier_out, mcand_out;




    // instantiate an array of mult_stage modules
    // this uses concatenation syntax for internal wiring, see lab 2 slides
    mult_stage mstage [`MULT_STAGES-1:0] (
        .clock (clock),
        .reset (reset),
        .start       ({internal_dones, start}), // forward prev done as next start
        .prev_sum    ({internal_product_sums, 128'h0}), // start the sum at 0, use the full 128 bits. 
        .mplier      ({internal_mpliers,      mplier}),
        .mcand       ({internal_mcands,       mcand}),
        .product_sum ({full_product_sum,    internal_product_sums}),
        .next_mplier ({mplier_out, internal_mpliers}),
        .next_mcand  ({mcand_out,  internal_mcands}),
        .done        ({done,       internal_dones}) // done when the final stage is done
    );

    // assign the output product to be either high 64 bits or low 64 bits of the full product sum, based on type of mult

    always_comb begin
        case (mul_type)
            MUL_ALU_MUL:    product = full_product_sum[63:0];    // low 64 bits (normal multiply)
            MUL_ALU_MULH:   product = full_product_sum[127:64];  // high 64 bits (signed x signed)
            MUL_ALU_MULHSU: product = full_product_sum[127:64];  // high 64 bits (signed x unsigned)
            MUL_ALU_MULHU:  product = full_product_sum[127:64];  // high 64 bits (unsigned x unsigned)
            default:        product = 64'b0;
        endcase
    end

endmodule

