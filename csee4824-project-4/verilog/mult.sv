
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


module mult (
    input logic clock, reset,
    input logic [63:0] mcand, mplier,
    input ALU_FUNC mult_func,
    input logic start,

    output logic [63:0] product,
    output logic done
);

    logic [(128*(`MULT_STAGES-1))-1:0] internal_product_sums;
    logic [(128*(`MULT_STAGES-1))-1:0] internal_mpliers;
    logic [(128*(`MULT_STAGES-1))-1:0] internal_mcands;
    logic [`MULT_STAGES-2:0] internal_dones;

    logic [127:0] full_product_sum;
    logic [127:0] mplier_out, mcand_out;

    //Extend inputs based on mult_type, in order to accomodate signed operations
    logic [127:0] extended_mcand, extended_mplier;


    //combinational block with handling signed/unsigned extension. 
    always_comb begin
        case (mult_func)
            ALU_MUL, ALU_MULH: begin
                extended_mcand  = {{64{mcand[63]}}, mcand};
                extended_mplier = {{64{mplier[63]}}, mplier};
            end
            ALU_MULHSU: begin
                extended_mcand  = {{64{mcand[63]}}, mcand};
                extended_mplier = {64'b0, mplier};
            end
            ALU_MULHU: begin
                extended_mcand  = {64'b0, mcand};
                extended_mplier = {64'b0, mplier};
            end
            default: begin
                extended_mcand  = {64'b0, mcand};
                extended_mplier = {64'b0, mplier};
            end
        endcase
    end


    // Instantiate the pipeline stages
    mult_stage mstage [`MULT_STAGES-1:0] (
        .clock(clock),
        .reset(reset),
        .start({internal_dones, start}),
        .prev_sum({internal_product_sums, 128'h0}),
        .mplier({internal_mpliers, extended_mplier}),
        .mcand({internal_mcands, extended_mcand}),
        .product_sum({full_product_sum, internal_product_sums}),
        .next_mplier({mplier_out, internal_mpliers}),
        .next_mcand({mcand_out, internal_mcands}),
        .done({done, internal_dones})
    );

    // Select output portion
    always_comb begin
        case (mult_func)
            ALU_MUL:    product = full_product_sum[63:0];    // low 64 bits
            ALU_MULH,
            ALU_MULHSU,
            ALU_MULHU:  product = full_product_sum[127:64];  // high 64 bits
            default:        product = 64'b0;
        endcase
    end

endmodule
