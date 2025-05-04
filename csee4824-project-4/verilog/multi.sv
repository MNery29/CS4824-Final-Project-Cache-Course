
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


// This is one stage of an 8 stage pipelined multiplier that multiplies
// two 64-bit integers and returns the low 64 bits of the result.
// This is not an ideal multiplier but is sufficient to allow a faster clock
// period than straight multiplication.


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


module multi (
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
