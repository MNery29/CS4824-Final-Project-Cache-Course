
// This is one stage of an 8 stage pipelined multiplier that multiplies
// two 64-bit integers and returns the low 64 bits of the result.
// This is not an ideal multiplier but is sufficient to allow a faster clock
// period than straight multiplication.

`include "verilog/sys_defs.svh"

//booth's algorithm support for multiplication
//you must sign extend the unsigned values before passing it here!
// i am assuming the inputs are all signed

//LEGEND ( i am using this notation to fit with diagrams i found online )
// https://sitams.org/wp-content/uploads/2023/05/Booths-Multiplication-Algorithm-for-Signed-2s-Complement.pdf
// QR -> multiplier
// BR -> multicand 
// AC -> partial product
// SC -> length of bits for XLEN = 32 -> 5

// so the product sum is actually {AC, QR} 


module mult_stage #(
    parameter int STEPS_PER_CYCLE  = 4     // # micro‑shifts to do in one clk
)
(
    input clock, reset, start,
    input [`XLEN*2-1:0] prev_sum, // this is {AC, QR}
    input [`XLEN-1:0] mcand, // This is BR
    input phantom_bit, // this is the extra bit we add to the end of QR, starts off as 0

    output logic [`XLEN*2-1:0] product_sum, 
    output logic out_phantom_bit,
    output logic done
);

    logic [1:0] qn;

    logic signed [`XLEN-1:0] ac, qr;

    always_comb begin
        qr = prev_sum[`XLEN-1:0];
        ac = prev_sum[(2*`XLEN)-1:`XLEN];
    end
    logic signed [`XLEN-1:0] next_ac;

    logic signed [`XLEN*2-1:0] shifted_product_sum;

    localparam int TRIPLE_W = 2*`XLEN + 1;
    logic signed [TRIPLE_W-1:0] work;          // {AC, QR, q‑1}
    int k;

    logic [2*XLEN-1:0] product_sum_next = work[TRIPLE_W-1:1];
    logic phantom_next = work[0];

    integer i;
    always_comb begin
        work = {prev_sum, phantom_bit};

        for (i=0; i < STEPS_PER_CYCLE; i = i+1 ) begin
            unique case (work[1:0])
                2'b01: work[TRIPLE_W-1:XLEN] = work[TRIPLE_W-1:XLEN] - $signed(mcand);
                2'b10: work[TRIPLE_W-1:XLEN] = work[TRIPLE_W-1:XLEN] + $signed(mcand);
                default: ;
            endcase
            work = work >>> 1;
        end
    end


     always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            product_sum <= '0;
            out_phantom_bit <= 1'b0;
            done <= 1'b0;
        end else if (start) begin
            product_sum <= product_sum_next;
            out_phantom_bit <= phantom_next;
            done <= 1'b1;
        end else begin
            done <= 1'b0;
        end
    end

endmodule
