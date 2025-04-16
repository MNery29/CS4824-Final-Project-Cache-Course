
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

//actually we will assume this only takes one STEP
// so basically one sc step

module mult_stage (
    input clock, reset, start,
    input [`XLEN*2-1:0] prev_sum, // this is {AC, QR}
    input [`XLEN-1:0] mcand, // This is BR
    input phantom_bit, // this is the extra bit we add to the end of QR, starts off as 0

    output logic [`XLEN*2-1:0] product_sum, 
    output logic out_phantom_bit,
    output logic done
);

    logic [`XLEN-1:0] ac, qr;

    always_comb begin
        qr = prev_sum[`XLEN-1:0];
        ac = prev_sum[(2*`XLEN)-1:`XLEN];
    end
    logic [`XLEN-1:0] next_ac;

    assign out_phantom_bit = qr[0];
    logic [`XLEN*2-1:0] shifted_product_sum;

    always_comb begin
        shifted_product_sum = {ac, qr} >> 1;
        next_ac = ac;
        if (qr[0] == 1'b0 && phantom_bit == 1'b0) begin
            // do nothing
            shifted_product_sum = {ac, qr} >> 1;
        end else if (qr[0] == 1'b1 && phantom_bit == 1'b0) begin
            // add BR to AC
            next_ac = ac + mcand;
            shifted_product_sum = {next_ac, qr} >> 1;
        end else if (qr[0] == 1'b0 && phantom_bit == 1'b1) begin
            // subtract BR from AC
            next_ac = ac + (~mcand + 1);
            shifted_product_sum = {next_ac, qr} >> 1;
        end else if (qr[0] == 1'b1 && phantom_bit == 1'b1) begin
            // do nothing
            shifted_product_sum = {ac, qr} >> 1;
        end
        else begin
            // do nothing
            shifted_product_sum = {ac, qr} >> 1;
        end
    end

    always_ff @(posedge clock) begin
        product_sum <= shifted_product_sum;
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            done <= 1'b0;
        end else begin
            done <= start;
        end
    end

endmodule
