/*
simple h3 hash function
*/

module h3hash #(
    parameter integer WAYS  = 8,
    parameter integer SETS_WIDTH = 5 // 32 so 2^5
)(
    input[`XLEN-1:0] index_in,
    output[WAYS*SETS_WIDTH-1:0] hashed_indices
);
logic [SETS_WIDTH*`XLEN*WAYS-1:0] big_matrix = {
        // Way 7 (5 rows of 32 bits each = 160 bits)
        32'hF7F7_7F7F, // row 0
        32'h7777_7777, // row 1
        32'hDEAD_BEEF, // row 2
        32'hAAAA_5555, // row 3
        32'h1357_9BDF, // row 4

        // Way 6
        32'hF6F6_6F6F,
        32'h6666_6666,
        32'hDEAD_BEEF,
        32'hAAAA_5555,
        32'h1357_9BDF,

        // Way 5
        32'hF5F5_5F5F,
        32'h5555_5555,
        32'hDEAD_BEEF,
        32'hAAAA_5555,
        32'h1357_9BDF,

        // Way 4
        32'hF4F4_4F4F,
        32'h4444_4444,
        32'hDEAD_BEEF,
        32'hAAAA_5555,
        32'h1357_9BDF,

        // Way 3
        32'hF3F3_3F3F,
        32'h3333_3333,
        32'hDEAD_BEEF,
        32'hAAAA_5555,
        32'h1357_9BDF,

        // Way 2
        32'hF2F2_2F2F,
        32'h2222_2222,
        32'hDEAD_BEEF,
        32'hAAAA_5555,
        32'h1357_9BDF,

        // Way 1
        32'hF1F1_1F1F,
        32'h1111_1111,
        32'hDEAD_BEEF,
        32'hAAAA_5555,
        32'h1357_9BDF,

        // Way 0
        32'hF0F0_0F0F,
        32'h0000_0000,
        32'hDEAD_BEEF,
        32'hAAAA_5555,
        32'h1357_9BDF
    };

    // logic [2:0] i;
    // logic [2:0] a;

    // for (a = 0; a < WAYS; a = a+1) begin
    //     for (i = 0; i < SETS_WIDTH; i = i + 1) begin
    //         hashed_indices[a*SETS_WIDTH + i] = 
    //                         ^(index_in & 
    //                         big_matrix[a*`XLEN*SETS_WIDTH + (i+1)*`XLEN - 1 : a*`XLEN*SETS_WIDTH + i*`XLEN]);
    //     end
    // end

    //force unroll:
    genvar a, i;
    generate
    for (a = 0; a < WAYS; a = a+1) begin : way_loop
        for (i = 0; i < SETS_WIDTH; i = i+1) begin : index_loop
        assign hashed_indices[a*SETS_WIDTH + i] =
            ^(index_in & big_matrix[a*`XLEN*SETS_WIDTH
                                    + (i+1)*`XLEN - 1
                                    : a*`XLEN*SETS_WIDTH
                                    + i*`XLEN]);
        end
    end
    endgenerate


endmodule;