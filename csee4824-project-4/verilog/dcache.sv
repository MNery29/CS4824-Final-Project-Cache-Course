/*
dcache specifications:
    256 bytes
    directly mapped
    non-blocking
    write-through
    cache line size of 4 bytes, 64 lines
    4 bytes = 2 offset length 
    64 lines = 6 index bits
    for a 32 bit address length: { Tag(24 bits) | Index(6 bits) | Offset(2 bits) }
*/

module dcache
#()
(
    input clk;
    input reset;
);

endmodule;