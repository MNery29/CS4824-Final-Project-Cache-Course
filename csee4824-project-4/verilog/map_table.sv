`include "verilog/sys_defs.svh"

module mt(
    input reset,
    input clock,
    input [4:0] rs1_addr, //Source register 1 address
    input [4:0] rs2_addr, //Source register 2 address
    input [4:0] r_dest, //Destination register address from Reservation Station

    input [4:0] tag_in, //Read port for tag
    input tag_in_valid,
    input [4:0] cdb_tag, //Read CDB tag broadcast 

    output [4:0] rs1_tag, //Tag output to RS opA
    output [4:0] rs2_tag, //Tag output to RS opB
    output rs1_valid, //Valid bits
    output rs2_valid,

    output [4:0] regfile_rs1_addr,
    output [4:0] regfile_rs2_addr,
    output [4:0] reg_write, //Register write destination
);

    logic [4:0] tags[31:0];
    logic has_tag[31:0];
    logic [4:0] indx;
    logic hit;

    assign regfile_rs1_addr = rs1_addr;
    assign regfile_rs2_addr = rs2_addr;

    always_comb begin
        hit = 0;
        for (int i = 0; i < 32; i++) begin
            if (has_tag[i] && (tags[i] == cbd_tag)) begin
                indx = i;
                hit = 1;
                break;
            end
        end
    end
    always_ff @posedge(clock) begin
        if (reset) begin
            has_tag[31:0] <= 0;
        end
        else begin
            if (tag_in_valid) begin
                tags[r_dest] <= tag_in;
                has_tag[r_dest] <= 1;
            end
            else begin
            end
            if (hit) begin
                reg_write <= indx;
            end
            else begin
                reg_write <= ZERO_REG;
            end
        end

        //Tag outputs to RS
        if ((rs1_addr != ZERO_REG) && (has_tag[rs1_addr])) begin
            rs1_valid <= 1;
            rs1_tag <= tags[rs1_addr];
        end else begin
            rs1_valid <= 0;
        end
        if ((rs2_addr != ZERO_REG) && (has_tag[rs2_addr])) begin
            rs2_valid <= 1;
            rs2_tag <= tags[rs2_addr];
        end else begin
            rs2_valid <= 0;
        end
    end


endmodule