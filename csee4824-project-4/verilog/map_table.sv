`include "verilog/sys_defs.svh"

module map_table(
    input reset,
    input clock,
    input [4:0] rs1_addr, //Source register 1 address
    input [4:0] rs2_addr, //Source register 2 address
    input [4:0] r_dest, //Destination register address from Reservation Station

    input [4:0] tag_in, //Read port for tag
    //input tag_in_valid,
    input [4:0] cdb_tag, //Read CDB tag broadcast 

    input read_cdb,

    output logic [4:0] rs1_tag, //Tag output to RS opA
    output logic [4:0] rs2_tag, //Tag output to RS opB

    output logic [4:0] regfile_rs1_addr,
    output logic [4:0] regfile_rs2_addr,
    output logic [4:0] reg_write //Register write destination
);

    logic [4:0] tags[31:0];
    logic has_tag[31:0];
    logic [4:0] indx;
    logic hit;

    assign regfile_rs1_addr = rs1_addr;
    assign regfile_rs2_addr = rs2_addr;
    
    always_ff @(posedge clock) begin

        if (reset) begin
            hit <= 0;
            for (int i = 0; i < 32; i++) begin
                has_tag[i] <= 1;
                tags[i] <= 5'b0;
            end
            reg_write <= `ZERO_REG;
        end

        else begin
            if (read_cdb) begin
                for (int i = 31; i >= 0; i--) begin
                    if (has_tag[i] && (tags[i] == cdb_tag)) begin
                        indx <= i;
                        hit <= 1;
                        tags[i] = 5'b0;
                        reg_write <= i;
                    end
                end
            end else begin
                //Read tag from RS
                if (r_dest != `ZERO_REG) begin
                    tags[r_dest] <= tag_in;
                end
                //Tag outputs to RS
                if (has_tag[rs1_addr]) begin
                    rs1_tag <= tags[rs1_addr];
                end
                if (has_tag[rs2_addr]) begin
                    rs2_tag <= tags[rs2_addr];
                end 
            end
        end

    end


endmodule