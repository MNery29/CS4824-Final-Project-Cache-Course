`include "verilog/sys_defs.svh"

module map_table(
    input reset,
    input clock,
    input [4:0] rs1_addr, //Source register 1 address
    input [4:0] rs2_addr, //Source register 2 address
    input [4:0] r_dest, //Destination register address from Reservation Station

    input [4:0] tag_in, //Read port for tag
    input dispatch_valid,
    input [4:0] cdb_tag_in, //Read CDB tag broadcast 
    input read_cdb, //CDB broadcast?
    input [4:0] retire_addr, //Read address of register retired to
    input retire_valid, //ROB retire?

    output logic [5:0] rs1_tag, //Tag output to RS opA, w/ ready in ROB bit
    output logic [5:0] rs2_tag, //Tag output to RS opB, w/ ready in ROB bit
    output logic rs1_tag_valid,
    output logic rs2_tag_valid,

    output logic [4:0] regfile_rs1_addr, //Pass throughs
    output logic [4:0] regfile_rs2_addr,
    //Needs a full signal probably?
);

    logic [4:0] tags[31:0];
    logic [31:0] ready_in_rob;
    logic has_tag[31:0];

    assign regfile_rs1_addr = rs1_addr;
    assign regfile_rs2_addr = rs2_addr;
    
    always_ff @(posedge clock) begin

        if (reset) begin
            hit <= 0;
            for (int i = 0; i < 32; i++) begin
                has_tag[i] <= 1;
                tags[i] <= 5'b0;
                ready_in_rob[i] <= 0;
            end
            reg_write <= `ZERO_REG;
        end

        else begin
            if (read_cdb) begin
                for (int i = 31; i >= 0; i--) begin //CAM to compare map table tag entries with CDB tag
                    if (has_tag[i] && (tags[i] == cdb_tag_in)) begin
                        ready_in_rob[i] = 1; //Set ready in ROB bit on tag match during complete
                    end
                end
            end else if (retire_valid) begin
                tags[retire_addr] = 5'b0; //Clear tag on retire
                ready_in_rob[i] = 0;
            end else if (dispatch_valid) begin
                //Set destination register tag
                if (r_dest != `ZERO_REG) begin
                    tags[r_dest] <= rs_tag_in;
                    ready_in_rob[i] = 0;
                end
                //Tag outputs to RS
                if (has_tag[rs1_addr]) begin
                    rs1_tag <= tags{[rs1_addr], ready_in_rob[rs1_addr]};
                end
                if (has_tag[rs2_addr]) begin
                    rs2_tag <= {tags[rs2_addr], ready_in_rob[rs2_addr]};
                end 
            end
        end

    end


endmodule