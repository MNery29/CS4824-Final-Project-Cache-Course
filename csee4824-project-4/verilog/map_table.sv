`include "verilog/sys_defs.svh"

module map_table (
    input reset,
    input clock,
    input [4:0] rs1_addr, //Source register 1 address
    input [4:0] rs2_addr, //Source register 2 address
    input [4:0] r_dest, //Destination register address from Reservation Station

    input [4:0] tag_in, //Read port for tag
    input load_entry,
    input [4:0] cdb_tag_in, //Read CDB tag broadcast 
    input read_cdb, //CDB broadcast?
    input [4:0] retire_addr, //Read address of register retired to
    input [4:0] retire_tag,
    input retire_entry, //ROB retire?

    output logic [5:0] rs1_tag, //Tag output to RS opA, w/ ready in ROB bit
    output logic [5:0] rs2_tag, //Tag output to RS opB, w/ ready in ROB bit

    output logic [4:0] regfile_rs1_addr, //Pass throughs
    output logic [4:0] regfile_rs2_addr
    //Debug outputs
    //output logic [7:0] tags_debug[31:0]
);

    logic [4:0] tags[31:0];
    logic ready_in_rob[31:0];
    logic has_tag[31:0];

    assign regfile_rs1_addr = rs1_addr;
    assign regfile_rs2_addr = rs2_addr;

    always_comb begin
        if (has_tag[rs1_addr]) begin
            rs1_tag = {tags[rs1_addr], ready_in_rob[rs1_addr]}; 
        end
        if (has_tag[rs2_addr]) begin
            rs2_tag = {tags[rs2_addr], ready_in_rob[rs2_addr]};
        end 
    end

    //Debug outputs NOW COMMENTED OUT POST CLEANUP
    //always_comb begin
    //    for (int i = 0; i < 32; i++) begin
    //        tags_debug[i] = {tags[i], ready_in_rob[i], has_tag[i]};
    //    end
    //end
    
    always_ff @(posedge clock) begin
        if (reset) begin
            for (int i = 0; i < 32; i++) begin
                has_tag[i] <= 1;
                tags[i] <= 5'b0;
                ready_in_rob[i] <= 0;
            end
        end

        else begin
            if (read_cdb) begin
                for (int i = 31; i >= 0; i--) begin //CAM to compare map table tag entries with CDB tag
                    if (has_tag[i] && (tags[i] == cdb_tag_in)) begin
                        ready_in_rob[i] <= 1; //Set ready in ROB bit on tag match during complete
                    end
                end
            end
            if ((retire_entry) && (retire_tag == tags[retire_addr])) begin
                tags[retire_addr] <= 5'b0; //Clear tag at destination register on retire
                ready_in_rob[retire_addr] <= 0;
            end
            if (load_entry) begin
                //Assign tag to destination register
                if (r_dest != `ZERO_REG) begin
                    tags[r_dest] <= tag_in;
                    ready_in_rob[r_dest] <= 0;
                end
            end
        end
    end

endmodule