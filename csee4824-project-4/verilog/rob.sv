/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob.sv                                              //
//                                                                     //
//  Description :  Re-order buffer for P6 architecture, ensures        //
//                 in-order appearance is maintained                   //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module rob(
    //standard input signals for module.
    input reset,
    input clk,

    //input signals from dispatch stage 
    //opcode
    input [4:0] dispatch_dest_reg,
    input [7:0] dispatch_opcode,
    input dispatch_valid,

    //input signals from execute stage 
    input [4:0] cdb_tag, //cdb - commnon data bus
    input [63:0] cdb_value,
    input cdb_valid,

    //input signals from retire stage
    input retire_valid,
    input branch_mispredict,

    //output signals to Regfile
    output [4:0] reg_dest,
    output [63:0] reg_value,
    output reg_valid,

    // output from map table: Tag for latest dispatch 
    output [4:0] map_table_tag,
    output map_table_tag_valid,
    //output signals to memory
    output [63:0] mem_addr,
    //rob full signal, for stalling/hazards   
    output rob_full
);
    //internal signals
    //rob entries - 5
    logic [63:0] rob_values[4:0];
    logic [7:0] rob_opcode[4:0];
    logic [4:0] rob_dest[4:0];
    logic rob_busy [4:0]; //if operation is still happening
    logic rob_ready[4:0]; //monitor if ROB values are ready to be committed

    //head and tail pointers for queue FIFO structure
    logic head[5:0];
    logic tail[5:0];

    assign rob_full = ((head[4:0] == tail[4:0]) && (head[5] != tail[5]));
    
    always_ff @(posedge clk) begin
        if(reset) begin
            rob_values[4:0] <= 0;
            rob_dest[4:0] <= 0;
            rob_ready[4:0] <= 0;
            head[4:0] <= 0;
            tail[4:0] <= 0;
        end
        else begin
            
            if(dispatch_valid) begin
                if (!rob_full) begin
                    rob_values[tail[4:0]] <= 0;
                    rob_dest[tail[4:0]] <= dispatch_dest_reg;
                    rob_opcode[tail[4:0]] <= dispatch_opcode; //maybe not necessary check if opcode is ever needed                   rob_ready[tail[4:0]] <= 0;
                    rob_busy[tail[4:0]] <= 1;
                    map_table_tag <= tail[4:0]; //Uses the fact tail doesn't update till next cycle to work. Check here if there are errors. 
                    map_table_tag_valid <= 1;   
                    tail[4:0] <= tail[4:0] + 1;
                end 
                map_table_tag_valid <= 0;
            end
            if (cdb_valid) begin
                rob_values[tag] <= cdb_value;
                rob_ready[tag] <= 1'b1;
                rob_busy[tag] <= 1'b0;
            end
            if (retire_valid) begin
                if (branch_mispredict) begin
                //    head[4:0] <= map_table_tag;
                end
                else begin
                    reg_dest <= rob_dest[head[4:0]];
                    reg_value <= rob_values[head[4:0]];
                    reg_valid <= rob_ready[head[4:0]];
                    mem_addr <= rob_values[head[4:0]]; //temporary placeholder, confirm
                    head[4:0] <= head[4:0] + 1;
                end  
            end
        end
    end



// valid bit needed for ROB

// set the head and tail to one more, store in memory for 

//keep valid bit in memory. 

//also save valid bit as part of memory for ROB, when memory is implement.

//also store valid bits