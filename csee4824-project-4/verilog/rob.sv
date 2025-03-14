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
    input reset; 
    input clk;

    //input signals from dispatch stage 
    //opcode
    input [4:0] dispatch_dest_reg;
    input [7:0] dispatch_opcode;
    input dispatch_valid;

    //input signals from execute stage 
    input [4:0] cdb_tag; //cdb - commnon data bus
    input [63:0] cdb_value;
    input cdb_valid;

    //input signals from retire stage
    input retire_valid;
    input branch_mispredict;

    //output signals to Regfile
    output [4:0] reg_dest;
    output [63:0] reg_value;
    output reg_valid;

    //output signals to memory
    output [63:0] mem_addr;

);

    //internal signals
    //rob entries - 5
    logic [63:0] rob_values[4:0];
    logic [4:0] rob_dest[4:0];
    logic rob_busy [4:0];
    logic rob_ready[4:0]; //monitor if ROB values are ready to be committed

    //head and tail pointers for queue FIFO structure
    logic head[4:0];
    logic tail[4:0];
    logic tail_next[4:0];

    assign tail_next = tail +1 > 31 ? 0 : tail + 1;
    rob_full = (tail_next == 0);
    
    always_ff @(posedge clk) begin
        if(reset) begin
            values[4:0] <= 0;
            dest[4:0] <= 0;
            ready[4:0] <= 0;
            head[4:0] <= 0;
            tail[4:0] <= 0;
        end
        else begin
            if(dispatch_valid) begin
                values[tail] <= 0;
                dest[tail] <= dispatch_dest_reg;
                ready[tail] <= 1'b0;
                tail <= tail_next;
            end
            if (cdb_valid) begin
                values[tail] <= cdb_value;
                ready[tail] <= 1'b1;
            end
            if () begin
                
            end
        end
    end
