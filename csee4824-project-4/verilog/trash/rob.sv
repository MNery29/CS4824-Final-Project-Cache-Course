/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob.sv                                              //
//                                                                     //
//  Description :  Re-order buffer for P6 architecture, ensures        //
//                 in-order appearance is maintained                   //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module rob(
    //standard input signals for module.
    input reset,
    input clock,

    //input signals from dispatch stage 
    //opcode
    input [4:0] dispatch_dest_reg,
    input [6:0] dispatch_opcode,
    input dispatch_valid,

    //input signals from complete stage from CDB
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

    // output for map table: Tag for latest dispatch 
    output [4:0] map_table_tag,
    output map_table_tag_valid,
    //output signals to memory
    output [63:0] mem_addr,
    output mem_valid,
    //rob full signal, for stalling/hazards   
    output rob_full
);

    typedef enum logic[1:0] { EMPTY, BUSY, READY  } rob_state; //used to track status of an instruction.
    //internal signals
    //rob entries - 5
    rob_state rob_status[31:0]; //status of each instruction in ROB
    logic [63:0] rob_values[31:0];
    logic [6:0] rob_opcode[31:0];
    logic [4:0] rob_dest[31:0];
    logic rob_busy [31:0]; //if operation is still happening
    logic rob_ready[31:0]; //monitor if ROB values are ready to be committed

    //head and tail pointers for queue FIFO structure
    logic [5:0] head, tail;

    assign rob_full = ((head[4:0] == tail[4:0]) && (head[5] != tail[5]));
    
    function  logic memory_retire(input [6:0] opcode);
        return (opcode == `RV32_STORE); //Store opcodes, like SB, SH, SW, SD, write to mem. 
    endfunction

    function  logic regfile_retire(input [6:0] opcode);
        return (
        (opcode == `RV32_LOAD)    || //For ops: LB, LH, LW, LBU, LHU, LWU, LD
        (opcode == `RV32_OP_IMM)  || //For ops: ADDI, SLTI, XORI, ORI, ANDI, SLLI, SRLI, SRAI
        (opcode == `RV32_OP)      || //For ops: ADD, SUB, SLL, SLT, XOR, SRL, SRA, OR, AND
        (opcode == `RV32_JALR_OP) || // JALR
        (opcode == `RV32_JAL_OP)  || // JAL
        (opcode == `RV32_LUI_OP)  || // LUI
        (opcode == `RV32_AUIPC_OP)   // AUIPC
    );
    endfunction




    always_ff @(posedge clk) begin
        if(reset) begin
            for (int i = 0; i < 32; i++) begin
                rob_values[i] <= 64'b0;
                rob_dest[i] <= 0;
                rob_opcode[i] <= 0;
                rob_status[i] <= EMPTY;
            end
            head <= 6'b0;
            tail <= 6'b0;
            map_table_tag <= 5'b0;
            map_table_tag_valid <= 0;
        end
        else begin
            //set default values
            reg_valid <= 1'b0;
            mem_valid <= 1'b0;
            map_table_tag_valid <= 1'b0;

            //dispatch stage logic, to handle incoming dispatch instruction. 
            if(dispatch_valid && !rob_full) begin
                rob_values[tail[4:0]] <= 0; //temporary placeholder, await cdb update. 
                rob_dest[tail[4:0]] <= dispatch_dest_reg; // set destination register 
                rob_opcode[tail[4:0]] <= dispatch_opcode; // set opcode
                rob_status[tail[4:0]] <= BUSY; // set status to busy till CDB udpate. 
                map_table_tag <= tail[4:0]; // update map table output w/ new entry 
                map_table_tag_valid <= 1'b1; // set valid bit for map table output
                tail <= tail + 1; //increment tail pointer, circular buffer style
            end else begin 
                map_table_tag_valid <= 0; //reset map table signal if no dispatch. 
            end 

            //execute stage logic, to handle incoming CDB update.

            if (cdb_valid) begin
                rob_values[cdb_tag] <= cdb_value;
                rob_status[cdb_tag] <= READY; // set status to ready, as value is now available for retirement
            end

            // retire stage logic to commit ready instructions. Check if an instruction is ready to retire.  
            if (retire_valid && rob_status[head[4:0]] == READY) begin
                if (branch_mispredict) begin
                    //placeholder for misprediction. Update in detail later. 
                    for (int i = 0; i < 32; i++) begin
                        rob_values[i] <= 64'b0;
                        rob_dest[i] <= 0;
                        rob_opcode[i] <= 0;
                        rob_status[i] <= EMPTY;
                    end
                    head <= tail; //reset head to tail to discard instructions. 
                end
                else begin
                    //Check if to write to regfile
                    if (regfile_retire(rob_opcode[head[4:0]])) begin
                        reg_dest <= rob_dest[head[4:0]];
                        reg_value <= rob_values[head[4:0]];
                        reg_valid <= 1'b1;
                    end

                    //Check if to write to memory
                    if (memory_retire(rob_opcode[head[4:0]])) begin
                        mem_addr <= rob_values[head[4:0]];
                        mem_valid <= 1'b1;
                    end

                    //update head pointer
                    head <= head + 1;
                end  
            end
        end
    end
endmodule



// valid bit needed for ROB

// set the head and tail to one more, store in memory for 

//keep valid bit in memory. 

//also save valid bit as part of memory for ROB, when memory is implement.

//also store valid bits