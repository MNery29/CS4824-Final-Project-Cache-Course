/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  rob.sv                                              //
//                                                                     //
//  Description :  Re-order buffer for P6 architecture, ensures        //
//                 in-order appearance is maintained                   //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

//Temp ROB for integration with dispatch stage

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module reorder_buffer(
    //standard input signals for module.
    input reset,
    input clock,

    //input signals from dispatch stage 
    //opcode
    input [4:0] dispatch_dest_reg,
    input [6:0] dispatch_opcode,
    input load_entry,
    //If RS needs to read value ready in ROB
    input rob_to_rs_read1,
    input [4:0]rob_read_tag1,
    input rob_to_rs_read2,
    input [4:0]rob_read_tag2,

    //input signals from execute stage 
    input [4:0] cdb_tag, //cdb - common data bus
    input [31:0] cdb_value,
    input cdb_valid,

    //input signals from retire stage
    input retire_entry,
    input rob_clear,

    //output signals to Regfile
    output logic [4:0] reg_dest,
    output logic [31:0] reg_value,
    output logic reg_valid,

    //output for reservation station/map table: Tag for latest dispatch 
    output [4:0] rob_tag_out,
    output [31:0] rob_to_rs_value1,
    output [31:0] rob_to_rs_value2,
    output logic rob_out_valid,
    
    //output signals to memory
    output logic [31:0] mem_addr,
    output logic mem_valid,
    //rob full signal, for stalling/hazards   
    output rob_full,
    //debug
    output logic [45:0] rob_debug [31:0],
    output logic [11:0] rob_pointers
);

    typedef enum logic[1:0] {EMPTY, BUSY, READY} rob_state; //used to track status of an instruction.
    //internal signals
    //rob entries - 5
    rob_state rob_status[31:0]; //status of each instruction in ROB
    logic [31:0] rob_values[31:0];
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

    //Outputs to RS
    assign rob_tag_out = tail[4:0] + 1'b1;

    assign rob_to_rs_value1 = rob_to_rs_read1 ? rob_values[rob_read_tag1] : 32'b0;
    assign rob_to_rs_value2 = rob_to_rs_read2 ? rob_values[rob_read_tag2] : 32'b0;

    //Debug outputs
    assign rob_pointers = {head, tail};
    always_comb begin
        for (int i = 0; i < 32; i++) begin
            rob_debug[i] = {rob_status[i], rob_opcode[i], rob_dest[i], rob_values[i]};
        end
    end

    always_ff @(posedge clock) begin
        if(reset) begin
            for (int i = 0; i < 32; i++) begin
                rob_values[i] <= 32'b0;
                rob_dest[i] <= 0;
                rob_opcode[i] <= 0;
                rob_status[i] <= EMPTY;
            end
            head <= 6'b0;
            tail <= 6'b0;
            reg_dest <= 5'b0;
            reg_value <= 32'b0;
            reg_valid <= 1'b0;
            mem_valid <= 1'b0;
            mem_addr <= 32'b0;
            rob_out_valid <= 0;

        end
        else begin
            //set default values
            rob_out_valid <= 0;

            //dispatch stage logic, to handle incoming dispatch instruction. 
            if (load_entry && !rob_full) begin
                rob_values[tail[4:0]] <= 32'b0; //temporary placeholder, await cdb update. 
                rob_dest[tail[4:0]] <= dispatch_dest_reg; // set destination register 
                rob_opcode[tail[4:0]] <= dispatch_opcode; // set opcode
                rob_status[tail[4:0]] <= BUSY; // set status to busy till CDB udpate. 
                rob_out_valid <= 1'b1; // set valid bit for output to RS/MT
                tail <= tail + 1; //increment tail pointer, circular buffer style
            end else begin 
                rob_out_valid <= 1'b0; //reset tag/value output valid if no dispatch. 
            end 

            //execute stage logic, to handle incoming CDB update.
            if (cdb_valid) begin
                rob_values[cdb_tag] <= cdb_value;
                rob_status[cdb_tag] <= READY; // set status to ready, as value is now available for retirement
            end

            // retire stage logic to commit ready instructions. Check if an instruction is ready to retire.  
            if (retire_entry && rob_status[head[4:0]] == READY) begin
                if (rob_clear) begin
                    //mispred or interrupt
                    for (int i = 0; i < 32; i++) begin
                        rob_values[i] <= 64'b0;
                        rob_dest[i] <= 0;
                        rob_opcode[i] <= 0;
                        rob_status[i] <= EMPTY;

                        reg_dest <= 5'b0;
                        reg_value <= 32'b0;
                        reg_valid <= 1'b0;
                        mem_valid <= 1'b0;
                        mem_addr <= 32'b0;
                    end
                    head <= tail; //reset head to tail to discard instructions. 
                end else begin
                    //update head pointer
                    reg_dest <= regfile_retire(rob_opcode[head[4:0]]) ? rob_dest[head[4:0]] : `ZERO_REG;
                    reg_value <= regfile_retire(rob_opcode[head[4:0]]) ? rob_values[head[4:0]] : 32'b0;
                    reg_valid <= regfile_retire(rob_opcode[head[4:0]]) ? 1'b1 : 1'b0;

                    mem_valid <= memory_retire(rob_opcode[head[4:0]]) ? 1'b1 : 1'b0;
                    mem_addr <= memory_retire(rob_opcode[head[4:0]]) ? rob_values[head[4:0]] : 32'b0;
                    head <= head + 1;
                end
            end else begin
                reg_dest <= 5'b0;
                reg_value <= 32'b0;
                reg_valid <= 1'b0;
                mem_valid <= 1'b0;
                mem_addr <= 32'b0;
            end
        end
    end
endmodule



// valid bit needed for ROB

// set the head and tail to one more, store in memory for 

//keep valid bit in memory. 

//also save valid bit as part of memory for ROB, when memory is implement.

//also store valid bits