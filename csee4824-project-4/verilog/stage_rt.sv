/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_rt.sv                                         //
//                                                                     //
//  Description :  instruction retire (RT) stage of the pipeline;      //
//                 determine instruction register destination and      //
//                 write result to regfile, prep for next PC           //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module stage_rt (

    input logic clock, 
    input logic reset,

    //inputs from ROB
    input ROB_RETIRE_PACKET rob_retire_packet, // packet from ROB

    input logic rob_ready, // ready bit from ROB
    input logic rob_valid, // valid bit from ROB

    // Commit control 
    input logic branch_mispredict, // branch mispredict bit from ROB
    input logic lsq_op_in_progress,

    //outputs
    output logic [31:0] retire_value, // data to write to register file
    output logic [4:0] retire_dest, // destination register to write to
    output logic retire_valid_out, // valid bit to register file
    output logic [4:0] retire_tag, // retire tag to maptable

    output logic halt,
    output logic illegal,
    output logic csr_op,

    output logic [31:0] npc,

    //memory outputs
    // output logic [63:0] mem_addr, // memory address to write to
    output logic [4:0] mem_tag,
    output logic mem_valid, // memory valid bit

    output logic clear_is,
    output logic stall_if,
    output logic clear_lsq,
    output logic [31:0] new_addr, // new address to write to
    output logic take_branch // take branch bit

);

//internal signals
logic [31:0] retire_value_reg;
logic [4:0] retire_dest_reg;
logic retire_valid_reg;
logic [4:0] retire_tag_reg;
logic [4:0] mem_tag_reg;
logic mem_valid_reg;
logic clear_is_reg;
logic stall_if_reg;
logic clear_lsq_reg;
logic take_branch_reg;

logic halt_reg;
logic illegal_reg;
logic csr_op_reg;

logic [31:0] npc_reg;

assign retire_value = take_branch ? npc_reg : retire_value_reg;
assign retire_dest = retire_dest_reg;
assign retire_valid_out = halt_reg ? (!lsq_op_in_progress && retire_valid_reg) : retire_valid_reg;
assign retire_tag = retire_tag_reg;
assign mem_tag = mem_tag_reg;
assign mem_valid = mem_valid_reg;
assign clear_is = clear_is_reg;
assign stall_if = stall_if_reg;
assign clear_lsq = clear_lsq_reg;
assign take_branch = take_branch_reg;
assign new_addr = take_branch ? retire_value_reg : 0; // if we take the branch, we need to set the new address to the value of the instruction

assign halt = !lsq_op_in_progress && halt_reg;
assign illegal = illegal_reg;
assign csr_op = csr_op_reg;

assign npc = npc_reg;

assign duplicate = rob_retire_packet.value == retire_value_reg 
        && rob_retire_packet.dest_reg == retire_dest_reg 
        && rob_retire_packet.tag[4:0] == retire_tag_reg 
        && rob_retire_packet.mem_valid == mem_valid_reg 
        && rob_retire_packet.halt == halt_reg 
        && rob_retire_packet.illegal == illegal_reg 
        && rob_retire_packet.csr_op == csr_op_reg;



always_ff @(posedge clock) begin
    // ok so we check to see if it is a branch, and if it is a branch, we check if we take the branch (we always assume no taking branches)
    if (reset) begin
        //cleare all retire outputs
        retire_value_reg <= 0;
        retire_dest_reg <= 0;
        retire_valid_reg<= 1'b0;
        retire_tag_reg <= 0;

        clear_is_reg <= 0;
        stall_if_reg <= 0;
        clear_lsq_reg <= 0;
        take_branch_reg <= 0;

        halt_reg <= 0;
        illegal_reg <= 0;
        csr_op_reg <= 0;

        //clear all mem outputs
        // mem_addr <= 64'b0;
        mem_tag_reg <= 0;
        mem_valid_reg <= 0;
        npc_reg <= 0;
    end else begin
        // if is a branch, and we predicted correct (not taken), then we can just ignore it
        if (rob_ready && rob_valid && !duplicate && !lsq_op_in_progress) begin
            // retiring an instruction: valid entry from ROB
            
            retire_value_reg <= rob_retire_packet.value;
            retire_dest_reg  <= rob_retire_packet.dest_reg;
            retire_valid_reg <= !rob_retire_packet.mem_valid || rob_retire_packet.take_branch;
            retire_tag_reg <= rob_retire_packet.tag[4:0];
            // mem_addr     <= rob_retire_packet.mem_addr;
            mem_tag_reg      <= rob_retire_packet.tag[4:0];
            mem_valid_reg    <= rob_retire_packet.mem_valid;

            halt_reg <= rob_retire_packet.halt;
            illegal_reg <= rob_retire_packet.illegal;
            csr_op_reg <= rob_retire_packet.csr_op;

            npc_reg <= rob_retire_packet.npc;
            if (rob_retire_packet.is_branch) begin
                // if it is a branch, we need to check if we take the branch
                if (rob_retire_packet.take_branch) begin
                    // if we take the branch, we need to clear the ROB and map table
                    clear_is_reg <= 1;
                    stall_if_reg <= 1;
                    clear_lsq_reg <= 1;
                    take_branch_reg <= 1;
                end else begin
                    clear_is_reg <= 0;
                    stall_if_reg <= 0;
                    clear_lsq_reg <= 0;
                    // if we do not take the branch, we just set the new address to zero
                    take_branch_reg <= 0;
                end
            end else begin
                clear_is_reg <= 0;
                stall_if_reg <= 0;
                clear_lsq_reg <= 0;
                // not a branch, so just set to default
                take_branch_reg <= 0;
            end
            
        end else begin
            //nothing to retire - set to default
            retire_value_reg <= 0;
            retire_dest_reg <= 0;
            retire_valid_reg <= 0;
            retire_tag_reg <= 0;
            clear_is_reg <= 0;
            stall_if_reg <= 0;
            clear_lsq_reg <= 0;
            halt_reg <= 0;
            take_branch_reg <= 0;
            illegal_reg <= 0;
            csr_op_reg <= 0;

            npc_reg <= 0;

        end
    end
end
endmodule