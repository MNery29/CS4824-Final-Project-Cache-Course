/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_if.sv                                         //
//                                                                     //
//  Description :  instruction fetch (IF) stage of the pipeline;       //
//                 fetch instruction, compute next PC location, and    //
//                 send them down the pipeline.                        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module stage_if (
    input             clock,          // system clock
    input             reset,          // system reset
    input             if_valid,       // only go to next PC when true
    input             take_branch,    // taken-branch signal
    input [`XLEN-1:0] branch_target,  // target pc: use if take_branch is TRUE
    input [63:0]      Icache_data_out, // data coming back from cache
    input             Icache_valid_out,// high when valid

    output IF_ID_PACKET      if_packet,
    output logic [`XLEN-1:0] proc2Icache_addr, // address sent to icache
    output logic stall_if // stall signal from IF to pass down pipeline
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching
    logic [`XLEN-1:0] prev_PC_reg;

    assign stall_if = ~Icache_valid_out;
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;             // initial PC value is 0 (the memory address where our program starts)
        end else if (take_branch) begin
            PC_reg <= branch_target; // update to a taken branch (does not depend on valid bit)
        end else if (if_valid && ~stall_if) begin //only update if valid and not told to stall
            PC_reg <= PC_reg + 4;    // or transition to next PC if valid
        end
    end

    //i think icache alr ignores the lower 3 bits
    assign proc2Icache_addr = PC_reg;

    //assign stall to whether icache is valid

    // this mux is because the Icache gives us 64 bits not 32 bits
    assign if_packet.inst = (if_valid && Icache_valid_out)
                          ? PC_reg[2] ? Icache_data_out[63:32] : Icache_data_out[31:0]
                          : `NOP;

    assign if_packet.PC  = PC_reg;
    assign if_packet.NPC = PC_reg + 4; // pass PC+4 down pipeline w/instruction

    assign if_packet.valid = if_valid && Icache_valid_out;

endmodule // stage_if
