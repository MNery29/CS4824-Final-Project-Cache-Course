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
    input [63:0]      Imem2proc_data, // data coming back from Instruction memory

    output IF_ID_PACKET      if_packet,
    output logic [`XLEN-1:0] proc2Imem_addr // address sent to Instruction memory
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;             // initial PC value is 0 (the memory address where our program starts)
        end else if (take_branch) begin
            PC_reg <= branch_target; // update to a taken branch (does not depend on valid bit)
        end else if (if_valid) begin
            PC_reg <= PC_reg + 4;    // or transition to next PC if valid
        end
    end

    // address of the instruction we're fetching (64 bit memory lines)
    // mem always gives us 8=2^3 bytes, so ignore the last 3 bits
    assign proc2Imem_addr = {PC_reg[`XLEN-1:3], 3'b0};

    // this mux is because the Imem gives us 64 bits not 32 bits
    assign if_packet.inst = (~if_valid) ? `NOP :
                            PC_reg[2] ? Imem2proc_data[63:32] : Imem2proc_data[31:0];

    assign if_packet.PC  = PC_reg;
    assign if_packet.NPC = PC_reg + 4; // pass PC+4 down pipeline w/instruction

    assign if_packet.valid = if_valid;

endmodule // stage_if
