/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  pipeline.sv                                         //
//                                                                     //
//  Description :  Top-level module of the verisimple pipeline;        //
//                 This instantiates and connects the 6 stages of our  //
//                 P6 pipeline together, along with the needed modules //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module pipeline (
    input        clock,             // System clock
    input        reset,             // System reset
    input [3:0]  mem2proc_response, // Tag from memory about current request
    input [63:0] mem2proc_data,     // Data coming back from memory
    input [3:0]  mem2proc_tag,      // Tag from memory about current reply

    output logic [1:0]       proc2mem_command, // Command sent to memory
    output logic [`XLEN-1:0] proc2mem_addr,    // Address sent to memory
    output logic [63:0]      proc2mem_data,    // Data sent to memory
`ifndef CACHE_MODE
    output MEM_SIZE          proc2mem_size,    // Data size sent to memory
`endif

    output logic [3:0]       pipeline_completed_insts,
    output EXCEPTION_CODE    pipeline_error_status,
    output logic [4:0]       pipeline_commit_wr_idx,
    output logic [`XLEN-1:0] pipeline_commit_wr_data,
    output logic             pipeline_commit_wr_en,
    output logic [`XLEN-1:0] pipeline_commit_NPC
);

    //////////////////////////////////////////////////
    //                IF Stage Wires                //
    //////////////////////////////////////////////////
    logic [`XLEN-1:0] proc2Icache_addr;
    logic [63:0]      Icache_data_out;
    logic             Icache_valid_out;
    logic             if_valid;
    logic             take_branch;
    logic [`XLEN-1:0] branch_target;
    logic             stall_if;
    IF_ID_PACKET      if_packet;

    //////////////////////////////////////////////////
    //                ID Stage Wires                //
    //////////////////////////////////////////////////
    ID_IS_PACKET      id_is_packet;
    IF_ID_PACKET      if_id_reg;

    logic [45:0]      id_rob_debug[31:0];
    logic [11:0]      id_rob_pointers;
    logic [7:0]       id_mt_tags[31:0];
    logic [74:0]      id_rs_debug;

    //////////////////////////////////////////////////
    //                IS Stage Wires                //
    //////////////////////////////////////////////////
    logic [`RS_SIZE-1:0] rs_ready_out;
    logic [31:0]         rs_opa_out   [`RS_SIZE];
    logic [31:0]         rs_opb_out   [`RS_SIZE];
    logic [5:0]          rs_tag_out   [`RS_SIZE];
    ALU_FUNC             rs_alu_func_out[`RS_SIZE];
    logic [31:0]         rs_npc_out     [`RS_SIZE];
    logic [31:0]         rs_inst_out    [`RS_SIZE];

    logic fu_ready = 1'b1;
    logic [`RS_SIZE-1:0] rs_issue_enable;
    IS_EX_PACKET is_packet;
    logic issue_valid;


    //////////////////////////////////////////////////
    //               I-Cache Wires                  //
    //////////////////////////////////////////////////
    logic [1:0]       proc2Imem_command;
    logic [`XLEN-1:0] proc2Imem_addr;
    logic [3:0]       Imem2proc_response;
    logic [63:0]      Imem2proc_data;
    logic [3:0]       Imem2proc_tag;

    //////////////////////////////////////////////////
    //           Temporary Branch Logic             //
    //////////////////////////////////////////////////
    assign if_valid = 1'b1;                // Always fetch for now
    assign take_branch = 1'b0;             // No branch resolution yet
    assign branch_target = 32'b0;          // Default branch target

    //////////////////////////////////////////////////
    //         Fetch Stage                          //
    //////////////////////////////////////////////////
    stage_if stage_if_0 (
        .clock(clock),
        .reset(reset),
        .if_valid(if_valid),
        .take_branch(take_branch),
        .branch_target(branch_target),
        .Icache_data_out(Icache_data_out),
        .Icache_valid_out(Icache_valid_out),
        .if_packet(if_packet),
        .proc2Icache_addr(proc2Icache_addr),
        .stall_if(stall_if)
    );

    //////////////////////////////////////////////////
    //                  I-Cache                     //
    //////////////////////////////////////////////////
    icache icache_0 (
        .clock(clock),
        .reset(reset),
        .Imem2proc_response(mem2proc_response),
        .Imem2proc_data(mem2proc_data),
        .Imem2proc_tag(mem2proc_tag),
        .proc2Icache_addr(proc2Icache_addr),
        .proc2Imem_command(proc2Imem_command),
        .proc2Imem_addr(proc2Imem_addr),
        .Icache_data_out(Icache_data_out),
        .Icache_valid_out(Icache_valid_out)
    );

    //////////////////////////////////////////////////
    //               Decode Stage                   //
    //////////////////////////////////////////////////
    stage_id stage_id_0 (
        .clock(clock),
        .reset(reset),
        .if_id_reg(if_packet),
        .id_is_packet(id_is_packet),
        .rob_debug(id_rob_debug),
        .rob_pointers_debug(id_rob_pointers),
        .mt_tags_debug(id_mt_tags),
        .rs_debug(id_rs_debug)
    );

    //////////////////////////////////////////////////
    //                Issue Stage                   //
    //////////////////////////////////////////////////
    stage_is stage_is_0 (
        .clock(clock),
        .reset(reset),
        .rs_ready_out(id_is_packet.rs_ready),
        .rs_opa_out(id_is_packet.opa),
        .rs_opb_out(id_is_packet.opb),
        .rs_tag_out(id_is_packet.dest_tag),
        .rs_alu_func_out(id_is_packet.alu_func),
        .rs_npc_out(id_is_packet.NPC),
        .rs_inst_out(id_is_packet.inst),
        .fu_ready(fu_ready),
        .issue_valid(issue_valid),
        .is_packet(is_packet),
        .rs_issue_enable(rs_issue_enable)
    );


    //////////////////////////////////////////////////
    //              Memory Access Logic             //
    //////////////////////////////////////////////////
    logic [`XLEN-1:0] proc2Dmem_addr;
    logic [`XLEN-1:0] proc2Dmem_data;
    logic [1:0]       proc2Dmem_command;
`ifndef CACHE_MODE
    MEM_SIZE          proc2Dmem_size;
`endif

    always_comb begin
        if (proc2Dmem_command != BUS_NONE) begin
            proc2mem_command = proc2Dmem_command;
            proc2mem_addr    = proc2Dmem_addr;
`ifndef CACHE_MODE
            proc2mem_size    = proc2Dmem_size;
`endif
        end else begin
            proc2mem_command = BUS_LOAD;
            proc2mem_addr    = proc2Imem_addr;
`ifndef CACHE_MODE
            proc2mem_size    = DOUBLE;
`endif
        end
        proc2mem_data = {32'b0, proc2Dmem_data};
    end

    //////////////////////////////////////////////////
    //               Pipeline Outputs               //
    //////////////////////////////////////////////////
    assign pipeline_completed_insts = 4'd0; // placeholder
    assign pipeline_error_status    = NO_ERROR;
    assign pipeline_commit_wr_en    = 1'b0;
    assign pipeline_commit_wr_idx   = 5'd0;
    assign pipeline_commit_wr_data  = 64'd0;
    assign pipeline_commit_NPC      = 64'd0;

endmodule // pipeline