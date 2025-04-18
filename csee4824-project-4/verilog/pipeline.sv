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
    IF_ID_PACKET      if_id_reg;
    
    //////////////////////////////////////////////////
    //                ID Stage Wires                //
    //////////////////////////////////////////////////
    ID_IS_PACKET      id_is_packet;
    ID_IS_PACKET      id_is_reg;
    logic [45:0]      id_rob_debug[31:0];
    logic [11:0]      id_rob_pointers;
    logic [7:0]       id_mt_tags[31:0];
    logic [74:0]      id_rs_debug;
    logic [`RS_SIZE-1:0] rs_issue_enable;

    //////////////////////////////////////////////////
    //                IS Stage Wires                //
    //////////////////////////////////////////////////
    IS_EX_PACKET      is_packet;
    IS_EX_PACKET      is_ex_reg;
    logic             issue_valid;
    logic fu_ready = 1'b1; // For now, always ready

    //////////////////////////////////////////////////
    //                 EX Stage Wires               //
    //////////////////////////////////////////////////
    ID_EX_PACKET id_ex_reg;   // The ID to EX stage register
    EX_MEM_PACKET ex_packet;  // Output Packet

    //////////////////////////////////////////////////
    //                CP Stage Wires                //
    //////////////////////////////////////////////////
    EX_CP_PACKET ex_cp_reg;
    CDB_PACKET cdb_packet;

    //////////////////////////////////////////////////
    //               RT Stage Wires                 //
    //////////////////////////////////////////////////
    logic [`XLEN-1:0] retire_value_out;
    logic [4:0]       retire_dest_out;
    logic             retire_valid_out;
    logic [`XLEN-1:0] mem_addr_out;
    logic             mem_valid_out;

    //////////////////////////////////////////////////
    //                ROB + Map Table Wires         //
    //////////////////////////////////////////////////
    DISPATCH_ROB_PACKET rob_dispatch_packet;
    ROB_DISPATCH_PACKET rob_dispatch_out;
    ROB_RETIRE_PACKET rob_retire_packet;
    logic rob_full;

    //////////////////////////////////////////////////
    //               I-Cache Wires                  //
    //////////////////////////////////////////////////
    logic [1:0]       proc2Imem_command;
    logic [`XLEN-1:0] proc2Imem_addr;
    logic [3:0]       Imem2proc_response;
    logic [63:0]      Imem2proc_data;
    logic [3:0]       Imem2proc_tag;

    //////////////////////////////////////////////////
    //               D-Cache Wires                  //
    //////////////////////////////////////////////////
    logic [63:0]dcache_data_out, // data coming back from cache
    logic [3:0] dcache_tag, // high when valid
    logic [3:0] dcache_response, // 0 = can't accept, other=tag of transaction]
    logic [1:0] dcache_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE
    logic [`XLEN-1:0] dcache_addr, // sending address to dcache

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
    //         IF/ID Pipeline Register              //
    //////////////////////////////////////////////////
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            if_id_reg <= '0;
        end else begin
            if_id_reg <= if_packet;
        end
    end


    //////////////////////////////////////////////////
    //               Decode Stage                   //
    //////////////////////////////////////////////////
    stage_id stage_id_0 (
        .clock(clock),
        .reset(reset),
        .if_id_reg(if_id_reg),

        .cdb_valid(cdb_packet.valid), // NEW
        .cdb_tag(cdb_packet.tag),     // NEW
        .cdb_value(cdb_packet.value), // (optional, if needed)

        .id_is_packet(id_is_packet), // this packet goes TO issue stage

        .rs1_issue(rs_issue_enable[0]),  // pass the rs_issue_enable signal
        .rs1_clear(rs_issue_enable[0]),  // for now, clearing on issue 

        .rob_debug(id_rob_debug),
        .rob_pointers_debug(id_rob_pointers),
        .mt_tags_debug(id_mt_tags),
        .rs_debug(id_rs_debug)
    );

    //////////////////////////////////////////////////
    //         ID/IS Pipeline Register              //
    //////////////////////////////////////////////////
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            id_is_reg <= '0;
        end else begin
            id_is_reg <= id_is_packet;
        end
    end



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
    //         IS/EX Pipeline Register              //
    //////////////////////////////////////////////////
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            is_ex_reg <= '0;
        end else begin
            is_ex_reg <= is_packet;
        end
    end

    //////////////////////////////////////////////////
    //                Execute Stage                 //
    //////////////////////////////////////////////////
    stage_ex stage_ex_0 (
        .id_ex_reg(is_ex_reg),
        .ex_packet(ex_packet)
    );

    //////////////////////////////////////////////////
    //           EX/CP Pipeline Register            //
    //////////////////////////////////////////////////
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            ex_cp_reg <= '0;
        end else begin
            ex_cp_reg <= ex_packet;
        end
    end

    //////////////////////////////////////////////////
    //               Complete Stage                 //
    //////////////////////////////////////////////////
    stage_cp stage_cp_0 (
        .clock(clock),
        .reset(reset),
        .ex_cp_packet(ex_cp_reg), // input packet from EX stage
        .cdb_packet_out(cdb_packet)
    );

    //////////////////////////////////////////////////
    //            Reorder Buffer (ROB)              //
    //////////////////////////////////////////////////
    reorder_buffer reorder_buffer_0 (
        .reset(reset),
        .clock(clock),
        .rob_dispatch_in(rob_dispatch_packet),
        .rob_dispatch_out(rob_dispatch_out),
        .rob_cdb_in(cdb_packet),
        .retire_entry(1'b0),
        .rob_clear(1'b0),
        .rob_retire_out(rob_retire_packet),
        .rob_to_rs_value1(),
        .rob_to_rs_value2(),
        .rob_full(rob_full),
        .rob_debug(id_rob_debug),
        .rob_pointers(id_rob_pointers)
    );

    //////////////////////////////////////////////////
    //                Map Table                     //
    //////////////////////////////////////////////////
    map_table map_table_0 (
        .reset(reset),
        .clock(clock),

        // Source register addresses (for reading tags)
        .rs1_addr(if_id_reg.inst.r.rs1),
        .rs2_addr(if_id_reg.inst.r.rs2),

        // Destination register address (where the result will eventually be written)
        .r_dest(if_id_reg.inst.r.rd),

        // ROB tag assigned to destination register
        .tag_in(rob_dispatch_out.tag),

        // Dispatch control: whether we are dispatching a new instruction
        .load_entry(dispatch_ok && if_id_reg.valid && has_dest_reg),

        // CDB broadcast: update map table when a result is ready
        .cdb_tag_in(cdb_packet.tag),
        .read_cdb(cdb_packet.valid),

        // Retirement: clear mappings when instructions retire
        .retire_addr(rob_retire_packet.dest_reg),
        .retire_tag(rob_retire_packet.tag),
        .retire_entry(rob_retire_packet.valid),

        // Outputs to the Reservation Station / Decode
        .rs1_tag(), // (connect later if needed)
        .rs2_tag(), // (connect later if needed)

        // Pass through register addresses for regfile reads
        .regfile_rs1_addr(), // (connect if needed)
        .regfile_rs2_addr(),

        // Debug
        .tags_debug(mt_tags_debug)
    );

    //////////////////////////////////////////////////
    //            CP/RT Pipeline Register           //
    //////////////////////////////////////////////////
    always_ff @(posedge clock or posedge reset) begin
        if (reset)
            cp_rt_reg <= '0;
        else
            cp_rt_reg <= rob_retire_packet;
    end


    //////////////////////////////////////////////////
    //               RT (Retire) Stage              //
    //////////////////////////////////////////////////
    stage_rt stage_rt_0 (
        .clock(clock),
        .reset(reset),
        .rob_retire_packet(cp_rt_reg),
        .branch_mispredict(1'b0),
        .retire_value(retire_value_out),
        .retire_dest(retire_dest_out),
        .retire_valid_out(retire_valid_out),
        .mem_addr(mem_addr_out),
        .mem_valid(mem_valid_out)
    );


    `DEFINE OWN_NONE = 2'b00;
    `DEFINE OWN_D    = 2'b01;
    `DEFINE OWN_I    = 2'b10;


    //////////////////////////////////////////////////
    //              Memory Access Logic             //
    //////////////////////////////////////////////////
    logic [1:0] owner_q, owner_d; // this will keep track of who sent the memory request at the last time step
    logic [`XLEN-1:0] proc2Dmem_addr;
    logic [`XLEN-1:0] proc2Dmem_data;
    logic [1:0]       proc2Dmem_command;
`ifndef CACHE_MODE
    MEM_SIZE          proc2Dmem_size;
`endif

    always_comb begin
        owner_d = owner_q;
        if (dcache_command != BUS_NONE) begin
            proc2mem_command = dcache_command;
            proc2mem_addr    = dcache_addr;
`ifndef CACHE_MODE
            proc2mem_size    = proc2Dmem_size;
`endif
            //if data mmodule sent the request
            owner_d = `OWN_D;
        end else begin
            proc2mem_command = BUS_LOAD;
            proc2mem_addr    = proc2Imem_addr;
`ifndef CACHE_MODE
            proc2mem_size    = DOUBLE;
`endif
            // then if instruction module sent the request
            owner_d = `OWN_I;
        end
        proc2mem_data = {32'b0, proc2Dmem_data};
    end


    module mem (
        .clk(clk),
        .proc2mem_addr(proc2mem_addr),
        .proc2mem_data(proc2mem_data),
    `ifndef CACHE_MODE
        .proc2mem_size(proc2mem_size),
    `endif
        .proc2mem_command(proc2mem_command),
        .mem2proc_response(mem2proc_response),
        .mem2proc_data(mem2proc_data),
        .mem2proc_tag(mem2proc_tag)
    );

    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            owner_q <= `OWN_NONE;
        // for now, we have to wait for data to respond, so not just tag
        else if (mem2proc_data != 0)   // memory sent a reply -> done
            owner_q <= `OWN_NONE;
        else
            owner_q <= owner_d;
    end
    
    always_comb begin
        // Default: de‑assert
        dcache_response     = 0;
        dcache_data_out     = 0;
        dcache_tag          = 0;

        Imem2proc_response  = 0;
        Imem2proc_data      = 0;
        Imem2proc_tag       = 0;

        case (owner_q)
            `OWN_D: begin
                dcache_response = mem2proc_response;
                dcache_data_out = mem2proc_data;
                dcache_tag      = mem2proc_tag;
            end
            `OWN_I: begin
                Imem2proc_response = mem2proc_response;
                Imem2proc_data     = mem2proc_data;
                Imem2proc_tag      = mem2proc_tag;
            end
            default: begin
            end
        endcase
    end

    //////////////////////////////////////////////////
    //               Pipeline Outputs               //
    //////////////////////////////////////////////////
    assign pipeline_commit_wr_en    = retire_valid_out;
    assign pipeline_commit_wr_idx   = retire_dest_out;
    assign pipeline_commit_wr_data  = retire_value_out;
    assign pipeline_commit_NPC      = cp_rt_reg.mem_addr; 
    assign pipeline_completed_insts = retire_valid_out ? 4'd1 : 4'd0;
    assign pipeline_error_status    = NO_ERROR;

endmodule // pipeline