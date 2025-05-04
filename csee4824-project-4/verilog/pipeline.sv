/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  pipeline.sv                                         //
//                                                                     //
//  Description :  Top-level module of the verisimple pipeline;        //
//                 This instantiates and connects the 6 stages of our  //
//                 P6 pipeline together, along with the needed modules //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

// needed when running for synthesis
// `include "verilog/stage_cp.sv"
// `include "verilog/stage_ex.sv"
// `include "verilog/stage_if.sv"
// `include "verilog/stage_id.sv"
// `include "verilog/stage_is.sv"
// `include "verilog/stage_mem.sv"
// `include "verilog/stage_rt.sv"
// `include "verilog/lsq.sv"
// `include "verilog/icache.sv"
// `include "verilog/dcache.sv"
// `include "verilog/map_table.sv"
// `include "test/mem.sv"

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
    output logic [`XLEN-1:0] pipeline_commit_NPC,
    output logic [45:0]      id_rob_debug[`ROB_SZ-1:0],
    output logic stall_if,
    output logic [`XLEN-1:0] proc2Icache_addr,
    output logic             Icache_valid_out,
    output logic [63:0] Icache_data_out,

    output logic [3:0]  mem2dcache_response, // 0 = can't accept, other=tag of transaction
    output logic [63:0] mem2dcache_data,    // data resulting from a load
    output logic [3:0]  mem2dcache_tag,
    
    
    //ID Input debug wires
    output IF_ID_PACKET if_id_reg,
    output logic if_stall,

    output logic cdb_valid,
    output logic [`ROB_TAG_BITS-1:0] cdb_tag,
    output logic [31:0] cdb_value,


    output logic [`RS_SIZE-1:0] rs_clear_vec,
    output logic rob_retire_entry,
    output logic rs_avail_out [`RS_SIZE],



    output logic  store_retire,


    output logic [4:0] rob_dest_reg,
    output logic [31:0] rob_to_regfile_value,
    output logic retire_entry,





    output logic lsq_free,


    output logic maptable_clear,
    output logic rob_clear,
    output logic rs_clear,



    //id stage debugging wires
    output [4:0] rs_tag_out [`RS_SIZE],
    // output logic rs1_ready,
    output logic [`RS_SIZE-1:0] rs_ready_out,

    output logic [5:0] mt_to_rs_tag1, mt_to_rs_tag2,
    output logic [4:0] mt_to_regfile_rs1, mt_to_regfile_rs2,

    output logic [31:0] rs1_value, rs2_value,
    output logic [31:0] rob_to_rs_value1, rob_to_rs_value2,

    output logic [31:0] rs1_opa_in, rs1_opb_in,

    output ALU_OPA_SELECT id_opa_select, 
    output ALU_OPB_SELECT id_opb_select,

    //if stage
    output IF_ID_PACKET if_packet,
    output INST rs_inst_out[`RS_SIZE],


   

    //IS stage debugging wires
    output IS_EX_PACKET is_packets [2:0],
    output IS_EX_PACKET is_ex_reg,
    output logic issue_valid,
    output logic [`RS_SIZE-1:0] rs_issue_enable,
   

    //EX stage debugging wires
    output EX_CP_PACKET ex_cp_reg,
    output EX_CP_PACKET ex_packet,
    output logic [2:0] fu_busy_signals,
    output logic cdb_busy, //this will stall the RS issue if ex stage is busy / full
    output logic rob_full,
    output logic rs1_available,
    output logic dispatch_ok,
    output logic [73:0] rs_debug [`RS_SIZE],
    output CDB_PACKET cdb_packet,
    // output logic take_conditional,
    output logic [`XLEN-1:0] opa_mux_out,
    output logic [`XLEN-1:0] opb_mux_out,

    //retire stage debugging wires
    output logic [`XLEN-1:0] retire_value_out,
    output logic [4:0]       retire_dest_out,
    output logic             retire_valid_out,
    output logic [4:0] retire_tag,
    output ROB_RETIRE_PACKET rob_retire_packet,
    output logic rob_ready, rob_valid,

    output logic [31:1] [`XLEN-1:0] debug_reg,

    //lsq debug wires
    output LSQ_PACKET lsq_packet,
    output lsq_entry_t lsq_out [7:0], // debugging
    output logic store_ready,
    output logic [4:0] store_tag, // tag of store ready to write

    output priv_addr_packet priv_addr_packet, 
    output logic cache_in_flight, //debugging
    output logic head_ready_for_mem, // debugging
    output logic [2:0] head_ptr, //points to OLDEST entry debugging
    output logic [2:0] tail_ptr, //points to next free entry debugging
    output logic [63:0]dcache_data_out, // data coming back from cache
    output logic [3:0] dcache_tag, // high when valid
    output logic [3:0] dcache_response, // 0 = can't accept, other=tag of transaction]
    output logic dcache_hit, // 1 if hit, 0 if miss
    output logic [1:0] dcache_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE
    output logic [63:0] dcache_data, // data going to cache for store
    output logic [`XLEN-1:0] dcache_addr, // sending address to dcache
    output logic [1:0] dcache_size, // size of data to send to cache

    output logic [4:0] mem_tag, // from rt stage
    output logic mem_valid, // from rt stage
    output EX_CP_PACKET cdb_lsq, // broadcast load data
    output logic halt_rt,
    output logic illegal_rt,
    output logic csr_op_rt,

    //dcache debugging:
    output logic [`XLEN-1:0] tag_to_addr [15:0],
    output logic tag_to_addr_valid [15:0],
    output logic [1:0] tag_to_memsize [15:0],
    output logic [63:0] tag_to_memdata [15:0], // this is for stores exclusively
    output logic tag_to_is_store [15:0],

    output logic halt_confirm,

    output logic  next_state,
    output logic  state,

    output logic [3:0] Imem2proc_response,

    output logic [63:0] cache_data [0:63], // 64 lines of 8 bytes
    output logic [22:0] cache_tag [0:63], // 64 lines of tag bits
    output logic cache_valid [0:63], // 64 lines of valid bits
    output logic cache_dirty [0:63], // 64 lines of dirty bits

    output logic [3:0] current_mem_tag,

    output logic lsq_op_in_progress,

    output logic [31:0] dcache_cur_addr,
    output logic [1:0] dcache_cur_command,
    output logic [63:0] dcache_cur_data,
    output logic [4:0] cache_tag_in_flight [15:0], //indexed by dcache_tag (3 bits)
    output logic cache_in_flight_valid [15:0], //indexed by dcache_tag (3 bits)
    output logic cache_offset_in_flight [15:0], //indexed by dcache_tag (3 bits) gets us whether it is top half of cache line or bottom half    
    output logic cache_in_flight_rd_unsigned [15:0], //indexed by dcache_tag (3 bits) gets us whether it is unsigned or signed
    output MEM_SIZE cache_in_flight_mem_size [15:0], //indexed by dcache_tag (3 bits) gets us whether it is 8, 16, or 32 bit


    output logic [1:0] owner_q, owner_d, // this will keep track of who sent the memory request at the last time step
    output logic [`XLEN-1:0] proc2Dmem_addr,
    output logic [`XLEN-1:0] proc2Dmem_data,
    output logic [1:0]       proc2Dmem_command,
    output logic wait_one_step,
    output logic next_wait_one_step,
    output logic read_data,

    output logic icache_rejected,
    output logic next_icache_rejected,
    output logic prev_icache_rejected,
    output logic pos_icache_rejected,

    output logic [`XLEN-1:0] dcache2mem_addr,
    output logic [63:0]      dcache2mem_data, // address for current command
    output logic [1:0]       dcache2mem_command, // `BUS_NONE `BUS_LOAD or `BUS_STORE
    output MEM_SIZE    dcache2mem_size,

    output logic [1:0] cycle_wait,
    output logic [1:0] next_cycle_wait, 

    output logic wb_eviction, // this will be high if we need to evict a DIRTY line from the cache
    output logic next_wb_eviction, // this will be high if we need to evict a DIRTY line from the cache

    output logic rs_entry_found,
    output logic [1:0] fu_select, // Selects which RS entry to load into

    output logic [1:0] next_mult_indx,
    output logic [1:0] next_alu1_indx,
    output logic [1:0] next_alu0_indx,
    output logic next_issued_alu0,
    output logic next_issued_alu1,
    output logic next_issued_mult,

    output logic hold_mult_valid,
    output logic hold_alu0_valid,
    output logic hold_alu1_valid,

    output EX_CP_PACKET hold_alu0_pkt,
    output EX_CP_PACKET hold_alu1_pkt,
    output EX_CP_PACKET hold_mult_pkt,
    output EX_CP_PACKET tmp_alu0_pkt,
    output EX_CP_PACKET tmp_alu1_pkt,
    output EX_CP_PACKET tmp_mult_pkt
);

    //////////////////////////////////////////////////
    //                IF Stage Wires                //
    //////////////////////////////////////////////////
    // logic [`XLEN-1:0] proc2Icache_addr;
    // logic             Icache_valid_out;
    // logic [63:0] Icache_data_out;
    logic             if_valid;
    // logic             stall_if;
    // IF_ID_PACKET      if_packet;
    
    //////////////////////////////////////////////////
    //                ID Stage Wires                //
    //////////////////////////////////////////////////
    //ID_IS_PACKET      id_is_packet; // NOT INCLUDED IN stage_id.sv
    //ID_IS_PACKET      id_is_reg; // NOT INCLUDED IN stage_id.sv FIX
    // logic [45:0]      id_rob_debug[31:0];
    logic [11:0]      id_rob_pointers;
    //logic [7:0]       id_mt_tags[31:0];
    //logic [74:0]      id_rs_debug;
    //logic [`RS_SIZE-1:0] rs_issue_enable;

    // logic [31:0] id_opA, id_opB;
    logic [31:0] rs_opa_out [`RS_SIZE];
    logic [31:0] rs_opb_out [`RS_SIZE];
    // logic [`ROB_TAG_BITS-1:0] id_tag;
    logic [31:0] rs_npc_out [`RS_SIZE];
    logic [31:0] rs_pc_out [`RS_SIZE];
    logic [31:0] npc_rt;
    // INST id_inst_out;
    // ALU_OPA_SELECT id_opa_select;
    // logic [`RS_SIZE-1:0][31:0] rs1_inst_out;
    // logic rs1_ready;
    // ALU_OPB_SELECT id_opb_select;
    logic id_has_dest_reg;
    logic [4:0] id_dest_reg_idx;
    // logic id_rd_mem, id_wr_mem, id_cond_branch, id_uncond_branch;
    logic rs_rd_mem_out [`RS_SIZE];
    logic rs_wr_mem_out [`RS_SIZE];
    ALU_FUNC rs_alu_func_out [`RS_SIZE];
    logic rs_cond_branch_out [`RS_SIZE];
    logic rs_uncond_branch_out [`RS_SIZE];

    ALU_OPA_SELECT rs_opa_select_out [`RS_SIZE];
    ALU_OPB_SELECT rs_opb_select_out [`RS_SIZE];
    logic halt_rt_hack;

    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            halt_rt_hack <= 0;
        end else if (halt_rt) begin
            halt_rt_hack <= 1;
        end 
        else begin
        end
    end

    // ROB_RETIRE_PACKET id_rob_retire_out;
    // logic rob_ready, rob_valid;

    // LSQ_PACKET lsq_packet;


    //////////////////////////////////////////////////
    //                IS Stage Wires                //
    //////////////////////////////////////////////////
    // IS_EX_PACKET      is_packet;
    // IS_EX_PACKET      is_ex_reg;
    // logic             issue_valid;
    // logic fu_ready;
    // logic [`RS_SIZE-1:0] rs_issue_enable;

    //////////////////////////////////////////////////
    //                 EX Stage Wires               //
    //////////////////////////////////////////////////
    // ID_EX_PACKET id_ex_reg;   // The ID to EX stage register
    // EX_MEM_PACKET ex_packet;  // Output Packet
    // CDB_PACKET cdb_packet_ex;
    // logic cdb_busy;
    // logic fu_busy; //this will stall the RS issue if ex stage is busy / full

    // we dont need this no more
    // assign fu_ready = !fu_busy;


    //////////////////////////////////////////////////
    //                CP Stage Wires                //
    //////////////////////////////////////////////////
    // EX_CP_PACKET ex_cp_reg;
    // CDB_PACKET cdb_packet;

    //////////////////////////////////////////////////
    //               RT Stage Wires                 //
    //////////////////////////////////////////////////
    // logic [`XLEN-1:0] retire_value_out;
    // logic [4:0]       retire_dest_out;
    // logic             retire_valid_out;
    logic [`XLEN-1:0] mem_addr_out;
    logic             mem_valid_out;

    logic clear_lsq;
    logic clear_is;
    logic stall_if_rt;

    logic take_branch;
    logic [31:0] new_addr;

    //////////////////////////////////////////////////
    //                ROB + Map Table Wires         //
    //////////////////////////////////////////////////
    DISPATCH_ROB_PACKET rob_dispatch_packet;
    ROB_DISPATCH_PACKET rob_dispatch_out;
    // ROB_RETIRE_PACKET rob_retire_packet;
    // logic rob_full;

    //////////////////////////////////////////////////
    //               I-Cache Wires                  //
    //////////////////////////////////////////////////
    logic [1:0]       proc2Imem_command;
    logic [`XLEN-1:0] proc2Imem_addr;
    // logic [3:0]       Imem2proc_response;
    logic [63:0]      Imem2proc_data;
    logic [3:0]       Imem2proc_tag;

    //////////////////////////////////////////////////
    //               D-Cache Wires                  //
    //////////////////////////////////////////////////

    logic [`XLEN-1:0] proc2Dcache_addr;
    logic [1:0] proc2Dcache_command; // `BUS_NONE `BUS_LOAD or `BUS_STORE

    // logic [3:0]  mem2dcache_response; // 0 = can't accept, other=tag of transaction
    // logic [63:0] mem2dcache_data;    // data resulting from a load
    // logic [3:0]  mem2dcache_tag;       // 0 = no value, other=tag of transaction

    // logic [`XLEN-1:0] dcache2mem_addr;
    // logic [63:0]      dcache2mem_data; // address for current command
    // logic [1:0]       dcache2mem_command; // `BUS_NONE `BUS_LOAD or `BUS_STORE
    // MEM_SIZE    dcache2mem_size;

    logic [63:0] hit_data; // data resulting from a load
    logic hit; // 1 if hit, 0 if miss
    logic [3:0] data_tag;
    logic [3:0] data_response;
    // logic [1:0] next_state; //for debugging 
    // logic [1:0] state; //for debugging 
    // logic [3:0] number_of_waits; //for debugging
    // logic [3:0] next_number_of_waits; //for debugging

    //////////////////////////////////////////////////
    //               LSQ Wires                     //
    //////////////////////////////////////////////////

    // priv_addr_packet priv_addr_packet; // this is correct // packet to send to memory stage
    // logic [63:0]dcache_data_out; // data coming back from cache
    // logic [3:0] dcache_tag; // high when valid
    // logic [3:0] dcache_response; // 0 = can't accept, other=tag of transaction]
    // logic dcache_hit; // 1 if hit, 0 if miss
    // logic [1:0] dcache_command; // `BUS_NONE `BUS_LOAD or `BUS_STORE
    // logic [63:0] dcache_data; // data going to cache for store
    // logic [`XLEN-1:0] dcache_addr; // sending address to dcache

    // logic [4:0] mem_tag; // from rt stage
    // logic mem_valid; // from rt stage
    // EX_CP_PACKET cdb_lsq; // broadcast load data

    // logic store_ready;
    // logic [4:0] store_tag; // tag of store ready to write
    // logic lsq_free; // stall dispatch if lsq is full
    // logic cache_in_flight; //debugging
    // logic head_ready_for_mem; // debugging
    // logic [2:0] head_ptr; //points to OLDEST entry debugging
    // logic [2:0] tail_ptr; //points to next free entry debugging
    //////////////////////////////////////////////////
    //           Temporary Branch Logic             //
    //////////////////////////////////////////////////
    assign if_valid = dispatch_ok && ~if_stall && ~stall_if_rt && !halt_rt_hack;                // Always fetch for now
    assign branch_target = 32'b0;          // Default branch target

    //////////////////////////////////////////////////
    //         Fetch Stage                          //
    //////////////////////////////////////////////////
    stage_if stage_if_0 (
        .clock(clock),
        .reset(reset),
        .if_valid(if_valid),
        .take_branch(take_branch),
        .branch_target(new_addr),
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
        .Imem2proc_response(Imem2proc_response),
        .Imem2proc_data(Imem2proc_data),
        .Imem2proc_tag(Imem2proc_tag),
        .proc2Icache_addr(proc2Icache_addr),
        .proc2Imem_command(proc2Imem_command),
        .proc2Imem_addr(proc2Imem_addr),
        .Icache_data_out(Icache_data_out),
        .Icache_valid_out(Icache_valid_out),
        .current_mem_tag(current_mem_tag)
    );
    // for now, no icache, i will pass through all the data
    // assign proc2Imem_addr = proc2Icache_addr;
    // assign Icache_data_out = Imem2proc_data;
    // assign Icache_valid_out = Imem2proc_tag != 0; // this means it is returning data

    //////////////////////////////////////////////////
    //                  D-Cache                     //
    //////////////////////////////////////////////////

    dcache dcache_0 (
        .clk(clock),
        .reset(reset),
        .proc2Dcache_addr(dcache_addr),
        .proc2Dcache_data(dcache_data),
        .proc2Dcache_command(dcache_command),
        .proc2Dcache_size(dcache_size),
        .mem2dcache_response(mem2dcache_response),
        .mem2dcache_data(mem2dcache_data),
        .mem2dcache_tag(mem2dcache_tag),
        .dcache2mem_addr(dcache2mem_addr),
        .dcache2mem_data(dcache2mem_data),
        .halt(halt_rt_hack),
        .dcache2mem_command(dcache2mem_command),
        // .dcache2mem_size(dcache2mem_size),
        .hit_data(dcache_data_out),
        .hit(dcache_hit),
        .data_tag(dcache_tag),
        .data_response(dcache_response),
        .state(state),
        .next_state(next_state),

        .cur_addr(dcache_cur_addr),
        .cur_command(dcache_cur_command),
        .cur_data(dcache_cur_data),

        .tag_to_addr(tag_to_addr),
        .tag_to_addr_valid(tag_to_addr_valid),
        .tag_to_memsize(tag_to_memsize),
        .tag_to_memdata(tag_to_memdata),
        .tag_to_is_store(tag_to_is_store),

        .cache_data(cache_data),
        .cache_tag(cache_tag),
        .cache_valid(cache_valid),
        .cache_dirty(cache_dirty),
        .halt_confirm(halt_confirm),

        .wb_eviction(wb_eviction),
        .next_wb_eviction(next_wb_eviction)
    );

    // for now lets just do passthroughs:
    // assign dcache2mem_addr = dcache_addr;
    // assign dcache2mem_data = dcache_data;
    // assign dcache2mem_command = dcache_command;


    // assign dcache_tag = mem2dcache_tag;
    // assign dcache_hit = mem2dcache_tag == mem2dcache_response && mem2dcache_response != 0;
    // assign dcache_response = mem2dcache_response;
    // assign dcache_data_out = mem2dcache_data;


    //////////////////////////////////////////////////
    //         IF/ID Pipeline Register              //
    //////////////////////////////////////////////////
    // IF_ID_PACKET      if_id_reg;
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            if_id_reg <= '0;
        end else begin
            if (clear_is || halt_rt_hack) begin
                if_id_reg <= '0;
            end else if (dispatch_ok) begin
                if_id_reg <= if_packet;
            end
        end
    end


    //////////////////////////////////////////////////
    //               Decode Stage                   //
    //////////////////////////////////////////////////
   
    // stage reset 
    logic stage_id_reset;
    assign stage_id_reset = reset || clear_is || halt_rt_hack;
    stage_id stage_id_0 (
        .clock(clock),
        .reset(stage_id_reset),
        .if_id_reg(if_id_reg),
        .if_stall(if_stall),

        .cdb_valid(cdb_packet.valid),
        .cdb_tag(cdb_packet.tag),
        .cdb_value(cdb_packet.value),
        .cdb_take_branch(cdb_packet.take_branch),

        .fu_busy(fu_busy_signals),
        .rs_clear_vec(rs_issue_enable), //this means its the first register

        .rob_retire_entry(retire_valid_out), // TODO: connect properly

        .store_retire(store_ready),
        .store_tag(store_tag),

        .rob_dest_reg(retire_dest_out),
        .rob_to_regfile_value(retire_value_out),
        .retire_entry(retire_valid_out),
        .retire_tag(retire_tag),
        // .rob_regfile_valid(retire_valid_out),

        .lsq_free(lsq_free),

        .rs_opa_out(rs_opa_out),
        .rs_opb_out(rs_opb_out),
        .rs_inst_out(rs_inst_out),
        .rs_opa_select_out(rs_opa_select_out),
        .rs_opb_select_out(rs_opb_select_out),
        .rs_tag_out(rs_tag_out),
        .rs_npc_out(rs_npc_out),
        .rs_pc_out(rs_pc_out),
        .rs_ready_out(rs_ready_out),
        .rs_avail_out(rs_avail_out),

        .has_dest_reg(id_has_dest_reg),
        .dest_reg_idx(id_dest_reg_idx),
        .rs_rd_mem_out(rs_rd_mem_out),
        .rs_wr_mem_out(rs_wr_mem_out),
        .rs_cond_branch_out(rs_cond_branch_out),
        .rs_uncond_branch_out(rs_uncond_branch_out),
        .rob_ready(rob_ready),
        .rob_valid(rob_valid),
        .rs_alu_func_out(rs_alu_func_out),
        .rob_retire_out(rob_retire_packet),

        .rob_pointers_debug(id_rob_pointers),
        .rob_debug(id_rob_debug),
        .rs_debug(rs_debug),

        .lsq_packet(lsq_packet),
        .rob_full(rob_full),
        .dispatch_ok(dispatch_ok),
        .rs1_available(rs1_available),
        .mt_to_rs_tag1(mt_to_rs_tag1),
        .mt_to_rs_tag2(mt_to_rs_tag2),
        .rs1_value(rs1_value),
        .rs2_value(rs2_value),
        .rob_to_rs_value1(rob_to_rs_value1),
        .rob_to_rs_value2(rob_to_rs_value2),

        .debug_reg(debug_reg),

        .mt_to_regfile_rs1(mt_to_regfile_rs1),
        .mt_to_regfile_rs2(mt_to_regfile_rs2),

        .rs1_opa_in(rs1_opa_in),
        .rs1_opb_in(rs1_opb_in),

        .rs_entry_found(rs_entry_found),
        .fu_select(fu_select) // Selects which RS entry to load into

    );

    //////////////////////////////////////////////////
    //         ID/IS Pipeline Register              //
    //////////////////////////////////////////////////
    //always_ff @(posedge clock or posedge reset) begin
    //    if (reset) begin
    //        id_is_reg <= '0; // IS ID Packet not defined yet FIX
    //    end else begin
    //        id_is_reg <= id_is_packet; // IS ID Packet not defined yet FIX
    //    end
    //end



    //////////////////////////////////////////////////
    //                Issue Stage                   //
    //////////////////////////////////////////////////

     stage_is stage_is_0 (
        .clock(clock),
        .reset(reset),

        // Functional unit ready signals (invert of busy)
        .fu_ready_alu0(!fu_busy_signals[0]),
        .fu_ready_alu1(!fu_busy_signals[1]),
        .fu_ready_mult(!fu_busy_signals[2]),

        // Reservation Station inputs from stage_id
        .rs_ready_out(rs_ready_out),
        .rs_opa_out(rs_opa_out),
        .rs_opb_out(rs_opb_out),
        .rs_opa_select_out(rs_opa_select_out),
        .rs_opb_select_out(rs_opb_select_out),
        .rs_inst_out(rs_inst_out),
        .rs_tag_out(rs_tag_out),
        .rs_alu_func_out(rs_alu_func_out),
        .rs_npc_out(rs_npc_out),
        .rs_pc_out(rs_pc_out),
        .rd_mem(rs_rd_mem_out),
        .wr_mem(rs_wr_mem_out),
        .cond_branch(rs_cond_branch_out),
        .uncond_branch(rs_uncond_branch_out),

        // Outputs
        .is_packets(is_packets),
        .rs_issue_enable(rs_issue_enable),

        .next_alu0_indx(next_alu0_indx),
        .next_alu1_indx(next_alu1_indx),
        .next_mult_indx(next_mult_indx),

        .next_issued_alu0(next_issued_alu0),
        .next_issued_alu1(next_issued_alu1),
        .next_issued_mult(next_issued_mult)
    );

    //////////////////////////////////////////////////
    //         IS/EX Pipeline Register              //
    //////////////////////////////////////////////////
    // always_ff @(posedge clock or posedge reset) begin
    //     if (reset) begin
    //         is_ex_reg <= '0;
    //     end else begin
    //         is_ex_reg <= is_packet;
    //     end
    // end

    //////////////////////////////////////////////////
    //                Execute Stage                 //
    //////////////////////////////////////////////////
    // EX_CP_PACKET ex_packet;
    logic ex_reset;
    assign ex_reset = reset || clear_is || halt_rt_hack;
 
    stage_ex stage_ex_0 (
         .clk(clock),
        .rst(ex_reset),
        .cdb_packet_busy(cdb_busy),
        .is_ex_reg(is_packets),
        .ex_cp_packet(ex_packet),
        // .take_conditional(take_conditional),
        .priv_addr_out(priv_addr_packet),
        .fu_busy_signals(fu_busy_signals),
        .mult_done(mult_done),

        .hold_alu0_valid(hold_alu0_valid),
        .hold_alu1_valid(hold_alu1_valid),
        .hold_mult_valid(hold_mult_valid),

        .hold_alu0_pkt(hold_alu0_pkt),
        .hold_alu1_pkt(hold_alu1_pkt),
        .hold_mult_pkt(hold_mult_pkt),
        .tmp_alu0_pkt(tmp_alu0_pkt),
        .tmp_alu1_pkt(tmp_alu1_pkt),
        .tmp_mult_pkt(tmp_mult_pkt)
    );

     //////////////////////////////////////////////////
    //              Memory Stage                    //
    //////////////////////////////////////////////////

    // //this is temporary while we wait for LSQ stage to be complete
    // // we will connect with LSQ wires
    //MEM_WB_PACKET mem_packet;


    // stage_mem stage_mem_0 (
    //      // Inputs
    //     .ex_mem_reg     (ex_mem_reg),
    //     .Dmem2proc_data (dcache_data_out), 

    //     // Outputs
    //     .mem_packet        (mem_packet),
    //     .proc2Dmem_command (dcache_command),
    //     // .proc2Dmem_size    (dcache_size),
    //     .proc2Dmem_addr    (dcache_addr),
    //     .proc2Dmem_data    (dcache_data)
    // );
    
    //////////////////////////////////////////////////
    //                LSQ Stage                     //
    //////////////////////////////////////////////////
    // so my idea for LSQ stage is the following:
    // we will issue the instruction in the reservation station
    // 
    logic lsq_reset;
    assign lsq_reset = reset || clear_lsq || halt_rt_hack;

    lsq lsq_0 (
        .clk(clock),
        .reset(lsq_reset),
        .dcache_data_out(dcache_data_out),
        .dcache_tag(dcache_tag),
        .dcache_response(dcache_response),
        .dcache_hit(dcache_hit),

        .mem_tag(mem_tag),
        .mem_valid(mem_valid),

        .dcache_cur_addr(dcache_cur_addr),
        .dcache_cur_command(dcache_cur_command),
        .dcache_cur_data(dcache_cur_data),

        .lsq_packet(lsq_packet),
        .cdb_in(cdb_packet), //check
        .priv_addr_in(priv_addr_packet),

        .cdb_out(cdb_lsq), // broadcast load data
        .dcache_command(dcache_command),
        .dcache_addr(dcache_addr), // sending address to dcache
        .dcache_data(dcache_data), // data for current command (if store)
        .dcache_size(dcache_size), // size of data to send to cache

        .lsq_op_in_progress(lsq_op_in_progress), // this is a signal to the ROB to stall

        .store_ready(store_ready), // let ROB know that store ready to write
        .store_ready_tag(store_tag), // tag of store ready to write
        .lsq_free(lsq_free),// if lsq has empty entry
        .cache_in_flight(cache_in_flight), //debugging
        .head_ready_for_mem(head_ready_for_mem), // debugging
        .head_ptr(head_ptr), //points to OLDEST entry debugging
        .tail_ptr(tail_ptr), //points to next free entry debugging
        .lsq_out(lsq_out), // debugging

        .cache_in_flight_valid(cache_in_flight_valid),
        .cache_tag_in_flight(cache_tag_in_flight),
        .cache_in_flight_rd_unsigned(cache_in_flight_rd_unsigned),
        .cache_offset_in_flight(cache_offset_in_flight),
        .cache_in_flight_mem_size(cache_in_flight_mem_size)
    );

    //////////////////////////////////////////////////
    //           EX/CP Pipeline Register            //
    //////////////////////////////////////////////////
    // always_ff @(posedge clock or posedge reset) begin
    //     if (reset || clear_lsq || halt_rt_hack) begin
    //         ex_cp_reg <= '0;
    //     end else begin
    //         if (!cdb_busy) begin
    //             ex_cp_reg <= ex_packet;
    //         end
    //     end
    // end

    //////////////////////////////////////////////////
    //               Complete Stage                 //
    //////////////////////////////////////////////////
    logic cp_reset;
    assign cp_reset = reset || clear_lsq || halt_rt_hack;
    stage_cp stage_cp_0 (
        .clock(clock),
        .reset(cp_reset),
        .ex_cp_packet(ex_packet), // input packet from EX stage
        .lsq_cp_packet(cdb_lsq), // input packet from LSQ stage
        .cdb_packet_out(cdb_packet),
        .ex_rejected(cdb_busy)
    );


    
    //////////////////////////////////////////////////
    //            Reorder Buffer (ROB)              //
    //////////////////////////////////////////////////
    // reorder_buffer reorder_buffer_0 (
    //     .reset(reset),
    //     .clock(clock),
    //     .rob_dispatch_in(rob_dispatch_packet),
    //     .rob_dispatch_out(rob_dispatch_out),
    //     .rob_cdb_in(cdb_packet),
    //     .retire_entry(1'b0),
    //     .rob_clear(1'b0),
    //     .rob_retire_out(rob_retire_packet),
    //     .rob_to_rs_value1(),
    //     .rob_to_rs_value2(),
    //     .rob_full(rob_full),
    //     .rob_debug(id_rob_debug),
    //     .rob_pointers(id_rob_pointers)
    // );

    //////////////////////////////////////////////////
    //                Map Table                     //
    //////////////////////////////////////////////////
    // map_table map_table_0 (
    //     .reset(reset),
    //     .clock(clock),

    //     // Source register addresses (for reading tags)
    //     .rs1_addr(if_id_reg.inst.r.rs1),
    //     .rs2_addr(if_id_reg.inst.r.rs2),

    //     // Destination register address (where the result will eventually be written)
    //     .r_dest(if_id_reg.inst.r.rd),

    //     // ROB tag assigned to destination register
    //     .tag_in(rob_dispatch_out.tag),

    //     // Dispatch control: whether we are dispatching a new instruction
    //     .load_entry(dispatch_ok && if_id_reg.valid && has_dest_reg),

    //     // CDB broadcast: update map table when a result is ready
    //     .cdb_tag_in(cdb_packet.tag),
    //     .read_cdb(cdb_packet.valid),

    //     // Retirement: clear mappings when instructions retire
    //     .retire_addr(rob_retire_packet.dest_reg),
    //     .retire_tag(rob_retire_packet.tag),
    //     .retire_entry(rob_retire_packet.valid),

    //     // Outputs to the Reservation Station / Decode
    //     .rs1_tag(), // (connect later if needed)
    //     .rs2_tag(), // (connect later if needed)

    //     // Pass through register addresses for regfile reads
    //     .regfile_rs1_addr(), // (connect if needed)
    //     .regfile_rs2_addr(),

    //     // Debug
    //     .tags_debug(mt_tags_debug)
    // );

    //////////////////////////////////////////////////
    //            CP/RT Pipeline Register           //
    //////////////////////////////////////////////////
    // ROB_RETIRE_PACKET cp_rt_reg;
    // always_ff @(posedge clock or posedge reset) begin
    //     if (reset)
    //         cp_rt_reg <= '0;
    //     else
    //         cp_rt_reg <= rob_retire_packet;
    // end


    //////////////////////////////////////////////////
    //               RT (Retire) Stage              //
    //////////////////////////////////////////////////
    stage_rt stage_rt_0 (
        .clock(clock),
        .reset(reset),
        .rob_retire_packet(rob_retire_packet),
        .rob_ready(rob_ready),
        .rob_valid(rob_valid),
        .branch_mispredict(1'b0),
        .lsq_op_in_progress(lsq_op_in_progress),
        .retire_value(retire_value_out),
        .retire_dest(retire_dest_out),
        .retire_valid_out(retire_valid_out),
        .retire_tag(retire_tag),
        .halt(halt_rt),
        .illegal(illegal_rt),
        .csr_op(csr_op_rt),
        .npc(npc_rt),
        .mem_tag(mem_tag),
        .mem_valid(mem_valid),
        .clear_is(clear_is),
        .stall_if(stall_if_rt),
        .clear_lsq(clear_lsq),

        .take_branch(take_branch),
        .new_addr(new_addr)
    );



    `define OWN_NONE 2'b00
    `define OWN_D    2'b01
    `define OWN_I    2'b10



    //////////////////////////////////////////////////
    //              Memory Access Logic             //
    //////////////////////////////////////////////////
    // logic [1:0] owner_q, owner_d; // this will keep track of who sent the memory request at the last time step
    // logic [`XLEN-1:0] proc2Dmem_addr;
    // logic [`XLEN-1:0] proc2Dmem_data;
    // logic [1:0]       proc2Dmem_command;
    // logic wait_one_step;
    // logic next_wait_one_step;
    // logic read_data;
`ifndef CACHE_MODE
    MEM_SIZE          proc2Dmem_size;
`endif

//     always_comb begin
//         next_wait_one_step = wait_one_step == 0 ? 0 : wait_one_step - 1;

//         owner_d = owner_q;
//         if (dcache2mem_command != BUS_NONE) begin
            
//             proc2mem_command = dcache2mem_command;
//             proc2mem_addr    = dcache2mem_addr;
//             proc2mem_data = dcache2mem_data;
// `ifndef CACHE_MODE
//             proc2mem_size    = dcache_size;
// `endif
//             //if data mmodule sent the request
//             owner_d = `OWN_D;
//             if (owner_d != owner_q) begin
//                 next_wait_one_step = 1;
//             end
//         end else begin
//             proc2mem_command = BUS_LOAD;
//             proc2mem_addr    = proc2Imem_addr;
// `ifndef CACHE_MODE
//             proc2mem_size    = DOUBLE;
// `endif
//             // then if instruction module sent the request
//             owner_d = `OWN_I;
//             if (owner_d != owner_q) begin
//                 next_wait_one_step = 1;
//             end
//         end
//     end


//     always_ff @(negedge clock or posedge reset) begin
//         if (reset) begin
//             owner_q <= `OWN_NONE;
//             wait_one_step <= 1;
//         // for now, we have to wait for data to respond, so not just tag
//         end
//         else begin
//             wait_one_step <= next_wait_one_step;
//             if (owner_q != owner_d && (read_data || owner_q == `OWN_NONE))begin
//                 owner_q <= owner_d;
//             end
//         end
//     end
    
//     assign Imem2proc_data     = mem2proc_data;
//     assign Imem2proc_tag      = mem2proc_tag;
//     assign mem2dcache_data = mem2proc_data;
//     assign mem2dcache_tag      = mem2proc_tag;
//     always_comb begin
//         // Default: de‑assert
//         mem2dcache_response     = 0;

//         Imem2proc_response  = 0;

//         read_data=0;

//         if_stall = 0;

//         case (owner_q)
//             `OWN_D: begin
//                 if_stall = 1'b1;
//                 if (wait_one_step == 0) begin
//                     mem2dcache_response = mem2proc_response;

//                     if (mem2proc_response != 0) begin
//                         read_data = 1'b1;
//                     end
//                 end
//             end
//             `OWN_I: begin
//                 if (owner_d != `OWN_D) begin
//                         Imem2proc_response = mem2proc_response;
//                         if (mem2proc_response != 0) begin
//                             read_data = 1'b1;
//                         end
//                     end
                
//                 end
//                 else begin
//                     read_data = 1'b1;
//                 end
                
//             end
//             default: begin
//             end
//         endcase
//     end

    // logic [1:0] cycle_wait;
    // logic [1:0] next_cycle_wait;
    assign next_icache_rejected = (dcache2mem_command == BUS_NONE) ? 1'b0 : 1'b1;


    assign if_stall = icache_rejected;
    assign Imem2proc_data     = mem2proc_data;
    assign Imem2proc_tag      = mem2proc_tag;
    assign mem2dcache_data = mem2proc_data;
    assign mem2dcache_tag      = mem2proc_tag;
    always_comb begin
        next_cycle_wait = cycle_wait == 0 ? 0 : cycle_wait - 1;
        // if (icache_rejected != next_icache_rejected) begin
        //     next_cycle_wait = 2'b10;
        // end

        if (pos_icache_rejected) begin
                proc2mem_command = dcache2mem_command;
                proc2mem_addr    = dcache2mem_addr;
                proc2mem_data = dcache2mem_data;
`ifndef CACHE_MODE
                proc2mem_size    = dcache_size;

`endif
            end
            else begin
                proc2mem_command = BUS_LOAD;
                proc2mem_addr    = proc2Imem_addr;
                proc2mem_data = 0;
`ifndef CACHE_MODE
                proc2mem_size    = DOUBLE;
`endif
            end
    end
    always_ff @(posedge clock or posedge reset) begin
        if (reset) begin
            pos_icache_rejected <= 1'b0;
        end
        else begin
            pos_icache_rejected <= next_icache_rejected;
        end
    end
    // stay on positive
    always_ff @(negedge clock or posedge reset) begin
        if (reset) begin
            icache_rejected <= 1'b0; 
            prev_icache_rejected <= 1'b0;  
        end else begin
            prev_icache_rejected <= icache_rejected;
            icache_rejected <= pos_icache_rejected;
        end
    end

    always_comb begin
        mem2dcache_response   = 0;
        Imem2proc_response  = 0;
        if (prev_icache_rejected ) begin
            if (icache_rejected) begin
                mem2dcache_response = mem2proc_response;
            end
        end
        else begin
            if (!icache_rejected) begin
                Imem2proc_response  = mem2proc_response;
            end
        end
    end

    //////////////////////////////////////////////////
    //               Pipeline Outputs               //
    //////////////////////////////////////////////////
    assign pipeline_commit_wr_en    = retire_valid_out;
    assign pipeline_commit_wr_idx   = retire_dest_out;
    assign pipeline_commit_wr_data  = retire_value_out;
    assign pipeline_commit_NPC      = npc_rt; 
    assign pipeline_completed_insts = retire_valid_out ? 4'd1 : 4'd0;
    assign pipeline_error_status = illegal_rt        ? ILLEGAL_INST :
                                   halt_rt_hack && halt_confirm          ? HALTED_ON_WFI :
                                    NO_ERROR;

endmodule // pipeline