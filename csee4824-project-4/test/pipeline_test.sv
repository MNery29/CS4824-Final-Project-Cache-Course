/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  pipeline_test.sv                                    //
//                                                                     //
//  Description :  Testbench module for the verisimple pipeline;       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

// P4 TODO: Add your own debugging framework. Basic printing of data structures
//          is an absolute necessity for the project. You can use C functions
//          like in test/pipeline_print.c or just do everything in verilog.
//          Be careful about running out of space on CAEN printing lots of state
//          for longer programs (alexnet, outer_product, etc.)


// these link to the pipeline_print.c file in this directory, and are used below to print
// detailed output to the pipeline_output_file, initialized by open_pipeline_output_file()
// import "DPI-C" function void open_pipeline_output_file(string file_name);
// import "DPI-C" function void print_header(string str);
// import "DPI-C" function void print_cycles();
// import "DPI-C" function void print_stage(string div, int inst, int npc, int valid_inst);
// import "DPI-C" function void print_reg(int wb_reg_wr_data_out_hi, int wb_reg_wr_data_out_lo,
//                                        int wb_reg_wr_idx_out, int wb_reg_wr_en_out);
// import "DPI-C" function void print_membus(int proc2mem_command, int mem2proc_response,
//                                           int proc2mem_addr_hi, int proc2mem_addr_lo,
//                                           int proc2mem_data_hi, int proc2mem_data_lo);
// import "DPI-C" function void print_close();


module testbench;
    // used to parameterize which files are used for memory and writeback/pipeline outputs
    // "./simv" uses program.mem, writeback.out, and pipeline.out
    // but now "./simv +MEMORY=<my_program>.mem" loads <my_program>.mem instead
    // use +WRITEBACK=<my_program>.wb and +PIPELINE=<my_program>.ppln for those outputs as well
    string program_memory_file;
    string writeback_output_file;
    // string pipeline_output_file;

    // variables used in the testbench
    logic        clock;
    logic        reset;
    logic [31:0] clock_count;
    logic [31:0] instr_count;
    int          wb_fileno;
    logic [63:0] debug_counter; // counter used for infinite loops, forces termination

    logic [1:0]       proc2mem_command;
    logic [`XLEN-1:0] proc2mem_addr;
    logic [63:0]      proc2mem_data;
    logic [3:0]       mem2proc_response;
    logic [63:0]      mem2proc_data;
    logic [3:0]       mem2proc_tag;
`ifndef CACHE_MODE
    MEM_SIZE          proc2mem_size;
`endif

    logic [3:0]       pipeline_completed_insts;
    EXCEPTION_CODE    pipeline_error_status;
    logic [4:0]       pipeline_commit_wr_idx;
    logic [`XLEN-1:0] pipeline_commit_wr_data;
    logic             pipeline_commit_wr_en;
    logic [`XLEN-1:0] pipeline_commit_NPC;

    logic [3:0]  mem2dcache_response; // 0 = can't accept, other=tag of transaction
    logic [63:0] mem2dcache_data;    // data resulting from a load
    logic [3:0]  mem2dcache_tag; 

   
    logic [45:0]      id_rob_debug[31:0];

    logic stall_if;
    logic [`XLEN-1:0] proc2Icache_addr;
    logic             Icache_valid_out;
    logic [63:0] Icache_data_out;

    

    //id stage debugging
    logic [`ROB_TAG_BITS-1:0] id_tag;
    logic rs1_ready;
    logic [5:0] mt_to_rs_tag1, mt_to_rs_tag2;

    logic [31:0] rs1_value, rs2_value;
    logic [31:0] rob_to_rs_value1, rob_to_rs_value2;

    INST id_inst_out;


    //IS stage debugging wires
    IS_EX_PACKET is_packet;
    logic if_stall;
    IS_EX_PACKET is_ex_reg;
    logic issue_valid;
    logic fu_ready;
    logic [`RS_SIZE-1:0] rs_issue_enable;

    //ID Debugging wires 
    logic cdb_valid;
    logic [`ROB_TAG_BITS-1:0] cdb_tag;
    logic [31:0] cdb_value;

    logic rs1_clear;

    logic rob_retire_entry;

    logic store_retire;

    logic [4:0] rob_dest_reg;
    logic [31:0] rob_to_regfile_value;
    logic [4:0] retire_tag;
    logic rs_halt_out;
    logic rs_illegal_out;
    logic rs_csr_op_out;

    logic maptable_clear;
    logic rob_clear;
    logic rs_clear;

    logic [31:0] rs1_opa_in;
    logic [31:0] rs1_opb_in;



    //EX stage debugging wires
    EX_CP_PACKET ex_cp_reg;
    EX_CP_PACKET ex_packet;
    logic fu_busy;
    logic cdb_busy; //this will stall the RS issue if ex stage is busy / full
    logic [`XLEN-1:0] opa_mux_out;
    logic [`XLEN-1:0] opb_mux_out;

    
    logic take_conditional;
    IF_ID_PACKET if_packet;
    IF_ID_PACKET if_id_reg;

    logic rob_full;
    logic rs1_available;
    logic dispatch_ok;

    logic [73:0] rs_debug;

    CDB_PACKET cdb_packet;

    logic [`XLEN-1:0] retire_value_out;
    logic [4:0]       retire_dest_out;
    logic             retire_valid_out;

    ROB_RETIRE_PACKET rob_retire_packet;

    logic rob_valid, rob_ready;

    logic [31:1] [`XLEN-1:0] debug_reg;

    logic [4:0] mt_to_regfile_rs1, mt_to_regfile_rs2;

    ALU_OPA_SELECT id_opa_select;
    ALU_OPB_SELECT id_opb_select;

    LSQ_PACKET lsq_packet;
    lsq_entry_t lsq_out [7:0]; // debugging
    logic store_ready;
    logic [4:0] store_tag; // tag of store ready to write
    logic lsq_free; // stall dispatch if lsq is full
    priv_addr_packet priv_addr_in;
    logic cache_in_flight; //debugging
    logic head_ready_for_mem; // debugging
    logic [2:0] head_ptr; //points to OLDEST entry debugging
    logic [2:0] tail_ptr; //points to next free entry debugging
    logic [63:0]dcache_data_out; // data coming back from cache
    logic [3:0] dcache_tag; // high when valid
    logic [3:0] dcache_response; // 0 = can't accept; other=tag of transaction]
    logic dcache_hit; // 1 if hit; 0 if miss
    logic [1:0] dcache_command; // `BUS_NONE `BUS_LOAD or `BUS_STORE
    logic [63:0] dcache_data; // data going to cache for store
    logic [`XLEN-1:0] dcache_addr; // sending address to 
    logic [1:0] dcache_size; // size of data being sent to cache
    logic [4:0] mem_tag; // from rt stage
    logic mem_valid; // from rt stage

    EX_CP_PACKET cdb_lsq; // broadcast load data


    logic illegal_rt;
    logic halt_rt;
    logic csr_op_rt;

    logic [`XLEN-1:0] tag_to_addr [15:0];
    logic tag_to_addr_valid [15:0];
    logic [1:0] tag_to_memsize [15:0];
    logic [63:0] tag_to_memdata [15:0]; // this is for stores exclusively
    logic tag_to_is_store [15:0];

    logic [1:0] state;
    logic [1:0] next_state;

    logic [3:0] Imeme2proc_response;

    logic [63:0] cache_data [0:63]; // 64 lines of 8 bytes
    logic [22:0] cache_tag [0:63]; // 64 lines of tag bits
    logic cache_valid [0:63]; // 64 lines of valid bits
    logic cache_dirty [0:63]; // 64 lines of dirty bits
    
    logic [3:0] current_mem_tag;

    logic lsq_op_in_progress;


    // logic [`XLEN-1:0] if_NPC_dbg;
    // logic [31:0]      if_inst_dbg;
    // logic             if_valid_dbg;
    // logic [`XLEN-1:0] if_id_NPC_dbg;
    // logic [31:0]      if_id_inst_dbg;
    // logic             if_id_valid_dbg;
    // logic [`XLEN-1:0] id_ex_NPC_dbg;
    // logic [31:0]      id_ex_inst_dbg;
    // logic             id_ex_valid_dbg;
    // logic [`XLEN-1:0] ex_mem_NPC_dbg;
    // logic [31:0]      ex_mem_inst_dbg;
    // logic             ex_mem_valid_dbg;
    // logic [`XLEN-1:0] mem_wb_NPC_dbg;
    // logic [31:0]      mem_wb_inst_dbg;
    // logic             mem_wb_valid_dbg;


    // Instantiate the Pipeline
    pipeline core (
        // Inputs
        .clock             (clock),
        .reset             (reset),
        .mem2proc_response (mem2proc_response),
        .mem2proc_data     (mem2proc_data),
        .mem2proc_tag      (mem2proc_tag),

        // Outputs
        .proc2mem_command (proc2mem_command),
        .proc2mem_addr    (proc2mem_addr),
        .proc2mem_data    (proc2mem_data),
`ifndef CACHE_MODE
        .proc2mem_size    (proc2mem_size),
`endif
        .mem2dcache_response (mem2dcache_response),
        .mem2dcache_data     (mem2dcache_data),
        .mem2dcache_tag      (mem2dcache_tag),

        .pipeline_completed_insts (pipeline_completed_insts),
        .pipeline_error_status    (pipeline_error_status),
        .pipeline_commit_wr_data  (pipeline_commit_wr_data),
        .pipeline_commit_wr_idx   (pipeline_commit_wr_idx),
        .pipeline_commit_wr_en    (pipeline_commit_wr_en),
        .pipeline_commit_NPC      (pipeline_commit_NPC),

        .id_rob_debug          (id_rob_debug),
        .Icache_valid_out     (Icache_valid_out),
        .if_stall           (if_stall),
        .proc2Icache_addr     (proc2Icache_addr),
        .stall_if             (stall_if),
        .Icache_data_out       (Icache_data_out),

        //id stage debugging
        .id_tag               (id_tag),
        .rs1_ready            (rs1_ready),
        .mt_to_rs_tag1       (mt_to_rs_tag1),
        .mt_to_rs_tag2       (mt_to_rs_tag2),
        .rs1_value            (rs1_value),
        .rs2_value            (rs2_value),
        .rob_to_rs_value1     (rob_to_rs_value1),
        .rob_to_rs_value2     (rob_to_rs_value2),

        //IS stage debugging wires
        .is_packet            (is_packet),
        .is_ex_reg            (is_ex_reg),
        .issue_valid          (issue_valid),
        .fu_ready             (fu_ready),
        .rs_issue_enable      (rs_issue_enable),
        //EX stage debugging wires
        .ex_cp_reg            (ex_cp_reg),
        .fu_busy              (fu_busy),
        .cdb_busy             (cdb_busy),

        .take_conditional       (take_conditional),

        .if_packet            (if_packet),
        .if_id_reg            (if_id_reg),

        .rob_full             (rob_full),
        .rs1_available        (rs1_available),
        .dispatch_ok          (dispatch_ok),

        .id_rs_debug             (rs_debug),

        .ex_packet            (ex_packet),

        .cdb_packet            (cdb_packet),
        .opa_mux_out         (opa_mux_out),
        .opb_mux_out         (opb_mux_out),


        .id_inst_out          (id_inst_out),

        .illegal_rt         (illegal_rt),
        .halt_rt            (halt_rt),
        .csr_op_rt          (csr_op_rt),



        //rt stage debugging wires
        .retire_value_out     (retire_value_out),
        .retire_dest_out      (retire_dest_out),
        .retire_valid_out     (retire_valid_out),
        .retire_tag          (retire_tag),
        .rob_retire_packet    (rob_retire_packet),

        .rob_valid           (rob_valid),
        .rob_ready           (rob_ready),

        .debug_reg           (debug_reg),

        .mt_to_regfile_rs1 (mt_to_regfile_rs1),
        .mt_to_regfile_rs2 (mt_to_regfile_rs2),
        .id_opa_select (id_opa_select),
        .id_opb_select (id_opb_select),

        //LSQ DEBUGGING
        .lsq_packet          (lsq_packet),
        .lsq_out             (lsq_out),
        .store_ready         (store_ready),
        .store_tag           (store_tag),
        .lsq_free            (lsq_free),
        .priv_addr_packet     (priv_addr_in),
        .cache_in_flight     (cache_in_flight),
        .head_ready_for_mem  (head_ready_for_mem),
        .head_ptr           (head_ptr),
        .tail_ptr           (tail_ptr),
        .dcache_data_out     (dcache_data_out),
        .dcache_tag          (dcache_tag),
        .dcache_response     (dcache_response),
        .dcache_hit          (dcache_hit),
        .dcache_command      (dcache_command),
        .dcache_data         (dcache_data),
        .dcache_addr         (dcache_addr),
        .dcache_size         (dcache_size),
        .mem_tag             (mem_tag),
        .mem_valid           (mem_valid),
        .cdb_lsq             (cdb_lsq),

        .rs1_opa_in         (rs1_opa_in),
        .rs1_opb_in         (rs1_opb_in),

        // dcache debugging
        .tag_to_addr         (tag_to_addr),
        .tag_to_addr_valid   (tag_to_addr_valid),
        .tag_to_memsize      (tag_to_memsize),
        .tag_to_memdata      (tag_to_memdata),
        .tag_to_is_store     (tag_to_is_store),

        .state             (state),
        .next_state        (next_state),

        .Imem2proc_response (Imeme2proc_response),

        .cache_data         (cache_data),
        .cache_tag          (cache_tag),
        .cache_valid        (cache_valid),
        .cache_dirty        (cache_dirty),

        .current_mem_tag     (current_mem_tag),

        .lsq_op_in_progress (lsq_op_in_progress)
        

        // .if_NPC_dbg       (if_NPC_dbg),
        // .if_inst_dbg      (if_inst_dbg),
        // .if_valid_dbg     (if_valid_dbg),
        // .if_id_NPC_dbg    (if_id_NPC_dbg),
        // .if_id_inst_dbg   (if_id_inst_dbg),
        // .if_id_valid_dbg  (if_id_valid_dbg),
        // .id_ex_NPC_dbg    (id_ex_NPC_dbg),
        // .id_ex_inst_dbg   (id_ex_inst_dbg),
        // .id_ex_valid_dbg  (id_ex_valid_dbg),
        // .ex_mem_NPC_dbg   (ex_mem_NPC_dbg),
        // .ex_mem_inst_dbg  (ex_mem_inst_dbg),
        // .ex_mem_valid_dbg (ex_mem_valid_dbg),
        // .mem_wb_NPC_dbg   (mem_wb_NPC_dbg),
        // .mem_wb_inst_dbg  (mem_wb_inst_dbg),
        // .mem_wb_valid_dbg (mem_wb_valid_dbg)
    );


    // Instantiate the Data Memory
    mem memory (
        // Inputs
        .clk              (clock),
        .proc2mem_command (proc2mem_command),
        .proc2mem_addr    (proc2mem_addr),
        .proc2mem_data    (proc2mem_data),
`ifndef CACHE_MODE
        .proc2mem_size    (proc2mem_size),
`endif

        // Outputs
        .mem2proc_response (mem2proc_response),
        .mem2proc_data     (mem2proc_data),
        .mem2proc_tag      (mem2proc_tag)
    );


    // Generate System Clock
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

     function automatic string mem_size_str (input MEM_SIZE sz);
        case (sz)
            BYTE   : return "BYTE";
            HALF   : return "HALF";
            WORD   : return "WORD";
            DOUBLE : return "DOUBLE";
            default: return "MS(?)";
        endcase
    endfunction
    // Task to display # of elapsed clock edges
    task show_clk_count;
        real cpi;
        begin
            cpi = (clock_count + 1.0) / instr_count;
            $display("@@  %0d cycles / %0d instrs = %f CPI\n@@",
                      clock_count+1, instr_count, cpi);
            $display("@@  %4.2f ns total time to execute\n@@\n",
                      clock_count * `CLOCK_PERIOD);
        end
    endtask // task show_clk_count
   // ------------------------------------------------------------
    // Pretty-printer for a 64-line cache (TAG_BITS = 23).
    // Call with:  print_cache();
    // ------------------------------------------------------------
    task automatic print_cache;
    int i;

    // Header
    $display("\n%-4s | V D | %-6s | %-23s",
                "Idx", "Tag", "Data[63:0]  (byte-wise)");
    $display("---- | --- | ------ | -----------------------");

    // Body
    for (i = 0; i < 64; i++) begin
        $display("%4d | %1b %1b | 0x%06h | %02x %02x %02x %02x %02x %02x %02x %02x",
                i,
                cache_valid[i],
                cache_dirty[i],
                cache_tag[i][22:0],        // 23-bit tag
                cache_data[i][63:56],      // byte 7 (MSB)
                cache_data[i][55:48],      // byte 6
                cache_data[i][47:40],      // byte 5
                cache_data[i][39:32],      // byte 4
                cache_data[i][31:24],      // byte 3
                cache_data[i][23:16],      // byte 2
                cache_data[i][15:8 ],      // byte 1
                cache_data[i][7 :0 ]       // byte 0 (LSB)
                );
    end

    $display("");   // trailing blank line
    endtask


    task automatic show_if_packet (input IF_ID_PACKET pkt);
        // Extract indices (0–31).  For formats that lack rs1/rs2 these
        // bits are architecturally zero, so the printout is harmless.
        automatic logic [4:0] rs1_idx = pkt.inst[19:15];
        automatic logic [4:0] rs2_idx = pkt.inst[24:20];

        if (!pkt.valid) begin
            $display("[%0t] IF   : (stall/invalid)", $time);
        end
        else begin
            $display("[%0t] IF   : PC = 0x%08h  NPC = 0x%08h inst = 0x%08h  rs1 = %0d  rs2 = %0d",
                    $time, pkt.PC, pkt.NPC, pkt.inst, rs1_idx, rs2_idx);
        end
    endtask
    task automatic show_rob_retire_packet (
        input ROB_RETIRE_PACKET pkt,
        input string            prefix = "RETIRE"
    );
        $display("[%0t] %s : tag=%0d  dest=%0d  reg_valid=%b mem_valid=%b  branch=%b",
                $time, prefix,
                pkt.tag, pkt.dest_reg,
                pkt.reg_valid, pkt.mem_valid, pkt.is_branch);

        $display("            value=0x%08h  mem_addr=0x%08h take_branch=%b NPC = 0x%08h",
                pkt.value, pkt.mem_addr, pkt.take_branch, pkt.npc);
    endtask
    function automatic string memsize_str (input logic [1:0] sz);
        // Assumes the standard MEM_SIZE encoding (`BYTE, HALF, WORD, DOUBLE`)
        case (sz)
            BYTE   : memsize_str = "B ";
            HALF   : memsize_str = "H ";
            WORD   : memsize_str = "W ";
            DOUBLE : memsize_str = "D ";
            default: memsize_str = "??";
        endcase
    endfunction

    task automatic dump_tag_map;
        int i;
        $display("\n=== Tag-to-Address Map ================================================");
        $display(" idx | V | isSt | Sz |          Address          |        Data");
        $display("-----+---+------+----+---------------------------+-----------------------");
        for (i = 0; i < 16; i++) begin
            if (tag_to_addr_valid[i]) begin
                $display(" %20d | %1b |  %1b   | %s | 0x%016h | 0x%016h",
                        i,
                        tag_to_addr_valid[i],
                        tag_to_is_store[i],
                        memsize_str(tag_to_memsize[i]),
                        tag_to_addr[i],
                        tag_to_memdata[i]);
            end
            else begin
                // Print an empty row so the chart always shows 16 lines
                $display(" %20d | 0 |  -   | -- | ----------------------- | ------------------", i);
            end
        end
        $display("=======================================================================\n");
    endtask

    task automatic show_id_stage (
        input [`ROB_TAG_BITS-1:0] id_tag,
        input logic               rs1_ready
    );
        $display("[%0t] ID   : tag=%0d  rs1_ready=%b",
                $time, id_tag, rs1_ready);
    endtask
    function string alu_func_str (ALU_FUNC f);
        case (f)
            ALU_ADD   : return "ADD";
            ALU_SUB   : return "SUB";
            ALU_AND   : return "AND";
            ALU_OR    : return "OR";
            ALU_XOR   : return "XOR";
            ALU_SLT   : return "SLT";
            ALU_SLTU  : return "SLTU";
            ALU_SLL   : return "SLL";
            ALU_SRL   : return "SRL";
            ALU_SRA   : return "SRA";
            ALU_MUL   : return "MUL";
            default   : return "UNK";
        endcase
    endfunction

    // ===============================================================
    //  Pretty-printer for a single LSQ entry
    //    – returns a formatted string, so you can use $display()
    // ===============================================================
    function automatic string lsq_entry_fmt
        (input int idx, input lsq_entry_t e);
        string s;
        $sformat(s,
            "[%0d] v:%0b %-2s  rob:%2d | addr:%08h (tag:%2d v:%0b) | data:%016h (tag:%2d v:%0b) | rd_unsigned: %0b mem_size: %s| ret:%0b",
            idx,
            e.valid,
            e.is_store ? "ST" : "LD",
            e.rob_tag,
            e.address,
            e.address_tag,
            e.address_valid,
            e.store_data,
            e.store_data_tag,
            e.store_data_valid,
            e.rd_unsigned,
            mem_size_str(e.mem_size),
            e.retired);
        return s;
    endfunction

    function automatic string priv_addr_pkt_fmt (input priv_addr_packet p);
        string s;
        if (!p.valid)
            return "(priv-addr: — invalid —)";

        $sformat(s,
            "(addr:%0h  tag:%0d  v:%0b)",
            p.addr,
            p.tag,
            p.valid);
        return s;
    endfunction

    // ===============================================================
    //  Convenience task to dump the whole LSQ array
    // ===============================================================
    task automatic dump_lsq
        (input lsq_entry_t lsq [8]);
        int i;
        $display("====== LSQ DUMP @ %0t ======", $time);
        for (i = 0; i < 8; i++) begin
            // if (lsq[i].valid)              // comment out this line
            $display("%s", lsq_entry_fmt(i, lsq[i]));
        end
        $display("===========================\n");
    endtask


    // ------------------------------------------------------------
    //  ID-input tests
    // ------------------------------------------------------------


    task automatic show_dispatch_inputs (
        input logic              clock,
        input logic              reset,
        input IF_ID_PACKET       if_id_reg,
        input logic              cdb_valid,
        input [`ROB_TAG_BITS-1:0] cdb_tag,
        input logic [31:0]       cdb_value,
        input logic              fu_busy,
        input logic              rs1_clear,
        input logic              rob_retire_entry,
        input logic              store_retire,
        input logic [4:0]        store_tag,
        input logic [4:0]        rob_dest_reg,
        input logic [31:0]       rob_to_regfile_value,
        input logic              lsq_free,
        input logic              maptable_clear,
        input logic              rob_clear,
        input logic              rs_clear
    );
        $display("[%0t] DISPATCH INPUTS:", $time);
        $display("    clock=%b  reset=%b", clock, reset);
        $display("    if_id_reg: PC=0x%08h  NPC=0x%08h  inst=0x%08h  valid=%b", 
                if_id_reg.PC, if_id_reg.NPC, if_id_reg.inst, if_id_reg.valid);
        $display("    CDB: valid=%b  tag=%0d  value=0x%08h", 
                cdb_valid, cdb_tag, cdb_value);
        $display("    fu_busy=%b  rs1_clear=%b", fu_busy, rs1_clear);
        $display("    rob_retire_entry=%b  rob_dest_reg=%0d  rob_to_regfile_value=0x%08h", 
                rob_retire_entry, rob_dest_reg, rob_to_regfile_value);
        $display("    store_retire=%b  store_tag=%0d", store_retire, store_tag);
        $display("    lsq_free=%b", lsq_free);
        $display("    clears: map_table=%b  rob=%b  rs=%b", 
                maptable_clear, rob_clear, rs_clear);
    endtask




    task automatic show_rs_debug (
        input logic [74:0] rs_debug,
        input string       prefix = "RS"
    );
        // unpack
        logic [31:0] opA     = rs_debug[73:42];
        logic        opA_v   = rs_debug[41];
        logic [31:0] opB     = rs_debug[40:09];
        logic        opB_v   = rs_debug[8];
        logic [4:0]  tag     = rs_debug[7:3];
        logic        in_use  = rs_debug[2];
        logic        ready   = rs_debug[1];
        logic        avail   = rs_debug[0];

        // print
        $display("[%0t] %s : opA=0x%08h (%s)  opB=0x%08h (%s)  tag=%0d  in_use=%b  ready=%b  avail=%b",
                $time, prefix,
                opA, opA_v ? "V" : "X",
                opB, opB_v ? "V" : "X",
                tag, in_use, ready, avail);
    endtask
      function automatic string opa_sel_str (input ALU_OPA_SELECT sel);
        case (sel)
            OPA_IS_RS1  : return "RS1";
            OPA_IS_NPC  : return "NPC";
            OPA_IS_PC   : return "PC";
            OPA_IS_ZERO : return "ZERO";
            default     : return "OPA(?)";
        endcase
    endfunction

    function automatic string opb_sel_str (input ALU_OPB_SELECT sel);
        case (sel)
            OPB_IS_RS2   : return "RS2";
            OPB_IS_I_IMM : return "I_IMM";
            OPB_IS_S_IMM : return "S_IMM";
            OPB_IS_B_IMM : return "B_IMM";
            OPB_IS_U_IMM : return "U_IMM";
            OPB_IS_J_IMM : return "J_IMM";
            default      : return "OPB(?)";
        endcase
    endfunction
    // ------------------------------------------------------------
    //  IS-stage packet
    // ------------------------------------------------------------
    task automatic show_is_packet (
        input IS_EX_PACKET pkt,
        input logic        issue_valid,
        input logic        fu_ready,
        input logic [`RS_SIZE-1:0] rs_issue_en
    );
        $display("[%0t] IS   : val=%b  tag=%0d  RS=%0d  func=%s  rd=%b wr=%b",
                $time, pkt.issue_valid, pkt.rob_tag, pkt.RS_tag,
                alu_func_str(pkt.alu_func),
                pkt.rd_mem, pkt.wr_mem);
        $display("            OPA=0x%08h  OPB=0x%08h  NPC=0x%08h  PC=0x%08h inst=0x%08h",
                pkt.OPA, pkt.OPB, pkt.NPC, pkt.PC, pkt.inst);
        $display("            issue_valid=%b  fu_ready=%b  rs_issue_enable=%b cond_branch=%b uncond_branch=%b",
                issue_valid, fu_ready, rs_issue_en, pkt.cond_branch, pkt.uncond_branch);
        $display(" opa_sel=%s  opb_sel=%s",
                opa_sel_str(pkt.opa_select), opb_sel_str(pkt.opb_select));
    endtask

    // ------------------------------------------------------------
    //  EX-stage commit packet
    // ------------------------------------------------------------
    task automatic show_ex_packet (
        input EX_CP_PACKET pkt,
        input logic        fu_busy,
        input logic        cdb_busy
    );
        $display("[%0t] EX   : val=%b  done=%b  tag=%0d  value=0x%08h take_branch=%b",
                $time, pkt.valid, pkt.done, pkt.rob_tag, pkt.value , pkt.take_branch);
        $display("            fu_busy=%b  cdb_busy=%b", fu_busy, cdb_busy);
    endtask
    task automatic show_regfile (
        input logic [31:1] [`XLEN-1:0] regs,   // <-- same layout as regfile
        input string                      hdr = "REGFILE"
    );
        $display("[%0t] === %s contents ===", $time, hdr);
        for (int i = 1; i <= 31; i++) begin
            // regs[i] is the XLEN-bit vector for x<i>
            $display("x%0d : 0x%0h", i, regs[i]);
        end
    endtask
    task automatic show_cdb_packet (
            input CDB_PACKET pkt,
            input string     prefix = "CDB"   // let caller override label
        );
            if (!pkt.valid) begin
                $display("[%0t] %s : (invalid)", $time, prefix);
            end
            else begin
                $display("[%0t] %s : tag=%0d  value=0x%08h take_branch=%b",
                        $time, prefix, pkt.tag, pkt.value, pkt.take_branch);
            end
        endtask

        function automatic string lsq_pkt_fmt (input LSQ_PACKET pkt);
        string s;

        if (!pkt.valid)          // early-out for bubble / NOP packets
            return "(LSQ_PACKET — invalid —)";

        $sformat(s,
            "(rob:%2d  data:%08h  dtag:%2d  dval:%0b  rd:%0b  wr:%0b  v:%0b rd_unsigned:%0b mem_size: %s)",
            pkt.rob_tag,
            pkt.store_data,
            pkt.store_data_tag,
            pkt.store_data_valid,
            pkt.rd_mem,
            pkt.wr_mem,
            pkt.valid,
            pkt.rd_unsigned,
            mem_size_str(pkt.mem_size));

        return s;
    endfunction

    // Show contents of a range of Unified Memory, in both hex and decimal
    task show_mem_with_decimal;
        input [31:0] start_addr;
        input [31:0] end_addr;
        int showing_data;
        begin
            $display("@@@");
            showing_data=0;
            for(int k=start_addr;k<=end_addr; k=k+1)
                if (memory.unified_memory[k] != 0) begin
                    $display("@@@ mem[%5d] = %x : %0d", k*8, memory.unified_memory[k],
                                                             memory.unified_memory[k]);
                    showing_data=1;
                end else if(showing_data!=0) begin
                    $display("@@@");
                    showing_data=0;
                end
            $display("@@@");
        end
    endtask // task show_mem_with_decimal

    task display_all_signals;
        begin
            $display("ROB contents:");
            for (int i = 0; i < 32; i++) begin
                $display("Status:%b Opcode:%b Dest:%b Value:%h", id_rob_debug[i][45:44], id_rob_debug[i][43:37], 
                    id_rob_debug[i][36:32], id_rob_debug[i][31:0]);
            end
            //display mem/ if stuff in pipeline
            $display("------------------------------------------------------------");
            $display("DCACHE DATA: ");
            dump_tag_map(); 
            print_cache();
            $display("IF STAGE CONTENT: ");
            $display("IF STALL : (BC OF DATA ARBITRATION : %0b)", if_stall);
            $display("PC=%x, VALID=%b STALL_IF= %b ICACHEDATA=%h",proc2Icache_addr, Icache_valid_out, stall_if, Icache_data_out);
            show_dispatch_inputs(clock, reset, if_id_reg, cdb_valid, cdb_tag, cdb_value, fu_busy, rs1_clear,
                rob_retire_entry, store_retire, store_tag, rob_dest_reg, rob_to_regfile_value,
                lsq_free, maptable_clear, rob_clear, rs_clear);
            $display("ID STAGE CONTENT: ");
            $display("ID INST OUT: %h", id_inst_out);
            $display("OPA SELECT =%s, OPB SELECT=%s", opa_sel_str(id_opa_select), opb_sel_str(id_opb_select));
            $display("REG INDX 1 =%d, REG INDX 2=%d", mt_to_regfile_rs1, mt_to_regfile_rs2);
            $display("ROB TAG=%d, RS1 READY=%b, mt_to_rs_tag1=%b, mt_to_rs_tag2=%b", id_tag, rs1_ready, mt_to_rs_tag1, mt_to_rs_tag2);
            $display("RS1 VALUE=%h, RS2 VALUE=%h", rs1_value, rs2_value);
            $display("RS A IN (FROM ID STAGE ): %h", rs1_opa_in);
            $display("RS B IN (FROM ID STAGE ): %h", rs1_opb_in);
            $display("ROB TO RS VALUE1=%h, ROB TO RS VALUE2=%h", rob_to_rs_value1, rob_to_rs_value2);
            show_if_packet(if_packet);
            show_if_packet(if_id_reg);
            show_id_stage   (id_tag, rs1_ready);
            $display("IS PACKEt: ");
            show_is_packet  (is_packet, issue_valid, fu_ready, rs_issue_enable);
            // $display("IS REGISTER: ");
            // show_is_packet  (is_ex_reg, issue_valid, fu_ready, rs_issue_enable);
            $display("EX take conditional ");
            $display("OPA MUX OUT=%h, OPB MUX OUT=%h", opa_mux_out, opb_mux_out);
            $display("take_conditional=%b", take_conditional);
            $display("First Ex packet");
            show_ex_packet  (ex_packet, fu_busy, cdb_busy);

            $display("ex reg");
            show_ex_packet  (ex_cp_reg, fu_busy, cdb_busy);
            show_rs_debug(rs_debug, "RS[0]");
            show_cdb_packet(cdb_packet, "CDB");
            $display("RETIRE STAGE INFORMATION: ");
            $display("LSQ IN PROGRESS : %b", lsq_op_in_progress);
            $display("RETIRE HALT =%b, RETIRE ILLEGAL=%b, RETIRE CSR OP=%b", halt_rt, illegal_rt, csr_op_rt);
            $display("RETIRE VALUE=%h, RETIRE DEST=%d, RETIRE VALID=%b RETIRE_TAG=%b", retire_value_out, retire_dest_out, retire_valid_out, retire_tag);
            //display rob full, rs1 available, dispatch ok
            $display("ROB FULL=%b RS1 AVAIL=%b DISPATCH OK=%b", rob_full, rs1_available, dispatch_ok);  

            $display("ROB READY=%b, ROB VALID=%b", rob_ready, rob_valid);
            show_rob_retire_packet(rob_retire_packet);

            $display("------------------------------------------------------------");
            $display("reg file information: ");
            show_regfile(debug_reg, "DEBUG_REG");

            $display("------------------------------------------------------------");
            $display("LSQ contents:");
            dump_lsq(lsq_out);

            $display("LSQ PACKET CONTENTS : %s ", lsq_pkt_fmt(lsq_packet));


            $display("PRIVATE ADDR contents: %s",priv_addr_pkt_fmt(priv_addr_in));
            $display("rest of the contents:");
            $display("cache in flight =%b", cache_in_flight);
            $display("store ready =%b, store tag=%d", store_ready, store_tag);
            $display("lsq free =%b", lsq_free);
            $display("head ready for mem =%b", head_ready_for_mem);
            $display("head ptr =%d", head_ptr);
            $display("tail ptr =%d", tail_ptr);
            $display("dcache data out =%h", dcache_data_out);
            $display("dcache tag =%b", dcache_tag);
            $display("dcache response =%b", dcache_response);
            $display("dcache hit =%b", dcache_hit);
            $display("dcache command =%b", dcache_command);
            $display("dcache data =%h", dcache_data);
            $display("dcache addr =%h", dcache_addr);
            $display("dcache size =%b", dcache_size);

            $display("state =%b", state);
            $display("next state =%b", next_state);

            $display("mem2dcache response =%b", mem2dcache_response);
            $display("mem2dcache data =%h", mem2dcache_data);
            $display("mem2dcache tag =%b", mem2dcache_tag);
            $display("mem tag =%d", mem_tag);
            $display("mem valid =%b", mem_valid);
            $display("CDB LSQ:");
            show_ex_packet(cdb_lsq, fu_busy, cdb_busy);

            $display("real memory modules signasl:");
            $display("proc2mem_command =%b", proc2mem_command);
            $display("proc2mem_addr =%h", proc2mem_addr);
            $display("proc2mem_data =%h", proc2mem_data);
            // $display("proc2mem_size =%s", mem_size_str(proc2mem_size));
            $display("mem2proc_response =%b", mem2proc_response);
            $display("mem2proc_data =%h", mem2proc_data);
            $display("mem2proc_tag =%b", mem2proc_tag);

            $display("Imem2proc response =%b", Imeme2proc_response);  
            $display("CURRENT MEM TAG =%b", current_mem_tag);  
            

            
            $display("------------------------------------------------------------");
        end
    endtask


    initial begin
        //$dumpvars;

        // P4 NOTE: You must keep memory loading here the same for the autograder
        //          Other things can be tampered with somewhat
        //          Definitely feel free to add new output files

        // set paramterized strings, see comment at start of module
        if ($value$plusargs("MEMORY=%s", program_memory_file)) begin
            $display("Loading memory file: %s", program_memory_file);
        end else begin
            $display("Loading default memory file: program.mem");
            program_memory_file = "program.mem";
        end
        if ($value$plusargs("WRITEBACK=%s", writeback_output_file)) begin
            $display("Using writeback output file: %s", writeback_output_file);
        end else begin
            $display("Using default writeback output file: writeback.out");
            writeback_output_file = "writeback.out";
        end
        // if ($value$plusargs("PIPELINE=%s", pipeline_output_file)) begin
        //     $display("Using pipeline output file: %s", pipeline_output_file);
        // end else begin
        //     $display("Using default pipeline output file: pipeline.out");
        //     pipeline_output_file = "pipeline.out";
        // end

        clock = 1'b0;
        reset = 1'b0;

        // Pulse the reset signal
        $display("@@\n@@\n@@  %t  Asserting System reset......", $realtime);
        reset = 1'b1;
        @(posedge clock);
        @(posedge clock);

        // store the compiled program's hex data into memory
        $readmemh(program_memory_file, memory.unified_memory);

        @(posedge clock);
        @(posedge clock);
        #1;
        // This reset is at an odd time to avoid the pos & neg clock edges

        reset = 1'b0;
        $display("@@  %t  Deasserting System reset......\n@@\n@@", $realtime);

        wb_fileno = $fopen(writeback_output_file);

        // Open the pipeline output file after throwing reset
        // open_pipeline_output_file(pipeline_output_file);
        // print_header("removed for line length");
    end


    // Count the number of posedges and number of instructions completed
    // till simulation ends
    always @(posedge clock) begin
        if(reset) begin
            clock_count <= 0;
            instr_count <= 0;
        end else begin
            clock_count <= (clock_count + 1);
            instr_count <= (instr_count + pipeline_completed_insts);
            // $display("______________POS EDGE CLOCK CYCLE!!!________________");
            // display_all_signals();
        end
        
    end



    always @(negedge clock) begin
        if(reset) begin
            $display("@@\n@@  %t : System STILL at reset, can't show anything\n@@",
                     $realtime);
            debug_counter <= 0;
        end else begin
            #2;
            // $display("______________NEGATIVE EDGE CLOCK CYCLE!!!________________");
            // display_all_signals();

            // print the pipeline debug outputs via c code to the pipeline output file
            // print_cycles();
            // $display(" @@ %h", id_opA);
            // print_stage(" ", if_inst_dbg,     if_NPC_dbg    [31:0], {31'b0,if_valid_dbg});
            // print_stage("|", if_id_inst_dbg,  if_id_NPC_dbg [31:0], {31'b0,if_id_valid_dbg});
            // print_stage("|", id_ex_inst_dbg,  id_ex_NPC_dbg [31:0], {31'b0,id_ex_valid_dbg});
            // print_stage("|", ex_mem_inst_dbg, ex_mem_NPC_dbg[31:0], {31'b0,ex_mem_valid_dbg});
            // print_stage("|", mem_wb_inst_dbg, mem_wb_NPC_dbg[31:0], {31'b0,mem_wb_valid_dbg});
            // print_reg(32'b0, pipeline_commit_wr_data[31:0],
            //     {27'b0,pipeline_commit_wr_idx}, {31'b0,pipeline_commit_wr_en});
            // print_membus({30'b0,proc2mem_command}, {28'b0,mem2proc_response},
            //     32'b0, proc2mem_addr[31:0],
            //     proc2mem_data[63:32], proc2mem_data[31:0]);
            // $fdisplay(wb_fileno, "@@  %t : System clock %d", $realtime, clock_count);

            // print register write information to the writeback output file
            if (pipeline_completed_insts > 0) begin
                if(pipeline_commit_wr_en)
                    $fdisplay(wb_fileno, "PC=%x, REG[%d]=%x",
                              pipeline_commit_NPC - 4,
                              pipeline_commit_wr_idx,
                              pipeline_commit_wr_data);
                else
                    $fdisplay(wb_fileno, "PC=%x, ---", pipeline_commit_NPC - 4);
            end

            // deal with any halting conditions
            if(pipeline_error_status != NO_ERROR || debug_counter > 50000000) begin
                $display("@@@ Unified Memory contents hex on left, decimal on right: ");
                show_mem_with_decimal(0,`MEM_64BIT_LINES - 1);
                // 8Bytes per line, 16kB total

                $display("@@  %t : System halted\n@@", $realtime);

                case(pipeline_error_status)
                    LOAD_ACCESS_FAULT:
                        $display("@@@ System halted on memory error");
                    HALTED_ON_WFI:
                        $display("@@@ System halted on WFI instruction");
                    ILLEGAL_INST:
                        $display("@@@ System halted on illegal instruction");
                    default:
                        $display("@@@ System halted on unknown error code %x",
                            pipeline_error_status);
                endcase
                $display("@@@\n@@");
                show_clk_count;
                // print_close(); // close the pipe_print output file
                $fclose(wb_fileno);
                #100 $finish;
            end
            debug_counter <= debug_counter + 1;
        end // if(reset)
    end

endmodule // testbench
