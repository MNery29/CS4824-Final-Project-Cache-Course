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

   
    logic [45:0]      id_rob_debug[31:0];

    logic stall_if;
    logic [`XLEN-1:0] proc2Icache_addr;
    logic             Icache_valid_out;
    logic [63:0] Icache_data_out;

    logic [`ROB_TAG_BITS-1:0] id_tag;
    logic rs1_ready;


    //IS stage debugging wires
    IS_EX_PACKET is_packet;
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
    logic [4:0] store_tag;

    logic [4:0] rob_dest_reg;
    logic [31:0] rob_to_regfile_value;

    logic lsq_free;

    logic maptable_clear;
    logic rob_clear;
    logic rs_clear;



    //EX stage debugging wires
    EX_CP_PACKET ex_cp_reg;
    EX_CP_PACKET ex_packet;
    logic fu_busy;
    logic cdb_busy; //this will stall the RS issue if ex stage is busy / full

    IF_ID_PACKET if_packet;
    IF_ID_PACKET if_id_reg;

    logic rob_full;
    logic rs1_available;
    logic dispatch_ok;

    logic [73:0] rs_debug;

    CDB_PACKET cdb_packet;

    



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

        .pipeline_completed_insts (pipeline_completed_insts),
        .pipeline_error_status    (pipeline_error_status),
        .pipeline_commit_wr_data  (pipeline_commit_wr_data),
        .pipeline_commit_wr_idx   (pipeline_commit_wr_idx),
        .pipeline_commit_wr_en    (pipeline_commit_wr_en),
        .pipeline_commit_NPC      (pipeline_commit_NPC),

        .id_rob_debug          (id_rob_debug),
        .Icache_valid_out     (Icache_valid_out),
        .proc2Icache_addr     (proc2Icache_addr),
        .stall_if             (stall_if),
        .Icache_data_out       (Icache_data_out),

        .id_tag               (id_tag),
        .rs1_ready            (rs1_ready),

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

        .if_packet            (if_packet),
        .if_id_reg            (if_id_reg),

        .rob_full             (rob_full),
        .rs1_available        (rs1_available),
        .dispatch_ok          (dispatch_ok),

        .id_rs_debug             (rs_debug),

        .ex_packet            (ex_packet),

        .cdb_packet            (cdb_packet)


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

    task automatic show_if_packet (
        input IF_ID_PACKET pkt
    );
        if (!pkt.valid) begin
            $display("[%0t] IF   : (stall/invalid)", $time);
        end
        else begin
            $display("[%0t] IF   : PC = 0x%08h  NPC = 0x%08h  inst = 0x%08h",
                    $time, pkt.PC, pkt.NPC, pkt.inst);
        end
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
        $display("            OPA=0x%08h  OPB=0x%08h  NPC=0x%08h  inst=0x%08h",
                pkt.OPA, pkt.OPB, pkt.NPC, pkt.inst);
        $display("            issue_valid=%b  fu_ready=%b  rs_issue_enable=%b is_branch=%b",
                issue_valid, fu_ready, rs_issue_en, pkt.is_branch);
    endtask

    // ------------------------------------------------------------
    //  EX-stage commit packet
    // ------------------------------------------------------------
    task automatic show_ex_packet (
        input EX_CP_PACKET pkt,
        input logic        fu_busy,
        input logic        cdb_busy
    );
        $display("[%0t] EX   : val=%b  done=%b  tag=%0d  value=0x%08h",
                $time, pkt.valid, pkt.done, pkt.rob_tag, pkt.value);
        $display("            fu_busy=%b  cdb_busy=%b", fu_busy, cdb_busy);
    endtask
    task automatic show_cdb_packet (
        input CDB_PACKET pkt,
        input string     prefix = "CDB"   // let caller override label
    );
        if (!pkt.valid) begin
            $display("[%0t] %s : (invalid)", $time, prefix);
        end
        else begin
            $display("[%0t] %s : tag=%0d  value=0x%08h",
                    $time, prefix, pkt.tag, pkt.value);
        end
    endtask

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
        end
    end


    always @(negedge clock) begin
        if(reset) begin
            $display("@@\n@@  %t : System STILL at reset, can't show anything\n@@",
                     $realtime);
            debug_counter <= 0;
        end else begin
            #2;
            $display("______________NEW CLOCK CYCLE________________");
            $display("ROB contents:");
            for (int i = 0; i < 32; i++) begin
                $display("Status:%b Opcode:%b Dest:%b Value:%h", id_rob_debug[i][45:44], id_rob_debug[i][43:37], 
                    id_rob_debug[i][36:32], id_rob_debug[i][31:0]);
            end
            //display mem/ if stuff in pipeline

            $display("IF STAGE CONTENT: ");
            $display("PC=%x, VALID=%b STALL_IF= %b ICACHEDATA=%h",proc2Icache_addr, Icache_valid_out, stall_if, Icache_data_out);
            
            show_if_packet(if_packet);
            show_if_packet(if_id_reg);
            //display what the ID packet is seeing 
            show_dispatch_inputs(
                clock, reset, if_id_reg,
                cdb_valid, cdb_tag, cdb_value,
                fu_busy, rs1_clear,
                rob_retire_entry,
                store_retire, store_tag,
                rob_dest_reg, rob_to_regfile_value,
                lsq_free,
                maptable_clear, rob_clear, rs_clear
            );



            show_id_stage   (id_tag, rs1_ready);
            show_is_packet  (is_packet, issue_valid, fu_ready, rs_issue_enable);
            $display("First Ex packet");
            show_ex_packet  (ex_packet, fu_busy, cdb_busy);
            $display("ex reg");
            show_ex_packet  (ex_cp_reg, fu_busy, cdb_busy);
            show_rs_debug(rs_debug, "RS[0]");
            show_cdb_packet(cdb_packet, "CDB");
            //display rob full, rs1 available, dispatch ok
            $display("ROB FULL=%b RS1 AVAIL=%b DISPATCH OK=%b", rob_full, rs1_available, dispatch_ok);
            $display("------------------------------------------------------------");


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
