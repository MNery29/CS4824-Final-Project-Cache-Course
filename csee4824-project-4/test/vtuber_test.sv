/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  vtuber_test.sv                                      //
//                                                                     //
//  Description :  Testbench module for the verisimple pipeline        //
//                 for the VisUal TestBencheR                          //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

extern void initcurses(int,int,int,int,int,int,int,int,int,int);
extern void flushpipe();
extern void waitforresponse();
extern void initmem();
extern int get_instr_at_pc(int);
extern int not_valid_pc(int);

module testbench;
    // used to parameterize which file is loaded into memory
    // "./vis_simv" still just uses program.mem
    // but now "./simv +MEMORY=<my_program>.mem" loads <my_program>.mem instead
    string program_memory_file;

    // Registers and wires used in the testbench
    logic        clock;
    logic        reset;
    logic [31:0] clock_count;
    logic [31:0] instr_count;
    int          wb_fileno;
    logic [63:0] debug_counter; // counter used for when pipeline infinite loops, forces termination

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

    logic [`XLEN-1:0] if_NPC_dbg;
    logic [31:0]      if_inst_dbg;
    logic             if_valid_dbg;
    logic [`XLEN-1:0] if_id_NPC_dbg;
    logic [31:0]      if_id_inst_dbg;
    logic             if_id_valid_dbg;
    logic [`XLEN-1:0] id_ex_NPC_dbg;
    logic [31:0]      id_ex_inst_dbg;
    logic             id_ex_valid_dbg;
    logic [`XLEN-1:0] ex_mem_NPC_dbg;
    logic [31:0]      ex_mem_inst_dbg;
    logic             ex_mem_valid_dbg;
    logic [`XLEN-1:0] mem_wb_NPC_dbg;
    logic [31:0]      mem_wb_inst_dbg;
    logic             mem_wb_valid_dbg;


    // Instantiate the Pipeline
    pipeline pipeline_0 (
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
        .proc2mem_size    (proc2mem_size),

        .pipeline_completed_insts (pipeline_completed_insts),
        .pipeline_error_status    (pipeline_error_status),
        .pipeline_commit_wr_data  (pipeline_commit_wr_data),
        .pipeline_commit_wr_idx   (pipeline_commit_wr_idx),
        .pipeline_commit_wr_en    (pipeline_commit_wr_en),
        .pipeline_commit_NPC      (pipeline_commit_NPC),

        .if_NPC_dbg       (if_NPC_dbg),
        .if_inst_dbg      (if_inst_dbg),
        .if_valid_dbg     (if_valid_dbg),
        .if_id_NPC_dbg    (if_id_NPC_dbg),
        .if_id_inst_dbg   (if_id_inst_dbg),
        .if_id_valid_dbg  (if_id_valid_dbg),
        .id_ex_NPC_dbg    (id_ex_NPC_dbg),
        .id_ex_inst_dbg   (id_ex_inst_dbg),
        .id_ex_valid_dbg  (id_ex_valid_dbg),
        .ex_mem_NPC_dbg   (ex_mem_NPC_dbg),
        .ex_mem_inst_dbg  (ex_mem_inst_dbg),
        .ex_mem_valid_dbg (ex_mem_valid_dbg),
        .mem_wb_NPC_dbg   (mem_wb_NPC_dbg),
        .mem_wb_inst_dbg  (mem_wb_inst_dbg),
        .mem_wb_valid_dbg (mem_wb_valid_dbg)
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


    // Count the number of posedges and number of instructions completed
    // till simulation ends
    always @(posedge clock) begin
        if (reset) begin
            clock_count <= 0;
            instr_count <= 0;
        end else begin
            clock_count <= (clock_count + 1);
            instr_count <= (instr_count + pipeline_completed_insts);
        end
    end


    initial begin
        clock = 0;
        reset = 0;

        // Call to initialize visual debugger
        // *Note that after this, all stdout output goes to visual debugger*
        // each argument is number of registers/signals for the group
        initcurses(
            5,  // IF
            4,  // IF/ID
            13, // ID
            17, // ID/EX
            4,  // EX
            14, // EX/MEM
            5,  // MEM
            9,  // MEM/WB
            3,  // WB
            2   // Miscellaneous
        );

        // Pulse the reset signal
        reset = 1'b1;
        @(posedge clock);
        @(posedge clock);

        // set paramterized strings, see comment at start of module
        if ($value$plusargs("MEMORY=%s", program_memory_file)) begin
            $display("Loading memory file: %s", program_memory_file);
        end else begin
            $display("Loading default memory file: program.mem");
            program_memory_file = "program.mem";
        end

        // Read program contents into memory array
        $readmemh(program_memory_file, memory.unified_memory);

        @(posedge clock);
        @(posedge clock);
        #1;
        // This reset is at an odd time to avoid the pos & neg clock edges
        reset = 1'b0;
    end


    always @(negedge clock) begin
        if (!reset) begin
            #2;

            // deal with any halting conditions
            if (pipeline_error_status!=NO_ERROR) begin
                #100
                $display("\nDONE\n");
                waitforresponse();
                flushpipe();
                $finish;
            end
        end
    end


    // This block is where we dump all of the signals that we care about to
    // the visual debugger.  Notice this happens at *every* clock edge.
    always @(clock) begin
        #2;

        // Dump clock and time onto stdout
        $display("c%h%7.0d",clock,clock_count);
        $display("t%8.0f",$time);
        $display("z%h",reset);

        // Dump register file contents
        $write("a");
        for(int i = 0; i < 32; i=i+1) begin
            $write("%h", pipeline_0.stage_id_0.regfile_0.registers[i]);
        end
        $display("");

        // Dump instructions and their validity for each stage
        $write("p");
        $write("%h%h%h%h%h%h%h%h%h%h ",
               if_inst_dbg,      if_valid_dbg,
               if_id_inst_dbg,   if_id_valid_dbg,
               id_ex_inst_dbg,   id_ex_valid_dbg,
               ex_mem_inst_dbg,  ex_mem_valid_dbg,
               mem_wb_inst_dbg,  mem_wb_valid_dbg);
        $display("");

        // Dump interesting register/signal contents onto stdout
        // format is "<reg group prefix><name> <width in hex chars>:<data>"
        // Current register groups (and prefixes) are:
        // f: IF   d: ID   e: EX   m: MEM    w: WB  v: misc. reg
        // g: IF/ID   h: ID/EX  i: EX/MEM  j: MEM/WB

        // IF signals (5) - prefix 'f'
        $display("fNPC 8:%h",         pipeline_0.if_packet.NPC);
        $display("finst 8:%h",        pipeline_0.if_packet.inst);
        $display("fImem_addr 8:%h",   pipeline_0.stage_if_0.proc2Imem_addr);
        $display("fPC_reg 8:%h",      pipeline_0.stage_if_0.PC_reg);
        $display("fvalid 1:%h",       pipeline_0.if_packet.valid);

        // IF/ID signals (4) - prefix 'g'
        $display("genable 1:%h",      pipeline_0.if_id_enable);
        $display("gNPC 16:%h",        pipeline_0.if_id_reg.NPC);
        $display("ginst 8:%h",        pipeline_0.if_id_reg.inst);
        $display("gvalid 1:%h",       pipeline_0.if_id_reg.valid);

        // ID signals (13) - prefix 'd'
        $display("drs1 8:%h",         pipeline_0.id_packet.rs1_value);
        $display("drs2 8:%h",         pipeline_0.id_packet.rs2_value);
        $display("ddest_reg 2:%h",    pipeline_0.id_packet.dest_reg_idx);
        $display("drd_mem 1:%h",      pipeline_0.id_packet.rd_mem);
        $display("dwr_mem 1:%h",      pipeline_0.id_packet.wr_mem);
        $display("dopa_sel 1:%h",     pipeline_0.id_packet.opa_select);
        $display("dopb_sel 1:%h",     pipeline_0.id_packet.opb_select);
        $display("dalu_func 2:%h",    pipeline_0.id_packet.alu_func);
        $display("dcond_br 1:%h",     pipeline_0.id_packet.cond_branch);
        $display("duncond_br 1:%h",   pipeline_0.id_packet.uncond_branch);
        $display("dhalt 1:%h",        pipeline_0.id_packet.halt);
        $display("dillegal 1:%h",     pipeline_0.id_packet.illegal);
        $display("dvalid 1:%h",       pipeline_0.id_packet.valid);

        // ID/EX signals (17) - prefix 'h'
        $display("henable 1:%h",      pipeline_0.id_ex_enable);
        $display("hNPC 16:%h",        pipeline_0.id_ex_reg.NPC);
        $display("hinst 8:%h",        pipeline_0.id_ex_reg.inst);
        $display("hrs1 8:%h",         pipeline_0.id_ex_reg.rs1_value);
        $display("hrs2 8:%h",         pipeline_0.id_ex_reg.rs2_value);
        $display("hdest_reg 2:%h",    pipeline_0.id_ex_reg.dest_reg_idx);
        $display("hrd_mem 1:%h",      pipeline_0.id_ex_reg.rd_mem);
        $display("hwr_mem 1:%h",      pipeline_0.id_ex_reg.wr_mem);
        $display("hopa_sel 1:%h",     pipeline_0.id_ex_reg.opa_select);
        $display("hopb_sel 1:%h",     pipeline_0.id_ex_reg.opb_select);
        $display("halu_func 2:%h",    pipeline_0.id_ex_reg.alu_func);
        $display("hcond_br 1:%h",     pipeline_0.id_ex_reg.cond_branch);
        $display("huncond_br 1:%h",   pipeline_0.id_ex_reg.uncond_branch);
        $display("hhalt 1:%h",        pipeline_0.id_ex_reg.halt);
        $display("hillegal 1:%h",     pipeline_0.id_ex_reg.illegal);
        $display("hvalid 1:%h",       pipeline_0.id_ex_reg.valid);
        $display("hcsr_op 1:%h",      pipeline_0.id_ex_reg.csr_op);

        // EX signals (4) - prefix 'e'
        $display("eopa_mux 8:%h",     pipeline_0.stage_ex_0.opa_mux_out);
        $display("eopb_mux 8:%h",     pipeline_0.stage_ex_0.opb_mux_out);
        $display("ealu_result 8:%h",  pipeline_0.ex_packet.alu_result);
        $display("etake_branch 1:%h", pipeline_0.ex_packet.take_branch);

        // EX/MEM signals (14) - prefix 'i'
        $display("ienable 1:%h",      pipeline_0.ex_mem_enable);
        $display("iNPC 8:%h",         pipeline_0.ex_mem_reg.NPC);
        $display("iinst 8:%h",        pipeline_0.ex_mem_inst_dbg);
        $display("irs2 8:%h",         pipeline_0.ex_mem_reg.rs2_value);
        $display("ialu_result 8:%h",  pipeline_0.ex_mem_reg.alu_result);
        $display("idest_reg 2:%h",    pipeline_0.ex_mem_reg.dest_reg_idx);
        $display("ird_mem 1:%h",      pipeline_0.ex_mem_reg.rd_mem);
        $display("iwr_mem 1:%h",      pipeline_0.ex_mem_reg.wr_mem);
        $display("itake_branch 1:%h", pipeline_0.ex_mem_reg.take_branch);
        $display("ihalt 1:%h",        pipeline_0.ex_mem_reg.halt);
        $display("iillegal 1:%h",     pipeline_0.ex_mem_reg.illegal);
        $display("ivalid 1:%h",       pipeline_0.ex_mem_reg.valid);
        $display("icsr_op 1:%h",      pipeline_0.ex_mem_reg.csr_op);
        // haven't updated VTUBER to use rd_unsigned yet
        $display("imem_size 1:%h",    {pipeline_0.ex_mem_reg.rd_unsigned, pipeline_0.ex_mem_reg.mem_size});

        // MEM signals (5) - prefix 'm'
        $display("mmem_data 16:%h",   pipeline_0.mem2proc_data);
        $display("mmem_result 8:%h",  pipeline_0.mem_wb_reg.result);
        $display("m2Dmem_data 16:%h", pipeline_0.proc2mem_data);
        $display("m2Dmem_addr 8:%h",  pipeline_0.proc2Dmem_addr);
        $display("m2Dmem_cmd 1:%h",   pipeline_0.proc2Dmem_command);

        // MEM/WB signals (9) - prefix 'j'
        $display("jenable 1:%h",      pipeline_0.mem_wb_enable);
        $display("jNPC 8:%h",         pipeline_0.mem_wb_NPC_dbg);
        $display("jinst 8:%h",        pipeline_0.mem_wb_inst_dbg);
        $display("jresult 8:%h",      pipeline_0.mem_wb_reg.result);
        $display("jdest_reg 2:%h",    pipeline_0.mem_wb_reg.dest_reg_idx);
        $display("jtake_branch 1:%h", pipeline_0.mem_wb_reg.take_branch);
        $display("jhalt 1:%h",        pipeline_0.mem_wb_reg.halt);
        $display("jillegal 1:%h",     pipeline_0.mem_wb_reg.illegal);
        $display("jvalid 1:%h",       pipeline_0.mem_wb_reg.valid);

        // WB signals (3) - prefix 'w'
        $display("wwr_data 8:%h",     pipeline_0.wb_regfile_data);
        $display("wwr_idx 2:%h",      pipeline_0.wb_regfile_idx);
        $display("wwr_en 1:%h",       pipeline_0.wb_regfile_en);

        // Misc signals(2) - prefix 'v'
        $display("vcompleted 1:%h",   pipeline_completed_insts);
        $display("vpipe_err 1:%h",    pipeline_error_status);

        // must come last
        $display("break");

        // This is a blocking call to allow the debugger to control when we
        // advance the simulation
        waitforresponse();
    end

endmodule // module testbench
