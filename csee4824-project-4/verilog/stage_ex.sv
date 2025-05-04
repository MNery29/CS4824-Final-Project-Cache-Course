/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_ex.sv                                         //
//                                                                     //
//  Description :  instruction execute (EX) stage of the pipeline;     //
//                 given the instruction command code CMD, select the  //
//                 proper input A and B for the ALU, compute the       //
//                 result, and compute the condition for branches, and //
//                 pass all the results down the pipeline.             //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"
`include "verilog/mult.sv"
`include "verilog/mult_stage.sv"

// ALU: computes the result of FUNC applied with operands A and B
// This module is purely combinational
module alu (
    input [`XLEN-1:0] opa,
    input [`XLEN-1:0] opb,
    ALU_FUNC          func,

    output logic [`XLEN-1:0] result
);




    logic signed [`XLEN-1:0]   signed_opa, signed_opb;
    logic signed [2*`XLEN-1:0] signed_mul, mixed_mul;
    logic        [2*`XLEN-1:0] unsigned_mul;

    assign signed_opa   = opa;
    assign signed_opb   = opb;

    // We let verilog do the full 32-bit multiplication for us.
    // This gives a large clock period.
    // You will replace this with your pipelined multiplier in project 4.
    assign signed_mul   = signed_opa * signed_opb;
    assign unsigned_mul = opa * opb;
    assign mixed_mul    = signed_opa * opb;

    always_comb begin
        case (func)
            ALU_ADD:    result = opa + opb;
            ALU_SUB:    result = opa - opb;
            ALU_AND:    result = opa & opb;
            ALU_SLT:    result = signed_opa < signed_opb;
            ALU_SLTU:   result = opa < opb;
            ALU_OR:     result = opa | opb;
            ALU_XOR:    result = opa ^ opb;
            ALU_SRL:    result = opa >> opb[4:0];
            ALU_SLL:    result = opa << opb[4:0];
            ALU_SRA:    result = signed_opa >>> opb[4:0]; // arithmetic from logical shift
            ALU_MUL:    result = signed_mul[`XLEN-1:0];
            ALU_MULH:   result = signed_mul[2*`XLEN-1:`XLEN];
            ALU_MULHSU: result = mixed_mul[2*`XLEN-1:`XLEN];
            ALU_MULHU:  result = unsigned_mul[2*`XLEN-1:`XLEN];

            default:    result = `XLEN'hfacebeec;  // here to prevent latches
        endcase
    end

endmodule // alu
// Conditional branch module: compute whether to take conditional branches
// This module is purely combinational
module conditional_branch (
    input [2:0]       func, // Specifies which condition to check
    input [`XLEN-1:0] rs1,  // Value to check against condition
    input [`XLEN-1:0] rs2,

    output logic take // True/False condition result
);

    logic signed [`XLEN-1:0] signed_rs1, signed_rs2;
    assign signed_rs1 = rs1;
    assign signed_rs2 = rs2;
    always_comb begin
        case (func)
            3'b000:  take = signed_rs1 == signed_rs2; // BEQ
            3'b001:  take = signed_rs1 != signed_rs2; // BNE
            3'b100:  take = signed_rs1 < signed_rs2;  // BLT
            3'b101:  take = signed_rs1 >= signed_rs2; // BGE
            3'b110:  take = rs1 < rs2;                // BLTU
            3'b111:  take = rs1 >= rs2;               // BGEU
            default: take = `FALSE;
        endcase
    end

endmodule // conditional_branch


module stage_ex (
    input logic clk, rst,
    input IS_EX_PACKET is_ex_reg[2:0], //One packet per FU, ALU0/1 and MULT
    input cdb_packet_busy,

    // output EX_MEM_PACKET ex_packet,
    output logic take_conditional_alu0,
    output logic take_conditional_alu1,
    output EX_CP_PACKET ex_cp_packet,
    //broad cast value + tag to cbd, so reorder buffer can be updated
    output priv_addr_packet priv_addr_out,

    //Output FU busy signals: 
    output logic [2:0] fu_busy_signals,
    output logic mult_done,

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
    //functional unit inputs
    logic [`XLEN-1:0] opa_mux_out0, opb_mux_out0;
    logic [`XLEN-1:0] opa_mux_out1, opb_mux_out1;
    logic [`XLEN-1:0] opa_mux_out2, opb_mux_out2;
    priv_addr_packet next_priv_addr_out;


    // logic [`XLEN-1:0] opa_mux_out, opb_mux_out;
    logic [31:0] next_val;
    logic take_branch;
    logic is_branch;
    // logic take_conditional;
    logic is_mem_op;

    //MULT logic inputs 
    logic [63:0] mult_result;
    logic mult_start;
    logic is_mult_inst;  

    // logic hold_mult_valid; 
    // logic hold_alu0_valid;
    // logic hold_alu1_valid;


    EX_CP_PACKET next_ex_cp_packet;

    // EX_CP_PACKET hold_alu0_pkt;
    // EX_CP_PACKET hold_alu1_pkt;
    // EX_CP_PACKET hold_mult_pkt;
    // EX_CP_PACKET tmp_alu0_pkt;
    // EX_CP_PACKET tmp_alu1_pkt;
    // EX_CP_PACKET tmp_mult_pkt;
    logic hold_alu0_is_mem_op;
    logic hold_alu1_is_mem_op;
    logic hold_mult_is_mem_op;
    EX_CP_PACKET new_pkt;


    //ALU Result Signals
    logic [`XLEN-1:0] alu0_result;
    logic [`XLEN-1:0] alu1_result;
    
    //Logic for which FU to select from: 
    logic [1:0] fu_index;
    always_comb begin
        next_ex_cp_packet = '{default: 0};
        if (!cdb_packet_busy && (tmp_mult_pkt.valid || (hold_mult_valid && hold_mult_pkt.valid)) ) begin //MULT 
            fu_index = 2'd2;
            if (tmp_mult_pkt.valid) begin
                next_ex_cp_packet = tmp_mult_pkt;
            end
            else begin
                next_ex_cp_packet = hold_mult_pkt;
            end
        end
        else if (!cdb_packet_busy && ((tmp_alu0_pkt.valid && !(is_ex_reg[0].rd_mem || is_ex_reg[0].wr_mem) )
                            || (hold_alu0_valid && hold_alu0_pkt.valid && !hold_alu0_is_mem_op))) begin //ALU0
            fu_index = 2'd0;
            if (tmp_alu0_pkt.valid) begin
                next_ex_cp_packet = tmp_alu0_pkt;
            end
            else begin
                next_ex_cp_packet = hold_alu0_pkt;
            end
        end
        else if (!cdb_packet_busy && ((tmp_alu1_pkt.valid && !(is_ex_reg[1].rd_mem || is_ex_reg[1].wr_mem))
                            || (hold_alu1_valid && hold_alu1_pkt.valid && !hold_alu1_is_mem_op))) begin //ALU1
            fu_index = 2'd1;
            if (tmp_alu1_pkt.valid) begin
                next_ex_cp_packet = tmp_alu1_pkt;
            end
            else begin
                next_ex_cp_packet = hold_alu1_pkt;
            end
        end
        else begin
            fu_index = 2'b11; // default (NOP)
        end
    end

    logic [1:0] fu_index_priv_addr;

    always_comb begin
        next_priv_addr_out = '{default: 0};
        if (((tmp_alu0_pkt.valid && (is_ex_reg[0].rd_mem || is_ex_reg[0].wr_mem) )
                            || (hold_alu0_valid && hold_alu0_pkt.valid && hold_alu0_is_mem_op))) begin//ALU0
            fu_index_priv_addr = 2'd0;
            next_priv_addr_out.valid = tmp_alu0_pkt.valid;
            next_priv_addr_out.tag   = tmp_alu0_pkt.rob_tag;
            next_priv_addr_out.addr  = tmp_alu0_pkt.value;
        end
        else if (((tmp_alu1_pkt.valid && (is_ex_reg[1].rd_mem || is_ex_reg[1].wr_mem))
                            || (hold_alu1_valid && hold_alu1_pkt.valid && hold_alu1_is_mem_op))) begin //ALU1
            fu_index_priv_addr = 2'd1;
            next_priv_addr_out.valid = tmp_alu1_pkt.valid;
            next_priv_addr_out.tag   = tmp_alu1_pkt.rob_tag;
            next_priv_addr_out.addr  = tmp_alu1_pkt.value;
        end
        else begin
            fu_index_priv_addr = 2'b11; // default (NOP)
        end
    end

    // assign ex_packet.dest_reg_idx = id_ex_reg.dest_reg_idx;

    // assign ex_packet.halt         = id_ex_reg.halt;
    // assign ex_packet.illegal      = id_ex_reg.illegal;
    // assign ex_packet.csr_op       = id_ex_reg.csr_op;
    // assign ex_cp_packet.valid        = is_ex_reg.issue_valid;

    // Break out the signed/unsigned bit and memory read/write size
    // assign ex_cp_packet.rd_unsigned  = is_ex_reg.inst.r.funct3[2]; // 1 if unsigned, 0 if signed
    // assign ex_cp_packet.mem_size     = MEM_SIZE'(is_ex_reg.inst.r.funct3[1:0]);

    //PACKETS FOR EACH UNIT: 


    //check if issue is a valid op and the function is a mult instruction
    //if so start mult. 
    
    logic mult_started;
    logic [4:0] mult_tag;

    logic [31:0] mult_opa;
    logic [31:0] mult_opb;
   

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            mult_started <= 0;
            // hold_valid <= 1'b0;
            // hold_pkt   <= '{default:0};
            hold_alu0_valid <= 1'b0;
            hold_alu1_valid <= 1'b0;
            hold_mult_valid <= 1'b0;
            hold_alu0_pkt <= '{default:0};
            hold_alu1_pkt <= '{default:0};
            hold_mult_pkt <= '{default:0};
            hold_alu0_is_mem_op <= 1'b0;
            hold_alu1_is_mem_op <= 1'b0;
            hold_mult_is_mem_op <= 1'b0;

            priv_addr_out <= '{default:0};
            ex_cp_packet <= '{default:0};
            
            mult_tag <= 5'b0;
        end
        else begin
            priv_addr_out <= next_priv_addr_out;
            ex_cp_packet <= next_ex_cp_packet;
            if (mult_start && !mult_done) begin
                mult_started <= 1;
                mult_tag <= is_ex_reg[2].rob_tag;
            end
            else if (mult_done) begin
                mult_started <= 0;
            end
            else begin
                mult_started <= 0;
                mult_opa <= opa_mux_out2;
                mult_opb <= opb_mux_out2;
            end

            hold_alu0_is_mem_op <= hold_alu0_valid ? hold_alu0_is_mem_op : (is_ex_reg[0].rd_mem || is_ex_reg[0].wr_mem);
            hold_alu1_is_mem_op <= hold_alu1_valid ? hold_alu1_is_mem_op : (is_ex_reg[1].rd_mem || is_ex_reg[1].wr_mem);
            hold_mult_is_mem_op <= hold_mult_valid ? hold_mult_is_mem_op : (is_ex_reg[2].rd_mem || is_ex_reg[2].wr_mem);

            if (hold_alu0_valid) begin
                hold_alu0_pkt <= hold_alu0_pkt;
            end
            else begin
                hold_alu0_pkt <= tmp_alu0_pkt;
            end
            if (hold_alu1_valid) begin
                hold_alu1_pkt <= hold_alu1_pkt;
            end
            else begin
                hold_alu1_pkt <= tmp_alu1_pkt;
            end
            if (hold_mult_valid) begin
                hold_mult_pkt <= hold_mult_pkt;
            end
            else begin
                hold_mult_pkt <= tmp_mult_pkt;
            end

            if ((tmp_alu0_pkt.valid || (hold_alu0_pkt.valid && hold_alu0_valid)) &&
                fu_index != 2'd0 && fu_index_priv_addr != 2'd0) begin
                    hold_alu0_valid <= 1'b1;
                end
            else begin
                hold_alu0_valid <= 1'b0;
            end

            if ((tmp_alu1_pkt.valid || (hold_alu1_pkt.valid && hold_alu1_valid)) &&
                fu_index != 2'd1 && fu_index_priv_addr != 2'd1) begin
                    hold_alu1_valid <= 1'b1;
                end
            else begin
                hold_alu1_valid <= 1'b0;
            end

            if ((tmp_mult_pkt.valid || (hold_mult_pkt.valid && hold_mult_valid)) &&
                fu_index != 2'd2) begin
                    hold_mult_valid <= 1'b1;
                end
            else begin
                hold_mult_valid <= 1'b0;
            end
        end
    end

    assign mult_start = (is_ex_reg[2].issue_valid && is_mult_inst) && !mult_started;




    assign is_mult_inst = (is_ex_reg[2].alu_func inside{ALU_MUL, ALU_MULH, ALU_MULHSU, ALU_MULHU});

    assign fu_busy_signals[0] = is_ex_reg[0].issue_valid || hold_alu0_valid ;                  // ALU0 is busy if it's issued
    assign fu_busy_signals[1] = is_ex_reg[1].issue_valid || hold_alu1_valid;                  // ALU1 busy
    assign fu_busy_signals[2] = (mult_started || is_ex_reg[2].issue_valid) || hold_mult_valid;     // MULT busy if issued and not done



    // ultimate "take branch" signal:
    // unconditional, or conditional and the condition is true

    // no branch 
    // assign ex_packet.take_branch = id_ex_reg.uncond_branch || (id_ex_reg.cond_branch && take_conditional);


    //ALU0 opA and opB selection: 
    always_comb begin
        case (is_ex_reg[0].opa_select)
            OPA_IS_RS1:  opa_mux_out0 = is_ex_reg[0].OPA;
            OPA_IS_NPC:  opa_mux_out0 = is_ex_reg[0].NPC;
            OPA_IS_PC:   opa_mux_out0 = is_ex_reg[0].PC;
            OPA_IS_ZERO: opa_mux_out0 = 0;
            default:     opa_mux_out0 = `XLEN'hdeadface;
        endcase
    end

    always_comb begin
        case (is_ex_reg[0].opb_select)
            OPB_IS_RS2:   opb_mux_out0 = is_ex_reg[0].OPB;
            OPB_IS_I_IMM: opb_mux_out0 = `RV32_signext_Iimm(is_ex_reg[0].inst);
            OPB_IS_S_IMM: opb_mux_out0 = `RV32_signext_Simm(is_ex_reg[0].inst);
            OPB_IS_B_IMM: opb_mux_out0 = `RV32_signext_Bimm(is_ex_reg[0].inst);
            OPB_IS_U_IMM: opb_mux_out0 = `RV32_signext_Uimm(is_ex_reg[0].inst);
            OPB_IS_J_IMM: opb_mux_out0 = `RV32_signext_Jimm(is_ex_reg[0].inst);
            default:      opb_mux_out0 = `XLEN'hfacefeed;
        endcase
    end

    // ALU1 operand selection
    always_comb begin
        case (is_ex_reg[1].opa_select)
            OPA_IS_RS1:  opa_mux_out1 = is_ex_reg[1].OPA;
            OPA_IS_NPC:  opa_mux_out1 = is_ex_reg[1].NPC;
            OPA_IS_PC:   opa_mux_out1 = is_ex_reg[1].PC;
            OPA_IS_ZERO: opa_mux_out1 = 0;
            default:     opa_mux_out1 = `XLEN'hdeadface;
        endcase
    end

    always_comb begin
        case (is_ex_reg[1].opb_select)
            OPB_IS_RS2:   opb_mux_out1 = is_ex_reg[1].OPB;
            OPB_IS_I_IMM: opb_mux_out1 = `RV32_signext_Iimm(is_ex_reg[1].inst);
            OPB_IS_S_IMM: opb_mux_out1 = `RV32_signext_Simm(is_ex_reg[1].inst);
            OPB_IS_B_IMM: opb_mux_out1 = `RV32_signext_Bimm(is_ex_reg[1].inst);
            OPB_IS_U_IMM: opb_mux_out1 = `RV32_signext_Uimm(is_ex_reg[1].inst);
            OPB_IS_J_IMM: opb_mux_out1 = `RV32_signext_Jimm(is_ex_reg[1].inst);
            default:      opb_mux_out1 = `XLEN'hfacefeed;
        endcase
    end

    // MULT operand selection
    always_comb begin
        case (is_ex_reg[2].opa_select)
            OPA_IS_RS1:  opa_mux_out2 = is_ex_reg[2].OPA;
            OPA_IS_NPC:  opa_mux_out2 = is_ex_reg[2].NPC;
            OPA_IS_PC:   opa_mux_out2 = is_ex_reg[2].PC;
            OPA_IS_ZERO: opa_mux_out2 = 0;
            default:     opa_mux_out2 = `XLEN'hdeadface;
        endcase
    end

    always_comb begin
        case (is_ex_reg[2].opb_select)
            OPB_IS_RS2:   opb_mux_out2 = is_ex_reg[2].OPB;
            OPB_IS_I_IMM: opb_mux_out2 = `RV32_signext_Iimm(is_ex_reg[2].inst);
            OPB_IS_S_IMM: opb_mux_out2 = `RV32_signext_Simm(is_ex_reg[2].inst);
            OPB_IS_B_IMM: opb_mux_out2 = `RV32_signext_Bimm(is_ex_reg[2].inst);
            OPB_IS_U_IMM: opb_mux_out2 = `RV32_signext_Uimm(is_ex_reg[2].inst);
            OPB_IS_J_IMM: opb_mux_out2 = `RV32_signext_Jimm(is_ex_reg[2].inst);
            default:      opb_mux_out2 = `XLEN'hfacefeed;
        endcase
    end


    //selecting what next_val to forward. 
    always_comb begin
        if (is_ex_reg[2].issue_valid)
            next_val = mult_result;
        else if (is_ex_reg[0].issue_valid)
            next_val = alu0_result;
        else if (is_ex_reg[1].issue_valid)
            next_val = alu1_result;
        else
            next_val = 32'h0;
    end


    // Instantiate the ALU
    alu alu_0 (
        // Inputs
        .opa(opa_mux_out0),
        .opb(opb_mux_out0),
        .func(is_ex_reg[0].alu_func),

        // Output
        .result(alu0_result)
    );

    alu alu_1 (
        // Inputs
        .opa(opa_mux_out1),
        .opb(opb_mux_out1),
        .func(is_ex_reg[1].alu_func),

        // Output
        .result(alu1_result)
    );




    //instantiate the MULT unit 
    mult mult_0 (
        //inputs
        .clock(clk),
        .reset(rst),
        .mcand({32'b0, mult_opa}),
        .mplier({32'b0, mult_opb}),
        .mult_func(is_ex_reg[2].alu_func),
        .start(mult_start),
        .product(mult_result),
        .done(mult_done)
    );

    // Instantiate the conditional branch module
    conditional_branch conditional_branch_0 (
        // Inputs
        .func(is_ex_reg[0].inst.b.funct3), // instruction bits for which condition to check
        .rs1(is_ex_reg[0].OPA),
        .rs2(is_ex_reg[0].OPB),

        // Output
        .take(take_conditional_alu0)
    );
    conditional_branch conditional_branch_1 (
        // Inputs
        .func(is_ex_reg[1].inst.b.funct3), // instruction bits for which condition to check
        .rs1(is_ex_reg[1].OPA),
        .rs2(is_ex_reg[1].OPB),

        // Output
        .take(take_conditional_alu1)
    );

    assign tmp_alu0_pkt.valid        = is_ex_reg[0].issue_valid;
    assign tmp_alu0_pkt.rob_tag      = is_ex_reg[0].rob_tag;
    assign tmp_alu0_pkt.take_branch  = is_ex_reg[0].uncond_branch || 
                                        (is_ex_reg[0].cond_branch && take_conditional_alu0);
    assign tmp_alu0_pkt.value        = (is_ex_reg[0].cond_branch || is_ex_reg[0].uncond_branch) ? 
                                        (is_ex_reg[0].uncond_branch ? alu0_result
                                                        : (tmp_alu0_pkt.take_branch ? alu0_result
                                                                        : is_ex_reg[0].NPC))
                                                : alu0_result; 
    assign tmp_alu0_pkt.done         = is_ex_reg[0].issue_valid;



    
    assign tmp_alu1_pkt.valid        = is_ex_reg[1].issue_valid;
    assign tmp_alu1_pkt.rob_tag      = is_ex_reg[1].rob_tag;
    assign tmp_alu1_pkt.take_branch  = is_ex_reg[1].uncond_branch || 
                                        (is_ex_reg[1].cond_branch && take_conditional_alu1);
    assign tmp_alu1_pkt.value        = (is_ex_reg[1].cond_branch || is_ex_reg[1].uncond_branch) ?
                                        (is_ex_reg[1].uncond_branch ? alu1_result
                                                        : (tmp_alu1_pkt.take_branch ? alu1_result
                                                                        : is_ex_reg[1].NPC))
                                                : alu1_result;
    assign tmp_alu1_pkt.done         = is_ex_reg[1].issue_valid;


    assign tmp_mult_pkt.valid        = mult_done;
    assign tmp_mult_pkt.rob_tag      = mult_tag;
    assign tmp_mult_pkt.take_branch  = 0;
    assign tmp_mult_pkt.value        = mult_result;
    assign tmp_mult_pkt.done         = mult_done;

    // assign ex_cp_packet = hold_valid ? hold_pkt : new_pkt;


    //priv addr out will only be fought between the alu0 vs alu1

    // assign is_mem_op = is_ex_reg[fu_index].rd_mem || is_ex_reg[fu_index].wr_mem;

    // assign priv_addr_out.valid = 
    // assign priv_addr_out.tag   = is_ex_reg[fu_index].rob_tag;
    // assign priv_addr_out.addr  = ex_cp_packet.value;

    // assign take_branch = is_ex_reg[fu_index].uncond_branch || 
    //                     (is_ex_reg[fu_index].cond_branch && take_conditional);

    // assign is_branch = is_ex_reg[fu_index].cond_branch || 
    //                 is_ex_reg[fu_index].uncond_branch;

    // assign new_pkt.valid =  (is_ex_reg[fu_index].issue_valid && 
    //                             (!is_mem_op) && 
    //                             (!is_mult_inst)) || mult_done;

    // assign new_pkt.rob_tag = mult_done ? mult_tag : (is_ex_reg[fu_index].rob_tag);

    // assign ex_cp_packet.value = cdb_packet_busy ? 
    //     (is_branch ? last_packet.value :
    //     is_ex_reg[fu_index].uncond_branch ? next_val :
    //     is_ex_reg[fu_index].cond_branch   ? (take_branch ? next_val : is_ex_reg[fu_index].NPC) :
    //                                         is_ex_reg[fu_index].NPC)
    //     : (is_mult_inst ? mult_result : next_val);
    // assign new_pkt.value      = is_branch
    //                             ? (is_ex_reg[fu_index].uncond_branch ? next_val
    //                                                     : (take_branch ? next_val
    //                                                                     : is_ex_reg[fu_index].NPC))
    //                             : (mult_done ? mult_result : next_val);
    // assign new_pkt.done = (is_ex_reg[fu_index].issue_valid && 
    //                         (!is_mem_op) && 
    //                         (!is_mult_inst)) ||mult_done ;

    // assign new_pkt.take_branch = take_branch;

endmodule // stage_ex