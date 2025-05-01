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

// ALU function code input
// probably want to leave these alone
// typedef enum logic [4:0] {
//     ALU_ADD     = 5'h00,
//     ALU_SUB     = 5'h01,
//     ALU_SLT     = 5'h02,
//     ALU_SLTU    = 5'h03,
//     ALU_AND     = 5'h04,
//     ALU_OR      = 5'h05,
//     ALU_XOR     = 5'h06,
//     ALU_SLL     = 5'h07,
//     ALU_SRL     = 5'h08,
//     ALU_SRA     = 5'h09,
//     ALU_MUL     = 5'h0a, // Mult FU
//     ALU_MULH    = 5'h0b, // Mult FU
//     ALU_MULHSU  = 5'h0c, // Mult FU
//     ALU_MULHU   = 5'h0d, // Mult FU
//     ALU_DIV     = 5'h0e, // unused
//     ALU_DIVU    = 5'h0f, // unused
//     ALU_REM     = 5'h10, // unused
//     ALU_REMU    = 5'h11  // unused
// } ALU_FUNC;


// this mul_unit uses booth's algorithm to multiply two 64-bit integers
// module mul_unit (
//     input  logic         clk,
//     input  logic         rst,
//     // Issue side
//     input IS_EX_PACKET req,

//     output logic         busy,
//     output EX_CP_PACKET out_packet
// );
//     // Right now, we are not using the op_codes, we are just doing
//     // regular o' signed multiplication

//     //------------------------------------------------------
//     //  Simple state machine: IDLE → BUSY → DONE
//     //------------------------------------------------------
//     typedef enum logic [1:0] {M_IDLE, M_RUN, M_DONE} mstate_e;
//     mstate_e state, nstate;

//     // Hold operands & tag while the underlying `mult` runs
//     logic [63:0] opa_q, opb_q;
//     logic  [5:0] tag_q;

//     // Outputs from the Booth pipeline
//     logic mul_done;
//     logic [63:0] mul_lo;

//     // Underlying pipelined multiplier (produces low 64 bits, assert when done)
//     booth_mult mul_pipe (
//         .clock  (clk),
//         .reset  (rst),
//         .mcand  (opa_q),
//         .mplier (opb_q),
//         .start  (state == M_IDLE && req.issue_valid),
//         .product(mul_lo),
//         .done   (mul_done)
//     );

//     // State transition
//     always_comb begin
//         case (state)
//             M_IDLE: nstate = (req.issue_valid) ? M_RUN : M_IDLE; //was set as just 'valid' changed to req.issue_valid
//             M_RUN : nstate = (mul_done) ? M_DONE : M_RUN;
//             M_DONE: nstate = M_IDLE;
//             default: nstate = M_IDLE;
//         endcase
//     end

//     // Registers
//     always_ff @(posedge clk or posedge rst) begin
//         if (rst) begin
//             state <= M_IDLE;
//         end else begin
//             state <= nstate;
//             if (req.issue_valid && state == M_IDLE) begin
//                 opa_q <= req.OPA;
//                 opb_q <= req.OPB;
//                 tag_q <= req.rob_tag;
//             end
//         end
//     end

//     //--------------------------------------------------
//     //  Busy / response signals
//     //--------------------------------------------------
//     assign busy         = (state != M_IDLE);
//     assign out_packet.valid    = (state == M_DONE);
//     assign out_packet.rob_tag     = tag_q[4:0];
//     assign out_packet.value   = mul_lo; // return lower 64 bits?
// endmodule

//funcitonal unit cluster
// module fu_cluster #(
//     parameter int NUM_MULS = 2
// )(
//     input  logic        clk,
//     input  logic        rst,
//     input  IS_EX_PACKET in_pkt,

//     output logic        issue_ready,
//     output EX_CP_PACKET out_pkt
// );

//     // ---------- Simple combinational ALU ----------
//     logic [63:0] simple_res;
//     logic signed [`XLEN-1:0] A, B;

//     always_comb begin //from just begin statement to always_comb
//         A = in_pkt.OPA; // what is p... changed to in_pkt
//         B = in_pkt.OPB;
//         unique case(in_pkt.alu_func)
//             ALU_ADD : simple_res = A + B;
//             ALU_SUB : simple_res = A - B;
//             ALU_AND : simple_res = A & B;
//             ALU_OR  : simple_res = A | B;
//             ALU_XOR : simple_res = A ^ B;
//             ALU_SLT : simple_res = (A < B);
//             ALU_SLTU: simple_res = (in_pkt.OPA < in_pkt.OPB);
//             ALU_SLL : simple_res = in_pkt.OPA << in_pkt.OPB[4:0];
//             ALU_SRL : simple_res = in_pkt.OPA >> in_pkt.OPB[4:0];
//             ALU_SRA : simple_res = A >>> in_pkt.OPB[4:0];
//             default : simple_res = `XLEN'hDEAD_BEEF;
//         endcase
//     end

//     logic is_mul = (in_pkt.alu_func == ALU_MUL || 
//                     in_pkt.alu_func == ALU_MULH ||
//                     in_pkt.alu_func == ALU_MULHSU ||
//                     in_pkt.alu_func == ALU_MULHU);

//     logic                mul_busy [NUM_MULS];
//     EX_CP_PACKET         mul_out  [NUM_MULS];
//     IS_EX_PACKET         mul_req;

//     assign mul_req = (in_pkt.issue_valid && is_mul) ? in_pkt : '0;

//     genvar i;
//     generate
//         for(i=0;i<NUM_MULS;i++) begin : G_MUL
//             mul_unit MU(
//                 .clk        (clk),
//                 .rst        (rst),
//                 .req        (mul_req),
//                 .busy       (mul_busy[i]),
//                 .out_packet (mul_out[i])
//             );
//         end
//     endgenerate

//     //logic any_mul_free = |~mul_busy; syntax errors

//     logic any_mul_free;

//     always_comb begin
//         any_mul_free = 0;
//         for (int i = 0; i < NUM_MULS; i++) begin
//             if (!mul_busy[i]) begin
//                 any_mul_free = 1;
//             end
//         end
//     end

//     // ---------- Ready / result select ----------
//     assign issue_ready = in_pkt.issue_valid && (is_mul ? any_mul_free : 1'b1);

//     always_comb begin
//         if (!is_mul) begin
//             out_pkt.valid   = in_pkt.issue_valid;
//             out_pkt.rob_tag = in_pkt.rob_tag;
//             out_pkt.value   = simple_res;
//         end else begin
//             out_pkt = mul_out[0];                 // first MUL only
//         end
//     end
// endmodule

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

    // input ID_EX_PACKET id_ex_reg,

    // output EX_MEM_PACKET ex_packet,
    output logic alu_busy,
    output logic take_conditional,
    output EX_CP_PACKET ex_cp_packet,
    //broad cast value + tag to cbd, so reorder buffer can be updated
    output priv_addr_packet priv_addr_out,

    output logic [`XLEN-1:0] opa_mux_out,
    output logic [`XLEN-1:0] opb_mux_out

);
    //functional unit inputs
    logic [`XLEN-1:0] opa_mux_out0, opb_mux_out0;
    logic [`XLEN-1:0] opa_mux_out1, opb_mux_out1;
    logic [`XLEN-1:0] opa_mux_out2, opb_mux_out2;


    // logic [`XLEN-1:0] opa_mux_out, opb_mux_out;
    logic [31:0] next_val;
    logic take_branch;
    logic is_branch;
    // logic take_conditional;

    //MULT logic inputs 
    logic [63:0] mult_result;
    logic mult_done;
    logic mult_start;
    logic is_mult_inst;   


    
    //Logic for which FU to select from: 
    logic [1:0] fu_index;
    always_comb begin
        if (is_ex_reg[2].issue_valid) //MULT
            fu_index = 2'd2;
        else if (is_ex_reg[0].issue_valid) //ALU0
            fu_index = 2'd0;
        else if (is_ex_reg[1].issue_valid) //ALU1
            fu_index = 2'd1;
        else
            fu_index = 2'd0; // default (NOP)
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
    assign mult_start = is_ex_reg[2].issue_valid && (is_ex_reg[2].alu_func inside{MUL_ALU_MUL, MUL_ALU_MULH, MUL_ALU_MULHSU, MUL_ALU_MULHU});

    assign is_mult_inst = (is_ex_reg[2].alu_func inside{MUL_ALU_MUL, MUL_ALU_MULH, MUL_ALU_MULHSU, MUL_ALU_MULHU});

    assign alu_busy = cdb_packet_busy || (is_mult_inst && !mult_done);


    EX_CP_PACKET last_packet;


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
        .mcand(opa_mux_out2),
        .mplier(opb_mux_out2),
        .mult_type(is_ex_reg[2].alu_func),
        .start(mult_start),
        .product(mult_result),
        .done(mult_done)
    );

    // Instantiate the conditional branch module
    conditional_branch conditional_branch_0 (
        // Inputs
        .func(is_ex_reg[fu_index].inst.b.funct3), // instruction bits for which condition to check
        .rs1(is_ex_reg[fu_index].OPA),
        .rs2(is_ex_reg[fu_index].OPB),

        // Output
        .take(take_conditional)
    );


    assign is_mem_op = is_ex_reg[fu_index].rd_mem || is_ex_reg[fu_index].wr_mem;

    assign priv_addr_out.valid = is_ex_reg[fu_index].issue_valid && is_mem_op;
    assign priv_addr_out.tag   = is_ex_reg[fu_index].rob_tag;
    assign priv_addr_out.addr  = ex_cp_packet.value;

    assign take_branch = is_ex_reg[fu_index].uncond_branch || 
                        (is_ex_reg[fu_index].cond_branch && take_conditional);

    assign is_branch = is_ex_reg[fu_index].cond_branch || 
                    is_ex_reg[fu_index].uncond_branch;

    assign ex_cp_packet.valid = cdb_packet_busy ? 
                                last_packet.valid : 
                                is_ex_reg[fu_index].issue_valid && 
                                (!is_mem_op) && 
                                (!is_mult_inst || mult_done);

    assign ex_cp_packet.rob_tag = cdb_packet_busy ? 
                                last_packet.rob_tag : 
                                is_ex_reg[fu_index].rob_tag;

    assign ex_cp_packet.value = cdb_packet_busy ? 
        (is_branch ? last_packet.value :
        is_ex_reg[fu_index].uncond_branch ? next_val :
        is_ex_reg[fu_index].cond_branch   ? (take_branch ? next_val : is_ex_reg[fu_index].NPC) :
                                            is_ex_reg[fu_index].NPC)
        : (is_mult_inst ? mult_result : next_val);

    assign ex_cp_packet.done = cdb_packet_busy ? 
                            last_packet.done : 
                            is_ex_reg[fu_index].issue_valid && 
                            (!is_mem_op) && 
                            (!is_mult_inst || mult_done);

    assign ex_cp_packet.take_branch = cdb_packet_busy ? 
                                    last_packet.take_branch : 
                                    take_branch;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            last_packet.valid <= 0;
            last_packet.rob_tag <= 0;
            last_packet.value <= 0;
            last_packet.done <= 0;
            last_packet.take_branch <= 0;
        end else begin
            last_packet.valid <= ex_cp_packet.valid;
            last_packet.rob_tag <= ex_cp_packet.rob_tag;
            last_packet.value <= ex_cp_packet.value;
            last_packet.done <= ex_cp_packet.done;
            last_packet.take_branch <= ex_cp_packet.take_branch;
        end
    end

endmodule // stage_ex
