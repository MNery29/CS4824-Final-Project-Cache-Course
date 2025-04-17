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
module mul_unit (
    input  logic         clk,
    input  logic         rst,
    // Issue side
    input IS_EX_PACKET req,

    output logic         busy,
    output EX_CP_PACKET out_packet,
);
    // Right now, we are not using the op_codes, we are just doing
    // regular o' signed multiplication

    //------------------------------------------------------
    //  Simple state machine: IDLE → BUSY → DONE
    //------------------------------------------------------
    typedef enum logic [1:0] {M_IDLE, M_RUN, M_DONE} mstate_e;
    mstate_e state, nstate;

    // Hold operands & tag while the underlying `mult` runs
    logic [63:0] opa_q, opb_q;
    logic  [5:0] tag_q;

    // Outputs from the Booth pipeline
    logic mul_done;
    logic [63:0] mul_lo;

    // Underlying pipelined multiplier (produces low 64 bits, assert when done)
    booth_mult mul_pipe (
        .clock  (clk),
        .reset  (rst),
        .mcand  (opa_q),
        .mplier (opb_q),
        .start  (state == M_IDLE && req.issue_valid),
        .product(mul_lo),
        .done   (mul_done)
    );

    // State transition
    always_comb begin
        case (state)
            M_IDLE: nstate = (valid) ? M_RUN : M_IDLE;
            M_RUN : nstate = (mul_done) ? M_DONE : M_RUN;
            M_DONE: nstate = M_IDLE;
            default: nstate = M_IDLE;
        endcase
    end

    // Registers
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= M_IDLE;
        end else begin
            state <= nstate;
            if (req.valid && state == M_IDLE) begin
                opa_q <= req.OPA;
                opb_q <= req.OPB;
                tag_q <= req.rob_tag;
            end
        end
    end

    //--------------------------------------------------
    //  Busy / response signals
    //--------------------------------------------------
    assign busy         = (state != M_IDLE);
    assign out_packet.valid    = (state == M_DONE);
    assign out_packet.rob_tag     = tag_q[4:0];
    assign out_packet.value   = mul_lo; // return lower 64 bits?
endmodule

//funcitonal unit cluster
module fu_cluster #(
    parameter int NUM_ADDERS = 2,
    parameter int NUM_MULS   = 2
)
(
    input logic clk,
    input logic rst,
    input IS_EX_PACKET in_packet,

    // it is ready to accept new instructions
    output logic issue_ready,
    output EX_CP_PACKET out_packet
);
    logic adders_busy [NUM_ADDERS-1:0];
    logic muls_busy  [NUM_MULS-1:0];

    logic IS_EX_PACKET adders_request [NUM_ADDERS-1:0];
    logic IS_EX_PACKET muls_request  [NUM_MULS-1:0];

    logic EX_CP_PACKET add_resp [NUM_ADDERS-1:0];
    logic EX_CP_PACKET mul_resp  [NUM_MULS-1:0];
    genvar ai;
    generate for (ai = 0; ai < NUM_ADDERS; ai++) begin : G_ADD
        always_ff @(posedge clk or posedge rst) begin
            if (rst) begin
                adders_busy[ai] <= 1'b0;
            end else begin
                // accept new op if free and dispatcher fires
                if (!adders_busy[ai] && in_packet.issue_valid && in_packet.alu_func==ALU_ADD && issue_ready && ai==0) begin
                    adders_request[ai] <= in_packet;
                    adders_busy[ai] <= 1'b1;
                end else if (adders[ai].busy) begin
                    adders_busy[ai] <= 1'b0; // done next cycle
                end
            end
        end

        assign add_resp[ai].valid = adders_busy[ai]; // finishes in 1‑cycle
        assign add_resp[ai].rob_tag   = adders_request[ai].rob_tag;
        assign add_resp[ai].value = {(adders_request[ai].OPA + adders_request[ai].OPB)}; // this is literally our adder
    end endgenerate

    genvar mi;
    generate for (mi = 0; mi < NUM_MULS; mi++) begin : G_MUL
        // Issue bus is one‑deep; only first free gets the op.
        // ok we need to handle other cases of ALU_MUL 
        assign muls_request[mi] = (in_packet.valid && (in_packet.alu_func==ALU_MUL || in_packet.alu_func==ALU_MULH || 
                                    in_packet.alu_func==ALU_MULHSU || in_packet.alu_func==ALU_MULHU) &&
                                    issue_ready && mi==0) ? issue_req : '{valid:0, op:FU_NOP, rs1:0, rs2:0, tag:0};

        mul_unit MU (
            .clk (clk),
            .rst (rst),
            .req (muls_request[mi]),
            .busy(muls_busy[mi]),
            .out_packet (mul_resp[mi])
        );

    end endgenerate

    logic any_add_free, any_mul_free;
    assign any_add_free = |(~{adders_busy}); // takes OR of all adders
    assign any_mul_free = |(~(muls_busy)); // takes OR of all muls

    always_comb begin
        unique case(in_packet.alu_func)
            ALU_ADD : issue_ready = in_packet.valid && any_add_free;
            ALU_MUL : issue_ready = in_packet.valid && any_mul_free;
            ALU_MULH: issue_ready = in_packet.valid && any_mul_free;
            ALU_MULHSU: issue_ready = in_packet.valid && any_mul_free;
            ALU_MULHU: issue_ready = in_packet.valid && any_mul_free;
            default: issue_ready = 1'b1; // logical ops execute in regfile‑bypass ALU
        endcase
    end

endmodule
// ALU: computes the result of FUNC applied with operands A and B
// This module is purely combinational
module alu 
(
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
    input ID_EX_PACKET id_ex_reg,

    output EX_MEM_PACKET ex_packet
);

    logic [`XLEN-1:0] opa_mux_out, opb_mux_out;
    logic take_conditional;

    // Pass-throughs
    assign ex_packet.NPC          = id_ex_reg.NPC;
    assign ex_packet.rs2_value    = id_ex_reg.rs2_value;
    assign ex_packet.rd_mem       = id_ex_reg.rd_mem;
    assign ex_packet.wr_mem       = id_ex_reg.wr_mem;
    assign ex_packet.dest_reg_idx = id_ex_reg.dest_reg_idx;
    assign ex_packet.halt         = id_ex_reg.halt;
    assign ex_packet.illegal      = id_ex_reg.illegal;
    assign ex_packet.csr_op       = id_ex_reg.csr_op;
    assign ex_packet.valid        = id_ex_reg.valid;

    // Break out the signed/unsigned bit and memory read/write size
    assign ex_packet.rd_unsigned  = id_ex_reg.inst.r.funct3[2]; // 1 if unsigned, 0 if signed
    assign ex_packet.mem_size     = MEM_SIZE'(id_ex_reg.inst.r.funct3[1:0]);

    // ultimate "take branch" signal:
    // unconditional, or conditional and the condition is true
    assign ex_packet.take_branch = id_ex_reg.uncond_branch || (id_ex_reg.cond_branch && take_conditional);

    // ALU opA mux
    always_comb begin
        case (id_ex_reg.opa_select)
            OPA_IS_RS1:  opa_mux_out = id_ex_reg.rs1_value;
            OPA_IS_NPC:  opa_mux_out = id_ex_reg.NPC;
            OPA_IS_PC:   opa_mux_out = id_ex_reg.PC;
            OPA_IS_ZERO: opa_mux_out = 0;
            default:     opa_mux_out = `XLEN'hdeadface; // dead face
        endcase
    end

    // ALU opB mux
    always_comb begin
        case (id_ex_reg.opb_select)
            OPB_IS_RS2:   opb_mux_out = id_ex_reg.rs2_value;
            OPB_IS_I_IMM: opb_mux_out = `RV32_signext_Iimm(id_ex_reg.inst);
            OPB_IS_S_IMM: opb_mux_out = `RV32_signext_Simm(id_ex_reg.inst);
            OPB_IS_B_IMM: opb_mux_out = `RV32_signext_Bimm(id_ex_reg.inst);
            OPB_IS_U_IMM: opb_mux_out = `RV32_signext_Uimm(id_ex_reg.inst);
            OPB_IS_J_IMM: opb_mux_out = `RV32_signext_Jimm(id_ex_reg.inst);
            default:      opb_mux_out = `XLEN'hfacefeed; // face feed
        endcase
    end

    // Instantiate the ALU
    alu alu_0 (
        // Inputs
        .opa(opa_mux_out),
        .opb(opb_mux_out),
        .func(id_ex_reg.alu_func),

        // Output
        .result(ex_packet.alu_result)
    );

    // Instantiate the conditional branch module
    conditional_branch conditional_branch_0 (
        // Inputs
        .func(id_ex_reg.inst.b.funct3), // instruction bits for which condition to check
        .rs1(id_ex_reg.rs1_value),
        .rs2(id_ex_reg.rs2_value),

        // Output
        .take(take_conditional)
    );

endmodule // stage_ex
