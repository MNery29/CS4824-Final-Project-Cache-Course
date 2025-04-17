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
    parameter int NUM_MULS = 2
)(
    input  logic        clk,
    input  logic        rst,
    input  IS_EX_PACKET in_pkt,

    output logic        issue_ready,
    output EX_CP_PACKET out_pkt
);

    // ---------- Simple combinational ALU ----------
    logic [63:0] simple_res;
    logic signed [`XLEN-1:0] A,B;
    begin
        A = p.OPA; B = p.OPB;
        unique case(p.alu_func)
            ALU_ADD : simple_res = A + B;
            ALU_SUB : simple_res = A - B;
            ALU_AND : simple_res = A & B;
            ALU_OR  : simple_res = A | B;
            ALU_XOR : simple_res = A ^ B;
            ALU_SLT : simple_res = (A < B);
            ALU_SLTU: simple_res = (p.OPA < p.OPB);
            ALU_SLL : simple_res = p.OPA << p.OPB[4:0];
            ALU_SRL : simple_res = p.OPA >> p.OPB[4:0];
            ALU_SRA : simple_res = A >>> p.OPB[4:0];
            default : simple_res = `XLEN'hDEAD_BEEF;
        endcase
    end

    logic is_mul = (in_pkt.alu_func == ALU_MUL || 
                    in_pkt.alu_func == ALU_MULH ||
                    in_pkt.alu_func == ALU_MULHSU ||
                    in_pkt.alu_func == ALU_MULHU);

    logic                mul_busy [NUM_MULS];
    EX_CP_PACKET         mul_out  [NUM_MULS];
    IS_EX_PACKET         mul_req;

    assign mul_req = (in_pkt.valid && is_mul) ? in_pkt : '0;

    genvar i;
    generate
        for(i=0;i<NUM_MULS;i++) begin : G_MUL
            mul_unit MU(
                .clk        (clk),
                .rst        (rst),
                .req        (mul_req),
                .busy       (mul_busy[i]),
                .out_packet (mul_out[i])
            );
        end
    endgenerate

    logic any_mul_free = |~mul_busy;

    // ---------- Ready / result select ----------
    assign issue_ready = in_pkt.valid && (is_mul ? any_mul_free : 1'b1);

    always_comb begin
        if (!is_mul) begin
            out_pkt.valid   = in_pkt.valid;
            out_pkt.rob_tag = in_pkt.rob_tag;
            out_pkt.value   = simple_res;
        end else begin
            out_pkt = mul_out[0];                 // first MUL only
        end
    end
endmodule
// ALU: computes the result of FUNC applied with operands A and B
// This module is purely combinational

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
