/* ********************************************
 *	COSE222 Lab #4
 *
 *	Module: pipelined_cpu.sv
 *  - Top design of the 5-stage pipelined RISC-V processor
 *  - Processor supports instructions described in Chapter 4 of COD book
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

// Packed structures for pipeline registers
// Pipe reg: IF/ID
typedef struct packed {
    logic   [63:0]  pc;
    logic   [31:0]  inst;
} pipe_if_id;

// Pipe reg: ID/EX
typedef struct packed {
    logic   [63:0]  rs1_dout;
    logic   [63:0]  rs2_dout;
    logic   [63:0]  imm64;
    logic   [2:0]   funct3;
    logic   [6:0]   funct7;
    logic           branch;
    logic           alu_src;
    logic   [1:0]   alu_op;
    logic           mem_read;
    logic           mem_write;
    logic   [4:0]   rs1;
    logic   [4:0]   rs2;
    logic   [4:0]   rd;         // rd for regfile
    logic           reg_write;
    logic           mem_to_reg;
} pipe_id_ex;

// Pipe reg: EX/MEM
typedef struct packed {
    logic   [63:0]  alu_result; // for address
    logic   [63:0]  rs2_dout;   // for store
    logic           mem_read;
    logic           mem_write;
    logic   [4:0]   rd;
    logic           reg_write;
    logic           mem_to_reg;
} pipe_ex_mem;

// Pipe reg: MEM/WB
typedef struct packed {
    logic   [63:0]  alu_result;
    logic   [63:0]  dmem_dout;
    logic   [4:0]   rd;
    logic           reg_write;
    logic           mem_to_reg;
} pipe_mem_wb;

module pipeline_cpu
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10,
              REG_WIDTH = 64,
              DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input           clk,            // System clock
    input           reset_b         // Asychronous negative reset
);

    // -------------------------------------------------------------------
    /* Instruction fetch stage:
     * - Accessing the instruction memory with PC
     * - Control PC udpates for pipeline stalls
     */

    // Program counter
    logic           pc_write;   // enable PC updates
    logic   [63:0]  pc_curr, pc_next;
    logic   [63:0]  pc_next_plus4, pc_next_branch;
    logic           branch;
    logic           regfile_zero;   // zero detection from regfile

    assign pc_next_plus4 = pc_curr + 3'd4;
    assign pc_next = (branch & regfile_zero) ? pc_next_branch : pc_next_plus4;      // FILL THIS. branch이고 Taken이면 branch로 이동.

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            pc_curr <= 'b0;
        end else begin
            if (pc_write) begin
               pc_curr <= pc_next; 
            end       // FILL THIS, pc_write이면 stall. 현재 PC 유지.
        end
    end

    // imem
    logic   [IMEM_ADDR_WIDTH-1:0]   imem_addr;
    logic   [31:0]  inst;   // instructions = an output of ????
    
    assign imem_addr = pc_curr[IMEM_ADDR_WIDTH+1:2];     // FILL THIS

    // instantiation: instruction memory
    imem #(
        .IMEM_DEPTH         (IMEM_DEPTH),
        .IMEM_ADDR_WIDTH    (IMEM_ADDR_WIDTH)
    ) u_imem_0 (
        .addr               ( imem_addr     ),
        .dout               ( inst          )
    );
    // -------------------------------------------------------------------

    // -------------------------------------------------------------------
    /* IF/ID pipeline register
     * - Supporting pipeline stalls and flush
     */
    pipe_if_id      id;         // THINK WHY THIS IS ID...
    logic           if_flush, if_stall;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            id <= 'b0;
        end else begin
            if (if_flush) begin     // Flush condition
                id <= #(`FF) 'b0;
            end else if (~if_stall) begin        // Stall condition
                id.pc <= #(`FF) pc_curr;
                id.inst <= #(`FF) inst;
            end
        end
    end
    // -------------------------------------------------------------------

    // ------------------------------------------------------------------
    /* Instruction decoder stage:
     * - Generating control signals
     * - Register file
     * - Immediate generator
     * - Hazard detection unit
     */
    
    // -------------------------------------------------------------------
    /* Main control unit:
     * Main control unit generates control signals for datapath elements
     * The control signals are determined by decoding instructions
     * Generating control signals using opcode = inst[6:0]
     */
    logic   [6:0]   opcode;
    //logic           branch;
    logic           alu_src, mem_to_reg;
    logic   [1:0]   alu_op;
    logic           mem_read, mem_write, reg_write; // declared above

    // COMPLETE THE MAIN CONTROL UNIT HERE

    assign opcode = id.inst[6:0];
    assign branch = (opcode == 7'b1100011) ? 1'b1 : 1'b0;      // branch instruction
    assign mem_read = (opcode == 7'b0000011) ? 1'b1 : 1'b0;    // ld instruction
    assign mem_write = (opcode == 7'b0100011) ? 1'b1 : 1'b0;   // sd instruction
    assign mem_to_reg = mem_read;
    assign reg_write = (opcode == 7'b0110011) | mem_read;      // ld or r-type
    assign alu_src = (mem_read | mem_write) ? 1'b1 : 1'b0;     // ld or sd instruction

    assign alu_op[0] = branch;
    assign alu_op[1] = (opcode == 7'b0110011);  // r-type

    // --------------------------------------------------------------------

    // ---------------------------------------------------------------------
    /* Immediate generator:
     * Generating immediate value from inst[31:0]
     */
    logic   [63:0]  imm64;
    logic   [63:0]  imm64_branch;  // imm64 left shifted by 1

    // COMPLETE IMMEDIATE GENERATOR HERE
    logic   [11:0]  imm12;

    assign imm12 = (branch) ? {id.inst[31], id.inst[7], id.inst[30:25], id.inst[11:8]} :        // FILL THIS
                ( (mem_read) ? id.inst[31:20] : {id.inst[31:25], id.inst[11:7]} );
    assign imm64 = { { 52{ imm12[11] } }, imm12 };                                  // FILL THIS
    assign imm64_branch = {imm64[62:0], 1'b0};                                      // FILL THIS

    // Computing branch target
    assign pc_next_branch = id.pc + imm64_branch;                                 // FILL THIS

    // ----------------------------------------------------------------------

    // ----------------------------------------------------------------------
    /* Hazard detection unit
     * - Detecting data hazards from load instrcutions
     * - Detecting control hazards from taken branches
     */
    pipe_id_ex      ex;         // Added.
    pipe_ex_mem     mem;        // Added.

    logic   [4:0]   rs1, rs2;

    logic           stall_by_load_use;
    logic   [1:0]   stall_by_regwr_branch;   // branch result is decided in ID stage, this is not explained in the textbook
    logic           flush_by_branch;
    
    logic           id_stall;

    assign stall_by_load_use = ex.mem_read & ( (ex.rd == rs1) | (ex.rd == rs2) );                           // FILL THIS: STALL BY LOAD-USE
    assign stall_by_regwr_branch[0] = branch & ( (ex.rd == rs1) | (ex.rd == rs2) );        // FILL THIS: STALL BY INST-BRANCH (CONDITION 1)
    assign stall_by_regwr_branch[1] = branch & ( (mem.rd == rs1) | (mem.rd == rs2) );     // FILL THIS: STALL BY INST-BRANCH (CONDITION 2)

    assign flush_by_branch = branch & regfile_zero;                                                       // FILL THIS: FLUSH CONDITION

    assign id_stall = |stall_by_regwr_branch | stall_by_load_use;
    assign if_flush = |flush_by_branch;                               // FILL THIS
    assign if_stall = |stall_by_regwr_branch | stall_by_load_use;     // FILL THIS
    assign pc_write = ~(|stall_by_load_use | stall_by_regwr_branch);                           // FILL THIS

    // ----------------------------------------------------------------------


    // regfile/
    pipe_mem_wb     wb;

    logic   [4:0]   rd;    // register numbers
    logic   [REG_WIDTH-1:0] rd_din;
    logic   [REG_WIDTH-1:0] rs1_dout, rs2_dout;
    
    assign rs1 = id.inst[19:15];         // FILL THIS
    assign rs2 = id.inst[24:20];         // FILL THIS
    assign rd = id.inst[11:7];           // FILL THIS
    // rd, rd_din, and reg_write will be determined in WB stage
    
    // instnatiation of register file
    regfile #(
        .REG_WIDTH          (REG_WIDTH)
    ) u_regfile_0 (
        .clk                ( clk ),
        .rs1                ( rs1 ),
        .rs2                ( rs2 ),
        .rd                 ( wb.rd ),          // WB의 rd를 연결
        .rd_din             ( rd_din ),
        .reg_write          ( wb.reg_write ),   // WB의 reg_write를 연결
        .rs1_dout           ( rs1_dout ),
        .rs2_dout           ( rs2_dout )
    );

    assign regfile_zero = (rs1_dout ^ rs2_dout) == 0;      // FILL THIS. xor로 같은지 확인. 같으면 0이 되는 것에 주의.

    // ------------------------------------------------------------------

    // -------------------------------------------------------------------
    /* ID/EX pipeline register
     * - Supporting pipeline stalls
     */
    //  pipe_id_ex      ex;         // THINK WHY THIS IS EX...
    logic   [6:0]   funct7;
    logic   [2:0]   funct3;

    // THE FOLLOWING SIGNALS WILL BE USED FOR ALU CONTROL
    assign funct7 = id.inst[31:25];         // FILL THIS
    assign funct3 = id.inst[14:12];         // FILL THIS

    // COMPLETE ID/EX PIPELINE REGISTER
    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            ex <= 'b0;
        end else begin
            if (id_stall) begin
                ex <= 'b0;
            end else begin
            // FILL THIS
                ex.rs1_dout <= rs1_dout;
                ex.rs2_dout <= rs2_dout;
                ex.imm64 <= imm64;
                ex.funct3 <= funct3;
                ex.funct7 <= funct7;
                ex.branch <= branch;
                ex.alu_src <= alu_src;
                ex.alu_op <= alu_op;
                ex.mem_read <= mem_read;
                ex.mem_write <= mem_write;
                ex.rs1 <= rs1;
                ex.rs2 <= rs2;
                ex.rd <= rd;                             // EX로 보내는 값은 현재 instruction의 rd 값. wb에서 온 값이 아님.
                ex.reg_write <= reg_write;
                ex.mem_to_reg <= mem_to_reg;
            end
        end
    end

    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    /* Excution stage:
     * - ALU & ALU control
     * - Data forwarding unit
     */

    // --------------------------------------------------------------------
    /* ALU control unit:
     * ALU control unit generate alu_control signal which selects ALU operations
     * Generating control signals using alu_op, funct7, and funct3 fileds
     */

    logic   [3:0]   alu_control;    // ALU control signal

    // COMPLETE ALU CONTROL UNIT
    always_comb begin
        if (ex.alu_op[1:1]) begin
            case (ex.funct3)
                3'b111 : alu_control = 4'b0000;
                3'b110 : alu_control = 4'b0001;
                default: alu_control = (ex.funct7[5:5]) ? 4'b0110 : 4'b0010;
            endcase
        end else begin
            alu_control = (ex.alu_op[0:0]) ? 4'b0110 : 4'b0010;
        end
    end

    // ---------------------------------------------------------------------

    // ----------------------------------------------------------------------
    /* Forwarding unit:
     * - Forwarding from EX/MEM and MEM/WB
     */
    logic   [1:0]   forward_a, forward_b;
    logic   [63:0]  alu_fwd_in1, alu_fwd_in2;   // outputs of forward MUXes

    // COMPLETE FORWARDING MUXES
    always_comb begin
        case (forward_a)
           2'b00 : alu_fwd_in1 = ex.rs1_dout;
           2'b10 : alu_fwd_in1 = mem.alu_result;
           2'b01 : alu_fwd_in1 = rd_din;
        endcase
    end
    always_comb begin
        case (forward_b)
           2'b00 : alu_fwd_in2 = ex.rs2_dout;
           2'b10 : alu_fwd_in2 = mem.alu_result;
           2'b01 : alu_fwd_in2 = rd_din;
        endcase
    end

    // COMPLETE THE FORWARDING UNIT
    // Need to prioritize forwarding conditions
    always_comb begin
        if
        ( mem.reg_write
        & (mem.rd != 0)
        & (mem.rd == ex.rs1) ) begin
            assign forward_a = 2'b10;
        end else if 
        ( wb.reg_write 
        & (wb.rd != 0) 
        & (wb.rd == ex.rs1) ) begin
            assign forward_a = 2'b01;
        end else begin
            assign forward_a = 2'b00;
        end
    end

    always_comb begin
        if
        ( mem.reg_write
        & (mem.rd != 0)
        & (mem.rd == ex.rs2) ) begin
            assign forward_b = 2'b10;
        end else if 
        ( wb.reg_write 
        & (wb.rd != 0) 
        & (wb.rd == ex.rs2) ) begin
            assign forward_b = 2'b01;
        end else begin
            assign forward_b = 2'b00;
        end
    end

    // -----------------------------------------------------------------------

    // ALU
    logic   [REG_WIDTH-1:0] alu_in1, alu_in2;
    logic   [REG_WIDTH-1:0] alu_result;
    logic           alu_zero;   // will not be used

    assign alu_in1 = alu_fwd_in1;                           // FILL THIS
    assign alu_in2 = ex.alu_src ? ex.imm64 : alu_fwd_in2;   // FILL THIS. ex.alu_src 값에 따라서 alu에서 사용하는 값이 달라짐.

    // instantiation: ALU
    alu #(
        .REG_WIDTH          (REG_WIDTH)
    ) u_alu_0 (
        .in1                (alu_in1),
        .in2                (alu_in2),
        .alu_control        (alu_control),
        .result             (alu_result),
        .zero               (alu_zero)
    );

    // -------------------------------------------------------------------------
    /* Ex/MEM pipeline register
     */
    // pipe_ex_mem     mem;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            mem <= 'b0;
        end else begin
           // FILL THIS
            mem.alu_result <= alu_result;
            mem.rs2_dout <= alu_fwd_in2;            // mem으로 넘기는 값은 forwarding 한 값을 넘겨야 함. imm 값이 아니라서 alu_fwd_in2.
            mem.mem_read <= ex.mem_read;
            mem.mem_write <= ex.mem_write;
            mem.rd <= ex.rd;
            mem.reg_write <= ex.reg_write;
            mem.mem_to_reg <= ex.mem_to_reg;
        end
    end

    // --------------------------------------------------------------------------
    /* Memory srage
     * - Data memory accesses
     */

    // dmem
    logic   [DMEM_ADDR_WIDTH-1:0]    dmem_addr;
    logic   [63:0]  dmem_din, dmem_dout;

    assign dmem_addr = mem.alu_result[DMEM_ADDR_WIDTH+2:3];     // FILL THIS
    assign dmem_din = mem.rs2_dout;                             // FILL THIS
    
    // instantiation: data memory
    dmem #(
        .DMEM_DEPTH         (DMEM_DEPTH),
        .DMEM_ADDR_WIDTH    (DMEM_ADDR_WIDTH)
    ) u_dmem_0 (
        .clk                (clk),
        .addr               (dmem_addr),
        .din                (dmem_din),
        .mem_read           (mem.mem_read   ),
        .mem_write          (mem.mem_write  ),
        .dout               (dmem_dout)
    );


    // -----------------------------------------------------------------------
    /* MEM/WB pipeline register
     */

    //pipe_mem_wb         wb;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            wb <= 'b0;
        end else begin
            // FILL THIS
            wb.alu_result = mem.alu_result;
            wb.dmem_dout = dmem_dout;
            wb.rd = mem.rd;
            wb.reg_write = mem.reg_write;
            wb.mem_to_reg = mem.mem_to_reg;
        end
    end

    // ----------------------------------------------------------------------
    /* Writeback stage
     * - Write results to regsiter file
     */
    
    assign rd_din = wb.mem_to_reg ? wb.dmem_dout : wb.alu_result;    // FILL THIS

endmodule