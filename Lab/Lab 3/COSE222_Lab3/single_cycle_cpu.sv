/* ********************************************
 *	COSE222 Lab #3
 *
 *	Module: top design of the single-cycle CPU (single_cycle_cpu.sv)
 *  - Top design of the single-cycle CPU
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

module single_cycle_cpu
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10,
              REG_WIDTH = 64,
              DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input           clk,            // System clock
    input           reset_b         // Asychronous negative reset
);

    // Wires for datapath elements
    logic   [IMEM_ADDR_WIDTH-1:0]   imem_addr;
    logic   [31:0]  inst;   // instructions = an output of ????

    logic   [4:0]   rs1, rs2, rd;    // register numbers
    logic   [REG_WIDTH-1:0] rd_din;
    logic           reg_write;
    logic   [REG_WIDTH-1:0] rs1_dout, rs2_dout;

    logic   [REG_WIDTH-1:0] alu_in1, alu_in2;
    logic   [3:0]   alu_control;    // ALU control signal
    logic   [REG_WIDTH-1:0] alu_result;
    logic           alu_zero;

    logic   [DMEM_ADDR_WIDTH-1:0]    dmem_addr;
    logic   [63:0]  dmem_din, dmem_dout;
    logic           mem_read, mem_write;

    // -------------------------------------------------------------------
    /* Main control unit:
     * Main control unit generates control signals for datapath elements
     * The control signals are determined by decoding instructions
     * Generating control signals using opcode = inst[6:0]
     */
    logic   [6:0]   opcode;
    logic           branch, alu_src, mem_to_reg;
    logic   [1:0]   alu_op;
    //logic         mem_read, mem_write, reg_write; // declared above

    // COMPLETE THE MAIN CONTROL UNIT HERE

    assign opcode = inst[6:0];

    always @(opcode) begin
        if (opcode == 51) begin     // R-format
            assign alu_src = 0;
            assign mem_to_reg = 0;
            assign reg_write = 1;
            assign mem_read = 0;
            assign mem_write = 0;
            assign branch = 0;
            assign alu_op = 2'b10;
        end if (opcode == 3) begin     // I-format (ld)
            assign alu_src = 1;
            assign mem_to_reg = 1;
            assign reg_write = 1;
            assign mem_read = 1;
            assign mem_write = 0;
            assign branch = 0;
            assign alu_op = 2'b00;
        end if (opcode == 35) begin     // S-format (sd) 
            assign alu_src = 1;
            // assign mem_to_reg = X;
            assign reg_write = 0;
            assign mem_read = 0;
            assign mem_write = 1;
            assign branch = 0;
            assign alu_op = 2'b00;
        end if (opcode == 99) begin     // SB-format (beq)
            assign alu_src = 0;
            // assign mem_to_reg = X;
            assign reg_write = 0;
            assign mem_read = 0;
            assign mem_write = 0;
            assign branch = 1;
            assign alu_op = 2'b01;
        end
    end


    // --------------------------------------------------------------------

    // --------------------------------------------------------------------
    /* ALU control unit:
     * ALU control unit generate alu_control signal which selects ALU operations
     * Generating control signals using alu_op, funct7, and funct3 fileds
     */
    logic   [6:0]   funct7;
    logic   [2:0]   funct3;

    // COMPLETE THE ALU CONTROL UNIT HERE

    assign funct7 = inst[31:25];
    assign funct3 = inst[14:12];
    
    always @(alu_op or funct7 or funct3) begin
        if (alu_op == 2'b00) begin
            alu_control <= 4'b0010;
        end if (alu_op[0:0] == 1'b1) begin
            alu_control <= 4'b0110;
        end if (alu_op[1:1] == 1'b1) begin
            if (funct3 == 3'b000) begin
                if (funct7 == 7'b0000000) begin
                    alu_control <= 4'b0010;
                end if (funct7 == 7'b0100000) begin
                    alu_control <= 4'b0110;
                end 
            end if (funct3 == 3'b111) begin
                alu_control <= 4'b0000;
            end if (funct3 == 3'b110) begin
                alu_control <= 4'b0001; 
            end
        end
    end


    // ---------------------------------------------------------------------


    // ----------------------------------------------------------------------
    /* Immediate generator:
     * Generating immediate value from inst[31:0]
     */
    logic   [63:0]  imm64;
    logic   [63:0]  imm64_branch;  // imm64 left shifted by 1
    logic   [11:0]  imm12;  // 12-bit immediate value extracted from inst

    // COMPLETE IMMEDIATE GENERATOR HERE
    
    always @(opcode or imm12 or inst) begin
        if (opcode == 3) begin     // ld
            imm12[11:0] = inst[31:20];
        end if (opcode == 35) begin     // sd
            imm12[11:5] = inst[31:25];
            imm12[4:0] = inst[11:7];
        end if (opcode == 99) begin     // beq
            imm12[11:11] = inst[31];
            imm12[9:4] = inst[30:25];
            imm12[3:0] = inst[11:8];
            imm12[10:10] = inst[7];
        end
    end

    assign imm64 = imm12;
    assign imm64_branch = imm64 << 1;

    // ----------------------------------------------------------------------

    // Program counter
    logic   [63:0]  pc_curr, pc_next;
    logic   [63:0]  pc_next_plus4, pc_next_branch;

    assign pc_next_plus4 = pc_curr + 4;    // FILL THIS

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            pc_curr <= 'b0;
        end else begin
            pc_curr <= pc_next;        // FILL THIS
        end
    end


    // MUXes:
    // COMPLETE MUXES HERE
    // PC_NEXT
    assign pc_next_branch = pc_curr + imm64_branch;   // FILL THIS
    assign pc_next = (branch & alu_zero) ? pc_next_branch : pc_next_plus4;

    // ALU inputs
    assign alu_in2 = alu_src ? imm64 : rs2_dout;

    // RF din
    assign rd_din = mem_to_reg ? dmem_dout : alu_result;

    // COMPLETE CONNECTIONS HERE
    // imem
    assign imem_addr = pc_curr >> 2; 

    // regfile
    assign rs1 = inst[19:15];
    assign rs2 = inst[24:20];
    assign rd = inst[11:7];

    assign alu_in1 = rs1_dout;
 
    // dmem
    assign dmem_addr = alu_result >> 3;
    assign dmem_din = rs2_dout;



    // -----------------------------------------------------------------------
    /* Instantiation of datapath elements
     * All input/output ports should be connected
     */
    
    // IMEM
    imem #(
        .IMEM_DEPTH         (IMEM_DEPTH),
        .IMEM_ADDR_WIDTH    (IMEM_ADDR_WIDTH)
    ) u_imem_0 (
        .addr               ( imem_addr     ),
        .dout               ( inst          )
    );

    // REGFILE
    regfile #( 
        .REG_WIDTH      (REG_WIDTH)
    ) u_regfile_0 (
        .clk            (clk), 
        .rs1            (rs1), 
        .rs2            (rs2), 
        .rd             (rd), 
        .rd_din         (rd_din), 
        .reg_write      (reg_write), 
        .rs1_dout       (rs1_dout), 
        .rs2_dout       (rs2_dout)
    );

    // ALU
    alu #(
        .REG_WIDTH          (REG_WIDTH)
    ) u_alu_0 (
        .in1                (alu_in1),            
        .in2                (alu_in2),            
        .alu_control        (alu_control),        
        .result             (alu_result),         
        .zero               (alu_zero)          
    );

    // DMEM
    dmem #(
        .DMEM_DEPTH         (DMEM_DEPTH),
        .DMEM_ADDR_WIDTH    (DMEM_ADDR_WIDTH)
    ) u_dmem_0 (
        .clk            (clk),
        .addr           (dmem_addr),
        .din            (dmem_din),
        .mem_read       (mem_read),
        .mem_write      (mem_write),
        .dout           (dmem_dout)
    );

endmodule