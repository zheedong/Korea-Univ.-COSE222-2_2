/* ***********************************************
 *  COSE222 Lab #1
 *
 *  Module: testbench including clock and reset_b signals (tb_.sv)
 *  -
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 **************************************************
 */

`timescale 1ns/1ps
`define CLK_T 10

module tb_alu
#(  parameter REG_WIDTH = 64 )();  // ALU input data width is equal to the width of register file

    logic   [REG_WIDTH-1:0] in1;    // Operand 1
    logic   [REG_WIDTH-1:0] in2;    // Operand 2
    logic   [3:0]   alu_control;    // ALU control signal
    logic   [REG_WIDTH-1:0] result; // ALU output
    logic                   zero;   // Zero detection

    logic           clk, reset_b;

    initial clk = 1'b1;
    always #(`CLK_T/2) clk = ~clk;

    initial begin
        #(10) in1 = 8; in2 = 4; alu_control = 4'b0010;
        #(10) in1 = 8; in2 = 4; alu_control = 4'b0110;
        #(10) in1 = 8; in2 = 4; alu_control = 4'b0000; 
        #(10) in1 = 8; in2 = 4; alu_control = 4'b0001; 
        #(10) in1 = 8; in2 = 8; alu_control = 4'b0110;
    end

    alu dut (in1, in2, alu_control, result, zero);
endmodule