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

module tb_regfile
#(  parameter   REG_WIDTH = 64 )();    // the width of register file

    logic   [4:0]   rs1;    // source register 1
    logic   [4:0]   rs2;    // source register 2
    logic   [4:0]   rd;     // destination register
    logic   [REG_WIDTH-1:0] rd_din;     // input data for rd
    logic           reg_write;      // RegWrite signal
    logic   [REG_WIDTH-1:0] rs1_dout;
    logic   [REG_WIDTH-1:0] rs2_dout;

    logic           clk, reset_b;

    initial clk = 1'b1;
    always #(`CLK_T/2) clk = ~clk;

    initial begin
        clk = 1'b1;
        reset_b = 1'b0;
        repeat (2) @ (posedge clk);
        #(1) reset_b = 1'b1;

        #(10) reg_write = 1; rd = 10; rd_din = 15;
        #(10) reg_write = 1; rd = 20; rd_din = 13;
        #(10) reg_write = 0; rs1 = 2; rs2 = 20;
        #(10) reg_write = 0; rs1 = 1; rs2 = 10;
    end

    regfile dut (clk, rs1, rs2, rd, rd_din, reg_write, rs1_dout, rs2_dout);
endmodule