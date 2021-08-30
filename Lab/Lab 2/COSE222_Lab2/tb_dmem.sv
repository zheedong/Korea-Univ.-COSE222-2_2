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

module tb_dmem
#(  parameter DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )();

    logic   [DMEM_ADDR_WIDTH-1:0]   addr;
    logic   [63:0]  din;
    logic           mem_read;
    logic           mem_write;
    logic   [63:0]  dout;

    logic           clk, reset_b;

    initial clk = 1'b1;
    always #(`CLK_T/2) clk = ~clk;

    initial begin
        clk = 1'b1;
        reset_b = 1'b0;
        repeat (2) @ (posedge clk);
        #(10) reset_b = 1'b1;
        #(10) addr = 10; din = 20; mem_read = 0; mem_write = 1;
        #(10) addr = 6; din = 10; mem_read = 0; mem_write = 1;
        #(10) addr = 7; din = 40; mem_read = 0; mem_write = 1;
        #(10) addr = 10;  mem_read = 1; mem_write = 0;
        #(10) addr = 6;  mem_read = 1; mem_write = 0;
        #(10) addr = 7;  mem_read = 1; mem_write = 0;
    end

    dmem dut (clk, addr, din, mem_read, mem_write, dout);    
endmodule