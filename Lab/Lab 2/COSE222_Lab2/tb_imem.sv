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

module tb_imem
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10 )();

    logic [IMEM_ADDR_WIDTH-1:0]   addr;
    logic [31:0]  dout;

    logic           clk, reset_b;

    initial clk = 1'b1;
    always #(`CLK_T/2) clk = ~clk;

    initial begin
        clk = 1'b1;
        reset_b = 1'b0;
        repeat (2) @ (posedge clk);
        #(1) reset_b = 1'b1;

        #(10) addr = 1;
        #(10) addr = 2;
        #(10) addr = 3;
        #(10) addr = 4;
    end

    imem dut (addr, dout);
endmodule