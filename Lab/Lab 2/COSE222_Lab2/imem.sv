/* ********************************************
 *	COSE222 Lab #2
 *
 *	Module: instruction memory (imem.sv)
 *	- 1 address input port
 *	- 32-bit 1 data output port
 *	- A single entry size is 32 bit, which is equivalent to the RISC-V instruction size
 *
 *	Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

module imem
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10 )
(
    input   logic [IMEM_ADDR_WIDTH-1:0]   addr,
    output  logic [31:0]  dout
);
    logic [31:0] inst_data [IMEM_DEPTH-1:0];
    assign dout = inst_data [addr];

    initial begin
        inst_data [1] = 1;
        inst_data [2] = 4;
        inst_data [3] = 12;
        inst_data [4] = 16;
    end

endmodule