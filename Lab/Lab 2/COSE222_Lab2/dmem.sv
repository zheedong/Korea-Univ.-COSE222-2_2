/* ********************************************
 *	COSE222 Lab #2
 *
 *	Module: data memory (dmem.sv)
 *	- 1 address input port
 *	- 32-bit 1 data input and output ports
 *	- A single entry size is 64-bit
 *
 *	Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

module dmem
#(  parameter DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input   logic        clk,
    input   logic [DMEM_ADDR_WIDTH-1:0]   addr,
    input   logic [63:0]  din,
    input   logic   mem_read,
    input   logic   mem_write,
    output  logic   [63:0]  dout
);

    /* Data memory does not receive the clock signal in the textbook.
     * Without clock we need to implement the data memory with latches.
     * However, you must avoid generating latches in real RTL design.
     * If latches are generated after synthesis, then it means your design includes critical bugs.
     * Hence, in this design you are requested to design the data memory with the clock signal.
     * That means the written data is updated at the rising edge of the clock signal.
     */

    // Actually RISC-V supports misaligned data accesses to memory, however it this design the data memory will only support
    // the aligned memory accesses.

    logic   [63:0]  data[0:DMEM_DEPTH-1];

    // Write operation:
    always_ff @ (posedge clk) begin
        if (mem_write) begin       
            data[addr] = din;
        end
    end

    // Read operation:
    // - dout = 0 if (mem_read==0) 
    always_ff @ (posedge clk) begin
        if (mem_read == 1) begin
            assign dout = data[addr];
        end
        else begin
            assign dout = 0;
        end
    end

endmodule