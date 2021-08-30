/* ***********************************************
 *  COSE222 Lab #3
 *
 *  Module: testbench for single_cycle_cpu.sv
 *  -
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 **************************************************
 */

`timescale 1ns/1ps
`define CLK_T 10        // FILL THIS

module tb_single_cycle_cpu ();

    logic           clk, reset_b;

    initial clk = 1'b1;
    always #(`CLK_T/2) clk = ~clk;

    initial begin
        clk = 1'b1;
        reset_b = 1'b0;
        repeat (2) @ (posedge clk);
        #(1) reset_b = 1'b1;
    end

    int fout;

    initial begin
        fout = $fopen("./report.txt", "w");
        if (fout)
            $display("File was opened successfully: %d", fout);
        else
            $display("Failed to open the file: %d", fout);

        repeat (20) @ (posedge clk);
        #(`CLK_T/2)
        for (int i = 0; i < 32; i++) begin
            $fwrite(fout, "RF[%02d]: %016X\n", i, dut.u_regfile_0.rf_data[i]);
        end
        for (int i = 0; i < 8; i++) begin
            $fwrite(fout, "DMEM[%02d]: %016X\n", i, dut.u_dmem_0.data[i]);
        end
        
        $fclose(fout);
        $stop(1);
    end

    // INSTANTIATE DUT
    single_cycle_cpu dut (clk, reset_b);
endmodule