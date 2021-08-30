`timescale 1ns/1ps

module tb_xor2();
    logic [7:0] a, b, c;

    initial begin
        a = 0; b = 255;
        while(b >= 0)
        begin
            #(10) a = a + 1; b = b - 1;        
        end
    end
    xor2 dut (a, b, c);
endmodule
