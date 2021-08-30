module xor2 (a, b, c);
    input [7:0] a, b;
    output [7:0] c;

    assign c = a ^ b;
endmodule
