`timescale 1ns / 1ps

module RISC_Processor_tb;
    reg clk;
    reg reset;

    RISC_Processor uut (.clk(clk), .reset(reset));

    always #5 clk = ~clk;
    always #10 reset = ~reset;

    initial begin
        clk = 0;
        reset = 1;
        #200 $finish;
    end
endmodule
