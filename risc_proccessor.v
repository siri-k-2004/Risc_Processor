`timescale 1ns / 1ps

// ALU Module
module ALU (
    input [3:0] A, B,
    input [3:0] opcode,
    output reg [3:0] result,
    output reg zero
);
    always @(*) begin
        case (opcode)
            4'b0000: result = A & B;
            4'b0001: result = A | B;
            4'b0010: result = A + B;
            4'b0011: result = A - B;
            4'b0100: result = A ^ B;
            4'b0101: result = ~A;
            4'b0110: result = A + 1;
            4'b0111: result = A - 1;
            default: result = 4'b0000;
        endcase
        zero = (result == 4'b0000);
    end
endmodule

// Register File
module RegisterFile (
    input clk,
    input regWrite,
    input [1:0] rs, rt, rd,
    input [3:0] writeData,
    output [3:0] readData1, readData2
);
    reg [3:0] registers [3:0];
    initial begin
        registers[0] = 4'b0001;
        registers[1] = 4'b0010;
        registers[2] = 4'b0011;
        registers[3] = 4'b0100;
    end
    assign readData1 = registers[rs];
    assign readData2 = registers[rt];
    always @(posedge clk) begin
        if (regWrite)
            registers[rd] <= writeData;
    end
endmodule

// Memory
module Memory (
    input clk,
    input memWrite,
    input memRead,
    input [3:0] address,
    input [3:0] writeData,
    output reg [3:0] readData
);
    reg [3:0] memory [15:0];
    integer i;
    initial begin
        for (i = 0; i < 16; i = i + 1)
            memory[i] = 4'b0000;
        memory[3] = 4'b1010; // Memory[3] = 10 (for LOAD test)
    end
    always @(posedge clk) begin
        if (memWrite)
            memory[address] <= writeData;
    end
    always @(*) begin
        if (memRead)
            readData = memory[address];
        else
            readData = 4'b0000;
    end
endmodule

// Control Unit
module ControlUnit (
    input [7:0] instruction,
    output [3:0] opcode,
    output [1:0] rs, rt
);
    assign opcode = instruction[7:4];
    assign rs = instruction[3:2];
    assign rt = instruction[1:0];
endmodule

// Main Processor Module
module RISC_Processor (
    input clk,
    input reset
);
    reg [3:0] PC;
    wire [7:0] instruction;
    wire [3:0] opcode;
    wire [1:0] rs, rt, rd;
    wire [3:0] readData1, readData2, ALU_result, mem_out;
    wire zero;

    reg regWrite, memWrite, memRead;
    reg [3:0] writeBack;
    reg [7:0] instruction_memory [15:0];

    initial begin
        PC = 0;
        regWrite = 0;
        memRead = 0;
        memWrite = 0;
        writeBack = 0;
        instruction_memory[0] = 8'b00100001; // ADD R0 = R0 + R1
        instruction_memory[1] = 8'b00110010; // SUB R0 = R0 - R2
        instruction_memory[2] = 8'b00000110; // AND R0 = R1 & R2
        instruction_memory[3] = 8'b00010110; // OR R0 = R1 | R2
        instruction_memory[4] = 8'b01000110; // XOR R0 = R1 ^ R2
        instruction_memory[5] = 8'b01010000; // NOT R0 = ~R0
        instruction_memory[6] = 8'b01100000; // INC R0
        instruction_memory[7] = 8'b01110000; // DEC R0
        instruction_memory[8] = 8'b10000011; // LOAD R0 from M[3]
        instruction_memory[9] = 8'b10010010; // STORE R0 to M[2]
    end

    assign instruction = instruction_memory[PC];
    assign rd = rs;

    ControlUnit CU(instruction, opcode, rs, rt);
    RegisterFile RF(clk, regWrite, rs, rt, rd, writeBack, readData1, readData2);
    ALU alu(readData1, readData2, opcode, ALU_result, zero);
    Memory mem(clk, memWrite, memRead, readData2, readData1, mem_out);

    always @(posedge clk or posedge reset) begin
        if (reset)
            PC <= 0;
        else
            PC <= PC + 1;

        case (opcode)
            4'b1000: begin // LOAD
                regWrite <= 1;
                memRead <= 1;
                memWrite <= 0;
                writeBack <= mem_out;
            end
            4'b1001: begin // STORE
                regWrite <= 0;
                memRead <= 0;
                memWrite <= 1;
                writeBack <= 4'b0000; // defined
            end
            default: begin // ALU ops
                regWrite <= 1;
                memRead <= 0;
                memWrite <= 0;
                writeBack <= ALU_result;
            end
        endcase
    end
endmodule
