`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 14.08.2025 01:52:27
// Design Name: 
// Module Name: msrv32_alu_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Testbench for msrv32_alu
// 
// Dependencies: msrv32_alu.v
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module msrv32_alu_tb;

// TB signals
reg [31:0] op_1_in, op_2_in;
reg [3:0]  opcode_in;
wire [31:0] result_out;

// Instantiate the DUT (Device Under Test)
msrv32_alu ALU (
    .op_1_in(op_1_in),
    .op_2_in(op_2_in),
    .opcode_in(opcode_in),
    .result_out(result_out)
);

// Task to initialize signals
task initialize;
begin
    op_1_in   = 32'd0;
    op_2_in   = 32'd0;
    opcode_in = 4'd0;
end
endtask

// Task to apply stimulus
task stimulus(
    input [31:0] op1,
    input [31:0] op2,
    input [3:0]  opcode
);
begin
    #10;
    op_1_in   = op1;
    op_2_in   = op2;
    opcode_in = opcode;
end 
endtask

// Main test sequence
initial begin
    initialize;
    stimulus(32'd20, 32'd40, 4'b0000); // Example: ADD
    stimulus(32'd20, 32'd40, 4'b1000); // Example: SUB
    stimulus(32'd60, 32'd50, 4'b0000); // Example: ADD
    #100 $finish;
end

// Monitor signal changes
initial
    $monitor("Time=%0t | Operand1=%d, Operand2=%d, Opcode=%b, Result=%d",
             $time, op_1_in, op_2_in, opcode_in, result_out);

endmodule
