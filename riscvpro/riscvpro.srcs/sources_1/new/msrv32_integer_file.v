`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.08.2025 22:52:37
// Design Name: 
// Module Name: msrv32_integer_file
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module msrv32_integer_file(
            // port declarations
            input ms_riscv32_mp_clk_in, ms_riscv32_mp_rst_in,
            //stage 2 connections
            input [4:0] rs_1_addr_in, rs_2_addr_in,
            output [31:0] rs_1_out, rs_2_out,
            // stage 3 connections
            input [4:0] rd_addr_in,
            input wr_en_in,
            input [31:0] rd_in
    );
    
    reg signed[31:0] reg_file [31:0];
    
    wire fwd_op1_enable, fwd_op2_enable;
    integer i;
    
    assign fwd_op1_enable = (rs_1_addr_in == rd_addr_in && wr_en_in == 1'b1) ? 1'b1 : 1'b0;
    assign fwd_op2_enable = (rs_2_addr_in == rd_addr_in && wr_en_in == 1'b1) ? 1'b1 : 1'b0;
    
    always @ (posedge ms_riscv32_mp_clk_in or posedge ms_riscv32_mp_rst_in)
        begin
            if(ms_riscv32_mp_rst_in)
                begin
                    for(i=0; i<32; i=i+1)
                        reg_file[i] <= 32'b0;
                    end
            else if(wr_en_in && rd_addr_in)
                begin
                    reg_file[rd_addr_in] <= rd_in;
                end
        end
        
    assign rs_1_out = fwd_op1_enable == 1'b1 ? rd_in : reg_file[rs_1_addr_in];
    assign rs_2_out = fwd_op2_enable == 1'b1 ? rd_in : reg_file[rs_2_addr_in];
endmodule
