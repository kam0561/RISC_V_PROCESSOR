`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.08.2025 23:28:27
// Design Name: 
// Module Name: msrv32_integer_file_tb
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
module msrv32_integer_file_tb();
    reg ms_riscv32_mp_clk_in;
    reg ms_riscv32_mp_rst_in;
    reg [4:0] rs_1_addr_in, rs_2_addr_in;
    reg [4:0] rd_addr_in;
    reg wr_en_in;
    reg [31:0] rd_in;
    wire [31:0] rs_1_out, rs_2_out;
    
    // Step1 : Instantiate the design INTEGER_FILE
    msrv32_integer_file INTF(
        ms_riscv32_mp_clk_in, ms_riscv32_mp_rst_in,
        // connections with pipeline stage 2
        rs_1_addr_in, rs_2_addr_in,
        rs_1_out, rs_2_out,
        // connections with pipeline stage 3
        rd_addr_in,
        wr_en_in,
        rd_in
    );
    
    // Step2 : Write a task named "initialize" to initialize the inputs of DUT
    task initialize;
        begin
            rs_1_addr_in = 0;
            rs_2_addr_in = 0;
            rd_addr_in = 0;
            wr_en_in = 0;
            rd_in = 0;
        end
    endtask
    
    // Step3 : Write clock generation logic with 50% duty cycle
    initial
    begin
        #10 ms_riscv32_mp_clk_in = 0;
        forever
            #10 ms_riscv32_mp_clk_in = ~ms_riscv32_mp_clk_in;
    end
    
    // Step4 : Write reset logic
    task reset;
        begin
            #10;
            ms_riscv32_mp_rst_in = 1;
            #10;
            ms_riscv32_mp_rst_in = 0;
        end
    endtask
    
    // Step5 : Declare tasks with arguments for generating the stimulus
    task stimulus_write(input [4:0] rd_addr,
                       input wr_en,
                       input [31:0] rd);
        begin
            @(negedge ms_riscv32_mp_clk_in);
            rd_addr_in = rd_addr;
            rd_in = rd;
            wr_en_in = wr_en;
        end
    endtask
    
    task stimulus_read(input [4:0] rsl_addr, rs2_addr,
                      input wr_en);
        begin
            @(negedge ms_riscv32_mp_clk_in);
            rs_1_addr_in = rsl_addr;
            rs_2_addr_in = rs2_addr;
            wr_en_in = wr_en;
        end
    endtask
    
    // Step6 : Call the tasks within the initial process
    initial
    begin
        initialize;
        reset;
        stimulus_write(5'd5, 1'b1, 32'd30);
        stimulus_write(5'd6, 1'b1, 32'd50);
        stimulus_read(5'd6, 5'd5, 1'b0);
        #100 $finish;
    end
    
    // Step7 : Process to monitor the changes in the variables
    initial
        $monitor("Inputs rd_in=%d, rs1_addr=%d, rs2_addr=%d, write_en=%b, Outputs rs_l_out=%d, rs_2_out=%d", rd_in, rs_1_addr_in, rs_2_addr_in, wr_en_in, rs_1_out, rs_2_out);
endmodule