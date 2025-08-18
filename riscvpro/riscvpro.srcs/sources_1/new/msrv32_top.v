`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 19.08.2025 00:45:14
// Design Name: 
// Module Name: msrv32_top
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


module msrv32_top #(
    parameter BOOT_ADDRESS = 32'h00000000
    
    )(
     input ms_riscv32_mp_clk_in,
     input ms_riscv32_mp_rst_in,
     //connection w real-time counter
     input [63:0] ms_riscv32_mp_rc_in,
     
     //connections w instruction memory
     output [31:0] ms_riscv32_mp_imaddr_out,
     input [31:0] ms_riscv32_mp_instr_in,
     input ms_riscv32_mp_instr_hready_in,
     
     //connections w data memory
     output [31:0] ms_riscv32_mp_dmaddr_out,
     output [31:0] ms_riscv32_mp_dmdata_out,
     output ms_riscv32_mp_dmwr_req_out,
     output [3:0] ms_riscv32_mp_dmwr_mask_out,
     input [31:0] ms_riscv32_mp_data_in,
     input ms_riscv32_mp_data_hready_in,
     input ms_riscv32_mp_hresp_in,
     output [1:0] ms_riscv32_mp_data_htrans_out,
     
     //connections w interrupt controller
     input ms_riscv32_mp_eirq_in,
     input ms_riscv32_mp_tirq_in,
     input ms_riscv32_mp_sirq_in
    );
    
    
endmodule
