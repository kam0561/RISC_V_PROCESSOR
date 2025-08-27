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
    output reg [31:0] ms_riscv32_mp_dmaddr_out,
    output reg [31:0] ms_riscv32_mp_dmdata_out,
    output reg ms_riscv32_mp_dmwr_req_out,
    output reg [3:0] ms_riscv32_mp_dmwr_mask_out,
    input [31:0] ms_riscv32_mp_data_in,
    input ms_riscv32_mp_data_hready_in,
    input ms_riscv32_mp_hresp_in,
    output reg [1:0] ms_riscv32_mp_data_htrans_out,
    
    //connections w interrupt controller
    input ms_riscv32_mp_eirq_in,
    input ms_riscv32_mp_tirq_in,
    input ms_riscv32_mp_sirq_in
);

    // Internal signals
    reg [31:0] pc;
    wire [31:0] instruction;
    wire [31:0] next_pc;
    
    // Instruction decode signals
    wire [6:0] opcode;
    wire [4:0] rd, rs1, rs2;
    wire [2:0] funct3;
    wire [6:0] funct7;
    wire [31:0] immediate;
    
    // Register file signals
    wire [31:0] rs1_data, rs2_data;
    reg rf_wr_en;
    reg [4:0] rf_rd_addr;
    reg [31:0] rf_rd_data;
    
    // ALU signals
    wire [31:0] alu_op1, alu_op2;
    wire [31:0] alu_result;
    reg [3:0] alu_opcode;
    
    // Control signals
    reg [1:0] alu_op1_sel, alu_op2_sel;
    reg [2:0] wb_sel;
    reg branch_taken;
    reg mem_read, mem_write;
    reg jump;
    
    // Pipeline registers (simplified 2-stage)
    reg [31:0] if_id_pc, if_id_instruction;
    reg [31:0] id_ex_pc, id_ex_rs1_data, id_ex_rs2_data, id_ex_immediate;
    reg [4:0] id_ex_rd;
    reg [2:0] id_ex_funct3;
    reg [6:0] id_ex_opcode;
    reg id_ex_rf_wr_en, id_ex_mem_read, id_ex_mem_write;
    
    // Instruction fetch
    assign instruction = ms_riscv32_mp_instr_in;
    assign ms_riscv32_mp_imaddr_out = pc;
    
    // PC logic
    always @(posedge ms_riscv32_mp_clk_in or posedge ms_riscv32_mp_rst_in) begin
        if (ms_riscv32_mp_rst_in) begin
            pc <= BOOT_ADDRESS;
        end else if (ms_riscv32_mp_instr_hready_in) begin
            if (jump || branch_taken) begin
                pc <= alu_result;
            end else begin
                pc <= pc + 4;
            end
        end
    end
    
    // IF/ID Pipeline Register
    always @(posedge ms_riscv32_mp_clk_in or posedge ms_riscv32_mp_rst_in) begin
        if (ms_riscv32_mp_rst_in) begin
            if_id_pc <= 32'b0;
            if_id_instruction <= 32'b0;
        end else if (ms_riscv32_mp_instr_hready_in) begin
            if_id_pc <= pc;
            if_id_instruction <= instruction;
        end
    end
    
    // Instruction decode
    assign opcode = if_id_instruction[6:0];
    assign rd = if_id_instruction[11:7];
    assign funct3 = if_id_instruction[14:12];
    assign rs1 = if_id_instruction[19:15];
    assign rs2 = if_id_instruction[24:20];
    assign funct7 = if_id_instruction[31:25];
    
    // Immediate generation (simplified)
    reg [31:0] imm_gen;
    always @(*) begin
        case (opcode)
            7'b0010011, 7'b0000011: // I-type
                imm_gen = {{20{if_id_instruction[31]}}, if_id_instruction[31:20]};
            7'b0100011: // S-type
                imm_gen = {{20{if_id_instruction[31]}}, if_id_instruction[31:25], if_id_instruction[11:7]};
            7'b1100011: // B-type
                imm_gen = {{19{if_id_instruction[31]}}, if_id_instruction[31], if_id_instruction[7], if_id_instruction[30:25], if_id_instruction[11:8], 1'b0};
            7'b0110111, 7'b0010111: // U-type
                imm_gen = {if_id_instruction[31:12], 12'b0};
            7'b1101111: // J-type
                imm_gen = {{11{if_id_instruction[31]}}, if_id_instruction[31], if_id_instruction[19:12], if_id_instruction[20], if_id_instruction[30:21], 1'b0};
            default:
                imm_gen = 32'b0;
        endcase
    end
    
    assign immediate = imm_gen;
    
    // ID/EX Pipeline Register
    always @(posedge ms_riscv32_mp_clk_in or posedge ms_riscv32_mp_rst_in) begin
        if (ms_riscv32_mp_rst_in) begin
            id_ex_pc <= 32'b0;
            id_ex_rs1_data <= 32'b0;
            id_ex_rs2_data <= 32'b0;
            id_ex_immediate <= 32'b0;
            id_ex_rd <= 5'b0;
            id_ex_funct3 <= 3'b0;
            id_ex_opcode <= 7'b0;
            id_ex_rf_wr_en <= 1'b0;
            id_ex_mem_read <= 1'b0;
            id_ex_mem_write <= 1'b0;
        end else begin
            id_ex_pc <= if_id_pc;
            id_ex_rs1_data <= rs1_data;
            id_ex_rs2_data <= rs2_data;
            id_ex_immediate <= immediate;
            id_ex_rd <= rd;
            id_ex_funct3 <= funct3;
            id_ex_opcode <= opcode;
            id_ex_rf_wr_en <= rf_wr_en;
            id_ex_mem_read <= mem_read;
            id_ex_mem_write <= mem_write;
        end
    end
    
    // Control unit (simplified)
    always @(*) begin
        // Default values
        rf_wr_en = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        jump = 1'b0;
        branch_taken = 1'b0;
        alu_op1_sel = 2'b00; // rs1
        alu_op2_sel = 2'b00; // rs2
        wb_sel = 3'b000;     // alu_result
        alu_opcode = 4'b0000;
        
        case (opcode)
            7'b0110011: begin // R-type
                rf_wr_en = 1'b1;
                alu_opcode = {funct7[5], funct3};
            end
            7'b0010011: begin // I-type (immediate)
                rf_wr_en = 1'b1;
                alu_op2_sel = 2'b01; // immediate
                alu_opcode = {1'b0, funct3};
                if (funct3 == 3'b101) // SRAI/SRLI
                    alu_opcode = {funct7[5], funct3};
            end
            7'b0000011: begin // Load
                rf_wr_en = 1'b1;
                mem_read = 1'b1;
                alu_op2_sel = 2'b01; // immediate
                wb_sel = 3'b001; // memory data
                alu_opcode = 4'b0000; // ADD
            end
            7'b0100011: begin // Store
                mem_write = 1'b1;
                alu_op2_sel = 2'b01; // immediate
                alu_opcode = 4'b0000; // ADD
            end
            7'b1100011: begin // Branch
                alu_op2_sel = 2'b00; // rs2
                case (funct3)
                    3'b000: alu_opcode = 4'b1000; // BEQ (SUB for comparison)
                    3'b001: alu_opcode = 4'b1000; // BNE (SUB for comparison)
                    default: alu_opcode = 4'b1000;
                endcase
                // Branch taken logic (simplified)
                case (funct3)
                    3'b000: branch_taken = (rs1_data == rs2_data); // BEQ
                    3'b001: branch_taken = (rs1_data != rs2_data); // BNE
                    default: branch_taken = 1'b0;
                endcase
            end
            7'b1101111: begin // JAL
                rf_wr_en = 1'b1;
                jump = 1'b1;
                alu_op1_sel = 2'b01; // PC
                alu_op2_sel = 2'b01; // immediate
                wb_sel = 3'b010; // PC + 4
                alu_opcode = 4'b0000; // ADD
            end
            7'b0110111: begin // LUI
                rf_wr_en = 1'b1;
                wb_sel = 3'b011; // immediate
            end
            default: begin
                // NOP or unsupported instruction
            end
        endcase
    end
    
    // ALU operand selection
    assign alu_op1 = (alu_op1_sel == 2'b01) ? id_ex_pc : id_ex_rs1_data;
    assign alu_op2 = (alu_op2_sel == 2'b01) ? id_ex_immediate : id_ex_rs2_data;
    
    // Write-back data selection
    always @(*) begin
        case (wb_sel)
            3'b000: rf_rd_data = alu_result;           // ALU result
            3'b001: rf_rd_data = ms_riscv32_mp_data_in; // Memory data
            3'b010: rf_rd_data = id_ex_pc + 4;         // PC + 4
            3'b011: rf_rd_data = id_ex_immediate;      // Immediate
            default: rf_rd_data = alu_result;
        endcase
    end
    
    // Memory interface
    always @(*) begin
        ms_riscv32_mp_dmaddr_out = alu_result;
        ms_riscv32_mp_dmdata_out = id_ex_rs2_data;
        ms_riscv32_mp_dmwr_req_out = id_ex_mem_write;
        ms_riscv32_mp_dmwr_mask_out = 4'b1111; // Full word access
        ms_riscv32_mp_data_htrans_out = (id_ex_mem_read || id_ex_mem_write) ? 2'b10 : 2'b00;
    end
    
    // Register file write control
    always @(*) begin
        rf_rd_addr = id_ex_rd;
    end
    
    // Module instantiations
    msrv32_integer_file register_file (
        .ms_riscv32_mp_clk_in(ms_riscv32_mp_clk_in),
        .ms_riscv32_mp_rst_in(ms_riscv32_mp_rst_in),
        .rs_1_addr_in(rs1),
        .rs_2_addr_in(rs2),
        .rs_1_out(rs1_data),
        .rs_2_out(rs2_data),
        .rd_addr_in(rf_rd_addr),
        .wr_en_in(id_ex_rf_wr_en),
        .rd_in(rf_rd_data)
    );
    
    msrv32_alu alu (
        .op_1_in(alu_op1),
        .op_2_in(alu_op2),
        .opcode_in(alu_opcode),
        .result_out(alu_result)
    );

endmodule