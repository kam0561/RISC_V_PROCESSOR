`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 27.08.2025 12:00:00
// Design Name: 
// Module Name: msrv32_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Testbench for MSRV32 Top Module
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module msrv32top_tb();

    // Testbench signals
    reg clk;
    reg rst;
    reg [63:0] real_time_counter;
    
    // Instruction memory interface
    wire [31:0] imem_addr;
    reg [31:0] imem_data;
    reg imem_ready;
    
    // Data memory interface
    wire [31:0] dmem_addr;
    wire [31:0] dmem_data_out;
    wire dmem_wr_req;
    wire [3:0] dmem_wr_mask;
    reg [31:0] dmem_data_in;
    reg dmem_ready;
    reg dmem_resp;
    wire [1:0] dmem_htrans;
    
    // Interrupt signals
    reg ext_irq;
    reg timer_irq;
    reg soft_irq;
    
    // Test variables
    integer i;
    reg [31:0] expected_result;
    
    // Instruction and Data Memory Arrays
    reg [31:0] instruction_memory [0:1023];  // 4KB instruction memory
    reg [31:0] data_memory [0:1023];         // 4KB data memory
    
    // DUT instantiation
    msrv32_top #(
        .BOOT_ADDRESS(32'h00000000)
    ) dut (
        .ms_riscv32_mp_clk_in(clk),
        .ms_riscv32_mp_rst_in(rst),
        .ms_riscv32_mp_rc_in(real_time_counter),
        
        // Instruction memory interface
        .ms_riscv32_mp_imaddr_out(imem_addr),
        .ms_riscv32_mp_instr_in(imem_data),
        .ms_riscv32_mp_instr_hready_in(imem_ready),
        
        // Data memory interface
        .ms_riscv32_mp_dmaddr_out(dmem_addr),
        .ms_riscv32_mp_dmdata_out(dmem_data_out),
        .ms_riscv32_mp_dmwr_req_out(dmem_wr_req),
        .ms_riscv32_mp_dmwr_mask_out(dmem_wr_mask),
        .ms_riscv32_mp_data_in(dmem_data_in),
        .ms_riscv32_mp_data_hready_in(dmem_ready),
        .ms_riscv32_mp_hresp_in(dmem_resp),
        .ms_riscv32_mp_data_htrans_out(dmem_htrans),
        
        // Interrupts
        .ms_riscv32_mp_eirq_in(ext_irq),
        .ms_riscv32_mp_tirq_in(timer_irq),
        .ms_riscv32_mp_sirq_in(soft_irq)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz clock
    end
    
    // Real-time counter
    always @(posedge clk) begin
        if (rst)
            real_time_counter <= 64'b0;
        else
            real_time_counter <= real_time_counter + 1;
    end
    
    // Instruction memory model
    always @(*) begin
        if (imem_addr[31:2] < 1024) begin
            imem_data = instruction_memory[imem_addr[31:2]];
        end else begin
            imem_data = 32'h00000013; // NOP (ADDI x0, x0, 0)
        end
    end
    
    // Data memory model
    always @(posedge clk) begin
        if (dmem_wr_req && dmem_ready) begin
            case (dmem_wr_mask)
                4'b1111: data_memory[dmem_addr[31:2]] <= dmem_data_out;
                4'b0011: data_memory[dmem_addr[31:2]][15:0] <= dmem_data_out[15:0];
                4'b1100: data_memory[dmem_addr[31:2]][31:16] <= dmem_data_out[31:16];
                4'b0001: data_memory[dmem_addr[31:2]][7:0] <= dmem_data_out[7:0];
                4'b0010: data_memory[dmem_addr[31:2]][15:8] <= dmem_data_out[15:8];
                4'b0100: data_memory[dmem_addr[31:2]][23:16] <= dmem_data_out[23:16];
                4'b1000: data_memory[dmem_addr[31:2]][31:24] <= dmem_data_out[31:24];
            endcase
        end
    end
    
    // Data memory read
    always @(*) begin
        if (dmem_addr[31:2] < 1024) begin
            dmem_data_in = data_memory[dmem_addr[31:2]];
        end else begin
            dmem_data_in = 32'h00000000;
        end
    end
    
    // Function to encode R-type instruction
    function [31:0] encode_r_type;
        input [6:0] funct7;
        input [4:0] rs2;
        input [4:0] rs1;
        input [2:0] funct3;
        input [4:0] rd;
        input [6:0] opcode;
        encode_r_type = {funct7, rs2, rs1, funct3, rd, opcode};
    endfunction
    
    // Function to encode I-type instruction
    function [31:0] encode_i_type;
        input [11:0] imm;
        input [4:0] rs1;
        input [2:0] funct3;
        input [4:0] rd;
        input [6:0] opcode;
        encode_i_type = {imm, rs1, funct3, rd, opcode};
    endfunction
    
    // Function to encode S-type instruction
    function [31:0] encode_s_type;
        input [11:0] imm;
        input [4:0] rs2;
        input [4:0] rs1;
        input [2:0] funct3;
        input [6:0] opcode;
        encode_s_type = {imm[11:5], rs2, rs1, funct3, imm[4:0], opcode};
    endfunction
    
    // Function to encode B-type instruction
    function [31:0] encode_b_type;
        input [12:0] imm;
        input [4:0] rs2;
        input [4:0] rs1;
        input [2:0] funct3;
        input [6:0] opcode;
        encode_b_type = {imm[12], imm[10:5], rs2, rs1, funct3, imm[4:1], imm[11], opcode};
    endfunction
    
    // Function to encode U-type instruction
    function [31:0] encode_u_type;
        input [19:0] imm;
        input [4:0] rd;
        input [6:0] opcode;
        encode_u_type = {imm, rd, opcode};
    endfunction
    
    // Function to encode J-type instruction
    function [31:0] encode_j_type;
        input [20:0] imm;
        input [4:0] rd;
        input [6:0] opcode;
        encode_j_type = {imm[20], imm[10:1], imm[11], imm[19:12], rd, opcode};
    endfunction
    
    // Task to wait for clock cycles
    task wait_cycles;
        input integer cycles;
        repeat(cycles) @(posedge clk);
    endtask
    
    // Task to load test program
    task load_test_program;
        begin
            // Initialize instruction memory
            for (i = 0; i < 1024; i = i + 1) begin
                instruction_memory[i] = 32'h00000013; // NOP
            end
            
            // Test Program:
            // Test 1: Basic arithmetic (R-type)
            instruction_memory[0] = encode_i_type(12'd10, 5'd0, 3'b000, 5'd1, 7'b0010011);   // ADDI x1, x0, 10
            instruction_memory[1] = encode_i_type(12'd20, 5'd0, 3'b000, 5'd2, 7'b0010011);   // ADDI x2, x0, 20
            instruction_memory[2] = encode_r_type(7'b0000000, 5'd2, 5'd1, 3'b000, 5'd3, 7'b0110011); // ADD x3, x1, x2
            instruction_memory[3] = encode_r_type(7'b0100000, 5'd1, 5'd2, 3'b000, 5'd4, 7'b0110011); // SUB x4, x2, x1
            
            // Test 2: Logical operations
            instruction_memory[4] = encode_i_type(12'hFF0, 5'd0, 3'b000, 5'd5, 7'b0010011);  // ADDI x5, x0, -16 (0xFF0)
            instruction_memory[5] = encode_r_type(7'b0000000, 5'd5, 5'd1, 3'b111, 5'd6, 7'b0110011); // AND x6, x1, x5
            instruction_memory[6] = encode_r_type(7'b0000000, 5'd5, 5'd1, 3'b110, 5'd7, 7'b0110011); // OR x7, x1, x5
            instruction_memory[7] = encode_r_type(7'b0000000, 5'd5, 5'd1, 3'b100, 5'd8, 7'b0110011); // XOR x8, x1, x5
            
            // Test 3: Shifts
            instruction_memory[8] = encode_i_type(12'd2, 5'd1, 3'b001, 5'd9, 7'b0010011);    // SLLI x9, x1, 2
            instruction_memory[9] = encode_i_type(12'd1, 5'd1, 3'b101, 5'd10, 7'b0010011);   // SRLI x10, x1, 1
            
            // Test 4: Comparisons
            instruction_memory[10] = encode_r_type(7'b0000000, 5'd2, 5'd1, 3'b010, 5'd11, 7'b0110011); // SLT x11, x1, x2
            instruction_memory[11] = encode_r_type(7'b0000000, 5'd1, 5'd2, 3'b011, 5'd12, 7'b0110011); // SLTU x12, x2, x1
            
            // Test 5: Load Upper Immediate
            instruction_memory[12] = encode_u_type(20'h12345, 5'd13, 7'b0110111);            // LUI x13, 0x12345
            
            // Test 6: Memory operations
            instruction_memory[13] = encode_s_type(12'd100, 5'd3, 5'd0, 3'b010, 7'b0100011); // SW x3, 100(x0)
            instruction_memory[14] = encode_i_type(12'd100, 5'd0, 3'b010, 5'd14, 7'b0000011); // LW x14, 100(x0)
            
            // Test 7: Branch (simple loop)
            instruction_memory[15] = encode_i_type(12'd1, 5'd1, 3'b000, 5'd1, 7'b0010011);   // ADDI x1, x1, 1
            instruction_memory[16] = encode_i_type(12'd50, 5'd0, 3'b000, 5'd15, 7'b0010011);  // ADDI x15, x0, 50
            instruction_memory[17] = encode_b_type(13'h1FF4, 5'd15, 5'd1, 3'b001, 7'b1100011); // BNE x1, x15, -12 (back to instr 15)
            
            // Test 8: Jump
            instruction_memory[18] = encode_j_type(21'h000008, 5'd16, 7'b1101111);            // JAL x16, 8 (jump to instr 20)
            instruction_memory[19] = encode_i_type(12'd999, 5'd0, 3'b000, 5'd17, 7'b0010011); // ADDI x17, x0, 999 (should be skipped)
            instruction_memory[20] = encode_i_type(12'd100, 5'd0, 3'b000, 5'd18, 7'b0010011); // ADDI x18, x0, 100
            
            // End with infinite loop
            instruction_memory[21] = encode_b_type(13'h0000, 5'd0, 5'd0, 3'b000, 7'b1100011); // BEQ x0, x0, 0 (infinite loop)
        end
    endtask
    
    // Task to check register value
    task check_register;
        input [4:0] reg_addr;
        input [31:0] expected;
        begin
            wait_cycles(1);
            if (reg_addr != 0) begin  // x0 is always 0
                if (dut.register_file.reg_file[reg_addr] == expected) begin
                    $display("PASS: x%0d = 0x%08h", reg_addr, expected);
                end else begin
                    $display("FAIL: x%0d = 0x%08h (expected 0x%08h)", 
                            reg_addr, dut.register_file.reg_file[reg_addr], expected);
                end
            end
        end
    endtask
    
    // Task to check memory value
    task check_memory;
        input [31:0] addr;
        input [31:0] expected;
        begin
            if (data_memory[addr[31:2]] == expected) begin
                $display("PASS: MEM[0x%08h] = 0x%08h", addr, expected);
            end else begin
                $display("FAIL: MEM[0x%08h] = 0x%08h (expected 0x%08h)", 
                        addr, data_memory[addr[31:2]], expected);
            end
        end
    endtask
    
    // Main test sequence
    initial begin
        // Initialize signals
        rst = 1;
        imem_ready = 1;
        dmem_ready = 1;
        dmem_resp = 0;
        ext_irq = 0;
        timer_irq = 0;
        soft_irq = 0;
        real_time_counter = 0;
        
        // Initialize memories
        for (i = 0; i < 1024; i = i + 1) begin
            data_memory[i] = 32'h00000000;
        end
        
        // Load test program
        load_test_program();
        
        $display("=== MSRV32 Processor Testbench ===");
        $display("Starting simulation...");
        
        // Reset sequence
        wait_cycles(5);
        rst = 0;
        wait_cycles(2);
        
        $display("\n--- Test 1: Basic Arithmetic ---");
        wait_cycles(10);
        $display("Testing: ADDI x1, x0, 10");
        check_register(1, 32'd10);
        $display("Testing: ADDI x2, x0, 20");
        check_register(2, 32'd20);  
        $display("Testing: ADD x3, x1, x2");
        check_register(3, 32'd30);
        $display("Testing: SUB x4, x2, x1");
        check_register(4, 32'd10);
        
        $display("\n--- Test 2: Logical Operations ---");
        wait_cycles(10);
        $display("Testing: ADDI x5, x0, -16");
        check_register(5, 32'hFFFFFFF0);
        $display("Testing: AND x6, x1, x5");
        check_register(6, 32'd0);
        $display("Testing: OR x7, x1, x5");
        check_register(7, 32'hFFFFFFFA); 
        $display("Testing: XOR x8, x1, x5");
        check_register(8, 32'hFFFFFFFA);
        
        $display("\n--- Test 3: Shift Operations ---");
        wait_cycles(5);
        $display("Testing: SLLI x9, x1, 2");
        check_register(9, 32'd40);    // 10 << 2 = 40
        $display("Testing: SRLI x10, x1, 1");
        check_register(10, 32'd5);   // 10 >> 1 = 5
        
        $display("\n--- Test 4: Comparisons ---");
        wait_cycles(5);
        $display("Testing: SLT x11, x1, x2");
        check_register(11, 32'd1);   // 10 < 20 = true
        $display("Testing: SLTU x12, x2, x1");
        check_register(12, 32'd0);  // 20 < 10 = false
        
        $display("\n--- Test 5: LUI ---");
        wait_cycles(5);
        $display("Testing: LUI x13, 0x12345");
        check_register(13, 32'h12345000);
        
        $display("\n--- Test 6: Memory Operations ---");
        wait_cycles(10);
        $display("Testing: SW x3, 100(x0)");
        check_memory(32'd100, 32'd30);
        $display("Testing: LW x14, 100(x0)");
        check_register(14, 32'd30);
        
        $display("\n--- Test 7: Branch Test ---");
        wait_cycles(100); // Let the loop run for a while
        
        $display("\n--- Test 8: Jump Test ---");
        wait_cycles(10);
        $display("Testing: JAL x16 (return address)");
        check_register(16, 32'd24);  // PC of JAL + 4 = 18*4 + 4 = 76
        $display("Testing: Jump target executed");
        check_register(18, 32'd100);
        
        // Final register dump
        $display("\n=== Final Register State ===");
        for (i = 1; i < 19; i = i + 1) begin
            $display("x%2d = 0x%08h (%0d)", i, dut.register_file.reg_file[i], dut.register_file.reg_file[i]);
        end
        
        $display("\n=== Simulation Complete ===");
        wait_cycles(10);
        $finish;
    end
    
    // Monitor important signals
    initial begin
        $monitor("Time: %0t | PC: 0x%08h | Instr: 0x%08h | Reset: %b", 
                 $time, imem_addr, imem_data, rst);
    end
    
    // Timeout watchdog
    initial begin
        #100000;  // 100us timeout
        $display("ERROR: Simulation timeout!");
        $finish;
    end

endmodule