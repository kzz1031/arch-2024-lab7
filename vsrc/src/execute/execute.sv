`ifndef __EXECUTE_SV
`define __EXECUTE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr_pkg.sv"
`include "src/execute/ALU_control.sv"
`include "src/execute/ALU.sv"
`include "src/execute/ALU_csr.sv"
`include "src/execute/data_a_select.sv"
`include "src/execute/divide.sv"
`include "src/execute/multiply.sv"
`else

`endif 
module execute
    import common::*;
    import csr_pkg::*;(
    input logic clk,
    input logic rst,
    input logic decode_valid,
    input u32 reg_decode_ins,
    input u5 reg_decode_rd,
    input logic is_word,
    input u64 reg_decode_pc,
    input u64 reg_decode_rd1,reg_decode_rd2,
    input msize_t reg_decode_msize,
    input logic reg_decode_sig,
    input u2  ctrl_ALU_op,
    input u64 reg_offset,
    input logic ctrl_ALU_src,
    input logic ctrl_mem_r,ctrl_mem_w,ctrl_reg_w,
    input logic stall,

    input csr_decode csr_inf,

    output logic reg_execute_mem_r,reg_execute_mem_w,reg_execute_reg_w,
    output u32 reg_execute_ins,
    output msize_t reg_execute_msize,
    output logic reg_execute_sig,
    output logic execute_valid,
    output u64  reg_execute_rd2,
    output u64 reg_execute_data_out,reg_execute_pc,reg_execute_csr_data_out,
    output logic reg_execute_zero_flag,
    output u5  reg_execute_rd,
    output logic execute_stall
);

ALU_CTR ALU_ctrl;
u64 ALU_data_out,data_a, csr_data_out;
logic mul_end,mul_valid,divide_end,divide_valid,ALU_zero_flag;
u64 multiply_data_out,divide_data_out,remain;
BRA take_branch;

always_ff @( posedge clk ) begin
    if( decode_valid )begin
        reg_execute_ins <= reg_decode_ins;
        reg_execute_mem_r <= ctrl_mem_r;
        reg_execute_mem_w <= ctrl_mem_w;
        reg_execute_pc <= reg_decode_pc;
        reg_execute_rd <= reg_decode_rd;
        reg_execute_reg_w  <= ctrl_reg_w;
        reg_execute_ins <= reg_decode_ins;
        reg_execute_rd2 <= reg_decode_rd2;
        reg_execute_msize <= reg_decode_msize;
        reg_execute_sig   <= reg_decode_sig;
        reg_execute_zero_flag <= ALU_zero_flag;
        reg_execute_csr_data_out  <= csr_data_out;
    end
    if( execute_valid ) execute_valid <= 0;
    else if(ALU_ctrl == MUL || ALU_ctrl == DIV || ALU_ctrl == DIVU || ALU_ctrl == REM || ALU_ctrl == REMU) begin
        if(mul_end & (!stall)) begin
            execute_valid <= 1; 
            reg_execute_data_out <= multiply_data_out;
        end
        else if(divide_end & (!stall)) begin
            execute_valid <= 1;
            reg_execute_data_out <= divide_data_out;
        end 
    end
    else if( (decode_valid & (!stall)) || ((!stall) & (reg_decode_pc != reg_execute_pc) ))begin
        execute_valid <= 1;
        reg_execute_data_out <= ALU_data_out;
    end 
end

assign execute_stall = divide_valid || mul_valid;

data_a_select data_a_select(
	.op				(OPC'(reg_decode_ins[6:0])),
	.pc				(reg_decode_pc),
	.reg_read_data_0(reg_decode_rd1),
	.data_a			(data_a)
);

ALU_control ALU_control(
    .funct3			(reg_decode_ins[14:12]),
    .funct7			(reg_decode_ins[31:25]),
    .ALU_ctrl		(ALU_ctrl),
	.take_branch    (take_branch),
    .ctrl_ALU_op	(ctrl_ALU_op));

ALU ALU(
    .data_a			(data_a),
    .data_b			(ctrl_ALU_src ? reg_offset : reg_decode_rd2),
    .data_out		(ALU_data_out),
    .zero			(ALU_zero_flag),
	.take_branch    (take_branch),
	.is_word		(is_word),
    .ALU_ctrl		(ALU_ctrl));

ALU_csr ALU_csr(
    .t              (csr_inf.t),
    .zimm           (reg_offset),
    .pc             (reg_decode_pc),
    .rd1            (reg_decode_rd1),
    .func3          (csr_inf.func3),
    .csr_data_out   (csr_data_out)
);

multiply multiply(
	.clk			(clk),
	.rst			(rst),
	.ALU_ctrl		(ALU_ctrl),
	.data_a			(reg_decode_rd1),
	.data_b			(reg_decode_rd2),
    .decode_valid   (decode_valid),
	.multiply_data_out(multiply_data_out),
	.mul_end		(mul_end),
	.mul_valid    	(mul_valid),
    .stall          (stall),
	.is_word		(is_word)
);

divide divide(
	.clk			(clk),
	.rst			(rst),
	.ALU_ctrl		(ALU_ctrl),
	.data_a			(reg_decode_rd1),
	.data_b			(reg_decode_rd2),
    .decode_valid   (decode_valid),
	.divide_data_out(divide_data_out),
	.remain 		(remain),
	.is_word 		(is_word),
	.divide_end		(divide_end),
	.divide_valid   (divide_valid),
    .stall          (stall)
);

endmodule

`endif 