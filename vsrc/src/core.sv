`ifndef __CORE_SV
`define __CORE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr_pkg.sv"
`include "src/fetch/fetch.sv"
`include "src/decode/decode.sv"
`include "src/execute/execute.sv"
`include "src/memory/memory.sv"
`include "src/writeback/writeback.sv"
`include "src/mmu/mmu.sv"
`endif
module core import common::*; 
			import csr_pkg::*;(
	input  logic       clk, reset,
	output ibus_req_t  ireq,
	input  ibus_resp_t iresp,
	output dbus_req_t  dreq,
	input  dbus_resp_t dresp,
	input  logic       trint, swint, exint
);
	/* TODO: Add your CPU-Core here. */
logic [63 : 0] RF [32];
logic [63 : 0] RF_next [32];
logic       ctrl_branch;  
logic 		decode_valid,reg_decode_sig,decode_stall;
logic 	    memory_valid,memory_stall;
logic       execute_valid,reg_execute_zero_flag,reg_execute_sig,reg_execute_reg_w;
logic       execute_stall;
u64 		reg_execute_data_out,reg_execute_pc,reg_execute_rd2;
u64 		reg_execute_csr_data_out,reg_memory_csr_data_out,reg_writeback_csr_data_out;
u64 		reg_writeback_data;
u5 			reg_writeback_rd;
logic 		writeback_valid,reg_writeback_reg_w;
logic signed [63 : 0] reg_offset;

csr_regs_t csr_regs,csr_regs_next;
csr_decode csr_inf;
logic is_csr;
u2 prvmode, prvmode_nxt;
u64 mmu_data;

always_ff @( posedge clk ) begin
	if(reset) prvmode <= 3;
end
always_comb begin
	if(csr_inf.mret) prvmode_nxt = 0;
	else if(csr_inf.ecall) prvmode_nxt = 2'b11;
	else prvmode_nxt = prvmode;
end

always_comb begin
	RF_next = RF;
    if (reg_writeback_reg_w)	RF_next[reg_writeback_rd] = csr_inf.csr_w ? csr_inf.t : reg_writeback_data;
	else                  		RF_next[reg_writeback_rd] = RF[reg_writeback_rd];
end // RF_next

u32 reg_fetch_ins;
u64 reg_fetch_pc,pc;
logic fetch_valid;

logic mmu_fetch_ok,mmu_memory_ok;

mmu mmu(
		.clk					(clk),
		.rst					(reset),
		.dreq					(dreq),
		.dresp					(dresp),
		.prvmode				(prvmode),
		.satp					(csr_regs.satp),
		.pc						(pc),
		.strobe					(strobe),
		.memory_busy			(memory_stall),
		.mem_store_data			(mem_store_data),
		.memory_addr            (reg_execute_data_out),
		.memory_r				(reg_execute_mem_r),
		.memory_w				(reg_execute_mem_w),
		.msize					(reg_execute_msize),
		.mmu_fetch_ok			(mmu_fetch_ok),
		.mmu_memory_ok			(mmu_memory_ok),
		.mmu_data				(mmu_data)
);

fetch fetch(.rst				(reset),
			.clk				(clk),
			.ireq				(ireq),
			.iresp				(iresp),
			.reg_fetch_ins		(reg_fetch_ins),
			.reg_fetch_pc		(reg_fetch_pc),
			.fetch_valid		(fetch_valid),
			.decode_valid		(decode_valid),
			.reg_offset			(reg_offset),
			.ctrl_branch		(ctrl_branch),
			.pc_branch			(reg_execute_data_out),
			.zero_flag			(reg_execute_zero_flag),
			.reg_execute_pc		(reg_execute_pc),
			.execute_valid		(execute_valid),	
			.stall				(execute_stall || memory_stall || decode_stall),
			.prvmode			(prvmode),
			.pc					(pc),
			.mmu_fetch_ok		(mmu_fetch_ok),
			.mmu_data			(mmu_data),
			.csr_regs			(csr_regs),
			.csr_inf			(csr_inf)
			);

logic [1:0] ctrl_ALU_op;
logic       ctrl_ALU_src;
logic       ctrl_reg_w;
logic       ctrl_mem_w;
logic       ctrl_mem_r;
logic       ctrl_mem_to_reg;
logic 		is_word;
u32   		reg_decode_ins;
u5 			reg_decode_rd;
// rs2
u64 		reg_decode_pc;
u32 		reg_busy; // 处理数据冒险
u64   		reg_decode_rd1,reg_decode_rd2;

msize_t reg_decode_msize;

decode decode(.clk          	(clk),
			  .rst  			(reset),
			  .reg_fetch_ins 	(reg_fetch_ins),
			  .reg_fetch_pc		(reg_fetch_pc),
			  .reg_decode_ins   (reg_decode_ins),
			  .reg_decode_sig	(reg_decode_sig),
			  .reg_decode_pc	(reg_decode_pc),
			  .reg_busy			(reg_busy),
			  .fetch_valid		(fetch_valid),
			  .decode_valid		(decode_valid),	
			  .ctrl_ALU_op		(ctrl_ALU_op),
			  .ctrl_ALU_src		(ctrl_ALU_src),
			  .ctrl_reg_w		(ctrl_reg_w),
			  .ctrl_mem_w		(ctrl_mem_w),
			  .ctrl_mem_r		(ctrl_mem_r),
			  .ctrl_mem_to_reg	(ctrl_mem_to_reg),
			  .ctrl_branch		(ctrl_branch),
			  .is_word			(is_word),
			  .reg_decode_rd1	(reg_decode_rd1),
			  .reg_decode_rd2	(reg_decode_rd2),
			  .reg_decode_msize	(reg_decode_msize),
			  .reg_decode_rd	(reg_decode_rd),
			  .reg_offset		(reg_offset),
			  .reg_writeback_data(reg_writeback_data),
			  .reg_writeback_csr_data_out(reg_writeback_csr_data_out),
			  .reg_writeback_rd	(reg_writeback_rd),

			  .csr_inf			(csr_inf),
			  .csr_regs_next	(csr_regs_next),
			  .csr_regs			(csr_regs),
			  .prvmode			(prvmode),
			  .is_csr			(is_csr),
			  .w_en				(reg_writeback_reg_w),
			  .RF				(RF),
			  .decode_stall		(decode_stall),
			  .stall			(memory_stall || execute_stall),
			  .pc				(pc));

u32 reg_execute_ins;
logic reg_execute_mem_w,reg_execute_mem_r;
msize_t reg_execute_msize;
u5 reg_execute_rd;

execute	execute(.clk					(clk),
				.rst					(reset),
				.decode_valid			(decode_valid),
				.reg_decode_ins 		(reg_decode_ins),
				.reg_decode_pc  		(reg_decode_pc),
				.reg_decode_msize		(reg_decode_msize),
				.reg_decode_rd			(reg_decode_rd),
				.reg_decode_sig			(reg_decode_sig),
				.is_word				(is_word),
				.ctrl_mem_w				(ctrl_mem_w),
				.ctrl_reg_w				(ctrl_reg_w),
			  	.ctrl_mem_r				(ctrl_mem_r),
				.ctrl_ALU_op			(ctrl_ALU_op),
				.reg_decode_rd1 		(reg_decode_rd1),
				.reg_decode_rd2 		(reg_decode_rd2),
				.reg_offset 			(reg_offset),
				.ctrl_ALU_src			(ctrl_ALU_src),
				.reg_execute_ins		(reg_execute_ins),
				.execute_valid			(execute_valid),
				.reg_execute_msize		(reg_execute_msize),
				.reg_execute_data_out	(reg_execute_data_out),
				.reg_execute_csr_data_out(reg_execute_csr_data_out),
				.reg_execute_zero_flag	(reg_execute_zero_flag),
				.reg_execute_mem_r		(reg_execute_mem_r),
				.reg_execute_mem_w		(reg_execute_mem_w),
				.reg_execute_sig 		(reg_execute_sig),
				.reg_execute_reg_w		(reg_execute_reg_w),
				.reg_execute_rd			(reg_execute_rd),
				.reg_execute_rd2		(reg_execute_rd2),
				.reg_execute_pc			(reg_execute_pc),
				.execute_stall			(execute_stall),
				.csr_inf				(csr_inf),
				.stall					(memory_stall)
				);

logic 	reg_memory_mem_r,reg_memory_reg_w,reg_memory_mem_w;
u64   	reg_memory_data_out;
u64 	reg_memory_ALU_data_out,reg_memory_pc,reg_memory_addr, mem_store_data;
u32 	reg_memory_ins;
strobe_t strobe;
u5		reg_memory_rd;

memory memory(
	.clk 					(clk),
	.rst					(reset),
	.dreq					(dreq),
	.dresp					(dresp),
	.reg_execute_rd 		(reg_execute_rd),
	.reg_execute_data_out	(reg_execute_data_out),
	.reg_execute_mem_r		(reg_execute_mem_r),
	.reg_execute_mem_w		(reg_execute_mem_w),
	.reg_execute_reg_w		(reg_execute_reg_w),
	.reg_execute_rd2		(reg_execute_rd2),
	.reg_execute_ins		(reg_execute_ins),
	.reg_execute_pc			(reg_execute_pc),
	.reg_execute_msize		(reg_execute_msize),
	.reg_execute_sig		(reg_execute_sig),
	.reg_execute_csr_data_out(reg_execute_csr_data_out),
	.reg_memory_reg_w		(reg_memory_reg_w),
	.reg_memory_mem_r		(reg_memory_mem_r),
	.reg_memory_mem_w		(reg_memory_mem_w),
	.reg_memory_ins			(reg_memory_ins),
	.reg_memory_pc			(reg_memory_pc),
	.reg_memory_data_out	(reg_memory_data_out),
	.reg_memory_ALU_data_out(reg_memory_ALU_data_out),
	.reg_memory_addr		(reg_memory_addr),
	.reg_memory_rd			(reg_memory_rd),
	.reg_memory_csr_data_out(reg_memory_csr_data_out),
	.execute_valid			(execute_valid),
	.memory_valid			(memory_valid),
	.memory_stall			(memory_stall),
	.satp					(csr_regs.satp),
	.prvmode				(prvmode),
	.strobe					(strobe),
	.mem_store_data			(mem_store_data),
	.mmu_data				(mmu_data),
	.mmu_memory_ok			(mmu_memory_ok)
);


writeback writeback(
	.clk					(clk),
	.rst					(reset),
	.memory_valid			(memory_valid),
	.reg_memory_ALU_data_out(reg_memory_ALU_data_out),
	.reg_memory_csr_data_out(reg_memory_csr_data_out),
	.reg_memory_data_out	(reg_memory_data_out),
	.reg_memory_rd			(reg_memory_rd),
	.reg_memory_reg_w		(reg_memory_reg_w),
	.reg_memory_ins			(reg_memory_ins),
	.reg_memory_pc			(reg_memory_pc),
	.reg_memory_mem_r		(reg_memory_mem_r),
	.reg_writeback_data		(reg_writeback_data),
	.reg_writeback_rd		(reg_writeback_rd),
	.reg_writeback_csr_data_out(reg_writeback_csr_data_out),
	.writeback_valid		(writeback_valid),
	.reg_writeback_reg_w	(reg_writeback_reg_w)
);

`ifdef VERILATOR
	DifftestInstrCommit DifftestInstrCommit(
		.clock              (clk),
		.coreid             (0),
		.index              (0),
		.valid              (writeback_valid),
		.pc                 (reg_memory_pc),
		.instr              (reg_memory_ins),
		.skip               (reg_memory_mem_w || reg_memory_mem_r ? reg_memory_addr[31] == 0 : 0),
		.isRVC              (0),
		.scFailed           (0),
		.wen                (reg_writeback_reg_w),
		.wdest              ({3'b0,reg_writeback_rd}),
		.wdata              (reg_writeback_data)
	);

	DifftestArchIntRegState DifftestArchIntRegState (
		.clock              (clk),
		.coreid             (0),
		.gpr_0              (0),
		.gpr_1              (RF_next[1]),
		.gpr_2              (RF_next[2]),
		.gpr_3              (RF_next[3]),
		.gpr_4              (RF_next[4]),
		.gpr_5              (RF_next[5]),
		.gpr_6              (RF_next[6]),
		.gpr_7              (RF_next[7]),
		.gpr_8              (RF_next[8]),
		.gpr_9              (RF_next[9]),
		.gpr_10             (RF_next[10]),
		.gpr_11             (RF_next[11]),
		.gpr_12             (RF_next[12]),
		.gpr_13             (RF_next[13]),
		.gpr_14             (RF_next[14]),
		.gpr_15             (RF_next[15]),
		.gpr_16             (RF_next[16]),
		.gpr_17             (RF_next[17]),
		.gpr_18             (RF_next[18]),
		.gpr_19             (RF_next[19]),
		.gpr_20             (RF_next[20]),
		.gpr_21             (RF_next[21]),
		.gpr_22             (RF_next[22]),
		.gpr_23             (RF_next[23]),
		.gpr_24             (RF_next[24]),
		.gpr_25             (RF_next[25]),
		.gpr_26             (RF_next[26]),
		.gpr_27             (RF_next[27]),
		.gpr_28             (RF_next[28]),
		.gpr_29             (RF_next[29]),
		.gpr_30             (RF_next[30]),
		.gpr_31             (RF_next[31])
	);

    DifftestTrapEvent DifftestTrapEvent(
		.clock              (clk),
		.coreid             (0),
		.valid              (0),
		.code               (0),
		.pc                 (0),
		.cycleCnt           (0),
		.instrCnt           (0)
	);

	DifftestCSRState DifftestCSRState(
		.clock              (clk),
		.coreid             (0),
		.priviledgeMode     (prvmode_nxt),
		.mstatus            (csr_regs_next.mstatus),
		.sstatus            (csr_regs_next.mstatus & 64'h800000030001e000),
		.mepc               (csr_regs_next.mepc),
		.sepc               (0),
		.mtval              (csr_regs_next.mtval),
		.stval              (0),
		.mtvec              (csr_regs_next.mtvec),
		.stvec              (0),
		.mcause             (csr_regs_next.mcause),
		.scause             (0),
		.satp               (csr_regs_next.satp),
		.mip                (csr_regs_next.mip),
		.mie                (0),
		.mscratch           (csr_regs_next.mscratch),
		.sscratch           (0),
		.mideleg            (0),
		.medeleg            (0)
	);
`endif
endmodule
`endif