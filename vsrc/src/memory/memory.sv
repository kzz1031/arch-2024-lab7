`ifndef __MEMORY_SV
`define __MEMORY_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "src/memory/load_data.sv"
`include "src/memory/store_data.sv"
`else

`endif 
module memory 
    import common::*;(
        input logic clk,
        input logic rst,
        input logic execute_valid,
        input u64 reg_execute_data_out,
        input u64 reg_execute_csr_data_out,
        output dbus_req_t dreq,
        input dbus_resp_t dresp,
        input logic reg_execute_mem_r,reg_execute_mem_w,
        input u5 reg_execute_rd,
        input msize_t reg_execute_msize,
        input u32 reg_execute_ins,
        input u64 reg_execute_pc,reg_execute_rd2,
        input logic reg_execute_reg_w,reg_execute_sig,
        input u64 satp,
        input u2 prvmode,

        input logic mmu_memory_ok,
        input u64 mmu_data,
        
        output logic reg_memory_reg_w,
        output logic reg_memory_mem_w,
        output logic reg_memory_mem_r,
        output u64 reg_memory_pc,
        output u32 reg_memory_ins,
        output u64 reg_memory_data_out,
        output logic memory_valid,
        output u5 reg_memory_rd,
        output u64 reg_memory_ALU_data_out,
        output u64 reg_memory_csr_data_out,
        output u64 reg_memory_addr,
        output logic memory_stall,

        output strobe_t strobe,
        output u64 mem_store_data
);

u64 mem_load_data;

always_ff @( posedge clk ) begin
    if(rst) memory_stall <= 0;
    if(execute_valid && (reg_execute_mem_r || reg_execute_mem_w)) begin
        memory_stall <= 1;
    end
    if(memory_stall & mmu_memory_ok) begin
        if(reg_execute_mem_r) reg_memory_data_out <= mem_load_data;
        memory_valid <= 1;
        memory_stall <= 0;
        reg_memory_rd  <= reg_execute_rd; 
        reg_memory_ins <= reg_execute_ins; 
        reg_memory_mem_r <= reg_execute_mem_r;
        reg_memory_mem_w <= reg_execute_mem_w;
        reg_memory_ALU_data_out <= reg_execute_data_out;
        reg_memory_reg_w <= reg_execute_reg_w;
        reg_memory_pc <= reg_execute_pc;
        reg_memory_addr <= reg_execute_data_out;
    end
    else if(execute_valid & !(reg_execute_mem_r || reg_execute_mem_w)) begin
        memory_valid <= 1;
        reg_memory_pc <= reg_execute_pc;
        reg_memory_rd  <= reg_execute_rd; 
        reg_memory_ins <= reg_execute_ins; 
        reg_memory_mem_r <= reg_execute_mem_r;
        reg_memory_ALU_data_out <= reg_execute_data_out;
        reg_memory_csr_data_out <= reg_execute_csr_data_out;
        reg_memory_reg_w <= reg_execute_reg_w;
        reg_memory_mem_w <= reg_execute_mem_w;
        end 
    if(memory_valid) memory_valid  <= 0;       
end

// assign dreq.valid = memory_stall;
// assign dreq.addr = (reg_execute_mem_r||reg_execute_mem_w) ? reg_execute_data_out : 0;

// always_comb begin 
// 	if(reg_execute_mem_r) begin
// 		dreq.strobe = 0;
// 		dreq.size = reg_execute_msize;
// 		dreq.data = 0;
// 	end	
// 	else if(reg_execute_mem_w) begin
// 		dreq.strobe = strobe;
// 		dreq.data = mem_store_data;
// 		dreq.size = reg_execute_msize;
// 	end	
// 	else begin
// 		dreq.data = 0;
// 		dreq.strobe = 0;
// 		dreq.size = msize_t'(3'b011);
// 	end
// end

store_data store_data(
	.mem_write_data (mem_store_data),
	.data			(reg_execute_rd2),
	.msize			(reg_execute_msize),
	.strobe			(strobe),
	.addr			(reg_execute_data_out[2:0])
);

load_data load_data(
	.msize			(reg_execute_msize),
	.mem_read_data	(mem_load_data),
	.addr			(reg_execute_data_out[2:0]),
	.sig 			(reg_execute_sig),
	.dr				(mmu_data)
);

endmodule

`endif 