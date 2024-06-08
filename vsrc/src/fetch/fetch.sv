`ifndef __FETCH_SV
`define __FETCH_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr_pkg.sv"
`include "src/fetch/fetch_mmu.sv"
`else

`endif 
module fetch 
    import common::*;
    import csr_pkg::*;(
    input logic rst,
    input logic clk,
    input dbus_resp_t dresp,
    input logic stall,
    input logic zero_flag,
    input u64 pc_branch,
    input logic ctrl_branch,
    input u64 reg_offset,
    input u64 reg_execute_pc,
    input logic decode_valid,
    input logic execute_valid,
    input logic writeback_valid,

    input logic mmu_fetch_ok,
    input u64 mmu_data,
    input csr_regs_t csr_regs,
    input csr_decode csr_inf,
    output ibus_req_t  ireq,
    output u32 reg_fetch_ins,
    output u64 reg_fetch_pc,
    output logic fetch_valid,
    output u64 pc,
    output u2 prvmode
);

logic delay,mmu_ok;

// assign ireq.valid = 1;
// assign ireq.addr = pc;
OPC op;
assign op = OPC'(reg_fetch_ins[6:0]);
u64 addr; 

always_ff @( posedge clk ) begin
    if(rst) begin
        fetch_valid <= 0;
        pc <= PCINIT;
        delay <= 0;     
    end 
    if(delay & dresp.data_ok & reg_execute_pc == reg_fetch_pc) begin
        if(csr_inf.mret & prvmode == 0) begin 
            pc <= csr_regs.mepc;
            delay <= 0;
        end
        else if(csr_inf.ecall & prvmode == 2'b11) begin
            pc <= csr_regs.mtvec;
            delay <= 0;
        end
        else begin
            delay <= 0;
            if(op == JAL || op == JALR) pc <= pc_branch;
            else if(zero_flag) pc <= reg_fetch_pc + reg_offset;
        end
end   
    else if( mmu_fetch_ok & (!fetch_valid) & (!stall) & (!delay)) begin 
        reg_fetch_ins <= (pc[2:0] == 0) ? mmu_data[31:0] : mmu_data[63:32];
        pc <= pc + 4; 
        reg_fetch_pc  <= pc;    
        fetch_valid <= 1;
    end 
    else if(fetch_valid & (!stall) & (!delay)) fetch_valid <= 0;

    if(ctrl_branch & decode_valid) delay <= 1;
end

endmodule

`endif 