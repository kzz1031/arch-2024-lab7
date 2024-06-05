`ifndef __FETCH_SV
`define __FETCH_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "src/fetch/fetch_mmu.sv"
`else

`endif 
module fetch 
    import common::*;(
    input logic rst,
    input logic clk,
    input  ibus_resp_t iresp,
    input logic stall,
    input logic zero_flag,
    input u64 pc_branch,
    input logic ctrl_branch,
    input u64 reg_offset,
    input u64 reg_execute_pc,
    input logic decode_valid,
    input logic execute_valid,
    input u64 satp,
    input u2 prvmode,
    output ibus_req_t  ireq,
    output u32 reg_fetch_ins,
    output u64 reg_fetch_pc,
    output logic fetch_valid
);

u64 pc;
logic delay,mmu_ok;
u3 jud;
assign ireq.valid = 1;
OPC op;
assign op = OPC'(reg_fetch_ins[6:0]);
u64 addr,mmu_data; 

fetch_mmu fetch_mmu(
    .clk        (clk),
    .rst        (rst),
    .satp       (satp),
    .ireq       (ireq),
    .pc         (pc),
    .iresp      (iresp),
    .mmu_ok     (mmu_ok),
    .mmu_data   (mmu_data),
    .prvmode    (prvmode)
);

always_ff @( posedge clk ) begin
    if(rst) begin
        fetch_valid <= 0;
        pc <= PCINIT;
        delay <= 0;     
    end 
    if(delay & mmu_ok & reg_execute_pc == reg_fetch_pc) begin
        delay <= 0;

        if(op == JAL || op == JALR) pc <= pc_branch;
        else if(zero_flag) pc <= reg_fetch_pc + reg_offset;
    end
    else if( mmu_ok & (!fetch_valid) & (!stall) & (!delay)) begin 
        reg_fetch_ins <= iresp.data;
        reg_fetch_pc  <= pc;    
        pc <= pc + 4;
        fetch_valid <= 1;
    end 
    else if(fetch_valid & (!stall) & (!delay)) fetch_valid <= 0;

    if(ctrl_branch & decode_valid) delay <= 1;
end

endmodule

`endif 