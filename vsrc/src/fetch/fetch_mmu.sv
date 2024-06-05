`ifndef __FETCHMMU_SV
`define __FETCHMMU_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 
module fetch_mmu
    import common::*;(
    input logic clk,
    input logic rst,
    input u64 satp,
    input u2 prvmode,
    input u64 pc,
    output ibus_req_t ireq,
    input ibus_resp_t iresp,
    output logic mmu_ok,
    output u32 mmu_data
);

u64 addr;
u3 stage;
logic tag;

always_ff @( posedge clk ) begin
    if(rst) begin
        stage <= 0; 
        addr <= 0;
    end    
end

always_comb begin
    mmu_ok = iresp.data_ok; 
end

	
endmodule

`endif 