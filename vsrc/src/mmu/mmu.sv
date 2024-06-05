`ifndef __MMU_SV
`define __MMU_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 
module mmu
    import common::*;(
    input logic clk,
    input logic rst,
    input u64 satp,
    input u2 prvmode,
    input u64 pc,
    output dbus_req_t dreq,
    input dbus_resp_t dresp,
    output logic mmu_fetch_ok,
    output logic mmu_memory_ok
);


	
endmodule

`endif 