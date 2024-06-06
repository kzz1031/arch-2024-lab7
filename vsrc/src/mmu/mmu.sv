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
    input logic memory_busy,//memory occupy
    input logic memory_w,
    input logic memory_r,
    input msize_t msize,
    input strobe_t  strobe,
    input u64 mem_store_data,
    input u64 memory_addr,
    output dbus_req_t dreq,
    input dbus_resp_t dresp,
    output logic mmu_fetch_ok,
    output logic mmu_memory_ok,
    output u64 mmu_data
);

always_comb begin
    dreq.valid = 1;
end
u64 addr,data;
logic src;
strobe_t strobe_m;
always_ff @( posedge clk ) begin
    if(rst) begin
        src <= 0;
        addr <= PCINIT;
    end
    if(dresp.data_ok) begin
        if(src == 1) begin
            src <= 0;
            strobe_m <= 0;
        end
        else if(memory_busy) begin
            src <= 1;
            strobe_m <= memory_w ? strobe : 0;
            data <= mem_store_data;
        end
        
    end
end

always_comb begin
    if(prvmode == 2'b11 || satp[63:60] != 4'b1000) begin
        mmu_data = dresp.data;
        dreq.strobe = strobe_m;
        dreq.data = mem_store_data;
        dreq.size = 3'b011;
        dreq.addr = src ? memory_addr : pc; 
        if(memory_busy) begin
            mmu_memory_ok = src & dresp.data_ok;
            mmu_data = dresp.data;
            mmu_fetch_ok = 0;
        end
        else begin
            mmu_fetch_ok = dresp.data_ok;
            mmu_data = dresp.data;
            mmu_memory_ok = 0;
        end
    end
    else begin
        dreq.addr = addr;
        mmu_data = dresp.data;
        dreq.strobe = strobe_m;
        dreq.data = mem_store_data;
        dreq.size = 3'b011;
        if(memory_busy) begin
            mmu_memory_ok = dresp.data_ok;
            mmu_data = dresp.data;
            mmu_fetch_ok = 0;
        end
        else begin
            mmu_fetch_ok = dresp.data_ok;
            mmu_data = dresp.data;
            mmu_memory_ok = 0;
        end
    end
end 
// always_comb begin
//     if(prvmode == 2'b11 || satp[63:60] != 4'b1000) begin
//         dreq.addr = addr;
//         mmu_data = dresp.data;
//         if(memory_busy) begin
//             if(memory_r) begin
//             dreq.strobe = strobe_m;
//             dreq.size = msize;
//             dreq.data = 0;
//             mmu_fetch_ok = 0; 
//             mmu_memory_ok = dresp.data_ok;
//             end
//         else begin
//             dreq.strobe = strobe_m;
//             dreq.data = mem_store_data;
//             dreq.size = msize;
//             mmu_fetch_ok = 0;
//             mmu_memory_ok = dresp.data_ok;
//             end  
//         end
//         else begin
//             dreq.strobe = 0;

//         end
//     end
// end 
	
endmodule

`endif 