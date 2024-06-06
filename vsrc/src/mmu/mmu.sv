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
u64 init_addr,data,nxt_addr;
logic src,tag;
strobe_t strobe_m;
u2 stage;


always_ff @( posedge clk ) begin
    if(dresp.data_ok & mmu_on) begin
        if(stage == 0) begin
            stage <= 2'b01;
            nxt_addr <= {8'b0, dresp.data[53:10], 12'b0} + {52'b0, init_addr[29:21], 3'b0};
        end
        else if(stage == 2'b01) begin
            stage <= 2'b10;
            nxt_addr <= {8'b0, dresp.data[53:10], 12'b0} + {52'b0, init_addr[20:12], 3'b0};
        end
        else if(stage == 2'b10) begin
            stage <= 2'b11;
            nxt_addr <= {8'b0, dresp.data[53:10], init_addr[11:0]};
        end
        else if(stage == 2'b11) begin
            stage <= 2'b0;
        end
    end   
end

always_ff @( posedge clk ) begin
    if(rst) begin
        src <= 0;
        stage <= 0;
    end
    if(dresp.data_ok & !tag) begin
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
    else if(dresp.data_ok & mmu_on) begin
        if(src == 1 && stage == 2'b11) begin
            src <= 0;
            strobe_m <= 0;
        end
        else if(memory_busy && src == 0) begin
            src <= 1;
            stage <= 0;
            strobe_m <= (memory_w && stage == 2'b11) ? strobe : 0;
            data <= mem_store_data;
        end
    end
end
logic mmu_on;
always_ff @( posedge clk ) begin
    if(rst)  mmu_on <= 0;
    if(!(prvmode == 2'b11 || satp[63:60] != 4'b1000)) begin
        if(dresp.data_ok) mmu_on <= 1;
    end
    else if(dresp.data_ok) mmu_on <= 0;
    
end
always_comb begin
    if(prvmode == 2'b11 || satp[63:60] != 4'b1000) begin
        init_addr = 0;
         // mmu translate needed?
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
        // mmu
        mmu_data = dresp.data;
        init_addr = src ? memory_addr : pc;

            dreq.strobe = strobe_m;
            dreq.data = mem_store_data;
            dreq.size = 3'b011;
        if(!mmu_on) begin
            dreq.addr = src ? memory_addr : pc;
            mmu_data = 0;
            mmu_fetch_ok = 0;
            mmu_memory_ok = 0;
        end
        else begin
            if(stage == 0) dreq.addr = {8'b0 , satp[43:0], 12'b0} + {52'b0, init_addr[38:30], 3'b0}; 
            else begin
                dreq.addr = nxt_addr;
            end
            if(memory_busy) begin
                mmu_memory_ok = src & dresp.data_ok & (stage == 2'b11);
                mmu_data = dresp.data;
                mmu_fetch_ok = 0;
            end
            else begin
                mmu_fetch_ok = dresp.data_ok & (stage == 2'b11);
                mmu_data = dresp.data;
                mmu_memory_ok = 0;
            end
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