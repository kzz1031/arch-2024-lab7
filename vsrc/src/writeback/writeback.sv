`ifndef __WRITEBACK_SV
`define __WRITEBACK_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 

module writeback 
    import common::*;(
        input logic clk,
        input logic rst,
        input logic memory_valid,
        input u64 reg_memory_ALU_data_out,
        input u64 reg_memory_data_out,
        input u64 reg_memory_pc,
        input u32 reg_memory_ins,
        input u5 reg_memory_rd,
        input logic reg_memory_reg_w,
        input logic reg_memory_mem_r,
        output logic reg_writeback_reg_w,
        output logic writeback_valid,
        output u64 reg_writeback_data,
        output u5 reg_writeback_rd
);
OPC op;
assign op = OPC'(reg_memory_ins[6:0]);
always_ff @( posedge clk ) begin
    if(memory_valid) begin
        reg_writeback_rd <= reg_memory_rd;
        writeback_valid  <= 1;
        reg_writeback_reg_w <= reg_memory_reg_w;
        if(op == JAL || op == JALR ) reg_writeback_data <= reg_memory_pc+4;
        else if(reg_memory_mem_r) reg_writeback_data <= reg_memory_data_out;
        else reg_writeback_data <= reg_memory_ALU_data_out;
    end
    if(writeback_valid) begin
        writeback_valid <= 0;
        reg_writeback_reg_w <= 0;
    end 
end

endmodule

`endif 