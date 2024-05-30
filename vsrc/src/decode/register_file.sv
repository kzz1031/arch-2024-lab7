`ifndef __RF_SV
`define __RF_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 
module register_file
    import common::*;(
    input logic clk,
    input logic rst,
    input u5 rs1,rs2,reg_w,
    input u64 w_data,
    input logic w_en,

    output u64 read_data_rs1,read_data_rs2,
    output logic [63 : 0] RF [32]
    );

    assign read_data_rs1 = RF[rs1];
    assign read_data_rs2 = RF[rs2];

    always_ff @(posedge clk or posedge rst) begin
        if(rst) begin
            integer i;
            for(i = 0; i< REG_NUM ; i++) RF[i] <= 64'('b0);
        end else begin
            if(w_en && reg_w != 0) begin
                RF[reg_w] <= w_data;
            end
        end
    end
endmodule
`endif