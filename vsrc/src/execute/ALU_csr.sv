`ifndef __ALUCSR_SV
`define __ALUCSR_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 
module ALU_csr
    import common::*;(
    input u64 t,
    input u64 zimm,
    input u64 rd1,
    input u64 pc,
    input u3 func3,
    output u64 csr_data_out
    );
    u64 temp_out;
    always_comb
    begin
        case (func3)
            3'b000: temp_out = pc; // ecall
            3'b011: temp_out = t & (~rd1);
            3'b111: temp_out = t & (~zimm);
            3'b010: temp_out = t | rd1;
            3'b110: temp_out = t | zimm;
            3'b001: temp_out = rd1;
            3'b101: temp_out = zimm;
             
            default: temp_out = 64'('b0);
        endcase
    end
assign csr_data_out = temp_out;

endmodule
`endif