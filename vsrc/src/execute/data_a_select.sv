`ifndef __ASELECT_SV
`define __ASELECT_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 
module data_a_select
    import common::*;(
        input OPC op,
        input u64 pc,
        input u64 reg_read_data_0,
        output u64 data_a
);
always_comb begin
	if(op == LUI || op == CSR) data_a = 0;
	else if(op == AUIPC || op == JAL) data_a = pc;
	else data_a = reg_read_data_0;
end // data_a select
endmodule
`endif 