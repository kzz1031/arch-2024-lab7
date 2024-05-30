`ifndef __IMM_SV
`define __IMM_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 
module immediate_generator
    import common::*;(
    input u32 ins,
    output logic signed [63 : 0] offset    
    );
    
    OPC opc;
    logic [31:0] imm;
    
    assign opc = OPC'(ins[6:0]);
    
    always_comb begin
        case (opc)
            JALR, OPI, LOAD , OPIW    : imm = { {21{ins[31]}}, ins[30:25], ins[24:20] };
            STORE        : imm = { {21{ins[31]}}, ins[30:25], ins[11:7] };
            BRANCH       : imm = { {20{ins[31]}}, ins[7], ins[30:25], ins[11:8], 1'b0 };
            AUIPC, LUI   : imm = { {1{ins[31]}}, ins[30:20], ins[19:12], 12'b0 };    
            JAL          : imm = { {12{ins[31]}}, ins[19:12], ins[20], ins[30:25], ins[24:21], 1'b0 };
            CSR          : imm = { {27'b0}, ins[19:15]};
            default : imm = 32'b0;
        endcase
    end
    
    assign offset = 64'(signed'(imm));
endmodule
`endif 