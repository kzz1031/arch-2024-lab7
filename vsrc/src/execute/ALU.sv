`ifndef __ALU_SV
`define __ALU_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 
module ALU
    import common::*;(
    input u64 data_a,
    input u64 data_b,
    input ALU_CTR ALU_ctrl,
    input BRA take_branch,
    input logic is_word,
    output u64 data_out,
    output logic zero
    );
    u64 temp_out;
    u64 temp_sum;
    u64 temp_sub;
    u64 temp_sll;
    u32 temp_srl;
    u32 word;

    assign word = data_a[31:0];
    assign temp_sll = data_a << data_b[4:0];
    assign temp_srl = word >> data_b[4:0];
    assign temp_sum = data_a + data_b;
    assign temp_sub = data_a - data_b;
    //assign zero = (data_out == 0);
    always_comb
    begin
        case (ALU_ctrl)
            AND: temp_out  = data_a & data_b;
            OR:  temp_out  = data_a | data_b;
            ADD: temp_out  = is_word ? {{32{temp_sum[31]}}, temp_sum[31:0]} : temp_sum;
            SUB: temp_out  = is_word ? {{32{temp_sub[31]}}, temp_sub[31:0]} : temp_sub;
            XOR: temp_out  = data_a ^ data_b;
            SLL: temp_out  = is_word ? {{32{temp_sll[31]}}, temp_sll[31:0]} : {data_a << data_b[5:0]};
            SRL: temp_out  = is_word ? {{32{temp_srl[31]}}, temp_srl[31:0]} : {data_a >> data_b[5:0]};
            SRA: temp_out  = is_word ? {{32{data_a[31]}} , $signed(data_a[31 : 0]) >>> data_b[4:0]} : {$signed(data_a) >>> data_b[5:0]};
            SLT: temp_out  = {63'b0 , $signed(data_a) < $signed(data_b)};
            SLTU: temp_out = {63'b0, data_a < data_b}; 
            default: temp_out = 64'('b0);
        endcase
    end
assign data_out = temp_out;

    
    
    always_comb
    begin
        case (take_branch)
            BEQ: zero = (data_out == 0);
            BNE: zero = !(data_out == 0);
            BLT: zero = ( $signed(data_a) < $signed(data_b));
            BGE: zero = ( $signed(data_a) >= $signed(data_b));
            BLTU:zero = (data_a < data_b);
            BGEU:zero = (data_a >= data_b);
            default: zero = 0;
        endcase
     end

     
endmodule
`endif