`ifndef __DIVIDE_SV
`define __DIVIDE_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 

module divide
    import common::*;(
    input logic clk,
    input logic rst,
    input ALU_CTR ALU_ctrl,
    input u64 data_a,
    input u64 data_b,
    input logic is_word,
    input logic decode_valid,
    input logic stall,
    output u64 divide_data_out,
    output u64 remain,
    output logic divide_end,
    output logic divide_valid
);

u64 op1_abs,op2_abs;
logic op1_sig,op2_sig,is_sig;
logic out_select;
logic word;
u64 temp_out;

always_ff @( posedge clk or posedge rst ) 
begin
    if(rst) begin
        divide_valid <= 0;
        out_select <= 0;
    end 
    else if((ALU_ctrl == DIV || ALU_ctrl == DIVU) & decode_valid) begin
        divide_valid <= 1;
        out_select <= 0;
    end 
    else if((ALU_ctrl == REM || ALU_ctrl == REMU) & decode_valid) begin
        divide_valid <= 1;
        out_select <= 1;
    end
    else if(divide_end == 1) begin
        divide_valid <= 0;
        out_select <= 0;
    end 
end

always_comb begin
    if(ALU_ctrl == DIV || ALU_ctrl == REM)
    begin
        if(is_word) begin
            op1_abs = data_a[31] ? { 32'b0,{~data_a[31:0] + 1}} : { 32'b0,data_a[31:0] };
            op2_abs = data_b[31] ? { 32'b0,{~data_b[31:0] + 1}} : { 32'b0,data_b[31:0] };
        end
        else begin
            op1_abs = data_a[63] ? (~data_a + 1) : data_a;
            op2_abs = data_b[63] ? (~data_b + 1) : data_b;
        end
    end
    else
    begin
        if(is_word) begin
            op1_abs = { 32'b0, data_a[31:0] };
            op2_abs = { 32'b0, data_b[31:0] };
        end
        else begin
            op1_abs = data_a;
            op2_abs = data_b;
        end
    end
end

always_ff @( posedge clk ) 
begin
    if(rst) begin
        op1_sig <= 0;
        op2_sig <= 0;
        is_sig <= 0;
    end
    else if((ALU_ctrl == DIV || ALU_ctrl == REM) & decode_valid) begin
        if(is_word) begin
            op1_sig <= data_a[31];
            op2_sig <= data_b[31];
            word <= 1;
        end
        else begin
            op1_sig <= data_a[63];
            op2_sig <= data_b[63];
            word <= 0;
        end
        is_sig <= 1;
    end
    else if((ALU_ctrl == DIVU || ALU_ctrl == REMU) & decode_valid) begin
        op1_sig <= 0;
        op2_sig <= 0;
        is_sig <= 0;
        if(is_word) word <= 1;
        else        word <= 0;
    end
    else if(divide_end) begin
        op1_sig <= 0;
        op2_sig <= 0;
        is_sig <= 0;
    end
end

u128 temp_a,temp_b;
logic [6:0] shift_count;

always_ff @( posedge clk ) begin
    if (rst) begin
        temp_a <= 0;
        temp_b <= 0;
        shift_count <= 0;
        divide_end <= 0;
    end
    else if ((ALU_ctrl == DIV || ALU_ctrl == REM) & decode_valid) begin
        temp_a <= { 64'b0 , op1_abs };
        temp_b <= { op2_abs, 64'b0 };
        shift_count <= 0;
    end
    else if ((ALU_ctrl == DIVU || ALU_ctrl == REMU) & decode_valid) begin
        temp_a <= { 64'b0 , op1_abs };
        temp_b <= { op2_abs, 64'b0 };
        shift_count <= 0;
    end
    else if (divide_end & (!stall)) begin
        shift_count <= 0;
        divide_end <= 0;
    end
    else if(temp_b == 0 && divide_valid) begin
        shift_count <= 0;
        divide_end <= 1;
    end
    else if (divide_valid) begin
        if (shift_count <= 64) begin
            if (temp_a[127:64] >= temp_b[127:64]) temp_a <= temp_a - temp_b + 1'b1;
            else if (shift_count == 64) begin
                divide_end <= 1;    
            end
            else begin
                temp_a <= temp_a << 1;
                shift_count <= shift_count + 1;
            end 
        end
    end
end
always_comb begin
    if(temp_b == 0) begin
        temp_out = -1;
        if(is_sig) remain = op1_sig ? -temp_a[63:0] : temp_a[63:0];
        else       remain = temp_a[63:0];
    end
    else if(is_sig) begin
        temp_out = (op1_sig ^ op2_sig) ? (~temp_a[63:0] + 1):temp_a[63:0];
        remain   = op1_sig ? -temp_a[127:64] : temp_a[127:64];
    end
    else begin
        temp_out = temp_a[63:0];
        remain   = temp_a[127:64];
    end
end

always_comb begin
    if(word) divide_data_out = out_select ? {{32{remain[31]}},remain[31:0]} : {{32{temp_out[31]}},temp_out[31:0]};
    else     divide_data_out = out_select ? remain : temp_out; 
    
end

endmodule
`endif
