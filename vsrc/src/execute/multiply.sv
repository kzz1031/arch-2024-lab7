`ifndef __MULTIPLY_SV
`define __MULTIPLY_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 
module multiply
    import common::*;(
    input logic clk,
    input logic rst,
    input ALU_CTR ALU_ctrl,
    input u64 data_a,
    input u64 data_b,
    input logic is_word,
    input logic decode_valid,
    input logic stall,
    output u64 multiply_data_out,
    output logic mul_end,
    output logic mul_valid
);

always_ff @( posedge clk or posedge rst) 
begin
    if(rst) mul_valid <= 0;
    else if(ALU_ctrl == MUL & decode_valid) mul_valid <= 1;
    else if(mul_end == 1) mul_valid <= 0;
end
u64 op1,op2,op1_abs,op2_abs;
logic op1_sig,op2_sig,word;
u64 multiplier;
assign op1_sig = op1[63];
assign op2_sig = op2[63];
assign op1_abs = data_a[63] ? (~data_a+1) : data_a;
assign op2_abs = data_b[63] ? (~data_b+1) : data_b;

assign mul_end = mul_valid & (multiplier == 0);

always_ff @( posedge clk ) 
begin
    if(rst) begin
        op1 <= 0;
        op2 <= 0;
    end
    else if(ALU_ctrl == MUL & decode_valid) begin
        op1 <= data_a;
        op2 <= data_b;
    end
    else if(mul_end) begin
        op1 <= 0;
        op2 <= 0;
    end
    
end

always_ff @( posedge clk ) begin
    if(rst) word <= 0;
    else if(ALU_ctrl == MUL & decode_valid) begin
        if(is_word) word <= 1;
        else        word <= 0;
    end
         
end

u128 multiplicand, product;
always_ff @( posedge clk ) begin
    if(mul_valid) multiplicand <= {multiplicand[126:0],1'b0};
    else if(ALU_ctrl == MUL & decode_valid) multiplicand <= {64'b0,op1_abs};
end
always_ff @( posedge clk ) begin
    if(mul_valid) multiplier <= {1'b0,multiplier[63:1]};
    else if(ALU_ctrl == MUL & decode_valid) multiplier <= op2_abs;
end

u128 product_partial, product_temp;
assign product_partial = multiplier[0] ? multiplicand : 128'b0;

always_ff @( posedge clk ) begin
    if(mul_valid) product_temp <= product_temp + product_partial;   
    else if(ALU_ctrl == MUL & decode_valid) product_temp <= 0;
end

assign product = op1_sig ^ op2_sig ? (~product_temp + 1) : product_temp;

always_comb begin 
    if(word) multiply_data_out = {{32{product[31]}},product[31:0]};
    else     multiply_data_out = product[63:0];
    
end
endmodule
`endif