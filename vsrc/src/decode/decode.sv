`ifndef __DECODE_SV
`define __DECODE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr_pkg.sv"
`include "src/decode/immediate_generator.sv"
`include "src/decode/register_file.sv"
`include "src/csr/csr.sv"
`else

`endif 
module decode
    import common::*;
    import csr_pkg::*;(
    input u32   reg_fetch_ins,
    input logic fetch_valid,
    input logic clk,
    input logic rst,
    input u64 reg_fetch_pc,
    input u5 reg_writeback_rd,
    input logic w_en,
    input u64   reg_writeback_data,
    input u64   reg_writeback_csr_data_out,
    input logic stall,

    output u32 reg_busy,
    output logic decode_valid,
    output u32   reg_decode_ins,

    output logic [1 : 0] ctrl_ALU_op,
    output logic ctrl_ALU_src,
    output logic ctrl_reg_w,ctrl_mem_r,ctrl_mem_w,ctrl_mem_to_reg,

    output csr_decode csr_inf,
    output csr_regs_t    csr_regs,
    output csr_regs_t    csr_regs_next,

    output logic is_word,
    output logic is_csr,

    output logic ctrl_branch,
    output msize_t reg_decode_msize,
    output logic reg_decode_sig,
    output logic [63 : 0] RF [32],
    output u64 reg_decode_rd1,reg_decode_rd2,
    output u5 reg_decode_rd,
    output u64 reg_offset,reg_decode_pc,
    output logic decode_stall
);

u64 read_data_rs1,read_data_rs2,offset,csr_rd;

register_file register_file(
    .clk			    (clk),
    .rst			    (rst),
    .rs1			    (reg_fetch_ins[19:15]),
    .rs2		        (reg_fetch_ins[24:20]),
    .reg_w			    (reg_writeback_rd),
	.read_data_rs1      (read_data_rs1),
    .read_data_rs2      (read_data_rs2),
    .w_data			    (csr_inf.csr_w ? csr_inf.t : reg_writeback_data),
    .w_en               (w_en),
	.RF				    (RF));

csr csr(
    .clk            (clk),
    .rst            (rst),
    .csr            (reg_fetch_ins[31:20]),
    .csr_regs       (csr_regs),
    .csr_regs_next  (csr_regs_next),
    .csr_rd         (csr_rd),
    .csr_wd         (reg_writeback_csr_data_out),
    .w_en           (w_en & csr_inf.csr_w)
);
immediate_generator immediate_generator(
    .ins			(reg_fetch_ins),
    .offset			(offset));

u5 rs1,rs2;
assign rs1 = reg_fetch_ins[19:15];
assign rs2 = reg_fetch_ins[24:20];

always_ff @( posedge clk ) begin
    if(rst) begin
        decode_stall <= 0;
        decode_valid <= 0; 
        is_csr       <= 0;   
    end 
    if( decode_stall & (!stall) & !(reg_busy[rs1] || reg_busy[rs2] )) begin
        decode_stall <= 0;
        decode_valid <= (reg_fetch_pc != reg_decode_pc);
    end  
    else if( fetch_valid & (!decode_stall) &(!stall) & (!(reg_busy[rs1] || reg_busy[rs2])) ) decode_valid <= 1;
    else if( fetch_valid  & (reg_busy[rs1] || reg_busy[rs2]))    decode_stall <= 1;
    
    if( decode_valid )      decode_valid <= 0;
end

OPC op;
assign op = OPC'(reg_fetch_ins[6:0]);

always_ff @(posedge clk) begin
    if((fetch_valid & (!stall) & (!(reg_busy[rs1] || reg_busy[rs2]))) || (decode_stall & (!stall) & !(reg_busy[rs1] || reg_busy[rs2]))) begin
        reg_offset     <= offset;
        reg_decode_rd1 <= read_data_rs1;
        reg_decode_rd2 <= read_data_rs2;
        reg_decode_pc     <= reg_fetch_pc;
        reg_decode_ins    <= reg_fetch_ins;
        reg_decode_rd     <= reg_fetch_ins[11:7];
        reg_decode_sig    <= reg_fetch_ins[14];
        reg_decode_msize  <= msize_t'({1'b0,reg_fetch_ins[13:12]});
    end
    
end

always_ff@( posedge clk ) begin
    if(rst)  begin
        reg_busy <= 0;
    end
    if(w_en) reg_busy[reg_writeback_rd] <= 0;
    if( (fetch_valid & (!stall) & (!(reg_busy[rs1] || reg_busy[rs2]))) || (decode_stall & (!stall) & !(reg_busy[rs1] || reg_busy[rs2]))) 
        case (op)
        LOAD:
        begin
            ctrl_ALU_op     <= 2'b00;
            ctrl_ALU_src    <= 1'b1;
            ctrl_reg_w      <= 1'b1;
            reg_busy[reg_fetch_ins[11:7]] <= reg_fetch_pc != reg_decode_pc;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b1;
            ctrl_mem_to_reg <= 1'b1;
            ctrl_branch     <= 1'b0;
            is_word         <= 1'b0;
            is_csr          <= 1'b0;
        end
        
        STORE:
        begin
            ctrl_ALU_op     <= 2'b00;
            ctrl_ALU_src    <= 1'b1;
            ctrl_reg_w      <= 1'b0;
            ctrl_mem_w      <= 1'b1;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b0;
            is_word         <= 1'b0;
            is_csr          <= 1'b0;
        end

        ARITH:
        begin            
            ctrl_ALU_op     <= 2'b10;
            ctrl_ALU_src    <= 1'b0;
            ctrl_reg_w      <= 1'b1;
            reg_busy[reg_fetch_ins[11:7]] <= reg_fetch_pc != reg_decode_pc;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b0;
            is_word         <= 1'b0;
            is_csr          <= 1'b0;
        end

        BRANCH:
        begin 
            ctrl_ALU_op     <= 2'b01;
            ctrl_ALU_src    <= 1'b0;
            ctrl_reg_w      <= 1'b0;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b1;
            is_word         <= 1'b0;
            is_csr          <= 1'b0;
        end
        LUI :
        begin
            ctrl_ALU_op     <= 2'b00;
            ctrl_ALU_src    <= 1'b1;
            ctrl_reg_w      <= 1'b1;
            reg_busy[reg_fetch_ins[11:7]] <= reg_fetch_pc != reg_decode_pc;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b0;
            is_word         <= 1'b0;
            is_csr          <= 1'b0;
        end
        OPI :
        begin
            ctrl_ALU_op     <= 2'b11;
            ctrl_ALU_src    <= 1'b1;
            ctrl_reg_w      <= 1'b1;
            reg_busy[reg_fetch_ins[11:7]] <= reg_fetch_pc != reg_decode_pc;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b0;
            is_word         <= 1'b0;
            is_csr          <= 1'b0;
        end
        AUIPC :
        begin 
            ctrl_ALU_op     <= 2'b00;
            ctrl_ALU_src    <= 1'b1;
            ctrl_reg_w      <= 1'b1;
            reg_busy[reg_fetch_ins[11:7]] <= reg_fetch_pc != reg_decode_pc;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b0;
            is_word         <= 1'b0;
            is_csr          <= 1'b0;
        end
        JAL :
        begin 
            ctrl_ALU_op     <= 2'b00;
            ctrl_ALU_src    <= 1'b1;
            ctrl_reg_w      <= 1'b1;
            reg_busy[reg_fetch_ins[11:7]] <= reg_fetch_pc != reg_decode_pc;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b1;
            is_word         <= 1'b0;
            is_csr          <= 1'b0;
        end
        JALR :
        begin 
            ctrl_ALU_op     <= 2'b00;
            ctrl_ALU_src    <= 1'b1;
            ctrl_reg_w      <= 1'b1;
            reg_busy[reg_fetch_ins[11:7]] <= reg_fetch_pc != reg_decode_pc;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b1;
            is_word         <= 1'b0;
            is_csr          <= 1'b0;
        end
        ARITHW:
        begin            
            ctrl_ALU_op     <= 2'b10;
            ctrl_ALU_src    <= 1'b0;
            ctrl_reg_w      <= 1'b1;
            reg_busy[reg_fetch_ins[11:7]] <= reg_fetch_pc != reg_decode_pc;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b0;
            is_word         <= 1'b1;
            is_csr          <= 1'b0;
        end
        OPIW :
        begin
            ctrl_ALU_op     <= 2'b11;
            ctrl_ALU_src    <= 1'b1;
            ctrl_reg_w      <= 1'b1;
            reg_busy[reg_fetch_ins[11:7]] <= reg_fetch_pc != reg_decode_pc;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b0;
            is_word         <= 1'b1;
            is_csr          <= 1'b0;
        end
        CSR :
        begin
            ctrl_reg_w      <= 1'b1;
            csr_inf.csr_w   <= 1'b1;
            csr_inf.csr     <= reg_fetch_ins[31:20];
            csr_inf.func3   <= reg_fetch_ins[14:12];
            csr_inf.rs1     <= reg_fetch_ins[19:15];
            csr_inf.t       <= csr_rd;
            is_csr          <= 1'b1;
        end
        default:
        begin
            ctrl_ALU_op     <= 2'b00;
            ctrl_ALU_src    <= 1'b0;
            ctrl_reg_w      <= 1'b0;
            ctrl_mem_w      <= 1'b0;
            ctrl_mem_r      <= 1'b0;
            ctrl_mem_to_reg <= 1'b0;
            ctrl_branch     <= 1'b0;
            is_word         <= 1'b0;
            is_csr          <= 1'b0;
        end
    endcase   
    end
endmodule

`endif 