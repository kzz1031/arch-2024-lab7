`ifndef __CSR_SV
`define __CSR_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/csr_pkg.sv"
`else

`endif 
module csr
    import common::*;
    import csr_pkg::*;(
    input logic clk,
    input logic rst,
    input u12 csr,
    input u64 csr_wd,
    input logic w_en,
    input logic mret,
    input csr_decode csr_inf,
    output u64 pc,
    output csr_regs_t   csr_regs_next,
    output csr_regs_t   csr_regs,
    output u2  prvmode,
    output u64 csr_rd
    );

    always_ff @( posedge clk ) begin
        if(rst) begin
            csr_regs <= '0;
			csr_regs.mcause[1] <= 1'b1;
			csr_regs.mepc[31] <= 1'b1;
        end
        if(w_en) begin
             if(csr_inf.mret) begin 
                pc <= csr_regs.mepc;
            end
            else if(csr_inf.ecall) begin
                pc <= csr_regs.mtvec;
            end
            if(csr_inf.mret) begin
                csr_regs.mstatus.mie <= csr_regs.mstatus.mpie;
                csr_regs.mstatus.mpie <= 1;
                csr_regs.mstatus.mpp <= 0;
                prvmode <= csr_regs.mstatus.mpp;
            end
            if(csr_inf.ecall) begin 
                csr_regs.mepc <= csr_wd;
                prvmode <= 2'b11;
                csr_regs.mcause <= {60'b0, 4'b1000};
                csr_regs.mstatus.mie <= 0;
                csr_regs.mstatus.mpie <= csr_regs.mstatus.mie;
                csr_regs.mstatus.mpp <= prvmode;
            end
            unique case(csr)
                CSR_MIE: csr_regs.mie <= csr_wd;
			    CSR_MIP: csr_regs.mip <= csr_wd;
			    CSR_MTVEC: csr_regs.mtvec <= csr_wd;
			    CSR_MSTATUS: csr_regs.mstatus <= csr_wd;
			    CSR_MSCRATCH: csr_regs.mscratch <= csr_wd;
			    CSR_MEPC: csr_regs.mepc <= csr_wd;
			    CSR_MCAUSE: csr_regs.mcause <= csr_wd;
			    CSR_MCYCLE: csr_regs.mcycle <= csr_wd;
			    CSR_MTVAL: csr_regs.mtval <= csr_wd;
                CSR_SATP: csr_regs.satp <= csr_wd;
                CSR_MHARTID: csr_regs.mhartid <= csr_wd;
                default: begin
                    
                end 
            endcase
        end
    end
    always_comb begin
        if(mret) begin
            csr_regs_next.mstatus.mie = csr_regs.mstatus.mpie;
            csr_regs_next.mstatus.mpie = 1;
            csr_regs_next.mstatus.mpp = 0;
        end
        if(csr_inf.ecall) begin
            csr_regs_next.mstatus.mpp = prvmode;
            csr_regs_next.mstatus.mie = 0;
            csr_regs_next.mepc = csr_wd;
            csr_regs_next.mcause = {60'b0, 4'b1000};
            csr_regs_next.mstatus.mpie = csr_regs.mstatus.mie;
        end
        unique case(csr)
            CSR_MIE: csr_rd = csr_regs.mie;
			CSR_MIP: csr_rd = csr_regs.mip;
			CSR_MTVEC: csr_rd = csr_regs.mtvec;
			CSR_MSTATUS: csr_rd = csr_regs.mstatus;
			CSR_MSCRATCH: csr_rd = csr_regs.mscratch;
			CSR_MEPC: csr_rd = csr_regs.mepc;
			CSR_MCAUSE: csr_rd = csr_regs.mcause;
			CSR_MCYCLE: csr_rd = csr_regs.mcycle;
			CSR_MTVAL: csr_rd = csr_regs.mtval;
            CSR_SATP: csr_rd = csr_regs.satp;
            CSR_MHARTID: csr_rd = csr_regs.mhartid;
			default: begin
				csr_rd = '0;
			end
        endcase 
    end
    u12 tag; //
    always_comb begin 
        tag = CSR_MCAUSE;
    end
    
    always_comb begin
        if(w_en) begin
            tag = {11'b0,1'b1};
            unique case(csr)
                CSR_MIE: csr_regs_next.mie = csr_wd;
                CSR_MIP: csr_regs_next.mip = csr_wd;
                CSR_MTVEC: csr_regs_next.mtvec = csr_wd;
                CSR_MSTATUS: csr_regs_next.mstatus = csr_wd;
                CSR_MSCRATCH: csr_regs_next.mscratch = csr_wd;
                CSR_MEPC: csr_regs_next.mepc = csr_wd;
                CSR_MCAUSE: csr_regs_next.mcause = csr_wd;
                CSR_MCYCLE: csr_regs_next.mcycle = csr_wd;
                CSR_MTVAL: csr_regs_next.mtval = csr_wd;
                CSR_SATP: csr_regs_next.satp = csr_wd;
                CSR_MHARTID: csr_regs_next.mhartid = csr_wd;
                default: begin
                end 
            endcase
        end
        else tag = {10'b0,2'b11};
        
end
endmodule
`endif