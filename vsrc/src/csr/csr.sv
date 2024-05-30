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
    output csr_regs_t   csr_regs_next,
    output csr_regs_t   csr_regs,
    output u64 csr_rd
    );

    always_ff @( posedge clk ) begin
        if(rst) begin
            csr_regs <= '0;
			csr_regs.mcause[1] <= 1'b1;
			csr_regs.mepc[31] <= 1'b1;
        end
        if(w_en) begin
            case(csr)
                CSR_MIE: csr_regs.mie <= csr_wd;
			    CSR_MIP: csr_regs.mip <= csr_wd;
			    CSR_MTVEC: csr_regs.mtvec <= csr_wd;
			    CSR_MSTATUS: csr_regs.mstatus <= csr_wd;
			    CSR_MSCRATCH: csr_regs.mscratch <= csr_wd;
			    CSR_MEPC: csr_regs.mepc <= csr_wd;
			    CSR_MCAUSE: csr_regs.mcause <= csr_wd;
			    CSR_MCYCLE: csr_regs.mcycle <= csr_wd;
			    CSR_MTVAL: csr_regs.mtval <= csr_wd;
                default: begin
                    
                end 
            endcase
        end
    end
    always_comb begin
        case(csr)
            CSR_MIE: csr_rd = csr_regs.mie;
			CSR_MIP: csr_rd = csr_regs.mip;
			CSR_MTVEC: csr_rd = csr_regs.mtvec;
			CSR_MSTATUS: csr_rd = csr_regs.mstatus;
			CSR_MSCRATCH: csr_rd = csr_regs.mscratch;
			CSR_MEPC: csr_rd = csr_regs.mepc;
			CSR_MCAUSE: csr_rd = csr_regs.mcause;
			CSR_MCYCLE: csr_rd = csr_regs.mcycle;
			CSR_MTVAL: csr_rd = csr_regs.mtval;
			default: begin
				csr_rd = '0;
			end
        endcase 
    end
    always_ff @( posedge clk ) begin
        case(csr)
            CSR_MIE: csr_regs_next.mie = csr_wd;
			CSR_MIP: csr_regs_next.mip = csr_wd;
			CSR_MTVEC: csr_regs_next.mtvec = csr_wd;
			CSR_MSTATUS: csr_regs_next.mstatus = csr_wd;
			CSR_MSCRATCH: csr_regs_next.mscratch = csr_wd;
			CSR_MEPC: csr_regs_next.mepc = csr_wd;
			CSR_MCAUSE: csr_regs_next.mcause = csr_wd;
			CSR_MCYCLE: csr_regs_next.mcycle = csr_wd;
			CSR_MTVAL: csr_regs_next.mtval = csr_wd;
            default: begin
                    
            end 
        endcase
end
endmodule
`endif