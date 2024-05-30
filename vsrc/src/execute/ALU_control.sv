`ifndef __ALU_CONTROL_SV
`define __ALU_CONTROL_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 

module ALU_control
    import common::*;(
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic [1:0] ctrl_ALU_op,
    output BRA take_branch,
    output ALU_CTR ALU_ctrl
    );
    always_comb
    begin
        case (ctrl_ALU_op)
            2'b00   : ALU_ctrl = ADD; // ld/sd
            2'b01   : begin
                ALU_ctrl = SUB;
            end
            2'b10   : begin
                if (funct7 == 7'b0000000 && funct3 == 3'b000)       ALU_ctrl = ADD; // add
                else if (funct7 == 7'b0100000 && funct3 == 3'b000)  ALU_ctrl = SUB; // sub
                else if (funct7 == 7'b0000000 && funct3 == 3'b111)  ALU_ctrl = AND; // and
                else if (funct7 == 7'b0000000 && funct3 == 3'b110)  ALU_ctrl = OR;  // or
                else if (funct7 == 7'b0000000 && funct3 == 3'b100)  ALU_ctrl = XOR;  // xor
                else if (funct7 == 7'b0000000 && funct3 == 3'b001)  ALU_ctrl = SLL;
                else if (funct7 == 7'b0000000 && funct3 == 3'b010)  ALU_ctrl = SLT;
                else if (funct7 == 7'b0000000 && funct3 == 3'b011)  ALU_ctrl = SLTU;
                else if (funct7 == 7'b0000000 && funct3 == 3'b101)  ALU_ctrl = SRL;
                else if (funct7 == 7'b0100000 && funct3 == 3'b101)  ALU_ctrl = SRA;
                else if (funct7 == 7'b0000001 && funct3 == 3'b100)  ALU_ctrl = DIV;
                else if (funct7 == 7'b0000001 && funct3 == 3'b101)  ALU_ctrl = DIVU;
                else if (funct7 == 7'b0000001 && funct3 == 3'b110)  ALU_ctrl = REM;
                else if (funct7 == 7'b0000001 && funct3 == 3'b111)  ALU_ctrl = REMU;
                else if (funct7 == 7'b0000001 && funct3 == 3'b000)  ALU_ctrl = MUL;
                else                                                ALU_ctrl = AND; // default
            end
            2'b11    : begin //OPI
                if (funct3 == 3'b000)       ALU_ctrl = ADD; // add
                else if (funct3 == 3'b111)  ALU_ctrl = AND; // and
                else if (funct3 == 3'b110)  ALU_ctrl = OR;  // or
                else if (funct3 == 3'b100)  ALU_ctrl = XOR;  // xor
                else if (funct3 == 3'b010)  ALU_ctrl = SLT;
                else if (funct3 == 3'b011)  ALU_ctrl = SLTU;
                else if (funct3 == 3'b101 && (funct7 == 7'b0100000 || funct7 == 7'b0100001))  ALU_ctrl = SRA;
                else if (funct3 == 3'b101 && (funct7 == 7'b0000000 || funct7 == 7'b0000001))  ALU_ctrl = SRL;
                else if (funct3 == 3'b001)  ALU_ctrl = SLL;
                else                        ALU_ctrl = AND; // default
            end
            default : ALU_ctrl = AND; 
        endcase
    end
    always_comb begin 
        if (funct3 == 3'b000)      take_branch = BEQ;
        else if (funct3 == 3'b001) take_branch = BNE;
        else if (funct3 == 3'b100) take_branch = BLT;
        else if (funct3 == 3'b101) take_branch = BGE;
        else if (funct3 == 3'b110) take_branch = BLTU;
        else if (funct3 == 3'b111) take_branch = BGEU;
        else take_branch = NONE;
    end   
endmodule
`endif