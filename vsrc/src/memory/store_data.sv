`ifndef __WRITEDATA_SV
`define __WRITEDATA_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 
module store_data
    import common::*;(
    input msize_t msize,
    input u64 data,
    input u3 addr,
    output strobe_t strobe,
    output u64 mem_write_data
);
always_comb begin
    mem_write_data = 64'b0;
	case (msize)
        MSIZE1: begin
            case (addr)
                3'b000: begin
                    strobe = 8'b00000001;
                    mem_write_data[7:0] = data[7:0];
                end
                3'b001: begin
                    strobe = 8'b00000010;
                    mem_write_data[15:8] = data[7:0];
                end
                3'b010: begin
                    strobe = 8'b00000100;
                    mem_write_data[23:16] = data[7:0];
                end
                3'b011: begin
                    strobe = 8'b00001000;
                    mem_write_data[31:24] = data[7:0];
                end
                3'b100: begin
                    strobe = 8'b00010000;
                    mem_write_data[39:32] = data[7:0];
                end
                3'b101: begin
                    strobe = 8'b00100000;
                    mem_write_data[47:40] = data[7:0];
                end
                3'b110: begin
                    strobe = 8'b01000000;
                    mem_write_data[55:48] = data[7:0];
                end
                3'b111: begin
                    strobe = 8'b10000000;
                    mem_write_data[63:56] = data[7:0];
                end
                default: begin
                    mem_write_data = 0; 
                    strobe = 0;
                end 
            endcase
        end
        MSIZE2: begin
            case (addr[2:1])
                2'b00: begin
                    strobe = 8'b00000011;
                    mem_write_data[15:0] = data[15:0]; 
                end
                2'b01: begin
                    strobe = 8'b00001100;
                    mem_write_data[31:16] = data[15:0]; 
                end
                2'b10: begin
                    strobe = 8'b00110000;
                    mem_write_data[47:32] = data[15:0]; 
                end
                2'b11: begin
                    strobe = 8'b11000000;
                    mem_write_data[63:48] = data[15:0]; 
                end
                default: begin
                    strobe = 0;
                    mem_write_data = 0;
                end
            endcase
        end
        MSIZE4: begin
            case (addr[2])
                1'b0: begin
                    strobe = 8'b00001111;
                    mem_write_data[31:0] = data[31:0];
                end
                1'b1: begin
                    strobe = 8'b11110000;
                    mem_write_data[63:32] = data[31:0];
                end
                default: begin
                    strobe = 0;
                    mem_write_data = 0;
                end
            endcase
        end
        MSIZE8: begin
            strobe = 8'b11111111;
            mem_write_data = data;
        end
        default: begin
            strobe = 0;
            mem_write_data = 0;
        end
    endcase
end
endmodule

`endif 