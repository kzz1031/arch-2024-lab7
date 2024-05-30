`ifndef __READDATA_SV
`define __READDATA_SV

`ifdef VERILATOR
`include "include/common.sv"
`else

`endif 
module load_data
    import common::*;(
    input msize_t msize,
    input u64 dr,
    input logic sig,
    input u3 addr,
    output u64 mem_read_data
);
always_comb begin
	case (msize)
        MSIZE1: begin
            case (addr)
                3'b000: mem_read_data = sig ?  {{56'b0},dr[7:0]} : {{56{dr[7]}},dr[7:0]};
                3'b001: mem_read_data = sig ?  {{56'b0},dr[15:8]} : {{56{dr[15]}},dr[15:8]};
                3'b010: mem_read_data = sig ?  {{56'b0},dr[23:16]} : {{56{dr[23]}},dr[23:16]};
                3'b011: mem_read_data = sig ?  {{56'b0},dr[31:24]} : {{56{dr[31]}},dr[31:24]};
                3'b100: mem_read_data = sig ?  {{56'b0},dr[39:32]} : {{56{dr[39]}},dr[39:32]};
                3'b101: mem_read_data = sig ?  {{56'b0},dr[47:40]} : {{56{dr[47]}},dr[47:40]};
                3'b110: mem_read_data = sig ?  {{56'b0},dr[55:48]} : {{56{dr[55]}},dr[55:48]};
                3'b111: mem_read_data = sig ?  {{56'b0},dr[63:56]} : {{56{dr[63]}},dr[63:56]};
                default:mem_read_data = 0; 
            endcase
        end
        MSIZE2: begin
            case (addr[2:1])
                2'b00: mem_read_data = sig ?  {{48'b0},dr[15:0]} : {{48{dr[15]}},dr[15:0]};
                2'b01: mem_read_data = sig ?  {{48'b0},dr[31:16]} : {{48{dr[31]}},dr[31:16]};
                2'b10: mem_read_data = sig ?  {{48'b0},dr[47:32]} : {{48{dr[47]}},dr[47:32]};
                2'b11: mem_read_data = sig ?  {{48'b0},dr[63:48]} : {{48{dr[63]}},dr[63:48]}; 
                default:mem_read_data = 0;
            endcase
        end
        MSIZE4: begin
            case (addr[2])
                1'b0:  mem_read_data = sig ?  {{32'b0},dr[31:0]} : {{32{dr[31]}},dr[31:0]};
                1'b1:  mem_read_data = sig ?  {{32'b0},dr[63:32]} : {{32{dr[63]}},dr[63:32]};
                default: mem_read_data = 0;
            endcase
        end
        MSIZE8: begin
            mem_read_data = dr;
        end
        default: mem_read_data = 0;
    endcase
end // pc select
endmodule

`endif 