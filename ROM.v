`timescale 1ns / 1ps

module ROM (ROM_data, ROM_addr);
output reg [3:0] ROM_data;
input [2:0] ROM_addr;

always @(ROM_addr) begin
	case (ROM_addr)
         3'd0: ROM_data = 4'b0000; // 0
         3'd1: ROM_data = 4'b1100; // c
         3'd2: ROM_data = 4'b0110; // 6
         3'd3: ROM_data = 4'b0111; // 7
         3'd4: ROM_data = 4'b1000; // 8
         3'd5: ROM_data = 4'b0001; // 1
         3'd6: ROM_data = 4'b1101; // d
         3'd7: ROM_data = 4'b1110; // e
    endcase
end
endmodule

