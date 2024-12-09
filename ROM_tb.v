`timescale 1ns / 1ps

module ROM_tb;
    reg [2:0] ROM_addr;           
    wire [3:0] ROM_data;         
    
    ROM uut (ROM_data, ROM_addr);

    initial begin
        ROM_addr = 3'd0;

        repeat (8) begin
            #10;
            ROM_addr = ROM_addr + 1;
        end
        $stop;
    end
endmodule


