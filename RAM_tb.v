`timescale 1ns / 1ps

module RAM_tb;
    reg i_clk;
    reg i_rst;
    reg i_write_en;
    reg [2:0] i_addr;
    reg [7:0] i_write_data;
    wire [7:0] o_read_data;

    RAM uut (i_clk,i_rst,i_write_en,i_addr,i_write_data,o_read_data);

    initial i_clk = 0;
    always #5 i_clk = ~i_clk;

    initial begin
        // Initialize 
        i_rst = 1; 
        i_write_en = 0; 
        i_addr = 0; 
        i_write_data = 0;
        #10 i_rst = 0;

        // Write values
        i_write_en = 1;
        for (integer i = 0; i < 8; i = i + 1) begin
            i_addr = i[2:0];      
            i_write_data = i + 8'haa; // Example data pattern starting from aa all the way to b1
            #10; 
        end

        // Read values 
        i_write_en = 0;
        for (integer i = 0; i < 8; i = i + 1) begin
            i_addr = i[2:0];    
            #10;
        end
        $stop;
    end
endmodule


