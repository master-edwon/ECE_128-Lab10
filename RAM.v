module RAM (
    input i_clk,                 
    input i_rst,                 
    input i_write_en,            
    input [2:0] i_addr,          
    input [7:0] i_write_data,
    output reg [7:0] o_read_data 
);
    reg [7:0] mem [0:7];     // Memory array: 8 locations, each 8 bits wide

    always @(posedge i_clk) begin
        if (i_rst) begin
            // Reset to zero
            mem[0] <= 8'b0;
            mem[1] <= 8'b0;
            mem[2] <= 8'b0;
            mem[3] <= 8'b0;
            mem[4] <= 8'b0;
            mem[5] <= 8'b0;
            mem[6] <= 8'b0;
            mem[7] <= 8'b0;
        end 
        else begin
            if (i_write_en)
                mem[i_addr] <= i_write_data; // Write operation
            else
                o_read_data <= mem[i_addr];  // Read operation
        end
    end
endmodule


