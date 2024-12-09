module TOP(clk, rst, addr1, addr2, reg_addr, result);
    input clk;
    input rst;
    input [2:0] addr1;
    input [2:0] addr2;
    input [2:0] reg_addr;
    output [7:0] result;

    wire w_rf, DA, SA, SB;
    wire [2:0] adr;
    wire [2:0] st_out;
    wire [2:0] w_ram;
    wire [3:0] data;
    wire[3:0] A, B;
    wire[7:0] m_result;
   
    cu uut1(clk, rst, addr1, addr2, w_rf, adr, DA,SA,SB, st_out, w_ram);    
    ROM uut2(data, adr);  
    RF uut3(A, B, SA, SB, data, DA, w_rf, rst, clk);  
    Combinational_Multiplier uut4(A, B, m_result);
    RAM uut5(clk, rst, w_ram, reg_addr, m_result, result);
   
endmodule

module Combinational_Multiplier(
    input [3:0] a, 
    input [3:0] b, 
    output [7:0] product 
);
    wire [7:0] partial0, partial1, partial2, partial3;

    // partial products
    assign partial0 = b[0] ? {4'b0, a} : 8'b0;      // Least significant bit
    assign partial1 = b[1] ? {3'b0, a, 1'b0} : 8'b0; // Shifted by 1
    assign partial2 = b[2] ? {2'b0, a, 2'b0} : 8'b0; // Shifted by 2
    assign partial3 = b[3] ? {1'b0, a, 3'b0} : 8'b0; // Shifted by 3

    // Sum
    assign product = partial0 + partial1 + partial2 + partial3;
endmodule


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

module cu (

input clk, reset,
  input [2:0] adr1,
  input [2:0] adr2,
  output reg w_rf,
  output reg [2:0] adr,
  output reg DA,SA,SB,
  output reg [2:0] st_out,
  output reg [2:0] w_ram
);

   
 
parameter S0_idle = 0 , S1_send_adr1 = 1 , S2_send_adr2 = 2 ,S3_multiply = 3 ,S4_write_ram = 4,S5_read_ram=5 ;
reg [2:0] PS,NS ;

    always@(posedge clk or posedge reset)
        begin
            if(reset)
                PS <= S0_idle;  
            else    
                PS <= NS ;
        end  


 

    always@(*)
        begin
           
            case(PS)
S0_idle:begin
NS = S1_send_adr1;
                                w_rf <=1'b1;
                                w_ram <=1'b1;
            st_out <= 3'b000;
end

S1_send_adr1:begin
w_rf <=1'b1;
adr<=adr1;
DA <=1'b0;
SA <=1'b0;
SB <=1'b1;
st_out <= 3'b001;
NS = S2_send_adr2;
end

S2_send_adr2:begin
w_rf <=1'b1;
adr<=adr2;
NS = S3_multiply;
DA <=1'b1;
SA <=1'b0;
SB <=1'b1;
            st_out <= 3'b010;
  end

               S3_multiply: begin
NS = S4_write_ram;
                                st_out <= 3'b011;
w_ram<=1;
       end

                                S4_write_ram: begin
                                st_out <= 3'b100;
NS = S5_read_ram;
end

S5_read_ram: begin
                                w_ram<=0;
                                //done <=1;
                                st_out <= 3'b101;
if(!reset) begin
NS = S5_read_ram;
end
else begin
NS = S0_idle;
end
end
endcase
end



endmodule


/*Here I have created a register file using only 2 registers just t make simple. So 1 bit
is enough to represent the register Address
DA- 1bit adress of register where I want to write data
SA- 1bit adress of register, from where I want to read data
SB- 1bit adress of register, from where I want to read data
For most mathemtical operation, we need two data, so 2 registers
source address SA, SB and 1 destination DA adress to write data*/

module RF(A, B, SA, SB, D, DA, W, rst, clk);
output [3:0]A; // A bus
output [3:0]B; // B bus
input SA; // Select A - A Address
input SB; // Select B - B Address
input [3:0]D; // Data input
input DA; // Data destination address
input W; // write enable
input rst; // positive logic asynchronous reset
input clk;

wire [1:0]load_enable;
wire [3:0]R00, R01;


Decoder1to2 decoder (load_enable, DA, W);
RegisterNbit reg00 (D,R00,load_enable[0], rst, clk); //D-in, R00-out
RegisterNbit reg01 (D,R01,load_enable[1], rst, clk);
Mux2to1Nbit muxA (A,R00, R01, SA);
Mux2to1Nbit muxB (B,R00, R01,SB);

endmodule

module RegisterNbit(D, Q,  L, R, clock);
parameter N = 4; // number of bits
output reg [N-1:0]Q; // registered output
input [N-1:0]D; // data input
input L; // load enable
input R; // positive logic asynchronous reset
input clock; // positive edge clock

always @(posedge clock or posedge R) begin
if(R)
Q <= 0;
else if(L)
Q <= D;
else
Q <= Q;
end
endmodule

module Decoder1to2(m, S, en);
input S; // select
input en; // enable (positive logic)
output [1:0]m; // 32 minterms

assign m[0] = ~S&en;
assign m[1] = S&en;

endmodule

module Mux2to1Nbit(o, i1,i2, s);
   input [3:0] i1,i2;
   input  s;
   output reg  [3:0] o;
 
always @(s or i1 or i2)
begin
   case (s)
      1'b0 : o = i1;
      1'b1 : o = i2;
      default : o = 4'b0;
   endcase
end
endmodule


