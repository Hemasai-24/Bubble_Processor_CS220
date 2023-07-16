//pds4,5
module IF(clk,instruction,ad,A,B,out,imd,out_pc);
input clk;reg [31:0] inst;input [31:0]instruction;output reg ad;input [31:0] A,B;output reg [31:0] out;

 reg [5:0]  opcode;reg [4:0] rs,rd,rt,shamt;reg [5:0] funct;reg [15:0] imd_i; reg[31:0] sum;input [5:0] imd;
 reg[25:0] imd_j;reg [5:0] pc;reg [5:0] pc_r;
 output reg [5:0] out_pc;

reg [31:0] register[31:0]; 
initial begin
  register[0]<=31'd0;
  register[1]<=31'd0;
  register[2]<=31'd0;
  register[3]<=31'd0;
  register[4]<=31'd0;
  register[5]<=31'd0;
  register[6]<=31'd0;
  register[7]<=31'd0;
  register[8]<=31'd0;
  register[9]<=31'd0;
  register[10]<=31'd0;
  register[11]<=31'd0;
  register[12]<=31'd0;
  register[13]<=31'd0;
  register[14]<=31'd0;
  register[15]<=31'd0;
  register[16]<=31'd0;
  register[17]<=31'd0;
  register[18]<=31'd0;
  register[19]<=31'd0;
  register[20]<=31'd0;
  register[21]<=31'd0;
  register[22]<=31'd0;
  register[23]<=31'd0;
  register[24]<=31'd0;
  register[25]<=31'd0;
  register[26]<=31'd0;
  register[27]<=31'd0;
  register[28]<=31'd0;
  register[29]<=31'd0;
  register[30]<=31'd0;
  register[31]<=31'd0;
end

 reg clk_r,rst_r,wrt_enb_r;
 reg [5:0] add_r,add_r1,add_r2;reg [31:0] data_in_r,data_in_r1,data_in_r2;
 wire [31:0] data_out_w,data_out_w1,data_out_w2;reg mode_r;
 reg rtype;reg itype;reg jtype;reg [31:0] a,b;reg [31:0] sub;wire [5:0] pc_out;

 reg [31:0] input1_r,input2_r;reg cin_r;wire cout_w;wire [31:0] sum_w,sub_w;reg cout; wire [31:0] res;

initial begin
   rtype<=1'd0;
   itype<=1'd0;
   jtype<=1'd0;
   ad<=1'd0;
   pc<=6'd0;
   out<=0;
   out_pc<=6'd0;
end
 always@(posedge clk)
 begin 
    clk_r<=clk;rst_r<=1'd1;wrt_enb_r<=1'd1;add_r<=pc;data_in_r<=instruction;data_in_r1<=A;data_in_r2<=B;mode_r<=1'd1;inst<=data_out_w;
    a<=data_out_w1;b<=data_out_w2;
    input1_r<=a;input2_r<=b;cin_r<=1'd0;cout<=cout_w;sum<=sum_w;sub<=sub_w;pc_r<=pc;
 end 

memoryo g1(clk_r,rst_r,wrt_enb_r,add_r,data_in_r,mode_r,data_out_w);//instruction 1
memoryo g2(clk_r,rst_r,wrt_enb_r,add_r1,data_in_r1,mode_r,data_out_w1);//input1
memoryo g3(clk_r,rst_r,wrt_enb_r,add_r2,data_in_r2,mode_r,data_out_w2);//input2
 ALU g4(data_in_r1,data_in_r2,imd,data_out_w[31:26],res,data_out_w[5:0],pc);
 branch g5(data_in_r1,data_in_r2,imd,pc_r,data_out_w[31:26],pc_out); 

always@(posedge clk)
begin
    
       opcode<=inst[31:26];
  
      if (opcode == 1'd0) begin
            rtype<=1'd1;
            rs <= inst[25:21];
            rt <= inst[20:16];
            rd <= inst[15:11];
            shamt <= inst[10:6];
            funct <= inst[5:0];
            end
     else if (opcode != 2 & opcode != 3)  begin
           itype<=1'd1;
              rs <= inst[25:21];
              rt <= inst[20:16];
              imd_i <= inst[15:0];
            end
            else begin
                jtype<=1'd1;
              imd_j <= inst[25:0];
            end
            if(opcode<6'd10&&opcode>=6'd0) begin
              assign out=res;
              pc=pc+1;
            end
            if(opcode<6'd17&&opcode>=6'd11) begin
              out_pc=pc_out;
              pc=pc_out;
            end
 end
endmodule
 module memoryo(clk,rst,wrt_enb,add,data_in,mode,data_out);
    input clk,rst,wrt_enb,mode;
    input [31:0]data_in;
    input [5:0]add;
    output reg [31:0]data_out;
    reg [31:0] data;
    reg [31:0] memory [31:0];
    always @(posedge clk or posedge rst)
     begin if(wrt_enb) begin
        if(!mode)
         begin
             $display("data_out=%b",data_out);
             data_out = data;
           memory[add]=data_in;
           
        end 
        else 
         begin
          
             data_out = data_in; 
             memory[add] = data_in;
             data=data_in;
       
        end
     end
 
    end
 endmodule 
module branch(input1,input2,input3,pc,opcode,out_pc);
input [31:0] input1,input2;input [5:0] input3;input [5:0] opcode;input [5:0] pc;  output reg [5:0] out_pc;
always @(*)
begin
     case (opcode)
      6'd11:begin if(input1==input2) begin 
      out_pc=pc+6'd1+input3;
      end 
      end
      6'd12:begin if(input1!=input2) begin 
      out_pc=pc+6'd1+input3;
      end end
      6'd13:begin if(input1>input2) begin 
      out_pc=pc+6'd1+input3;
      end end
      6'd14: begin if(input1<=input2) begin 
      out_pc=pc+6'd1+input3;
      end end 
      6'd15: begin if(input1>input2) begin 
      out_pc=pc+6'd1+input3;
      end end
      6'd16:begin if(input1>=input2) begin 
      out_pc=pc+6'd1+input3;
        //$display(" ha %b",out_pc);
      //$display("hi %b",pc);
      end end
    endcase
end
endmodule
module adder(A,B,cin,cout,sum);
    input [31:0] A;
    input [31:0] B;
    input cin;
    output [31:0] sum;output cout;
    genvar i;wire[32:0] c;assign c[0]=cin;
    for(i=0;i<32;i=i+1)
    begin :loop
        full_adder df(A[i],B[i],c[i],c[i+1],sum[i]);
    end
assign cout=c[32];
endmodule
module full_adder(a,b,cin,cout,s);
    input a,b,cin;
    output s,cout;
    
    assign s=(a^b)^cin;
    assign cout=(a&b)|(a&cin)|(b&cin);
endmodule
module full_subtractor(a,b,d,bout);
input [31:0] a,b;
//input [31:0] b;
output [31:0] d;output bout;
wire [32:0] b_in;
wire a12,b12,d12;
not g1(a12,a[0]);
not g2(b12,b[0]);
and g4(d12,a[0],b12);
and g3(b_in[0],a12,b[0]);
or g5(d[0],b_in[0],d12);

genvar i;
for(i=1;i<32;i=i+1)
begin
    if(i>0) sub f(a[i],b[i],b_in[i-1],d[i],b_in[i]);
end
assign bout=b_in[32];
endmodule

module sub(a,b,b_in,d,b_out);
input a,b,b_in;
output d,b_out;
wire d1,d2,b1,a1,a2,a3;
xor g1(d1,a,b);
xor g2(d,d1,b_in);
xor g3(a1,1,a);
and g4(a2,a1,b);
xor g5(d2,d1,1);
and g6(a3,d2,b_in);
or g7(b_out,a2,a3);
endmodule

module andh(in1,in2,out);
input [31:0] in1,in2;output [31:0] out;
assign out=in1 & in2;
endmodule
module orh(in1,in2,out);
input [31:0] in1,in2;output [31:0] out;
assign out=in1 | in2;
endmodule
module xorh(in1,in2,out);
input [31:0] in1,in2;output [31:0] out;
assign out=in1^in2;
endmodule
module rshift(in1,in2,out);
input [31:0] in1,in2;output [31:0] out;
assign out=in1>>in2;
endmodule
module lshift(in1,in2,out);
input [31:0] in1,in2;output [31:0] out;
assign out=in1<<in2;
endmodule
module ALU (input1,input2,input3,opcode,out,funct,pc);
  input [31:0] input1;input[5:0] funct;input [5:0] pc;
  input [31:0] input2;input [5:0] input3;
  input [5:0] opcode;
  output reg [31:0] out;output reg cout;reg [31:0] sum,sub;wire [31:0] w1,w2,w3,w4,w5;output reg bout;
  reg [31:0] input1_r,input2_r;wire [31:0] sum_w,sub_w;wire cout_w;reg cin_r;wire bout_w; 
  always @(*)
  begin
   input1_r<=input1;input2_r<=input2;cin_r<=1'd0;cout<=cout_w;sum<=sum_w;sub<=sub_w;bout<=bout_w;
  end
   adder fg(input1_r,input2_r,cin_r,cout_w,sum_w);
   full_subtractor gh(input1_r,input2_r,sub_w,bout_w);
   rshift hi(input1,input2,w1);
   lshift ij(input1,input2,w2);
   andh jk(input1,input2,w3);
   orh kl(input1,input2,w4);
   xorh lm(input1,input2,w5);

  always @(*) begin
   case (opcode)
      6'd0:
      if(funct==6'd32) begin
       out = sum;//addu
      end 
      else if(funct==6'd34) begin
          out = sub;//subu
      end
      6'd1|6'd2: out = w3;// and,andi
      6'd3|6'd4: out = w4;//  or,ori
      6'd5|6'd6: out = w5;//  xor,xori

      6'd7: out = w2;// shift left
      6'd8: out = w1;// shift right 
      6'd9: out = (input1 < input2) ? 1 : 0; //slt
      default: out = 0; 
    endcase
  end
endmodule
 module tb;      
 reg clk;reg [31:0]instruction;reg [5:0]imd;
 wire ad;wire [31:0] out;reg [31:0] A,B;wire [5:0] out_pc;
 IF uut(.clk(clk),.instruction(instruction),.ad(ad),.A(A),.B(B),.out(out),.imd(imd),.out_pc(out_pc));

 always #10 clk<=~clk;
  initial begin
  
     clk=1;
    instruction=32'd32;
    //instruction=32'b01000000110010000000000000000000;
    A=32'd6;
    B=32'd4;
    imd=6'd10;
    #100
    clk=1;
    instruction=32'd34;
    //instruction=32'b01000000110010000000000000000000;
    A=32'd6;
    B=32'd4;
    imd=6'd10;
    
      clk=1;
   // instruction=32'd34;
    instruction=32'b01000000110010000000000000000000;//16
    A=32'd6;
    B=32'd4;
    imd=6'd10;
    #100
    clk=1;
   // instruction=32'd34;
    instruction=32'b00110000110010000000000000000000;//12
    A=32'd6;
    B=32'd4;
    imd=6'd10;
    #1000 $finish;
 end
 
  initial begin 
    #100 $monitor("addition=%b time=%g,out=%b,imd=%b,out_pc=%b",ad,$time,out,imd,out_pc); 
  end
 endmodule

