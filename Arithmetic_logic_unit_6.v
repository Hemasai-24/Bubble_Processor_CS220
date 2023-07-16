//pds6-alu
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
module ALU (input1,input2,opcode,out,cout,bout);
  input [31:0] input1;
  input [31:0] input2;
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
      6'd0: out = sum;//addu
      6'd1: out = sub;//subu
      6'd2|6'd12: out = w3;// and,andi
      6'd3|6'd13: out = w4;//  or,ori
      6'd4|6'd14: out = w5;//  xor,xori
      6'd5: out = w2;// shift left
      6'd6: out = w1;// shift right 
      6'd7: out = (input1 < input2) ? 1 : 0; //slt
      default: out = 0; 
    endcase
  end

endmodule

module tb;
reg [31:0] input1,input2;wire cout; wire bout;
reg [5:0] opcode;
wire [31:0] out;
ALU uut(.input1(input1),.input2(input2),.opcode(opcode),.out(out),.cout(cout),.bout(bout));
initial begin
      $monitor("%b %b %b\n ",opcode,out,cout);
      input1=32'b10000000000000000000000000100010;
      input2=32'b10000000000000000000000000100010;
      opcode=6'd1;
      #100
      $finish;
end
endmodule





















































































