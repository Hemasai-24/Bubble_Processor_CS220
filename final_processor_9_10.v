module control_unit(clk,reset,out);
input clk,reset;
output reg [31:0] out;
reg[5:0] pc;wire [5:0]pc_out;
reg [31:0] curr_inst,curr_data,data_in;
reg [5:0] opcode,funct;reg [15:0] imd_i;
reg [4:0] rs,rt,rd,shamt;reg [25:0] imd_j;
reg[1:0] state;reg rtype,itype,jtype;reg flag,mode;
reg [31:0]register[31:0];
//intermediate registers for always
reg [5:0] add1;reg [4:0] add2;reg mode_r;
wire [31:0] data_out1,data_out2;
reg [31:0] input1,input2;wire [31:0] result;reg flag_r;reg [31:0] data_in_r;
initial begin
  for(i=0;i<32;i=i+1) 
      register[i]<=0;
      pc<=10;
      add1<=0;
      add2<=0;
end
always @(posedge clk)
begin
  add1<=pc;mode_r<=mode;curr_inst<=data_out1;curr_data<=data_out1;data_in_r<=data_in;
  // $display("add1 =%b add2=%b time=%d curr_inst=%b",add1,add2,$time,curr_inst);
end
instruction_memory a1(clk,mode_r,flag_r,add1,add2,data_in_r,data_out1);//instruction memory and data emmory
ALU a2(clk,register[rs],register[rt],shamt,funct,imd_i,imd_j,opcode,result,pc,pc_out);
integer i;
always @(posedge clk) begin
  if(reset) begin
    for(i=0;i<32;i=i+1) begin
      register[i]<=0;
    end
    rtype<=1'd0;
    itype<=1'd0;
    jtype<=1'd0;
    pc<=6'd10;
    out<=0;
    opcode<=0;
    state = 2'd0;
    flag<=0;
    mode<=0;
  end
  else if(state==0) begin
    // $display("opcode=%b pc=%b inst=%b register=%b",opcode,pc,curr_inst,register[1]);
    opcode<=curr_inst[31:26];
    state = 2'd1;
   // $display("state=%b",state);
  if(state==1) begin
    if(opcode >= 6'd0) begin
      if(opcode<=6'd10) begin
        rtype = 1'd1;itype = 1'd0;jtype = 1'd0;
        rs<=curr_inst[25:21];
        rt<=curr_inst[20:16];
        rd<=curr_inst[15:11];
        shamt <=curr_inst[10:6];
        funct <=curr_inst[5:0];
        state = 2'd2;
       // $display("state=%b",state);
        flag<=0;
      end
      else if(opcode!=18) begin
        itype = 1'd1;rtype = 1'd0;jtype = 1'd0;
        imd_i <= curr_inst[15:0];
        rs <= curr_inst[25:21];
        rt <= curr_inst[20:16];
        state = 2'd2;  
      end
      else begin 
        itype =1'd0;rtype =1'd0;jtype =1'd1;
        imd_j <= curr_inst[25:0];
        state = 2'd2;
        end
    end end 
    if(state==2) begin
      if(rtype==1'd1) begin
        register[rd]<=result;
        out<=result;
        pc = pc+1;
        state = 2'd0;
        //$display("state=%d",state);
        //$display("register[rd]=%d time=%d",register[rd],$time);
      end
      else if(jtype==1'd1) begin 
        pc =pc_out;
        state = 2'd0;
      end
      else if(opcode==16) begin
        flag<=1;
        add2<=imd_i + rs;
        register[rt]<=curr_data;
        state = 2'd0;
        pc =pc+1;
        mode<=0;
      end
      else if(opcode==17)begin
        flag<=1;
        mode<=1;
        add2<=imd_i + rs; 
        data_in<=register[rt];
        state = 2'd0;
        pc = pc+1;
      end
      if(opcode==19)begin
        register[rt]<=result;
        state = 2'd0;
        pc= pc+1;
      end
     // $display(" j state=%d",state);
      end
    end 
  end 
endmodule
module tb;
reg clk,reset;
wire [31:0] out;
control_unit uut(.clk(clk),.reset(reset),.out(out));
always #9 clk=~clk;
initial begin
 // $monitor("clk=%b time=%g,output=%b",clk,$time,out);
end
initial begin
  clk=1;
  reset=1;
  #100
  reset=0;
  #1000
  $finish;
  end
endmodule
module instruction_memory(clk,mode,flag,add1,add2,data_in,data_out);
input clk,mode,flag;input [5:0] add1;input [4:0] add2;
input [31:0] data_in;
output  reg [31:0] data_out;
reg [9:0] data_mem[31:0];//flag=1;
reg [31:0] instruction_mem [31:0];//flag=0;
    initial begin                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    instruction_mem[10] = {6'b000000,5'd0,5'd0,5'd1,5'd0,6'b100000};//s0=0
    instruction_mem[11] = {6'b000000,5'd0,5'd0,5'd2,5'd0,6'b100000};//s1=0
    instruction_mem[12] = {6'b000000,5'd0,5'd0,5'd7,5'd0,6'b100000};
    instruction_mem[13] = {6'b001000,5'd7,5'd7,16'd9};//s6=9
    instruction_mem[14] = {6'b000000,5'd2,5'd8,5'd16,5'd0,6'b100000};//add t7,s7,s1 //loop start
    instruction_mem[15] = {6'b100011,5'd16,5'd9,16'd0};//lw t0,0(t7)
    instruction_mem[16] = {6'b100011,5'd16,5'd10,16'd1};//lw t1,1(t7)
    instruction_mem[17] = {6'b000000,5'd9,5'd10,5'd11,5'd0,6'b101010};//slt t2,t0,t1
    instruction_mem[18] = {6'b000101,5'd0,5'd11,16'd21};//bne t2,zero,increment 
    instruction_mem[19] = {6'b101011,5'd16,5'd10,16'd0};//sw t1,0(t7)
    instruction_mem[20] = {6'b101011,5'd16,5'd9,16'd1};// sw t0,1(t7)
    instruction_mem[21] = {6'b001000,5'd2,5'd2,16'd1};//increment start , addi s1,s1,1
    instruction_mem[22] = {6'b000000,5'd7,5'd1,5'd6,5'd0,6'b100010};//sub s5,s6,s0
    instruction_mem[23] = {6'b000101,5'd2,5'd6,16'd14};// bne s1,s5,loop
    instruction_mem[24] = {6'b001000,5'd1,5'd1,16'd1};// addi s0,s0,1
    instruction_mem[25] = {6'b000000,5'd0,5'd0,5'd2,5'd0,6'b100000};// reset s1=0
    instruction_mem[26] = {6'b000101,5'd1,5'd7,16'd14};// bne s0,s6,loop
    data_mem[0] = 32'b00000000000000000000000000000011;//3
    data_mem[1] = 32'b00000000000000000000000000001010;//10
    data_mem[2] = 32'b00000000000000000000000000000010;//2
    data_mem[3] = 32'b00000000000000000000000000000100;//4
    data_mem[4] = 32'b00000000000000000000000000000101;//5
    data_mem[5] = 32'b00000000000000000000000000000110;//6
    data_mem[6] = 32'b00000000000000000000000000000000;//0
    data_mem[7] = 32'b00000000000000000000000000000111;//7
    data_mem[8] = 32'b00000000000000000000000000000001;//1
    data_mem[9] = 32'b00000000000000000000000000001000;//8
    end
always @(posedge clk) begin
  if(mode)//write
  begin
    if(flag) begin
       data_mem[add2]<=data_in;
    end
  end
  else begin//read
    if(flag) begin
       data_out<=data_mem[add2];
    end
    else begin
     // $display("add1=%b",add1);
      //$display("data=%b",data_out);
      data_out<=instruction_mem[add1];
    end
  end
    if(add1 >=6'b010010) begin
                    $display("%d",data_mem[0]);
                    $display("%d",data_mem[1]);
                    $display("%d",data_mem[2]);
                    $display("%d",data_mem[3]);
                    $display("%d",data_mem[4]);
                    $display("%d",data_mem[5]);
                    $display("%d",data_mem[6]);
                    $display("%d",data_mem[7]);
                    $display("%d",data_mem[8]);
                    $display("%d",data_mem[9]);
                    end
                    $display("add1=%b",add1);
                    $display("data_out=%b",data_out);
end 

endmodule

module ALU (clk,rs_data,rt_data,shamt_data,funct,imd_i,imd_j,opcode,out,pc,out_pc);
  input clk;
  input [31:0] rs_data,rt_data;
  input [15:0] imd_i;input [25:0]imd_j;input [5:0]opcode,pc,funct;
  input [4:0]  shamt_data;
  output reg [5:0] out_pc;
  output reg [31:0] out;
  reg [31:0] sum,sub;wire [31:0] w1,w2,w3,w4,w5;
  reg [31:0] input1_r,input2_r;wire [31:0] sum_w,sub_w;reg cin_r;

  always @(posedge clk)
  begin
   input1_r<=rs_data;input2_r<=rt_data;cin_r<=1'd0;sum<=sum_w;sub<=sub_w;
  end
   adder fg(input1_r,input2_r,cin_r,sum_w);
   full_subtractor gh(input1_r,input2_r,sub_w);
   rshift hi(input1_r,input2_r,w1);
   lshift ij(input1_r,input2_r,w2);
   andh jk(input1_r,input2_r,w3);
   orh kl(input1_r,input2_r,w4);
   xorh lm(input1_r,input2_r,w5);
  always @(*) begin
   case (opcode)
      6'd0:
      if(funct==6'd32) begin
       out <= sum;//addu
      end 
      else if(funct==6'd34) begin
       out <= sub;//subu
      end
    else if(funct==6'd30) begin
      input1_r<=$signed(rs_data);
      input2_r<=$signed(rt_data);
       out <=sum;//add
      end
      6'd1|6'd2: out <= w3;// and,andi
      6'd3|6'd4: out <= w4;//  or,ori
      6'd5|6'd6: out <= w5;//  xor,xori
      6'd7: out <= w2;// shift left//sll
      6'd8: out <= w1;// shift right//srl 
      6'd9: out <= ($signed(rs_data))< $signed(rt_data)? 1 : 0; //slt
      6'd10:begin if(rs_data==rt_data) begin 
      out_pc<= pc + 6'd1+imd_i;
      end end
      6'd11:begin if(rs_data!=rt_data) begin 
      out_pc<=imd_i;
      end end
      6'd12:begin if(rs_data>rt_data) begin 
      out_pc<=pc+ 6'd1+imd_i;
      end end
      6'd13: begin if(rs_data<=rt_data) begin 
      out_pc<=pc+ 6'd1+imd_i;
      end end 
      6'd14: begin if(rs_data>rt_data) begin 
      out_pc<=pc+ 6'd1+imd_i;
      end end
      6'd15:begin if(rs_data>=rt_data) begin 
      out_pc<=pc+6'd1+imd_i;
      end end
      6'd18:begin  
      out_pc=imd_j;
      end
      6'd19:begin
        out<=rs_data+imd_i;
      end
      default: begin out = 0; out_pc=0; end
    endcase
  end
endmodule
module adder(A,B,cin,sum);
    input [31:0] A;
    input [31:0] B;
    input cin;
    output [31:0] sum;wire cout;
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
module full_subtractor(a,b,d);
input [31:0] a,b;
//input [31:0] b;
output [31:0] d;wire bout;
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

