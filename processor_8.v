//pds_8
module IF(clk,reset,instruction,ad,out,out_pc);
input clk,reset;reg [31:0] inst;input [31:0]instruction;output reg ad;output reg [31:0] out;

 reg [5:0]  opcode;reg [4:0] rs,rd,rt,shamt;reg [5:0] funct;reg [5:0] imd_i; reg[31:0] sum;
 reg[5:0] imd_j;reg [5:0] pc;reg [5:0] pc_r;
 output reg [5:0] out_pc;

reg [31:0] register[31:0]; 

 reg clk_r,rst_r,wrt_enb_r;
 reg [5:0] add_r;reg [31:0] data_in_r;
 wire [31:0] data_out_w;reg mode_r;
 reg rtype;reg itype;reg jtype;wire [5:0] pc_out;
 reg [31:0] input1_r,input2_r;reg cin_r;wire cout_w;reg cout; wire [31:0] res;

 always@(posedge clk)
 begin 
    clk_r<=clk;rst_r<=1'd1;wrt_enb_r<=1'd1;add_r<=pc;data_in_r<=instruction;mode_r<=1'd1;inst<=data_out_w;
    cin_r<=1'd0;cout<=cout_w;pc_r<=pc;
 end 

memoryo g1(clk_r,rst_r,wrt_enb_r,add_r,data_in_r,mode_r,data_out_w);//instruction 1
integer i;
    
always@(posedge clk)
begin 
    //$display("data_out_w=%b ",data_out_w);
    if(reset) begin
       for(i=0;i<32;i=i+1)
        begin:loop
        register[i]<=10000000000000000000000000000001;
        end
       rtype<=1'd0;
       itype<=1'd0;
       jtype<=1'd0;
       ad<=1'd0;
       pc<=6'd0;
       out<=0;
       out_pc<=6'd0;
    end
    else begin
    opcode<=inst[31:26];
    //$display("opcode sg=%b ",opcode);
    if(opcode >= 6'd0&&opcode< 6'd10)begin
            rtype<=1'd1;
            rs <= inst[25:21];
            rt <= inst[20:16];
            rd <= inst[15:11];
            shamt <= inst[10:6];
            funct <= inst[5:0];
            end
     else if (opcode != 17 & opcode != 18)  begin
           itype<=1'd1;
              rs <= inst[25:21];
              rt <= inst[20:16];
              imd_i <= inst[5:0];
     end
            else begin
                jtype<=1'd1;
              imd_j <= inst[5:0];
            end
    end  end
 ALU gh(register[rs],register[rt],shamt,funct,imd_i,imd_j,opcode,res,pc_r,pc_out);
    always @(posedge clk) begin
            if(rtype==1'd1) begin
              assign out=res;
              pc=pc+1;
            end
            if(jtype|itype==1'd1) 
            begin
               // $display("hoi ghhkjdvkld");
              out_pc=pc_out;
              pc=pc_out;
            end
 end 
endmodule

module ALU (rs_data,rt_data,shamt_data,funct,imd_i,imd_j,opcode,out,pc,out_pc);
  input [31:0] rs_data,rt_data;
  input [5:0] imd_i,imd_j,opcode,pc,funct;
 input [4:0]  shamt_data;
  output reg [5:0] out_pc;
  output reg [31:0] out;
  reg [31:0] sum,sub;wire [31:0] w1,w2,w3,w4,w5;output reg bout;reg cout;
  reg [31:0] input1_r,input2_r;wire [31:0] sum_w,sub_w;wire cout_w;reg cin_r;wire bout_w; 

  always @(*)
  begin
   input1_r<=rs_data;input2_r<=rt_data;cin_r<=1'd0;cout<=cout_w;sum<=sum_w;sub<=sub_w;bout<=bout_w;
  end
   adder fg(input1_r,input2_r,cin_r,cout_w,sum_w);
   full_subtractor gh(input1_r,input2_r,sub_w,bout_w);
   rshift hi(input1_r,input2_r,w1);
   lshift ij(input1_r,input2_r,w2);
   andh jk(input1_r,input2_r,w3);
   orh kl(input1_r,input2_r,w4);
   xorh lm(input1_r,input2_r,w5);

  always @(*) begin
    //$display("opcodealu=%b ",opcode);
   case (opcode)
      6'd0:
      if(funct==6'd32) begin
       out = sum;//addu
      end 
      else if(funct==6'd34) begin
          out = sub;//subu
      end
    else if(funct==6'd30) begin
         input1_r=$signed(rs_data);
         input2_r=$signed(rt_data);
         out=sum;
      end
      6'd1|6'd2: out = w3;// and,andi
      6'd3|6'd4: out = w4;//  or,ori
      6'd5|6'd6: out = w5;//  xor,xori
      6'd7: out = w2;// shift left
      6'd8: out = w1;// shift right 
      6'd9: out = ($signed(rs_data))< $signed(rt_data)? 1 : 0; //slt
      6'd10:begin
         if(rs_data==rt_data) begin 
          out_pc=pc+6'd1+imd_i;
         end 
         end
      6'd11:begin if(rs_data!=rt_data) begin 
      out_pc=pc+6'd1+imd_i;
      end end
      6'd13:begin if(rs_data>rt_data) begin 
      out_pc=pc+6'd1+imd_i;
      end end
      6'd14: begin if(rs_data<=rt_data) begin 
      out_pc=pc+6'd1+imd_i;
      end end 
      6'd15: begin if(rs_data>rt_data) begin 
      out_pc=pc+6'd1+imd_i;
      end end
      6'd16:begin 
        if(rs_data>=rt_data) begin 
      out_pc=pc+6'd1+imd_i; 
       
      end end
      6'd17:begin  
        out_pc=imd_j;
       end 
      default: begin out = 0; out_pc=0; end
    endcase
    
  end
endmodule
 module tb;      
 reg clk;reg [31:0]instruction;reg [5:0]imd;reg reset;
 wire ad;wire [31:0] out;reg [31:0] A,B;wire [5:0] out_pc;
 IF uut(.clk(clk),.reset(reset),.instruction(instruction),.ad(ad),.out(out),.out_pc(out_pc));
 always #10 clk<=~clk;
  initial begin
   reset=1;
     clk=1;
    //instruction=32'd32;
    instruction=32'b00000000000000000000000000100000;
    #10
    reset=0;
     clk=1;
    //instruction=32'd32;
    instruction=32'b00000000000000000000000000100000;
    #100
    clk=1;
    instruction=32'd34;
    #1000 $finish;
 end
 
  initial begin 
    #500 $display("addition=%b time=%g,out=%b,out_pc=%b",ad,$time,out,out_pc); 
      //  #200 $display("addition=%b time=%g,out=%b,imd=%b,out_pc=%b",ad,$time,out,imd,out_pc); 
      //      #300 $display("addition=%b time=%g,out=%b,imd=%b,out_pc=%b",ad,$time,out,imd,out_pc); 
      //          #400 $display("addition=%b time=%g,out=%b,imd=%b,out_pc=%b",ad,$time,out,imd,out_pc); 
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
             $display("data_out=%b",data_out);
        end
     end
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