//pds7-branching
module branch(input1,input2,input3,pc,opcode);
input [31:0] input1,input2;input [5:0] input3;input [5:0] opcode;output reg [4:0] pc;
initial begin
    pc=6'd0;
end
always @(*)
begin
     case (opcode)
      6'd4:begin if(input1==input2) begin 
      pc=pc+6'd4+input3*4;
      end 
      end
      6'd5:begin if(input1!=input2) begin 
      pc=pc+6'd1+input3;
      end end
      6'd7:begin if(input1>input2) begin 
      pc=pc+6'd1+input3;
      end end
      6'd1: begin if(input1<=input2) begin 
      pc=pc+6'd1+input3;
      end end 
      6'd11: begin if(input1>input2) begin 
      pc=pc+6'd1+input3;
      end end
      6'd6:begin if(input1>=input2) begin 
      pc=pc+6'd1+input3;
      end end
    endcase
end
endmodule
module tb;
reg [31:0] input1,input2;reg [5:0] input3;
reg [5:0] opcode;wire  [4:0] pc;
branch uut(.input1(input1),.input2(input2),.input3(input3),.opcode(opcode),.pc(pc));
initial begin
      $monitor("%b %b \n ",opcode,pc);
      input1=32'b10000000000000000000000000100010;
      input2=32'b10000000000000000000000000100010;
      input3=6'd10;
      opcode=6'd1;
      #100
      $finish;
end

endmodule