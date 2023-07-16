module memory(clk,rst,wrt_enb,add,data_in,mode,data_out);
    input clk,rst,wrt_enb,mode;
    input [31:0]data_in;
    input [4:0]add;
    output reg [31:0]data_out;
    reg [31:0]data;
    reg [31:0] memory [31:0];
    
    always @(posedge clk)
     begin
        if(mode == 1'b1)
         begin
            data_out = data;
           data = memory[add];
        
        end
        else
         begin
            data_out = data;
            memory[add] = data_in;
            data = data_in;
            
        end
    end
endmodule

module memory_tb;
    reg clk,rst,wrt_enb,mode;
    reg [31:0]data_in;
    wire [31:0]data_out;
    reg [4:0]add;
    memory uut(clk,rst,wrt_enb,add,data_in,mode,data_out);
    always 
    #10 clk = ~clk;
    initial
     begin
        
        rst=0;wrt_enb=1;mode=0;clk=0;
         add = 5'b00001;
        data_in = 32'b0;
    #20;
    mode =0;
    add = 5'b00011;
    data_in = 32'b1;
    #20;
    add = 5'b00101;
    data_in = 32'd2;
    #20;
    mode =1 ;
    add = 5'b00001;
    #20;
    add = 5'b00101;
    #100 $finish;
        
     end

   initial 
begin
    $monitor("time=%g data_out=%b",$time,data_out);
end
endmodule