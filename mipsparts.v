`timescale 1ns/1ps
`define mydelay 1

//------------------------------------------------
// mipsparts.v
// David_Harris@hmc.edu 23 October 2005
// Components used in MIPS processor
//------------------------------------------------


module forward_unit (input clk,
                     input [4:0] rsD, rtD, rsE, rtE,
							input regwriteM, regwriteW, /* ??? */
							input [4:0] writeregM, writeregW,
							output reg forwardAD,
							output reg forwardBD,
							output reg [1:0] forwardAE,
							output reg [1:0] forwardBE,
							input [1:0] alusrcAE);

  always @(posedge clk)
  begin
    if (writeregW != 5'b0 && regwriteM) begin
      if (rsE == writeregM) forwardAE <= #`mydelay 2'b10;
      else if (rsE == writeregW) forwardAE <= #`mydelay 2'b01;
      else forwardAE <= #`mydelay 2'b00;
	   if (rtE == writeregM) forwardBE <= #`mydelay 2'b10;
      else if (rtE == writeregW) forwardBE <= #`mydelay 2'b01;
      else forwardBE <= #`mydelay 2'b00;
	   if (rsD == writeregW) forwardAD <= #`mydelay 1'b1;
      else forwardAD <= #`mydelay 1'b0;
	   if (rtD == writeregW) forwardBD <= #`mydelay 1'b1;
	   else forwardBD <= #`mydelay 1'b0;
	 end
	 else begin
	   forwardBD <= #`mydelay 1'b0;
		forwardAD <= #`mydelay 1'b0;
		forwardBE <= #`mydelay 2'b00;
		forwardAE <= #`mydelay 2'b00;
	 end
  end
  
endmodule

module hazard_unit (input clk,
                    input [4:0] rtD, rsE, rtE,
						  input memtoregE, memtoregM, branch,
                    output reg stallF, stallD, flushE);
						  
	always @(posedge clk)
	begin
	  if (memtoregE) begin
	    if ((rtD == rsE) || (rtD == rtE)) begin 
		   stallF <= #`mydelay 1'b1;
			stallD <= #`mydelay 1'b1;
		 end
	    else begin 
		   stallF <= #`mydelay 1'b0;
			stallD <= #`mydelay 1'b0;
		 end 
	  end
	  else if (branch) begin
		 flushE <= #`mydelay 1'b1;
	  end
	  else begin
	  	 stallF <= #`mydelay 1'b0;
	    stallD <= #`mydelay 1'b0;
		 flushE <= #`mydelay 1'b0;
	  end
	end

endmodule

module start_fetch(input clk,
                   input reset, stallF,
						 input [31:0] pcS,
						 output reg [31:0] pcF);
						  
						  
  always @(posedge clk)
  begin
    if(reset) pcF <= #`mydelay 32'b0;
	 else if (stallF) pcF <= #`mydelay pcF;
	 else pcF <= #`mydelay pcS;
  end
endmodule

module fetch_decode(input clk,
                    input reset,
                    input stallD, branch,
						  input [31:0] pcF,
                    input [31:0] instrF,
						  output reg [31:0] pcD,
                    output reg [31:0] instrD);
						  
						  
  always @(posedge clk)
  begin
/*
    if(reset || branch) begin
	   instrD <= 32'b0;
	   pcD <= 32'b0;
	 end
	 */
	 if (reset) begin
	   instrD <= #`mydelay 32'b0;
		pcD <= #`mydelay 32'b0;
	 end
	 else if (stallD) begin
	   instrD <= #`mydelay instrD;
		pcD <= #`mydelay pcD;
	 end
	 else begin
	   instrD <= #`mydelay instrF;
		pcD <= #`mydelay pcF;
    end
  end

endmodule

module decode_execute(input clk,
                    input reset, flushE,
                    input [31:0] pcD, output reg [31:0] pcE,
                    input [31:0] immD, output reg [31:0] immE,
						  input [4:0] rsD, output reg [4:0] rsE,
						  input [4:0] rtD, output reg [4:0] rtE,
						  input [4:0] rdD, output reg [4:0] rdE,
						  input [31:0] rd0D, output reg [31:0] rd0E,
						  input [31:0] rd1D, output reg [31:0] rd1E,
						  input regwriteD, output reg regwriteE,
						  input memtoregD, output reg memtoregE,
						  input memwriteD, output reg memwriteE,
						  input branchD, output reg branchE,
						  input [2:0] aluopD, output reg [2:0] aluopE,
						  input [1:0] alusrcBD, output reg [1:0] alusrcBE,
						  input [1:0] alusrcAD, output reg [1:0] alusrcAE,
						  input [1:0] regdstD, output reg [1:0] regdstE,
						  input zeroD, output reg zeroE);
  always @(posedge clk)
  begin
    if (reset || flushE) begin
	   pcE <= 32'b0;
	   immE <= 32'b0;
	   rsE <= 5'b0;
	   rtE <= 5'b0;
	   rdE <= 5'b0;
	   rd0E <= 32'b0;
	   rd1E <= 32'b0;
	   regwriteE <= 1'b0;
	   memtoregE <= 1'b0;
	   memwriteE <= 1'b0;
	   branchE <= 1'b0;
	   aluopE <= 2'b0;
	   alusrcBE <= 2'b0;
	   alusrcAE <= 2'b0;
	   regdstE <= 2'b0;
		zeroE <= 1'b0;
	 end
	 else begin
      pcE <= pcD;
	   immE <= immD;
	   rsE <= rsD;
	   rtE <= rtD;
	   rdE <= rdD;
	   rd0E <= rd0D;
	   rd1E <= rd1D;
	   regwriteE <= regwriteD;
	   memtoregE <= memtoregD;
	   memwriteE <= memwriteD;
	   branchE <= branchD;
	   aluopE <= aluopD;
	   alusrcBE <= alusrcBD;
	   alusrcAE <= alusrcAD;
	   regdstE <= regdstD;
		zeroE <= zeroD;
	 end
  end

endmodule

module execute_memory(input clk,
                    input reset,
                    input [4:0] writeregE, output reg [4:0] writeregM,
					     input [31:0] writedataE, output reg [31:0] writedataM,
	 					  input [31:0] aluoutE, output reg [31:0] aluoutM,
						  input memwriteE, output reg memwriteM,
						  input memtoregE, output reg memtoregM,
						  input regwriteE, output reg regwriteM);

  always @(posedge clk)
  begin
    if (reset) begin
	   writeregM <= 5'b0;
      writedataM <= 32'b0;
      aluoutM <= 32'b0;
      memwriteM <= 1'b0;
      memtoregM <= 1'b0;
      regwriteM <= 1'b0;
	 end
	 else begin
      writeregM <= writeregE;
      writedataM <= writedataE;
      aluoutM <= aluoutE;
      memwriteM <= memwriteE;
      memtoregM <= memtoregE;
      regwriteM <= regwriteE;
    end
  end
endmodule


module memory_wb(input clk,
               input reset,
               input regwriteM, output reg regwriteW,
					input memtoregM, output reg memtoregW,
					input [31:0] readdataM, output reg [31:0] readdataW,
					input [31:0] aluoutM, output reg [31:0] aluoutW,
					input [4:0] writeregM, output reg [4:0] writeregW);
  always @(posedge clk)
  begin
    if (reset) begin
      regwriteW <= 1'b0;
	   memtoregW <= 1'b0;
	   readdataW <= 32'b0;
	   aluoutW <= 32'b0;
	   writeregW <= 5'b0;
	 end
	 else begin
      regwriteW <= regwriteM;
	   memtoregW <= memtoregM;
	   readdataW <= readdataM;
	   aluoutW <= aluoutM;
	   writeregW <= writeregM;
	 end
  end
endmodule

module regfile(input         clk, 
               input         we, 
               input  [4:0]  ra1, ra2, wa, 
               input  [31:0] wd, 
               output [31:0] rd1, rd2);

  reg [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally
  // write third port on rising edge of clock
  // register 0 hardwired to 0

  always @(posedge clk)
    if (we) rf[wa] <= #`mydelay wd;	

  assign #`mydelay rd1 = (ra1 != 0) ? rf[ra1] : 0;
  assign #`mydelay rd2 = (ra2 != 0) ? rf[ra2] : 0;

endmodule


module alu(input [31:0] a, b,
           input [2:0] alucont,
			  output reg [31:0] result);

  wire [31:0] b2;
  wire sltu;
  wire [32:0] sum;
  
  assign b2 = alucont[2] ? ~b : b;
  assign sum[32:0] = a + b2 + alucont[2];
  assign sltu = ~sum[32];
  
  always@(*)
    case(alucont[1:0])
      2'b00: result <= a & b;
      2'b01: result <= a | b;
      2'b10: result <= sum;
      2'b11: result <= {31'b0, sltu};
	endcase
	
endmodule

module adder(input [31:0] a, b,
             output [31:0] y);

  assign #`mydelay y = a + b;
endmodule



module sl2(input  [31:0] a,
           output [31:0] y);

  // shift left by 2
  assign #`mydelay y = {a[29:0], 2'b00};
endmodule



module sign_zero_ext(input      [15:0] a,
                     input             signext,
                     output reg [31:0] y);
              
   always @(*)
	begin
	   if (signext)  y <= {{16{a[15]}}, a[15:0]};
	   else          y <= {16'b0, a[15:0]};
	end

endmodule

module sign_ext(input [15:0] a,
                output [31:0] y);

	assign #`mydelay y = {{16{a[15]}}, a[15:0]};

endmodule


module shift_left_16(input      [31:0] a,
		               input         shiftl16,
                     output reg [31:0] y);

   always @(*)
	begin
	   if (shiftl16) y = {a[15:0],16'b0};
	   else          y = a[31:0];
	end
              
endmodule



module flopr #(parameter WIDTH = 8)
              (input                  clk, reset,
               input      [WIDTH-1:0] d, 
               output reg [WIDTH-1:0] q);

  always @(posedge clk, posedge reset)
    if (reset) q <= #`mydelay 0;
    else       q <= #`mydelay d;

endmodule



module flopenr #(parameter WIDTH = 8)
                (input                  clk, reset,
                 input                  en,
                 input      [WIDTH-1:0] d, 
                 output reg [WIDTH-1:0] q);
 
  always @(posedge clk, posedge reset)
    if      (reset) q <= #`mydelay 0;
    else if (en)    q <= #`mydelay d;

endmodule

module mux2 #(parameter WIDTH = 8)
             (input  [WIDTH-1:0] d0, d1, 
              input              s, 
              output [WIDTH-1:0] y);

  assign #`mydelay y = s ? d1 : d0; 

endmodule

module mux3 #(parameter WIDTH = 8)
              (input [WIDTH-1:0] d0, d1, d2,
				   input [1:0] s,
					output reg [WIDTH-1:0] y);
  always @(*)
    case (s)
      2'b00: #`mydelay y <= d0;
      2'b01: #`mydelay y <= d1;
      2'b10: #`mydelay y <= d2;
    endcase  
endmodule

module mux4 #(parameter WIDTH = 8)
              (input [WIDTH-1:0] d0, d1, d2, d3,
				   input [1:0] s,
					output reg [WIDTH-1:0] y);

  always @(*)
    case (s)
      2'b00: #`mydelay y <= d0;
      2'b01: #`mydelay y <= d1;
      2'b10: #`mydelay y <= d2;
      2'b11: #`mydelay y <= d3;
    endcase

endmodule

