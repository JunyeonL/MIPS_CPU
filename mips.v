`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  /*
  wire        alusrc, regdst,;
  wire [2:0]  alucontrol;
  */
  wire shift16;
  wire [31:0] instrD;
  wire [31:0] pcplusF, pcplusD, pcplusE, pcbranch;
  wire [31:0] rd0, rd1, rd0D, rd1D, rd0E, rd1E;
  wire [31:0] immD, immE;
  wire regwriteD, regwriteE, regwriteM;
  wire memtoregD, memtoregE, memtoregM;
  wire branch, branchD, branchE;
  wire memwriteD, memwriteE, memwriteM;
  wire [31:0] regdataW;
  wire [2:0] aluopD, aluopE;
  wire [1:0] regdstD, regdstE;
  wire jump, jumpR;
  
  wire [31:0] aluoutE, aluoutM, aluoutW;
  wire [31:0] writedataE, writedataM;
  wire [31:0] bmuxo, jmuxo;
  wire [31:0] pcS;
  wire [4:0] writeregE, writeregM, writeregW;
  wire [1:0] alusrcAD, alusrcBD, alusrcAE, alusrcBE;
  
  wire [4:0] rsE, rtE, rdE;
  wire forwardAD, forwardBD;
  wire [1:0] forwardAE, forwardBE;
  wire [31:0] readdataM, readdataW;
  
  wire flushE, stallD, stallF;
  
  forward_unit fu(.clk(clk),
                  .rsD(instrD[25:21]), .rtD(instrD[20:16]), .rsE(rsE), .rtE(rtE),
					   .writeregM(writeregM), .regwriteM(regwriteM),
					   .writeregW(writeregW), .regwriteW(regwriteW),
					   .forwardAD(forwardAD), .forwardAE(forwardAE),
					   .forwardBD(forwardBD), .forwardBE(forwardBE),
					   .alusrcAE(alusrcAE));
  
  hazard_unit hu(.clk(clk), .rtD(instrD[20:16]), .rsE(rsE), .rtE(rtE),
                 .memtoregE(memtoregE), .memtoregM(memtoregM), .branch(branch),
                 .stallF(stallF), .stallD(stallD), .flushE(flushE));
					  
  mux2 #(32) Bmux(
      .d0       (pcplusF),
		.d1       (pcbranch),
		.s        (branch),
		.y        (bmuxo));

  mux2 #(32) Jmux(
      .d0       (bmuxo),
		.d1       ({pcplusD[31:28], instrD[25:0], 2'b00}),
		.s        (jump),
		.y        (jmuxo));
  
  mux2 #(32) JRmux(
      .d0       (jmuxo),
		.d1       (rd0),
		.s        (jumpR),
		.y        (pcS));

  /*****************************************************/
  /* 0000000000000000000000000000000000000000000000000 */
  /*****************************************************/
  start_fetch sf(.clk(clk),
                 .reset(reset), .stallF(stallF),
					  .pcS(pcS), .pcF(pc));
	
  adder pcplus(.a (pc), .b (32'b100), .y (pcplusF));

  
  /*****************************************************/
  /* 1111111111111111111111111111111111111111111111111 */
  /*****************************************************/
  fetch_decode fd(.clk(clk), .reset(reset),
                  //.branch(branch),
						.stallD(stallD),
                  .instrF(instr), .instrD(instrD),
						.pcF(pcplusF),	.pcD(pcplusD));

  controller c(.op			(instrD[31:26]),
               .funct		(instrD[5:0]),
					.jump			(jump),
					.jumpR		(jumpR),
					.shift16    (shift16),
					.regwrite	(regwriteD),
					.memtoreg	(memtoregD),
					.memwrite	(memwriteD),
					.branch		(branchD),
					.alucontrol	(aluopD),
					.alusrcB		(alusrcBD),
					.alusrcA		(alusrcAD),
					.regdst		(regdstD));

  regfile rf(
    .clk     (clk),
    .we      (regwriteW),
    .ra1     (instrD[25:21]),
    .ra2     (instrD[20:16]),
    .wa      (writeregW),
    .wd      (regdataW),
    .rd1     (rd0),
    .rd2     (rd1));

  wire [31:0] tmp_immD;
	 
  sign_ext signext(
    .a       (instrD[15:0]),
    .y       (tmp_immD));

  mux2 #(32) immdsel(
    .d0      (tmp_immD),
	 .d1      ({instrD[15:0], 16'b0}),
	 .s       (shift16),
	 .y       (immD));
	 
  wire [31:0] rdtmp;

  mux4 #(32) rdsrc(
    .d0       (rd0),
	 .d1       (32'b0),
	 .d2       (32'b0),
	 .d3       (32'b0),
	 .s        (alusrcAD),
	 .y        (rdtmp));
  
  mux2 #(32) rdsrc0(
    .d0       (rdtmp),
	 .d1       (regdataW),
	 .s        (forwardAD),
	 .y        (rd0D));

  mux2 #(32) rdsrc1(
    .d0       (rd1),
	 .d1       (regdataW),
	 .s        (forwardBD),
	 .y        (rd1D));
	 
  
  /*****************************************************/
  /* 2222222222222222222222222222222222222222222222222 */
  /*****************************************************/
  decode_execute de(.clk(clk), .flushE(flushE),
						  .reset(reset),
                    .pcD(pcplusD), .pcE(pcplusE),
                    .immD(immD), .immE(immE),
						  .rsD(instrD[25:21]), .rsE(rsE),
						  .rtD(instrD[20:16]), .rtE(rtE),
						  .rdD(instrD[15:11]), .rdE(rdE),
						  .rd0D(rd0D), .rd0E(rd0E),
						  .rd1D(rd1D), .rd1E(rd1E),
						  .regwriteD(regwriteD), .regwriteE(regwriteE),
						  .memtoregD(memtoregD), .memtoregE(memtoregE),
						  .memwriteD(memwriteD), .memwriteE(memwriteE),
						  .branchD(branchD), .branchE(branchE),
						  .aluopD(aluopD), .aluopE(aluopE),
						  .alusrcBD(alusrcBD), .alusrcBE(alusrcBE),
						  .alusrcAD(alusrcAD), .alusrcAE(alusrcAE),
						  .regdstD(regdstD), .regdstE(regdstE));


  wire [31:0] srca, srcb;
						
  mux3 #(32) rd0e(
    .d0   (rd0E),
	 .d1   (regdataW),
	 .d2   (aluoutM),
	 .s    (forwardAE),
	 .y    (srca));
						  
  mux3 #(32) rd1e(
    .d0   (rd1E),
	 .d1   (regdataW),
	 .d2   (aluoutM),
	 .s    (forwardBE),
	 .y    (writedataE));

  mux3 #(32) rd1ee(
    .d0   (writedataE),
	 .d1   (immE),
	 .d2   (pcplusE),
	 .s    (alusrcBE),
	 .y    (srcb));

  alu alu(
    .a       (srca),
    .b       (srcb),
    .alucont (aluopE),
    .result  (aluoutE));

  assign branch = branchE & aluoutE;

  mux4 #(5) wreg(
    .d0   (rtE),
	 .d1   (rdE),
	 .d2   (5'b11111),
	 .d3   (5'b0),
	 .s    (regdstE),
	 .y    (writeregE));
						  
  adder pcadd(.a (pcplusE), .b ({immE[29:0], 2'b00}), .y (pcbranch));
						 
  /*****************************************************/
  /* 3333333333333333333333333333333333333333333333333 */
  /*****************************************************/
  execute_memory em(.clk(clk),
                    .reset(reset),
                    .writeregE(writeregE), .writeregM(writeregM),
					     .writedataE(writedataE), .writedataM(writedataM),
	 					  .aluoutE(aluoutE), .aluoutM(aluoutM),
						  .memwriteE(memwriteE), .memwriteM(memwriteM),
						  .memtoregE(memtoregE), .memtoregM(memtoregM),
						  .regwriteE(regwriteE), .regwriteM(regwriteM));	

  assign memwrite = memwriteM;
  assign memaddr = aluoutM;
  assign memwritedata = writedataM;
  assign readdataM = memreaddata;

  
  /*****************************************************/
  /* 4444444444444444444444444444444444444444444444444 */
  /*****************************************************/
  memory_wb mw(.clk(clk),
               .reset(reset),
               .regwriteM(regwriteM), .regwriteW(regwriteW),
					.memtoregM(memtoregM), .memtoregW(memtoregW),
					.readdataM(readdataM), .readdataW(readdataW),
					.aluoutM(aluoutM), .aluoutW(aluoutW),
					.writeregM(writeregM), .writeregW(writeregW));

  mux2 #(32) lastmux(
    .d0  (aluoutW),
	 .d1  (readdataW),
	 .s   (memtoregW),
	 .y   (regdataW));
	 
endmodule

module controller(input  [5:0] op, funct,
						output [2:0] alucontrol,
						output [1:0] regdst,
						output [1:0] alusrcA, alusrcB,
						output       jump, jumpR, regwrite, memtoreg, memwrite, branch, shift16);

  wire [1:0] aluop;

  assign alusrcA = 2'b0;
  
  maindec md(
    .op       (op),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .alusrc   (alusrcB),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
    .aluop    (aluop),
	 .shift16  (shift16));
	 
  aludec ad( 
    .funct      (funct),
    .aluop      (aluop), 
	 .jumpR      (jumpR),
    .alucontrol (alucontrol));
 
endmodule

module maindec(input  [5:0] op,
               output       memtoreg, memwrite,
               output       branch,
					output [1:0] alusrc,
               output [1:0] regdst,
					output       regwrite, jump, shift16,
               output [1:0] aluop);

  reg [11:0] controls;

  assign {shift16, regwrite, regdst, alusrc, branch,
			 memwrite, memtoreg, jump, aluop} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 12'b010100000011; // Rtype
      6'b100011: controls <= #`mydelay 12'b010001001000; // LW
      6'b101011: controls <= #`mydelay 12'b000001010000; // SW
      6'b000100: controls <= #`mydelay 12'b000000100001; // BEQ
		6'b000101: controls <= #`mydelay 12'b000000100001; // BNEZ : JUNYEON
      6'b001000, 
      6'b001001: controls <= #`mydelay 12'b010001000000; // ADDI, ADDIU: only difference is exception
		6'b001010: controls <= #`mydelay 12'b010101000000; // SLTI : JUNYEON
      6'b001101: controls <= #`mydelay 12'b010001000010; // ORI
      6'b001111: controls <= #`mydelay 12'b110001000000; // LUI
      6'b000010: controls <= #`mydelay 12'b000000000100; // J
		6'b000011: controls <= #`mydelay 12'b011010000100; // JAL
      default:   controls <= #`mydelay 12'bxxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
				  output     jumpR,
              output     [2:0] alucontrol);
				  
  reg [3:0] controls;
  
  assign {jumpR, alucontrol} = controls;
  
  always @(*)
    case(aluop)
      2'b00: controls <= #`mydelay 4'b0010;  // add
      2'b01: controls <= #`mydelay 4'b0110;  // sub
      2'b10: controls <= #`mydelay 4'b0001;  // or
      default: case(funct)          // RTYPE
			 6'b100000,
          6'b100001: controls <= #`mydelay 4'b0010; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: controls <= #`mydelay 4'b0110; // SUB, SUBU: only difference is exception
          6'b100100: controls <= #`mydelay 4'b0000; // AND
          6'b100101: controls <= #`mydelay 4'b0001; // OR
			 6'b001000: controls <= #`mydelay 4'b1000; // JR
          6'b101010,
			 6'b101011: controls <= #`mydelay 4'b0111; // SLT, SLTU
          default:   controls <= #`mydelay 4'b0xxx; // ???
        endcase
    endcase
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg,
                input         alusrc, regdst,
                input         regwrite, jump, jumpR, linkra, condz, branch,
                input  [2:0]  alucontrol,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata);

  wire [4:0]  writereg;
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch, pcreturn;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result;
  wire        shift;
  wire zero;
  reg pcsrc;
  
  always @(*)
  begin
    if(~condz) pcsrc <= branch & zero;
	 else pcsrc <= branch & ~zero;
  end

  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (signimm),
    .y (signimmsh));
				 
  adder pcadd2(
    .a (pcplus4),
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) Br_mux(
    .d0  (pcplus4),
    .d1  (pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));
	 
  mux2 #(32) Jump_mux(
    .d0  (pcnextbr),
    .d1  ({pcplus4[31:28], instr[25:0], 2'b00}),
    .s   (jump),
    .y   (pcreturn));

  mux2 #(32) JR_mux(
    .d0  (pcreturn),
	 .d1  (srca),
	 .s   (jumpR),
	 .y   (pcnext));

  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    .ra1     (instr[25:21]),
    .ra2     (instr[20:16]),
    .wa      (writereg),
    .wd      (result),
    .rd1     (srca),
    .rd2     (writedata));

  mux3 #(5) wrmux(
    .d0  (instr[20:16]),
    .d1  (instr[15:11]),
	 .d2  (5'b11111),   /* ra register */
	 .s1  (regdst),
	 .s2  (linkra),
	 .y   (writereg));

  mux3 #(32) resmux(
    .d0 (aluout),
	 .d1 (readdata),
	 .d2 (pcplus4),
	 .s1 (memtoreg),
	 .s2 (linkra),
	 .y  (result));
	 
  sign_zero_ext sze(
    .a       (instr[15:0]),
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (shiftedimm[31:0]));
	 
  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (writedata),
    .d1 (shiftedimm[31:0]),
    .s  (alusrc),
    .y  (srcb));

  alu alu(
    .a       (srca),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout),
    .zero    (zero));

endmodule
