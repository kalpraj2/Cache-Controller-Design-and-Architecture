


module cachecModule(input clk, input reset, input [31:0] pa, input writeORread, input [7:0] inByte, input [255:0] dataBlock, output [7:0] cacheOutput,output hom);

	wire hit,replace,finalWrite, finalRead, bufferWrite,dataSel;
	wire [1:0] way,way0,way1;
	wire [3:0] validBits;
	
	wire [255:0] writeBlock,bufferBlock;
	
	CtrlCkt controlSignal(clk, reset, writeORread, hit, replace, finalWrite, finalRead, bufferWrite, dataSel);
	fourWaySACache tagComp(clk, reset, way, replace | finalWrite, pa[31:8], pa[7:5], hit, way0, validBits);
	mux2to1_256b mx256btm0(dataBlock,{32{inByte}}, dataSel,writeBlock);
	dataCache dataOp(clk, reset,way,replace | finalWrite, finalRead,dataSel,writeBlock,pa[7:5],pa[4:0],cacheOutput, bufferBlock);
	buffer buffCac0(clk, reset,bufferWrite,pa[31:5],bufferBlock);
	FIFO fifoReplacement0(clk,reset,replace,pa[7:5], validBits, way1);
	mux2to1_2b mx2btm0(way1, way0, finalRead, way);
	assign hom =hit;
endmodule


module comparator(input [23:0] in1, input [23:0] in2,output reg compOut);
  always@(in1,in2)
  if(in1==in2) begin compOut=1'b1;end
  else begin compOut=1'b0;end
endmodule


module CtrlCk( input clk, input reset, input write, input hit, output reg replace,output reg finalWrite,output reg finalRead,output reg bufferWrite,output reg sel);
	reg [3:0] state,next_state;
	
	always @ (negedge clk or posedge reset or negedge reset ) //(negedge clk or reset)
	begin
		if(reset==0) state=next_state;
		else begin state=0; next_state=0; end
	end
	
	always @ (state or reset)
	begin	
	  if(reset==0)
		case(state)
			0:begin 
					replace=0;
					finalWrite=0;bufferWrite=0;
					sel=0;
					finalRead=0;
					if(hit && ~write) next_state=1;                     //read hit
					else if(hit && write) next_state=3;                 //write hit
					else next_state=2;
				end
			1:begin 
					replace=0;
					finalWrite=0;
					if(write)  
					   bufferWrite=1;
					else       
					   bufferWrite=0;       
					sel=0;
					finalRead=1;
					next_state=0;
				end
			2:begin 
					replace=1;
					finalWrite=write;bufferWrite=0;
					sel=0;
					finalRead=0;
					if(write)  next_state=3;           //write miss
					else       next_state=1;           //read miss
				end
			3:begin
			    replace=0;
			    finalWrite=1;bufferWrite=0;	
			    sel=1;
			    finalRead=1;
			    next_state=1;
			  end
		endcase			
	end	
endmodule



module D_FF(input clk, input reset, input write, input d, output reg q);
  always @(negedge clk)
  if(reset) q=0;
  else
    if(write) q=d;
endmodule



module dataAr(input clk, input reset, input [31:0] we0, input [31:0] we1, input [31:0] we2, input [31:0] we3, input [31:0] we4, input [31:0] we5, input [31:0] we6, input [31:0] we7, input [255:0] block, output [255:0] blockOut0, output [255:0] blockOut1, output [255:0] blockOut2, output [255:0]  blockOut3, output [255:0]  blockOut4, output [255:0]  blockOut5, output [255:0]  blockOut6, output [255:0]  blockOut7);
  dataBlock block0(clk,reset,we0,block,blockOut0);
  dataBlock block1(clk,reset,we1,block,blockOut1);
  dataBlock block2(clk,reset,we2,block,blockOut2);
  dataBlock block3(clk,reset,we3,block,blockOut3);
  dataBlock block4(clk,reset,we4,block,blockOut4);
  dataBlock block5(clk,reset,we5,block,blockOut5);
  dataBlock block6(clk,reset,we6,block,blockOut6);
  dataBlock block7(clk,reset,we7,block,blockOut7);
endmodule


module dataBl(input clk, input reset, input [31:0] write, input [255:0] block,output [255:0] blockData);
  dataByte data00(clk,reset,write[0],block[7:0],blockData[7:0]);
  dataByte data01(clk,reset,write[1],block[15:8],blockData[15:8]);
  dataByte data02(clk,reset,write[2],block[23:16],blockData[23:16]);
  dataByte data03(clk,reset,write[3],block[31:24],blockData[31:24]);
  dataByte data04(clk,reset,write[4],block[39:32],blockData[39:32]);
  dataByte data05(clk,reset,write[5],block[47:40],blockData[47:40]);
  dataByte data06(clk,reset,write[6],block[55:48],blockData[55:48]);
  dataByte data07(clk,reset,write[7],block[63:56],blockData[63:56]);
  dataByte data08(clk,reset,write[8],block[71:64],blockData[71:64]);
  dataByte data09(clk,reset,write[9],block[79:72],blockData[79:72]);
  dataByte data10(clk,reset,write[10],block[87:80],blockData[87:80]);
  dataByte data11(clk,reset,write[11],block[95:88],blockData[95:88]);
  dataByte data12(clk,reset,write[12],block[103:96],blockData[103:96]);
  dataByte data13(clk,reset,write[13],block[111:104],blockData[111:104]);
  dataByte data14(clk,reset,write[14],block[119:112],blockData[119:112]);
  dataByte data15(clk,reset,write[15],block[127:120],blockData[127:120]);
  dataByte data16(clk,reset,write[16],block[135:128],blockData[135:128]);
  dataByte data17(clk,reset,write[17],block[143:136],blockData[143:136]);
  dataByte data18(clk,reset,write[18],block[151:144],blockData[151:144]);
  dataByte data19(clk,reset,write[19],block[159:152],blockData[159:152]);
  dataByte data20(clk,reset,write[20],block[167:160],blockData[167:160]);
  dataByte data21(clk,reset,write[21],block[175:168],blockData[175:168]);
  dataByte data22(clk,reset,write[22],block[183:176],blockData[183:176]);
  dataByte data23(clk,reset,write[23],block[191:184],blockData[191:184]);
  dataByte data24(clk,reset,write[24],block[199:192],blockData[199:192]);
  dataByte data25(clk,reset,write[25],block[207:200],blockData[207:200]);
  dataByte data26(clk,reset,write[26],block[215:208],blockData[215:208]);
  dataByte data27(clk,reset,write[27],block[223:216],blockData[223:216]);
  dataByte data28(clk,reset,write[28],block[231:224],blockData[231:224]);
  dataByte data29(clk,reset,write[29],block[239:232],blockData[239:232]);
  dataByte data30(clk,reset,write[30],block[247:240],blockData[247:240]);
  dataByte data31(clk,reset,write[31],block[255:248],blockData[255:248]);
endmodule


module dataB(input clk, input reset, input write, input [7:0] data,output [7:0] byteData);
  D_FF d00(clk, reset,write,data[0],byteData[0]);
  D_FF d01(clk, reset,write,data[1],byteData[1]);
  D_FF d02(clk, reset,write,data[2],byteData[2]);
  D_FF d03(clk, reset,write,data[3],byteData[3]);
  D_FF d04(clk, reset,write,data[4],byteData[4]);
  D_FF d05(clk, reset,write,data[5],byteData[5]);
  D_FF d06(clk, reset,write,data[6],byteData[6]);
  D_FF d07(clk, reset,write,data[7],byteData[7]);
endmodule


module dataCache(input clk, input reset, input [1:0] way, input write, input dataRead, input dataSel, input [255:0] block, input [2:0] index,input [4:0] offset,output [7:0] bytedata, output [255:0] selBlock);
  wire [3:0] wyOt;
  wire [7:0] dcOt,wEn0,wEn1,wEn2,wEn3,muxByte;
  wire [31:0] muxWr,decOut;
  wire [255:0] muxblockOut0,muxblockOut1,muxblockOut2,muxblockOut3,
  blockOut00,blockOut01,blockOut02,blockOut03,blockOut04,blockOut05,blockOut06,blockOut07,
  blockOut10,blockOut11,blockOut12,blockOut13,blockOut14,blockOut15,blockOut16,blockOut17,
  blockOut20,blockOut21,blockOut22,blockOut23,blockOut24,blockOut25,blockOut26,blockOut27,
  blockOut30,blockOut31,blockOut32,blockOut33,blockOut34,blockOut35,blockOut36,blockOut37;
  and nd00(wEn0[0],dcOt[0],wyOt[0]&write);and nd01(wEn0[1],dcOt[1],wyOt[0]&write);and nd02(wEn0[2],dcOt[2],wyOt[0]&write);and nd03(wEn0[3],dcOt[3],wyOt[0]&write);
  and nd04(wEn0[4],dcOt[4],wyOt[0]&write);and nd05(wEn0[5],dcOt[5],wyOt[0]&write);and nd06(wEn0[6],dcOt[6],wyOt[0]&write);and nd07(wEn0[7],dcOt[7],wyOt[0]&write);
  and nd10(wEn1[0],dcOt[0],wyOt[1]&write);and nd11(wEn1[1],dcOt[1],wyOt[1]&write);and nd12(wEn1[2],dcOt[2],wyOt[1]&write);and nd13(wEn1[3],dcOt[3],wyOt[1]&write);
  and nd14(wEn1[4],dcOt[4],wyOt[1]&write);and nd15(wEn1[5],dcOt[5],wyOt[1]&write);and nd16(wEn1[6],dcOt[6],wyOt[1]&write);and nd17(wEn1[7],dcOt[7],wyOt[1]&write);
  and nd20(wEn2[0],dcOt[0],wyOt[2]&write);and nd21(wEn2[1],dcOt[1],wyOt[2]&write);and nd22(wEn2[2],dcOt[2],wyOt[2]&write);and nd23(wEn2[3],dcOt[3],wyOt[2]&write);
  and nd24(wEn2[4],dcOt[4],wyOt[2]&write);and nd25(wEn2[5],dcOt[5],wyOt[2]&write);and nd26(wEn2[6],dcOt[6],wyOt[2]&write);and nd27(wEn2[7],dcOt[7],wyOt[2]&write);
  and nd30(wEn3[0],dcOt[0],wyOt[3]&write);and nd31(wEn3[1],dcOt[1],wyOt[3]&write);and nd32(wEn3[2],dcOt[2],wyOt[3]&write);and nd33(wEn3[3],dcOt[3],wyOt[3]&write);
  and nd34(wEn3[4],dcOt[4],wyOt[3]&write);and nd35(wEn3[5],dcOt[5],wyOt[3]&write);and nd36(wEn3[6],dcOt[6],wyOt[3]&write);and nd37(wEn3[7],dcOt[7],wyOt[3]&write);
  decoder5to32 offSetDc(offset,decOut);
  mux2to1_32b mx32b0({32{1'b1}},decOut, dataSel, muxWr);
  dataArray dtWy0(clk,reset,{32{wEn0[0]}} & muxWr,{32{wEn0[1]}} & muxWr,{32{wEn0[2]}} & muxWr,{32{wEn0[3]}} & muxWr,{32{wEn0[4]}} & muxWr,{32{wEn0[5]}} & muxWr,{32{wEn0[6]}} & muxWr,{32{wEn0[7]}} & muxWr,block,blockOut00,blockOut01,blockOut02,blockOut03,blockOut04,blockOut05,blockOut06,blockOut07);
  dataArray dtWy1(clk,reset,{32{wEn1[0]}} & muxWr,{32{wEn1[1]}} & muxWr,{32{wEn1[2]}} & muxWr,{32{wEn1[3]}} & muxWr,{32{wEn1[4]}} & muxWr,{32{wEn1[5]}} & muxWr,{32{wEn1[6]}} & muxWr,{32{wEn1[7]}} & muxWr,block,blockOut10,blockOut11,blockOut12,blockOut13,blockOut14,blockOut15,blockOut16,blockOut17);
  dataArray dtWy2(clk,reset,{32{wEn2[0]}} & muxWr,{32{wEn2[1]}} & muxWr,{32{wEn2[2]}} & muxWr,{32{wEn2[3]}} & muxWr,{32{wEn2[4]}} & muxWr,{32{wEn2[5]}} & muxWr,{32{wEn2[6]}} & muxWr,{32{wEn2[7]}} & muxWr,block,blockOut20,blockOut21,blockOut22,blockOut23,blockOut24,blockOut25,blockOut26,blockOut27);
  dataArray dtWy3(clk,reset,{32{wEn3[0]}} & muxWr,{32{wEn3[1]}} & muxWr,{32{wEn3[2]}} & muxWr,{32{wEn3[3]}} & muxWr,{32{wEn3[4]}} & muxWr,{32{wEn3[5]}} & muxWr,{32{wEn3[6]}} & muxWr,{32{wEn3[7]}} & muxWr,block,blockOut30,blockOut31,blockOut32,blockOut33,blockOut34,blockOut35,blockOut36,blockOut37);
  mux8to1_256b mx256b0(blockOut00,blockOut01,blockOut02,blockOut03,blockOut04,blockOut05,blockOut06,blockOut07,index,muxblockOut0);
  mux8to1_256b mx256b1(blockOut10,blockOut11,blockOut12,blockOut13,blockOut14,blockOut15,blockOut16,blockOut17,index,muxblockOut1);
  mux8to1_256b mx256b2(blockOut20,blockOut21,blockOut22,blockOut23,blockOut24,blockOut25,blockOut26,blockOut27,index,muxblockOut2);
  mux8to1_256b mx256b3(blockOut30,blockOut31,blockOut32,blockOut33,blockOut34,blockOut35,blockOut36,blockOut37,index,muxblockOut3);
  mux4to1_256b mx4to1b256(muxblockOut0,muxblockOut1,muxblockOut2,muxblockOut3,way,selBlock);
  mux32to1_8b mx32b00(selBlock[7:0],selBlock[15:8],selBlock[23:16],selBlock[31:24],selBlock[39:32],selBlock[47:40],selBlock[55:48],selBlock[63:56],selBlock[71:64],selBlock[79:72],selBlock[87:80],selBlock[95:88],selBlock[103:96],selBlock[111:104],selBlock[119:112],selBlock[127:120],selBlock[135:128],selBlock[143:136],selBlock[151:144],selBlock[159:152],selBlock[167:160],selBlock[175:168],selBlock[183:176],selBlock[191:184],selBlock[199:192],selBlock[207:200],selBlock[215:208],selBlock[223:216],selBlock[231:224],selBlock[239:232],selBlock[247:240],selBlock[255:248],offset,muxByte);
  mux2to1_8b mx8b(8'd0,muxByte,dataRead,bytedata);
endmodule



module decoder2to4(input [1:0] in,output reg [3:0] wayOut);
  always@(in)
  case(in)
    2'b00: wayOut=4'b0001;
    2'b01: wayOut=4'b0010;
    2'b10: wayOut=4'b0100;
    2'b11: wayOut=4'b1000;
  endcase
endmodule


module decoder3to8(input [2:0] in,output reg [7:0] decOut);
  always@(in)
  case(in)
    3'b000: decOut=8'b00000001;
    3'b001: decOut=8'b00000010;
    3'b010: decOut=8'b00000100;
    3'b011: decOut=8'b00001000;
    3'b100: decOut=8'b00010000;
    3'b101: decOut=8'b00100000;
    3'b110: decOut=8'b01000000;
    3'b111: decOut=8'b10000000;
  endcase
endmodule



module decoder5to32(input [4:0] in,output reg [31:0] decoder_out);
  always @ (in)
    case (in)
      5'h00 : decoder_out = 32'h00000001;
      5'h01 : decoder_out = 32'h00000002;
      5'h02 : decoder_out = 32'h00000004;
      5'h03 : decoder_out = 32'h00000008;
      5'h04 : decoder_out = 32'h00000010;
      5'h05 : decoder_out = 32'h00000020;
      5'h06 : decoder_out = 32'h00000040;
      5'h07 : decoder_out = 32'h00000080;
      5'h08 : decoder_out = 32'h00000100;
      5'h09 : decoder_out = 32'h00000200;
      5'h0A : decoder_out = 32'h00000400;
      5'h0B : decoder_out = 32'h00000800;
      5'h0C : decoder_out = 32'h00001000;
      5'h0D : decoder_out = 32'h00002000;
      5'h0E : decoder_out = 32'h00004000;
      5'h0F : decoder_out = 32'h00008000;
      5'h10 : decoder_out = 32'h00010000;
      5'h11 : decoder_out = 32'h00020000;
      5'h12 : decoder_out = 32'h00040000;
      5'h13 : decoder_out = 32'h00080000;
      5'h14 : decoder_out = 32'h00100000;
      5'h15 : decoder_out = 32'h00200000;
      5'h16 : decoder_out = 32'h00400000;
      5'h17 : decoder_out = 32'h00800000;
      5'h18 : decoder_out = 32'h01000000;
      5'h19 : decoder_out = 32'h02000000;
      5'h1A : decoder_out = 32'h04000000;
      5'h1B : decoder_out = 32'h08000000;
      5'h1C : decoder_out = 32'h10000000;
      5'h1D : decoder_out = 32'h20000000;
      5'h1E : decoder_out = 32'h40000000;
      5'h1F : decoder_out = 32'h80000000;
    endcase
endmodule



module encoder4to2(input [3:0] in,output reg [1:0] out);
  always @ (in)
  begin
    if(in[0]) out=2'b00;
    else if(in[1]) out=2'b01;
    else if(in[2]) out=2'b10;
    else out=2'b11;
  end
endmodule



module FIFO(input clk,input reset,input replace,input [2:0] index, input  [3:0] validBits, output [1:0] finalWay);

wire [7:0] decOut;
decoder3to8 ind(index,decOut); 

wire [1:0] priority0,priority1,priority2,priority3,
priot00,priot01,priot02,priot03,priot04,priot05,priot06,priot07,
priot10,priot11,priot12,priot13,priot14,priot15,priot16,priot17,
priot20,priot21,priot22,priot23,priot24,priot25,priot26,priot27,
priot30,priot31,priot32,priot33,priot34,priot35,priot36,priot37;

wire [7:0] we;
assign we = decOut & {8{replace}};
priorityArray arr0(clk,reset,we,priority0,priot00,priot01,priot02,priot03,priot04,priot05,priot06,priot07);
priorityArray arr1(clk,reset,we,priority1,priot10,priot11,priot12,priot13,priot14,priot15,priot16,priot17);
priorityArray arr2(clk,reset,we,priority2,priot20,priot21,priot22,priot23,priot24,priot25,priot26,priot27);
priorityArray arr3(clk,reset,we,priority3,priot30,priot31,priot32,priot33,priot34,priot35,priot36,priot37);

wire [1:0] muxOut0,muxOut1,muxOut2,muxOut3;
mux8to1_2b m0(priot00,priot01,priot02,priot03,priot04,priot05,priot06,priot07,index,muxOut0);
mux8to1_2b m1(priot10,priot11,priot12,priot13,priot14,priot15,priot16,priot17,index,muxOut1);
mux8to1_2b m2(priot20,priot21,priot22,priot23,priot24,priot25,priot26,priot27,index,muxOut2);
mux8to1_2b m3(priot30,priot31,priot32,priot33,priot34,priot35,priot36,priot37,index,muxOut3);
wire validReplace;    // it is 0 when replacement is based on atleast 1 invalid cache block
wire [1:0] validWayOut,priorityWayOut;
validityEncoder decision(validBits,validReplace,validWayOut); 
priorityEncoder_4x2 encode({muxOut3[1]|muxOut3[0],muxOut2[1]|muxOut2[0],muxOut1[1]|muxOut1[0],muxOut0[1]|muxOut0[0]},priorityWayOut);

mux2to1_2b mx2b00(validWayOut,priorityWayOut,validReplace,finalWay);
updatePriority update(finalWay,muxOut0,muxOut1,muxOut2,muxOut3,priority0,priority1,priority2,priority3);
endmodule



module fourWaySACache(input clk, input reset, input [1:0] way, input replace, input [23:0] tag, input [2:0] index, output iHit, output [1:0] wayOut, output [3:0] validComp);
  wire hit,set0,set1,cmpOut0,cmpOut1,cmpOut2,cmpOut3,
  valot00,valot01,valot02,valot03,valot04,valot05,valot06,valot07,
  valot10,valot11,valot12,valot13,valot14,valot15,valot16,valot17,
  valot20,valot21,valot22,valot23,valot24,valot25,valot26,valot27,
  valot30,valot31,valot32,valot33,valot34,valot35,valot36,valot37;
  
  wire [3:0] wyOt;
  wire [7:0] dcOt, wrEn0,wrEn1,wrEn2,wrEn3;
  
  wire [23:0] tagComp0,tagComp1,tagComp2,tagComp3,
  tagOut00,tagOut01,tagOut02,tagOut03,tagOut04,tagOut05,tagOut06,tagOut07,
  tagOut10,tagOut11,tagOut12,tagOut13,tagOut14,tagOut15,tagOut16,tagOut17,
  tagOut20,tagOut21,tagOut22,tagOut23,tagOut24,tagOut25,tagOut26,tagOut27,
  tagOut30,tagOut31,tagOut32,tagOut33,tagOut34,tagOut35,tagOut36,tagOut37;
  
  decoder3to8 ind(index, dcOt);
  decoder2to4 wydc(way, wyOt);
  and an00(wrEn0[0],dcOt[0],wyOt[0]&replace);and an01(wrEn0[1],dcOt[1],wyOt[0]&replace);and an02(wrEn0[2],dcOt[2],wyOt[0]&replace);and an03(wrEn0[3],dcOt[3],wyOt[0]&replace);
  and an04(wrEn0[4],dcOt[4],wyOt[0]&replace);and an05(wrEn0[5],dcOt[5],wyOt[0]&replace);and an06(wrEn0[6],dcOt[6],wyOt[0]&replace);and an07(wrEn0[7],dcOt[7],wyOt[0]&replace);
  and an10(wrEn1[0],dcOt[0],wyOt[1]&replace);and an11(wrEn1[1],dcOt[1],wyOt[1]&replace);and an12(wrEn1[2],dcOt[2],wyOt[1]&replace);and an13(wrEn1[3],dcOt[3],wyOt[1]&replace);
  and an14(wrEn1[4],dcOt[4],wyOt[1]&replace);and an15(wrEn1[5],dcOt[5],wyOt[1]&replace);and an16(wrEn1[6],dcOt[6],wyOt[1]&replace);and an17(wrEn1[7],dcOt[7],wyOt[1]&replace);
  and an20(wrEn2[0],dcOt[0],wyOt[2]&replace);and an21(wrEn2[1],dcOt[1],wyOt[2]&replace);and an22(wrEn2[2],dcOt[2],wyOt[2]&replace);and an23(wrEn2[3],dcOt[3],wyOt[2]&replace);
  and an24(wrEn2[4],dcOt[4],wyOt[2]&replace);and an25(wrEn2[5],dcOt[5],wyOt[2]&replace);and an26(wrEn2[6],dcOt[6],wyOt[2]&replace);and an27(wrEn2[7],dcOt[7],wyOt[2]&replace);
  and an30(wrEn3[0],dcOt[0],wyOt[3]&replace);and an31(wrEn3[1],dcOt[1],wyOt[3]&replace);and an32(wrEn3[2],dcOt[2],wyOt[3]&replace);and an33(wrEn3[3],dcOt[3],wyOt[3]&replace);
  and an34(wrEn3[4],dcOt[4],wyOt[3]&replace);and an35(wrEn3[5],dcOt[5],wyOt[3]&replace);and an36(wrEn3[6],dcOt[6],wyOt[3]&replace);and an37(wrEn3[7],dcOt[7],wyOt[3]&replace);tagOut25,tagOut26,tagOut27);
  mux8to1_24b mx24b0(tagOut00,tagOut01,tagOut02,tagOut03,tagOut04,tagOut05,tagOut06,tagOut07,index,tagComp0);
  mux8to1_24b mx24b1(tagOut10,tagOut11,tagOut12,tagOut13,tagOut14,tagOut15,tagOut16,tagOut17,index,tagComp1);
  mux8to1_24b mx24b2(tagOut20,tagOut21,tagOut22,tagOut23,tagOut24,tagOut25,tagOut26,tagOut27,index,tagComp2);
  mux8to1_24b mx24b3(tagOut30,tagOut31,tagOut32,tagOut33,tagOut34,tagOut35,tagOut36,tagOut37,index,tagComp3);
  mux8to1_1b mx1b0(valot00,valot01,valot02,valot03,valot04,valot05,valot06,valot07,index,validComp[0]);
  mux8to1_1b mx1b1(valot10,valot11,valot12,valot13,valot14,valot15,valot16,valot17,index,validComp[1]);
  mux8to1_1b mx1b2(valot20,valot21,valot22,valot23,valot24,valot25,valot26,valot27,index,validComp[2]);
  mux8to1_1b mx1b3(valot30,valot31,valot32,valot33,valot34,valot35,valot36,valot37,index,validComp[3]);
  comparator cmp0(tag,tagComp0,cmpOut0);
  comparator cmp1(tag,tagComp1,cmpOut1);
  comparator cmp2(tag,tagComp2,cmpOut2);
  comparator cmp3(tag,tagComp3,cmpOut3);
  encoder4to2 encodeWay({{cmpOut3 & validComp[3]},{cmpOut2 & validComp[2]},{cmpOut1 & validComp[1]},{cmpOut0 & validComp[0]}}, wayOut);
  or in0(set0,cmpOut0 & validComp[0],cmpOut1 & validComp[1]);
  or in1(set1,cmpOut2 & validComp[2],cmpOut3 & validComp[3]);
  or fin(iHit, set0, set1);
  missRegister hitReg(reset,tag,index, iHit, hit);
endmodule


module missRegister(input reset,input [23:0] in1, input [2:0] in2, input miss, output reg out);
	always@(in1,in2,reset)
		begin
		  if(reset)
		    out=1'b0;
		  else
		    out=miss;
		end
endmodule



module mux2to1_2b(input [1:0] in1,input [1:0] in2, input sel,output reg [1:0] muxOut);
  always@(in1,in2,sel)
  case(sel)
    1'b0: muxOut=in1;
    1'b1: muxOut=in2;
  endcase
endmodule




module mux2to1_8b(input [7:0] zero, input [7:0] inpt, input dataRead, output reg [7:0] dataOut);	
	always@(inpt or zero or dataRead)
		case (dataRead)
			1'b0: dataOut=zero;
			1'b1: dataOut=inpt;
		endcase	
endmodule




module mux2to1_32b(input [31:0] in1, input [31:0] in2, input sel, output reg [31:0] muxOut);  
  always@(in1 or in2 or sel)
    case (sel)
      1'b0: muxOut=in1;
      1'b1: muxOut=in2;
    endcase 
endmodule



module mux2to1_256b(input [255:0] in1,input [255:0] in2, input sel,output reg [255:0] muxOut);
  always@(in1,in2,sel)
  case(sel)
    1'b0: muxOut=in1;
    1'b1: muxOut=in2;
  endcase
endmodule



module mux4to1_256b(input [255:0] in1,input [255:0] in2, input [255:0] in3, input [255:0] in4, input[1:0] sel,output reg[255:0] muxOut);
  always@(in1,in2,in3,in4,sel)
  case(sel)
    2'b00: muxOut=in1;
    2'b01: muxOut=in2;
    2'b10: muxOut=in3;
    2'b11: muxOut=in4;
  endcase
endmodule




module mux8to1_1b(input in1, input in2, input in3, input in4, input in5, input in6, input in7, input in8,input[2:0] sel,output reg valot);
  always@(in1,in2,in3,in4,in5,in6,in7,in8,sel)
  case(sel)
    3'b000: valot=in1;
    3'b001: valot=in2;
    3'b010: valot=in3;
    3'b011: valot=in4;
    3'b100: valot=in5;
    3'b101: valot=in6;
    3'b110: valot=in7;
    3'b111: valot=in8;
  endcase
endmodule



module mux8to1_2b(input [1:0]in1, input [1:0]in2, input [1:0]in3, input [1:0]in4, input [1:0]in5, input [1:0]in6, input [1:0]in7, input [1:0]in8,input[2:0] sel,output reg [1:0]priot);
  always@(in1,in2,in3,in4,in5,in6,in7,in8,sel)
  case(sel)
    3'b000: priot=in1;
    3'b001: priot=in2;
    3'b010: priot=in3;
    3'b011: priot=in4;
    3'b100: priot=in5;
    3'b101: priot=in6;
    3'b110: priot=in7;
    3'b111: priot=in8;
  endcase
endmodule




module mux8to1_24b(input [23:0] in1,input [23:0] in2, input [23:0] in3, input [23:0] in4, input [23:0] in5,input [23:0] in6, input [23:0] in7, input [23:0] in8, input[2:0] sel,output reg[23:0] tagOut);
  always@(in1,in2,in3,in4,in5,in6,in7,in8,sel)
  case(sel)
    3'b000: tagOut=in1;
    3'b001: tagOut=in2;
    3'b010: tagOut=in3;
    3'b011: tagOut=in4;
    3'b100: tagOut=in5;
    3'b101: tagOut=in6;
    3'b110: tagOut=in7;
    3'b111: tagOut=in8;
  endcase
endmodule



module mux8to1_256b(input [255:0] in1,input [255:0] in2, input [255:0] in3, input [255:0] in4, input [255:0] in5,input [255:0] in6, input [255:0] in7, input [255:0] in8, input[2:0] sel,output reg[255:0] blockOut);
  always@(in1,in2,in3,in4,in5,in6,in7,in8,sel)
  case(sel)
    3'b000: blockOut=in1;
    3'b001: blockOut=in2;
    3'b010: blockOut=in3;
    3'b011: blockOut=in4;
    3'b100: blockOut=in5;
    3'b101: blockOut=in6;
    3'b110: blockOut=in7;
    3'b111: blockOut=in8;
  endcase
endmodule



module mux32to1_8b(input [7:0] in01,input [7:0] in02, input [7:0] in03, input [7:0] in04, input [7:0] in05,input [7:0] in06, input [7:0] in07, input [7:0] in08,
  input [7:0] in09,input [7:0] in10, input [7:0] in11, input [7:0] in12, input [7:0] in13,input [7:0] in14, input [7:0] in15, input [7:0] in16,
  input [7:0] in17,input [7:0] in18, input [7:0] in19, input [7:0] in20, input [7:0] in21,input [7:0] in22, input [7:0] in23, input [7:0] in24,
  input [7:0] in25,input [7:0] in26, input [7:0] in27, input [7:0] in28, input [7:0] in29,input [7:0] in30, input [7:0] in31, input [7:0] in32,
 input[4:0] sel,output reg [7:0] muxOut);
  always@(in01,in02,in03,in04,in05,in06,in07,in08,in09,in10,in11,in12,in13,in14,in15,in16,in17,in18,in19,in20,in21,in22,in23,in24,in25,in26,in27,in28,in29,in30,in31,in32,sel)
  case(sel)
    5'b00000: muxOut=in01;5'b00001: muxOut=in02;5'b00010: muxOut=in03;5'b00011: muxOut=in04;
    5'b00100: muxOut=in05;5'b00101: muxOut=in06;5'b00110: muxOut=in07;5'b00111: muxOut=in08
    5'b10100: muxOut=in21;5'b10101: muxOut=in22;5'b10110: muxOut=in23;5'b10111: muxOut=in24;
    5'b11000: muxOut=in25;5'b11001: muxOut=in26;5'b11010: muxOut=in27;5'b11011: muxOut=in28;
    5'b11100: muxOut=in29;5'b11101: muxOut=in30;5'b11110: muxOut=in31;5'b11111: muxOut=in32;
  endcase
endmodule



module priorityArray(input clk, input reset, input [7:0] we, input [1:0] priority,output [1:0] priot0, 
output [1:0] priot1, output [1:0] priot2, output [1:0] priot3,output [1:0] priot4, 
output [1:0] priot5, output [1:0] priot6, output [1:0] priot7);
  priorityBlock p0(clk,reset,we[0],priority,priot0);
  priorityBlock p1(clk,reset,we[1],priority,priot1);
  priorityBlock p2(clk,reset,we[2],priority,priot2)
  priorityBlock p4(clk,reset,we[4],priority,priot4);
  priorityBlock p5(clk,reset,we[5],priority,priot5);
  priorityBlock p6(clk,reset,we[6],priority,priot6);
  priorityBlock p7(clk,reset,we[7],priority,priot7);
endmodule


module priorityBlock(input clk, input reset, input write, input [1:0] priority,output [1:0] priorityData);
  D_FF d00(clk, reset,write,priority[0],priorityData[0]);
  D_FF d01(clk, reset,write,priority[1],priorityData[1]);
endmodule


module priorityEncoder_4x2(input [3:0] in,output reg [1:0] out);
  always @ (in)
  begin
    if(in[3]==0) out=2'b11;
    else if(in[2]==0) out=2'b10;
    else if(in[1]==0) out=2'b01;
    else if(in[0]==0) out=2'b00;   
  end
endmodule



module tagArray(input clk, input reset, input [7:0] we, input [23:0] tag, output [23:0] tagOut0, output [23:0] tagOut1, output [23:0] tagOut2, output [23:0]  tagOut3, output [23:0]  tagOut4, output [23:0]  tagOut5, output [23:0]  tagOut6, output [23:0]  tagOut7);
  tagBlock t0(clk,reset,we[0],tag,tagOut0);
  tagBlock t1(clk,reset,we[1],tag,tagOut1);
  tagBlock t2(clk,reset,we[2],tag,tagOut2);
  tagBlock t3(clk,reset,we[3],tag,tagOut3);
  tagBlock t4(clk,reset,we[4],tag,tagOut4);
  tagBlock t5(clk,reset,we[5],tag,tagOut5);
  tagBlock t6(clk,reset,we[6],tag,tagOut6);
  tagBlock t7(clk,reset,we[7],tag,tagOut7);
endmodule




module tagBlock(input clk, input reset, input write, input [23:0] tag,output [23:0] tagData);
  D_FF d00(clk, reset,write,tag[0],tagData[0]);
  D_FF d01(clk, reset,write,tag[1],tagData[1]);
  D_FF d02(clk, reset,write,tag[2],tagData[2]);
  D_FF d03(clk, reset,write,tag[3],tagData[3]);

  D_FF d14(clk, reset,write,tag[14],tagData[14]);
  D_FF d15(clk, reset,write,tag[15],tagData[15]);
  D_FF d16(clk, reset,write,tag[16],tagData[16]);
  D_FF d17(clk, reset,write,tag[17],tagData[17]);
  D_FF d18(clk, reset,write,tag[18],tagData[18]);
  D_FF d19(clk, reset,write,tag[19],tagData[19]);
  D_FF d20(clk, reset,write,tag[20],tagData[20]);
  D_FF d21(clk, reset,write,tag[21],tagData[21]);
  D_FF d22(clk, reset,write,tag[22],tagData[22]);
  D_FF d23(clk, reset,write,tag[23],tagData[23]);
endmodule



module updatePriority(input [1:0] way,input [1:0] in0,input [1:0] in1,input [1:0] in2,input [1:0] in3,
output reg [1:0] out0,output reg [1:0] out1,output reg  [1:0] out2,output reg [1:0] out3);  
  always @ (way,in0,in1,in2,in3)
  begin
    out0=in0;
    out1=in1;
    out2=in2;
    out3=in3;
    case(way)
      2'b00:  
      begin
        if(in1>in0)
          out1=in1-2'b01;
        if(in2>in0)
          out2=in2-2'b01;
        if(in3>in0)
          out3=in3-2'b01;
        out0=2'b11;
      end
      2'b01:  
      begin
        if(in0>in1)
          out0=in0-2'b01;
        if(in2>in1)
          out2=in2-2'b01;
        if(in3>in1)
          out3=in3-2'b01;
        out1=2'b11;
      end
      2'b10: 
      begin
        if(in0>in2)
          out0=in0-2'b01;
   
      end 
      2'b11:
      begin
        if(in0>in3)
          out0=in0-2'b01;
        if(in1>in3)
          out1=in1-2'b01;
        if(in2>in3)
          out2=in2-2'b01;
        out3=2'b11;
      end
    endcase
  end
endmodule


module vaildArray(input clk, input reset, input [7:0] we, input validBit,output valot0, output valot1, output valot2, output valot3,output valot4, output valot5, output valot6, output valot7);
  D_FF v0(clk, reset,we[0],validBit,valot0);
  D_FF v1(clk, reset,we[1],validBit,valot1);
  D_FF v2(clk, reset,we[2],validBit,valot2);
  D_FF v3(clk, reset,we[3],validBit,valot3);
  D_FF v4(clk, reset,we[4],validBit,valot4);
  D_FF v5(clk, reset,we[5],validBit,valot5);
  D_FF v6(clk, reset,we[6],validBit,valot6);
  D_FF v7(clk, reset,we[7],validBit,valot7);
endmodule


module validityEncoder(input [3:0] validBits,output reg invalidReplace, output reg [1:0] out); 
  always @ (validBits)
  begin  
  invalidReplace=1'b0;
  if(validBits[0]==0) out=2'b00;
  else if(validBits[1]==0)  out=2'b01;  
  else if(validBits[2]==0)  out=2'b10; 
  else if(validBits[3]==0)  out=2'b11;
  else invalidReplace=1'b1; 
  end
endmodule


module testbench;
  reg clk, reset, writeORread;
  reg [7:0] inbyte;
  reg [31:0] pa;
  reg [255:0] dataBlock;
  wire [7:0] cacheOutput;
  wire hom;
  cacheModule uut(clk, reset,pa, writeORread, inbyte, dataBlock, cacheOutput,hom);
  always #5 clk=~clk;
  initial
  begin
    clk=0; reset=1;//writeORread=0;inbyte=7'd9;dataBlock=256'h123456;pa=32'h9876ABC0;
    #10 reset=0;writeORread=0;inbyte=8'd9;dataBlock=256'h123456;pa=32'h9876ABC0;//read miss
    #30 writeORread=0;inbyte=8'h77;dataBlock=256'h123456;pa=32'hABCDABC0;       //read miss
    #30 writeORread=1;inbyte=8'hAA;pa=32'hABCDABD0;                             //write hit
    #30 writeORread=1;inbyte=8'hBB;dataBlock=256'h666666;pa=32'h12345678;       //write miss
    #40 $finish;
  end
endmodule



