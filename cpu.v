// ASM description; HD is where the program is read from, it can not be writen to
// Z is a variable
// X is a don't-care-value
// BYTE commands: ZZ(src)ZZ(dest)ZZZZ(command)
//
// HALT	0000; halt
// CLR	0001; clear (dest only)
// INC	0010; increment (dest only)
// DEC	0011; decrement (dest only)
// NOT	0100; not (dest only)
// AND	0101; bitwise and
// OR	0110; bitwise or
// XOR	0111; bitwise xor
// ADD	1000; add
// SUB	1001; subtract
// MUL	1010; multiply
// MOV	1011; copy
//
// Multi-BYTE commands; Currently BR is the only multi-byte command to function properly
//
// BR	XXXX1100 ZZZZZZZZ(address); branches the program to address
// CMP	ZZ(a)ZZ(b)1101 ZZ(condition #1)ZZ(condition #2)ZZ(conditio #3)XX (ZZZZZZZZ (ZZZZZZZ (ZZZZZZZZ)?)?)? (addresses); compares a to b and branches to the true condition's address or falls through; conditions are 01(Equal), 10(greater than), 11(less than); the addresses' are ommitable if there's no condition
// LOD	Z(src)Z(register flag)ZZ(dest)1110 ZZZZZZZZ; loads a RAM value to a register; src is a boolean choice between 0(RAM) and 1(HD); the register flag is a boolean flag telling whether you are reading the address directly (0) or from a register (1); If you are reading the address from a register it is represented by the two most significant bits
// SAV	XZ(register flag)ZZ(src)1111 ZZZZZZZZ; saves a register value to ram; the register flag is a boolean flag telling whether you are reading the address directly (0) or from a register (1); src is the register to write to; If you are reading the address from a register it is represented by the two most significant bits

module cmdsplitter(by, reg1, reg2, cmd);
	input [7:0] by;
	output reg [1:0] reg1, reg2;
	output reg [3:0] cmd;

	always @(by) begin
		reg1 = by[7:6];
		reg2 = by[5:4];
		cmd = by[3:0];
	end
endmodule

module counter(out, set, setby, clk, skip);
	output [7:0] out;
	reg [7:0] out;
	input set;
	input [7:0] setby;
	input clk;
	input skip;

	initial begin
		out = 0;
	end

	always @(posedge clk) begin
		if(~skip) begin
			out = out+1;
//			$display("count=%b",out);
		end
	end
	always @(posedge set) begin
		$display("setcount=%b",setby);
		out = setby;
	end
endmodule

module memram(by, selby, write, writeby);
	output [7:0] by;
	reg [7:0] by;
	input [7:0] selby;
	input write;
	input [7:0] writeby;

	reg [7:0] mem [0:255];

	always @(selby) begin
		by = mem[selby];
		$display("ram=%b",by);
	end
	always @(posedge write) begin
		$display("setram=%b",writeby);
		mem[selby] = writeby;
		$display("newram[%b]=%b",selby,mem[selby]);
	end
endmodule

module memhd(by, selby);
	output [7:0] by;
	reg [7:0] by;
	input selbi;
	input [7:0] selby;

	reg [7:0] mem [0:255];
	initial begin
		// test
		mem[0] = 8'b00000001; // CLR R1
		mem[1] = 8'b00010001; // CLR R2
		mem[2] = 8'b00000010; // INC R1
		mem[3] = 8'b00011000; // ADD R1, R2
		mem[4] = 8'b01001000; // ADD R2, R1
		mem[5] = 8'b00010001; // CLR R2
		mem[6] = 8'b00011010; // MUL R1, R2
		mem[7] = 8'b01001010; // Mul R2, R1
		mem[8] = 8'b00010010; // INC R2
		mem[9] = 8'b00001100; // BR 12
		mem[10] = 8'b0001100; // ^12
		mem[11] = 8'b11111111; // gibberish
		mem[12] = 8'b00000000; // HALT
		// fibs
/*		mem[0] = 8'b00000001; // CLR R1
		mem[1] = 8'b00010001; // CLR R2
		mem[2] = 8'b00100001; // CLR R3
		mem[3] = 8'b00010010; // INC R2
		mem[4] = 8'b01011111; // SAV R2, R3
		mem[5] = 8'b10000000; // ^R3
		mem[6] = 8'b10111110; // LOD R4,14
		mem[7] = 8'b00001110; // ^14
		mem[8] = 8'b00011000; // ADD R2,R1
		mem[9] = 8'b01001110; // LOD R1,R3
		mem[10] = 8'b10000000; // ^R3
		mem[11] = 8'b00100010; // INC R3
		mem[12] = 8'b01011111; // SAV R2,R3
		mem[13] = 8'b10000000; // ^R3
		mem[14] = 8'b10111101; // CMP R3,R4
		mem[15] = 8'b01000000; // ^BEQ 13
		mem[16] = 8'b00001101; // ^13
		mem[17] = 8'b00001100; // BR 6
		mem[18] = 8'b00000110; // ^6
		mem[19] = 8'b00000000; // HALT
		mem[20] = 8'b00010100; // 20*/
	end
	always @(selby) begin
		by = mem[selby];
	end
endmodule

module regs(one, sel1, two, sel2, write, writesel, writeby);
	output [7:0] one, two;
	reg [7:0] one,two;
	input [1:0] sel1,sel2;
	input write;
	input [1:0] writesel;
	input [7:0] writeby;

	reg [7:0] a;
	reg [7:0] b;
	reg [7:0] c;
	reg [7:0] d;

	always @(sel1,sel2) begin
		one = (sel1==0)?a:((sel1==1)?b:((sel1==2)?c:d));
		two = (sel2==0)?a:((sel2==1)?b:((sel2==2)?c:d));
/*		$display("reg00 = %b",a);
		$display("reg01 = %b",b);
		$display("reg10 = %b",c);
		$display("reg11 = %b",d);*/
	end
	always @(posedge write) begin 
		if(writesel==0) begin
			a = writeby;
		end
		else if(writesel==1) begin
			b = writeby;
		end
		else if(writesel==2) begin
			c = writeby;
		end
		else if(writesel==3) begin
			d = writeby;
		end
//		$display("setreg%b = %b",writesel,writeby);
	end
endmodule

module mux(write, writeby, writesel, ramwrite, ramsel, ramby, ramwriteby, skipcount, count, countwrite, countwriteby, halt, by, reg1, reg2, rega, regb, cmd, incout, decout, notout, andout, orout, xorout, addout, subout, mulout, divout, clk, cmpout);
	input [1:0] reg1;
	input [1:0] reg2;
	output reg [1:0] writesel;
	output reg write, ramwrite, countwrite;
	output reg [7:0] writeby, ramsel, ramwriteby, countwriteby;
	output reg halt, skipcount;
	input [7:0] rega, regb, by;
	input [3:0] cmd;
	input clk;
	input [1:0] cmpout;
	input [7:0] ramby;
	input [7:0] count;
	reg [7:0] counterswap;

	reg br, cmp, beq, bhi, blo, sav, lod, lodhd, lsreg;
	reg [1:0] b1, b2, b3, cmpval;

	initial begin
		br = 0;
		cmp = 0;
		beq = 0;
		bhi = 0;
		blo = 0;
		sav = 0;
		lod = 0;
		lodhd = 0;
		lsreg = 0;
		skipcount = 0;
		b1 = 0;
		b2 = 0;
		b3 = 0;
		cmpval = 0;
	end

	input [7:0] incout, decout, notout, andout, orout, xorout, addout, subout, mulout, divout;

	always @(posedge clk) begin
		assign write = 0;
		assign countwrite = 0;
		assign ramwrite = 0;

		if(br==1) begin
			assign countwrite = 1;
			assign countwriteby = by;
			assign br = 0;
		end
		else if(cmp==1) begin
			if(reg1>0) begin
				assign b1 = reg1;
			end
			if(reg2>0) begin
				assign b2 = reg2;
			end
			if(cmd>3) begin
				assign b3 = cmd-3;
			end
			assign cmp = 0;
		end
		else if(b1==1) begin // BEQ
			if(cmpval==1) begin
				countwrite = 1;
				countwriteby = by;
				assign b1 = 0;
				assign b2 = 0;
				assign b3 = 0;
			end
			else begin
				assign b1 = b2;
				assign b2 = b3;
				assign b3 = 0;
			end
		end
		else if(b1==2) begin // BHI
			if(cmpval==2) begin
				countwrite = 1;
				countwriteby = by;
				assign b1 = 0;
				assign b2 = 0;
				assign b3 = 0;
			end
			else begin
				assign b1 = b2;
				assign b2 = b3;
				assign b3 = 0;
			end
		end
		else if(b1==3) begin // BLO
			if(cmpval==3) begin
				countwrite = 1;
				countwriteby = by;
				assign b1 = 0;
				assign b2 = 0;
				assign b3 = 0;
			end
			else begin
				assign b1 = b2;
				assign b2 = b3;
				assign b3 = 0;
			end
		end
		else if(sav==1) begin
			if(~lsreg) begin
				$display("actually saving to %b", by);
				assign ramsel = by;
			end
			else begin
				$display("actually saving to R%b = %b", reg1, rega);
				assign ramsel = rega;
			end
			assign ramwrite = 1;
			assign sav = 0;
		end
		else if(lod==1) begin
			assign skipcount = 1;
			assign counterswap = count;
			if(lsreg==1) begin
				assign countwriteby = rega;
			end
			else begin
				assign countwriteby = by;
			end
			assign ramsel = by;
			assign lod = 0;
		end
		else if(skipcount==1) begin
			if(lodhd==1) begin
				assign writeby = by;
			end
			else begin
				assign writeby = ramby;
			end
			assign write = 1;
			assign skipcount = 0;
		end
		else begin
			assign writesel = reg2;
			if(cmd==0) begin
				assign halt = 1;
				$display("HALT");
			end
			if(cmd==1) begin // CLR
				assign writeby = 0;
				assign write = 1;
				$display("CLR");
			end
			if(cmd==2) begin // INC
				assign writeby = incout;
				assign write = 1;
				$display("INC");
			end
			if(cmd==3) begin // DEC
				assign writeby = decout;
				assign write = 1;
				$display("DEC");
			end
			if(cmd==4) begin // NOT
				assign writeby = notout;
				assign write = 1;
				$display("NOT");
			end
			if(cmd==5) begin // AND
				assign writeby = andout;
				assign write = 1;
				$display("AND");
			end
			if(cmd==6) begin // OR
				assign writeby = orout;
				assign write = 1;
				$display("OR");
			end
			if(cmd==7) begin // XOR
				assign writeby = xorout;
				assign write = 1;
				$display("XOR");
			end
			if(cmd==8) begin // ADD
				assign writeby = addout;
				assign write = 1;
				$display("ADD");
			end
			if(cmd==9) begin // SUB
				assign writeby = subout;
				assign write = 1;
				$display("SUB");
			end
			if(cmd==10) begin // MUL
				assign writeby = mulout;
				assign write = 1;
				$display("MUL");
			end
			if(cmd==11) begin // MOV
				assign writeby = rega;
				assign write = 1;
				$display("MOV");
			end
			if(cmd==12) begin // BR
				assign write = 0;
				assign br = 1;
				$display("BR");
			end
			if(cmd==13) begin // CMP
				assign write = 0;
				assign cmp = 1;
				assign cmpval = cmpout;
				$display("CMP");
			end
			if(cmd==14) begin // LOD
				assign write = 0;
				assign lod = 1;
				if(reg1==0) begin
					assign lodhd = 0;
					assign lsreg = 0;
				end
				else if(reg1==1) begin
					assign lodhd = 0;
					assign lsreg = 1;
				end
				else if(reg1==2) begin
					assign lodhd = 1;
					assign lsreg = 0;
				end
				else if(reg1==3) begin
					assign lodhd = 1;
					assign lsreg = 1;
				end
				$display("LOD");
			end
			if(cmd==15) begin // SAV
				assign write = 0;
				assign sav = 1;
				if(reg1==1) begin
					$display("saving reg");
					assign lsreg = 1;
				end
				else begin
					assign lsreg = 0;
				end
				$display("saving R%b = %b",reg2, regb);
				assign ramwriteby = regb;
				assign writeby = regb;
				$display("SAV");
			end
		end
	end
endmodule

module cpu(clk, halt);
	input clk;
	output wire halt;

	wire [3:0] cmd;
	wire [1:0] reg1;
	wire [1:0] reg2;
	wire [1:0] writesel;
	wire [7:0] rega;
	wire [7:0] regb;
	wire [7:0] mem;
	wire [7:0] by;
	wire [7:0] ramby;
	wire [7:0] writeby;
	wire [7:0] ramwriteby;
	wire [7:0] countwriteby;
	wire [7:0] ramsel;
	wire write;
	wire ramwrite;
	wire countwrite;
	wire [7:0] incout, decout, andout, orout, xorout, notout, addout, subout, mulout, divout;
	wire [1:0] cmpout;
	wire skipcount;

	counter cnt(mem, countwrite, countwriteby, clk, skipcount);
	memhd hdmem(by, mem);
	cmdsplitter split(by, reg1, reg2, cmd);

	regs allreg(rega, reg1, regb, reg2, write, writesel, writeby);

	inc cinc(incout, regb);
	dec cdec(decout, regb);
	cand ccand(andout, rega, regb);
	cor ccor(orout, rega, regb);
	cxor ccxor(xorout, rega, regb);
	cnot ccnot(notout, regb);
	add cadd(addout, rega, regb);
	sub csub(subout, rega, regb);
	mul cmul(mulout, rega, regb);
	div cdiv(divout, rega, regb);

	cmp ccmp(cmpout, rega, regb);

	memram rammem(ramby, ramsel, ramwrite, ramwriteby);

	mux cmux(write, writeby, writesel, ramwrite, ramsel, ramby, ramwriteby, skipcount, mem, countwrite, countwriteby, halt, by, reg1, reg2, rega, regb, cmd, incout, decout, notout, andout, orout, xorout, addout, subout, mulout, divout, clk, cmpout);

	always @(posedge write) begin
		$display("R%b=>%b",writesel,writeby);
	end

endmodule

module cmp(out, a, b);
	input [7:0] a,b;
	output [1:0] out;
	assign out = (a==b)?1:((a>b)?2:((a<b)?3:0));
endmodule

module inc(out, in);
	input [7:0] in;
	output [7:0] out;
	assign out = in+1;
endmodule

module dec(out, in);
	input [7:0] in;
	output [7:0] out;
	assign out = in-1;
endmodule

module cand(out, a, b);
	input [7:0] a,b;
	output [7:0] out;
	assign out = a&b;
endmodule

module cor(out, a,b);
	input [7:0] a,b;
	output [7:0] out;
	assign out = a|b;
endmodule

module cxor(out, a,b);
	input [7:0] a,b;
	output [7:0] out;
	assign out = a^b;
endmodule

module cnot(out, in);
	input [7:0] in;
	output [7:0] out;
	assign out = ~in;
endmodule

module add(out, a,b);
	input [7:0] a,b;
	output [7:0] out;
	assign out = a+b;
endmodule

module sub(out, a,b);
	input [7:0] a,b;
	output [7:0] out;
	assign out = a-b;
endmodule

module mul(out, a,b);
	input [7:0] a,b;
	output [7:0] out;
	assign out = a*b;
endmodule

module div(out, a,b);
	input [7:0] a,b;
	output [7:0] out;
	assign out = a/b;
endmodule

module stimulate;
	reg clk;
	wire halt;

	cpu ccpu(clk, halt);

	initial begin
		clk = 0;
		forever begin
			#1 clk = ~clk;
			if(halt)
				$finish;
		end
	end

endmodule
