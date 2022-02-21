module top_lab1( 
 input  CLOCK_20, rst_bar 
 ); 
  
 wire we_bar, re_bar, ram_en_bar; 
 wire [7:0] DATA_BUS, ADDR; 
 
 infinity_mp U0 (.clk(CLOCK_20), .rst_bar(rst_bar), .MData(DATA_BUS), .MAddr(ADDR), .ram_en_bar(ram_en_bar), .we_bar(we_bar), .re_bar(re_bar)); 
 
 ROM8x128  U1 (.clk(CLOCK_20), .re_bar(re_bar), .A(ADDR), .D(DATA_BUS)); 
 
 RAM8x128  U2 (.clk(CLOCK_20), .we_bar(we_bar), .re_bar(re_bar),  .ram_en_bar(ram_en_bar), .A(ADDR), .D(DATA_BUS)); 
 
endmodule 

module 21mux (
	input [7:0] one, 
	input [7:0] zero,
	input control,
	output [7:0] out
)
	reg [7:0] temp;
	always @(*) begin
		if (control == 1) begin
			temp = one;
		end
		else begin
			temp = zero;
		end
	end
	assign out = temp;
endmodule

module infinity_mp(
	input clk,rst_bar,
	output [7:0] MAddr,
	inout [7:0] MData,
	output ram_en_bar, we_bar, re_bar
	);
	wire [7:0] pcaddr,cuaddr;
	wire [2:0] opcode,oprand,ra,rb,wbvalue;
	wire re_bar,we_bar,wb, ram_en_bar;
	
	reg	[7:0] roma;
	reg [7:0] dataout;
	reg [7:0] data;

	//21mux pcmux(.one(wbvalue), .zero(pcaddr+1), .control(br), .out(pcin)) //PCmux, either pc=pc+1 or pc=rb
	Program_Counter pu0(.clk(clk),.pcin(pcaddr));
	PC_ctrl pc1(.wbvalue) //write me
	Ctrl_Unit pu1(.opcode(opcode),.ra(ra),.rb(rb),.re_bar(re_bar),.we_bar(we_bar),.wb(wb),.ram_en_bar(ram_en_bar),.MAddr(MAddr).MData(MData));
	reg_blk pu2(.instruction(MData),.wbvalue(wbvalue),.wb(wb),.opcode(opcode),.oprand(oprand),.ra(ra),.rb(rb));
	ALU_wbsel pu3(.opcode(opcode),oprand(oprand),datain(MData),.ra(ra),.rb(rb),.wbvalue(wbvalue));
	
);
	

//PC_ctrl
module PC_ctrl(
	input [2:0]	wbvalue,
	input	br,
	input	[7:0] pcaddr,

	output	[7:0] pcin

);
	wire [7:0] wwbvalue = {5'b00000,wbvalue};
	wire [7:0] upd_pcadder = pcaddr+1'b1;

	21mux pcmux(.one(wwbvalue), .zero(upd_pcadder), .control(br), .out(pcin)) //PCmux, either pc=pc+1 or pc=rb
	
endmodule

// PC Done - Ali
module Program_Counter (
	input clk,
	input [7:0] pcin,
	//input flag,
	output [7:0] pc
);
	reg [7:0] pcc;
	always @(posedge clk) begin
		pcc <= pcin;
	end
	
	assign pc = pcc;
endmodule






module reg_blk(
	input	[7:0]	instruction,
	input	[2:0]	wbvalue,
	input			wb,
	
	output	[2:0]	opcode,oprand,raout,rbout
);
	reg LDI = 0;
	reg [7:0] RF [0:7];
	reg [2:0] rb, ra;
	reg [7:0] instr;
	always @(*) begin
		instr = instruction;
		opcode = instr[7:6];
		rb = instr[2:0]; //rb is the address
		if (opcode == 2'b11) begin
			oprand = instr [5:3];
			// most recent edit:\
			if (oprand == 111) begin
				LDI = 1;
			end else begin
				LDI = 0;
			end
			//
		end
		else if (opcode != 2'b11) begin
			ra = instr [5:3];
		end
		if (wb == 1'b1) begin
			RF[rb] = wbvalue;
		end
	end
	assign raout = RF[ra];
	assign rbout = RF[rb];	
endmodule

module Ctrl_Unit (
	input [1:0] opcode,
	input [2:0] ra,
	input [2:0] rb,
	output re_bar,
	output we_bar,
	output wb,ram_en_bar,
	output [2:0] MAddr,
	output [2:0] MData
);
	always @(*) begin
		if	(opcode == 2'b00) begin //MOVE: RA -> RB
			re_barr = 1'b1;
			we_barr = 1'b1;
			ram_en_bar = 1'b1;
			wbb = 1'b1;
		end
		else if	(opcode == 2'b01) begin //LOAD: M[RA] -> RB
			re_barr = 1'b0;
			we_barr = 1'b1;	
			ram_en_bar = 1'b0;
			wbb = 1'b1;
		end
		else if (opcode == 2'b10) begin //STORE: RB -> M[RA]
			re_barr = 1'b1;
			we_barr = 1'b0;
			ram_en_bar = 1'b0;
			MAddrr = ra;
			MDataa = rb;
			wbb = 1'b0;
		end
		else if (opcode == 2'b11) begin //ALU
			re_barr = 1'b1;
			we_barr = 1'b1;
			wbb = 1'b1;
			ram_en_bar = 1'b1;
		end			
	end	
	
	assign re_bar = re_barr;
	assign we_bar = we_barr;
	assign wb = wbb;
	assign MAddr = MAddrr;
	assign MData = MDataa;
endmodule


module ALU_wbsel(
	input	[2:0] opcode,oprand,datain,ra,rb,pc,
	output	[2:0] wbvalue, pcwrite,
	output	br
	);
	reg [2:0] r1 = 3'b000;
	reg [2:0] ALU_out;
	reg [7:0] temp;
	always @(*) begin

		

		if (opcode == 2'b00) begin //Move
			ALU_out = ra;
			wbvalue = ALU_out;
			br = 1'b0;
		end
		else if (opcode == 2'b01) begin //Load
			wbvalue = datain;
			br = 1'b0;
		end
		else if (opcode == 2'b10) begin //store
			br = 1'b0;
		end
		else if (opcode == 2'b11) begin
			if (oprand == 3'b000) begin //and
				ALU_out = rb & r1;
				wbvalue = ALU_out;
				br = 1'b0;
			end
			else if (oprand == 3'b001) begin //plus
				ALU_out = rb + r1;
				wbvalue = ALU_out;
				br = 1'b0;
			end
			else if (oprand == 3'b010) begin  //minus
				ALU_out = rb - r1;
				wbvalue = ALU_out;
				br = 1'b0;
			end
			//new code added by Puchen
			else if (oprand == 3'b011) begin //BR
				//ALU_out = rb - r1;
				wbvalue = rb;
				pcwrite = rb;
				br = 1'b1;
			end
			
			else if (oprand == 3'b100) begin //BRZ
				wbvalue = rb;
				pcwrite = rb;
				if (ra == 3'b000) begin //z=1
					br = 1'b1;
				end else begin
					br = 1'b0;
				end
			end	

			else if (oprand == 3'b101) begin //BRN
				wbvalue = rb;
				pcwrite = rb;
				if (ra[2] == 1'b1) begin //N=1
					br = 1'b1;
				end else begin
					br = 1'b0;
				end
			end		
			
			else if (oprand == 3'b110) begin //JNL  need to figure out clock
				temp = rb;
				wbvalue = pc+1;
				pcwrite = temp;
				br = 1'b1;
			end	
			
			else if (oprand == 3'b111) begin //LDI  need to figure out clock
				wbvalue = pc+1'b1;
				br = 1'b0;
			end	
		end
			
 	end
endmodule

	
	
	
module ROM8x128 (
	input clk,
	input re_bar,
	input [7:0] A,
	output [7:0] D
	);
	reg [7:0] mem [0:127] ; 
	assign D = (~re_bar) ? mem[A] : 8'b0;
	initial begin
		$readmemb("code", mem); 
	end
endmodule

module RAM8x128 (D, in, A, we_bar, re_bar , ram_en_bar, clk);
	output [7:0] D;
	input [7:0] in;
	input [8:0] A;
	input we_bar,re_bar,ram_en_bar, clk;
	reg [7:0] out;
	reg [7:0] DATA[0:127];

	always @(posedge clk) begin
	if (ram_en_bar==1'b0) begin
		if(re_bar==1'b0) begin//READ
			out=DATA[A];
		end else 
		if(we_bar==1'b0) begin//WRITE
			DATA[A]=in;
		end
	end
	else begin
			out=8'bz;
	end
	end
	assign D = out;
endmodule