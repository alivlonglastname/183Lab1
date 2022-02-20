module top_lab1( 
 input  CLOCK_20, rst_bar 
 ); 
  
 wire we_bar, re_bar, ram_en_bar; 
 wire [7:0] DATA_BUS, ADDR; 
 
 infinity_mp U0 (.clk(CLOCK_20), .rst_bar(rst_bar), .MData(DATA_BUS), .MAddr(ADDR), .ram_en_bar(ram_en_bar), .we_bar(we_bar), .re_bar(re_bar)); 
 
 ROM8x128  U1 (.clk(CLOCK_20), .re_bar(re_bar), .A(ADDR), .D(DATA_BUS)); 
 
 RAM8x128  U2 (.clk(CLOCK_20), .we_bar(we_bar), .re_bar(re_bar),  .ram_en_bar(ram_en_bar), .A(ADDR), .D(DATA_BUS)); 
 
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
	
	Program_Counter pu0(.clk(clk),.pc(pcaddr));
	Ctrl_Unit pu1(.opcode(opcode),.ra(ra),.rb(rb),.re_bar(re_bar),.we_bar(we_bar),.wb(wb),.ram_en_bar(ram_en_bar),.MAddr(MAddr).MData(MData));
	reg_blk pu2(.instruction(MData),.wbvalue(wbvalue),.wb(wb),.opcode(opcode),.oprand(oprand),.ra(ra),.rb(rb));
	ALU_wbsel(.opcode(opcode),oprand(oprand),datain(MData),.ra(ra),.rb(rb),.wbvalue(wbvalue));
	
);
	


module Program_Counter (
	input clk,
	//input flag,
	output [7:0] pc
);
	reg [7:0] pcc;
	always @(posedge clk) begin
		if (flag == 1'b1) begin
			pcc <=pcc+1;
			
		end
	end
	
	assign pc = pcc;
endmodule

module Ctrl_Unit (
	input wire [7:0] instruction,
	input	[2:0]	wbvalue,
	//input			wb, //commented this out
	input pc; /* im not sure what size this should be */
	output wire pcctrl, wbctrl
	output re_bar,
	output we_bar,
	output wb,ram_en_bar,
	output [2:0] MAddr,
	output [2:0] MData,
	output	[2:0]	opcode,oprand,ra,rb
);
	reg [1:0] opcodeREG;
	reg [2:0] raa = 3'b000;
	reg [2:0] rbb;
	reg [2:0] oprandREG;
	reg wb = 1'b0;
	reg  [7:0] instr;
	reg alu; // True if opcode is 11, false otherwise
	reg pcctrl_temp, wbctrl_temp;

	always @(*) begin /* implement clk */
		// Instruction decoder: Are we dealing w RA or operand?
		instr = instruction; 
		opcodeREG = instr[7:6];
		rbb = instr[2:0];
		if (opcodeREG == 2'b11) begin // I EDITED THE CODE HERE (CHANGED != TO ==)
			oprandREG = instr [5:3];
			alu = 1;
		end
		else if (opcodeREG != 2'b11) begin // I EDITED THE CODE HERE (CHANGED == TO !=)
			raa = instr [5:3]; //if opcode is not 11, [5:3] is RA
			alu = 0;
		end
		if (wb == 1'b1) begin /* i think this should always update rb right?*/
			rb = wbvalue;
			wb = 1'b0;
		end
		// Instruction decoder END
		case (opcodeREG)
			2'b00 : begin //MOVE: RA -> RB
				re_barr = 1'b1;
				we_barr = 1'b1;
				ram_en_bar = 1'b1;
				wb = 1'b1; //EDITED: wbb -> wb
			end
			
			2'b01 : begin //LOAD: M[RA] -> RB
				re_barr = 1'b0;
				we_barr = 1'b1;	
				ram_en_bar = 1'b0;
				wb = 1'b1; //EDITED: wbb -> wb
			end
			
			2'b10 : begin //STORE: RB -> M[RA]
				re_barr = 1'b1;
				we_barr = 1'b0;
				ram_en_bar = 1'b0;
				MAddrr = ra;
				MDataa = rb;
				wb = 1'b0; //EDITED: wbb -> wb
			end
			
			2'b11 : begin //ALU
				re_barr = 1'b1;
				we_barr = 1'b1;
				wb = 1'b1; //EDITED: wbb -> wb
				ram_en_bar = 1'b0;
			end
		endcase
		#10 if (alu == 1) begin // delay needs to happen here because alu has to be calculated first
			case (oprandREG)
				3'b011: begin //BRANCH: RB->PC || pcctrl = 0
					pcctrl = 0;
				end
				
				3'b100 : begin //BRANCH IF Z: RB->PC if Z
					if (r1 == 0) begin /* COME BACK HERE: IDK WHICH NUMBER IM CHECKING TO SEE IF 0, IS IT R1 OR RB? */
						pcctrl = 0;
					end
					else begin
						pcctrl = 1;
					end
				end
				
				3'b101 : begin //BRANCH IF N: RB->PC if N
					if (r1 < 0) begin /* COME BACK HERE: IDK WHICH NUMBER IM CHECKING TO SEE IF 0, IS IT R1 OR RB? */
						pcctrl = 0;
					end
					else begin
						pcctrl = 1;
					end
				end
				
				3'b110 : begin //JUMP AND LINK : PC+1-> RB || RB->PC 
					/*check if this is ok: set pcctrl to 1 (so rb goes to pc), drive wbctrlmux with pc+1 by storing it in ra? */ 
					raa <= pc+1; // blocking statement because i want things to happen sequentially
					wbctrl = 1;
					pcctrl = 1;
				end
			endcase


		end
		
		
	end	
	// ASSIGNMENTS
	assign re_bar = re_barr;
	assign we_bar = we_barr;
	assign wb = wbb;
	assign MAddr = MAddrr;
	assign MData = MDataa;
	assign opcode = opcodeREG;
	assign oprand = oprandREG;
	assign ra=raa;
	assign rb=rbb;
	
endmodule




module ALU_wbsel(
	input	[2:0] opcode,oprand,datain,ra,rb,
	output	[2:0] wbvalue
	);
	reg [2:0] r1 = 3'b000;
	
	always @(*) begin
		if (opcode == 2'b00) begin //MOVE: RA -> RB | wbctrl = 1 | we = 1
			ALU_out = ra;
			wbvalue = ALU_out;
		end
		else if (opcode == 2'b01) begin //LOAD: M[RA] -> RB || wbctrl = 0 || we = 1
			wbvalue = datain; /*idk what this is, i think alu doesnt do anything here*/
		end
		
		else if (opcode == 2'b11) begin //Arithmetic operation
			if (oprand == 3'b000) begin //AND: (RB & R1) -> RB || wbctrl = 1 || we = 1
				ALU_out = rb & r1;
				wbvalue = ALU_out;
			end
			
			else if (oprand == 3'b001) begin //ADD: (RB + R1) -> RB || wbctrl = 1 || we = 1
				ALU_out = rb + r1;
				wbvalue = ALU_out;
			end
			
			else if (oprand == 3'b010) begin //EDITED: 011 TO 010
											//SUB: (RB - R1) -> RB || wbctrl = 1 || we = 1
				ALU_out = rb - r1;
				wbvalue = ALU_out;
			end
			else if (oprand == 3'b111) begin //LOAD IMMEDIATE: M[next byte] -> RB || PC+1 -> PC
				/*i dont actually remember what this is supposed to do so... implementing later uwu*/
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
	


