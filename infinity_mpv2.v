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

// reg_blk done
module reg_blk (
    input [2:0] A_addr, B_addr, wb_en, //if wb_en = 1, write into RB
    input [7:0] wb; //the value of wb
    output [7:0] A, B, R1
);

reg [7:0] rf [0:7]; //Register file

assign A = rf[A_addr];
assign B = rf[B_addr];

always @(*) begin
   if (wb_en) begin //if wb is enabled
       rf[B_addr] = wb;
   end
    
end


endmodule

// ALU done
module alu (
    input [7:0] A, B,
    output N, Z, 
    output [7:0] add, sub, aand, RA;
);
assign add = B+A;
assign sub = B-A;
assign aand = B & A;
assign Z = ~(A[0] | A[1] | A[2] | A[3] | A[4] | A[5] | A[6] | A[7]);
assign N = A[7];
endmodule

// instruction reg done
module intruction_reg(
		input [7:0]	instruction,
		output	[1:0]	opcode,
		output	[2:0]	oprand,
		output	[2:0]	A_addr,B_addr
);
	reg	[2:0]	ooprand, AA_addr;

	assign	opcode = instruction[7:6];
	assign	B_addr = instruction[2:0];
	always @(*) begin
		if	(opcode == 2'b11) begin
			ooprand = instruction[5:3];
		end
		else begin
			AA_addr = instruction[5:3];
		end
	end
    assign	A_addr = AA_addr;
	assign 	oprand = ooprand;
	
endmodule

module control(
	input	[1:0]	opcode,
	input	[2:0]	oprand,
	input	N,Z,
	output wr_bar, rd_bar, ram_en_bar, wb_en, br,
    output [2:0] sel
);


    reg re_barr, wr_barr, ram_en_barr, wbb_en, brr;
    reg [2:0] sell;
 	always @(*) begin
        brr = 0;
		if	(opcode == 2'b00) begin //MOVE: RA -> RB
			re_barr = 1'b1;
			we_barr = 1'b1;
			ram_en_bar = 1'b1;
			wbb_en = 1'b1;
            sell = 0;
		end
		else if	(opcode == 2'b01) begin //LOAD: M[RA] -> RB
			re_barr = 1'b0;
			we_barr = 1'b1;	
			ram_en_bar = 1'b0;
			wbb_en = 1'b1;
            sell = 1;
		end
		else if (opcode == 2'b10) begin //STORE: RB -> M[RA]
			re_barr = 1'b1;
			we_barr = 1'b0;
			ram_en_bar = 1'b0;
			MAddrr = ra;
			MDataa = rb;
			wbb_en = 1'b0;
		end
		else if (opcode == 2'b11) begin //ALU
			re_barr = 1'b1;
			we_barr = 1'b1;
			wbb_en = 1'b1;
			ram_en_bar = 1'b1;
            case (oprand)
                3'b000: begin // and
                    wbb_en = 1'b1;
                    sell = 2;
                end
                3'b001: begin // add
                    wbb_en = 1'b1;
                    sell = 3;
                end
                3'b010: begin //sub
                    wbb_en = 1'b1;
                    sell = 4;
                end

                3'b011: begin // branch
                    wbb_en = 1'b0;
                end
                3'b100: begin // biz
                    wbb_en = 1'b0;
                    if (Z) begin
                        brr = 1;
                    end
                end
                3'b101: begin //bin
                    wbb_en = 1'b0;
                    if (N) begin
                        brr = 1;
                    end
                end

                3'b110: begin //jl
                    wbb_en = 1'b1;
                end

                3'b111: begin //ldi
                    wbb_en = 1'b0;
                end
            endcase
		end			
	end	

assign sel = sell;
assign br = brr;
assign re_bar = re_barr;
assign we_bar = we_barr;
assign ram_en_bar = ram_en_barr;
assign wb_en = wbb_en;
// Move

// load

// store

// and

// add

// sub

// branch

// biz

// bin

// jl

// LDI

endmodule

module pc(

);
	always @(*) begin
		if (br == 1'b0) begin;
			pc_c = pc + 1'b1; //increment
		end
		else begin
			pc_c = 
	end
	
	always @(posedge clk) begin
		pc <= pc_c;
	end
endmodule