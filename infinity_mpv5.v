module top_lab1( 
 input  CLOCK_20, rst_bar 
 ); 
  
 wire we_bar, re_bar, ram_en_bar; 
 wire [7:0] DATA_BUS, ADDR; 
 
 infinity_mp U0 (.clk(CLOCK_20), .rst_bar(rst_bar), .MData(DATA_BUS), 
.MAddr(ADDR), .ram_en_bar(ram_en_bar), .we_bar(we_bar), .re_bar(re_bar)); 
 
 ROM8x128  U1 (.clk(CLOCK_20), .re_bar(re_bar), .A(ADDR), .D(DATA_BUS)); 
 
 RAM8x128  U2 (.clk(CLOCK_20), .we_bar(we_bar), .re_bar(re_bar), 
.ram_en_bar(ram_en_bar), .A(ADDR), .D(DATA_BUS)); 
 
endmodule 

module ROM8x128 (
	input clk,
	input re_bar,
	input [7:0] A,
	output [7:0] D
	);
	reg [7:0] mem [0:127] ; 
	reg count;
	always	@(posedge clk) begin
		if (count == 1'b0) begin
			count <= 1'b1;
		end
		else begin
			count <= 1'b0;
		end
	end
	assign D = ((~re_bar)&(~count)) ? mem[A] : 8'bzzzzzzzz;  //only output on the first clk cycle
	initial begin
		$readmemh("code", mem); /*changed to readmemh because we use hex */
	end
endmodule

module RAM8x128 (D, A, we_bar, re_bar , ram_en_bar, clk, rst);
	inout [7:0] D;
	//input [7:0] in;
	input [7:0] A;
	input we_bar,re_bar,ram_en_bar, clk;
	input rst;
	reg [7:0] out;
	reg [7:0] DATA[0:127];
	reg count;
	integer i;
	always @(*) begin
		
		if (rst == 0) begin
			for (i = 0; i < 128; i = i+1) begin
				DATA[i] = 8'b00000000;
			end
		end
	end
	always	@(posedge clk) begin
		if (count == 1'b0) begin
			count <= 1'b1;
		end
		else begin
			count <= 1'b0;
		end
	end
	always @(posedge clk) begin
		if ((ram_en_bar==1'b0)&count) begin //re/write on the second clk cyclr
			if(re_bar==1'b0) begin//READ
				out<=DATA[A];
			end else 
			if(we_bar==1'b0) begin//WRITE
				DATA[A]<=D;
			end
		end
		else begin
				out<=8'bz;
		end
	end
	assign D = out;
endmodule

module infinity_mp(
	input clk,rst_bar,
	output [7:0] MAddr,
	output [7:0] MData,
	output ram_en_bar, we_bar, re_bar
);
	wire reset;
    wire    [7:0]   instruction; //work me
    wire    [7:0]   pc, A, B, add, sub, aand,pc_to_wb;
    wire    [1:0]   opcode;
    wire    [2:0]   oprand, A_addr, A_addr_in, B_addr, sel;
    wire    N, Z, br, ram_en_barr, wb_en,freeza, wee_bar;
    wire    [7:0]   muxout,data;
    reg     count;
    reg     [7:0]   datar, dataout = 8'b00000000, MMdata = 8'b00000000; /*added MMData*/
    //mux1 determin input of rf a address
    assign  A_addr_in = (opcode == 2'b11) ? 3'b001 : A_addr;
    //mux2 determin input of writeback
    assign  muxout = sel[2] ? (sel[1] ? (sel[0] ? B : instruction ) : (sel[0] ? pc_to_wb : sub)) : (sel[1] ? (sel[0] ? add : aand) : (sel[0] ? data : A));
    assign we_bar = wee_bar; /* what is this */
    //this always block use to read/wrirte to data bus
    //Mdata drive by eprom on the first clock cycle
    //mdata drive by infinity_mp & ram on second clk cycle
	
    always @(posedge clk) begin
        if (count == 1'b0) begin
            count <= 1'b1;
            //dataout <= 8'bzzzzzzzz;
        end
        else begin 
			/* added the next line */
			//MMdata = dataout;
            dataout = B;
            datar <= MData;
            count <= 1'b0;
        end
    end

    assign data = datar;
    //assign MData = ((~wee_bar)&(~ram_en_bar)&count) ? dataout : 8'bzzzzzzzz;
	assign Mdata = 8'b00000000;/*added this and commented above line*/
	assign ram_en_bar = ram_en_barr;
	assign reset = rst_bar;
    instruction_reg u0(.clk(clk), .instructionin(MData),.opcode(opcode),.oprand(oprand),.A_addr(A_addr),.B_addr(B_addr),.freeza(freeza),.instructionout(instruction));//work on data
    pc      u1(.br(br),.clk(clk),.A(A),.B(B),.pc_out(MAddr),.pc_to_wb(pc_to_wb), .rst(reset));
    control u2(.opcode(opcode),.oprand(oprand),.N(N),.Z(Z),.freeza(freeza),.we_bar(wee_bar),.re_bar(re_bar),.ram_en_bar(ram_en_bar),.wb_en(wb_en),.br(br),.sel(sel));
    reg_blk u3(.clk(clk),.A_addr(A_addr_in),.B_addr(B_addr),.wb_en(wb_en),.wb(muxout),.A(A),.B(B), .rst(reset));
    alu     u4(.A(A),.B(B),.N(N),.Z(Z),.add(add),.sub(sub),.aand(aand));

endmodule

// reg_blk done 
//update reg on the second clk cycle
module reg_blk (
	input rst,
    input [2:0] A_addr, B_addr, //if wb_en = 1, write into RB
    input [7:0] wb, //the value of wb
    input   clk, wb_en, 
    output [7:0] A, B
);

	
    reg [7:0] rf [0:7]; //Register file
    reg count;
	always @(*) begin
		if (rst == 0) begin
			rf[0] = 8'b00000000;
			rf[1] = 8'b00000000;
			rf[2] = 8'b00000000;
			rf[3] = 8'b00000000;
			rf[4] = 8'b00000000;
			rf[5] = 8'b00000000;
			rf[6] = 8'b00000000;
			rf[7] = 8'b00000000;
		end
	end
    assign A = rf[A_addr];
    assign B = rf[B_addr];

    always @(posedge clk) begin
        if (count == 1'b0) begin
            count <= 1'b1;
        end
        else begin
            if (wb_en) begin //if wb is enabled
                rf[B_addr] <= wb;
            end
            count <= 1'b0;
        end
        
    end


endmodule

// ALU done
module alu (
    input [7:0] A, B,
    output N, Z, 
    output [7:0] add, sub, aand
);
assign add = B+A;
assign sub = B-A;
assign aand = B & A;
assign Z = ~(A[0] | A[1] | A[2] | A[3] | A[4] | A[5] | A[6] | A[7]);
assign N = A[7];
endmodule

// instruction reg done
module instruction_reg(
		input [7:0]	instructionin,
        input clk,
		output	[1:0]	opcode,
		output	[2:0]	oprand,
		output	[2:0]	A_addr,B_addr,
        output [7:0]  instructionout,
        output  freeza
);
	reg	[2:0]	ooprand = 3'b000, AA_addr = 3'b000, BB_addr = 3'b000;
    reg [1:0] oopcode = 2'b00;
    reg freeze = 1'b0;
    reg count = 1'b0;
    reg [7:0] temp;
    //reg [7:0] temp1;

	always @(posedge clk) begin   //add clock here. It is better to have regester clocked here to make sure it will only store one number per cycle
        if  (count == 1'b0) begin
            temp <= instructionin; // temp store current instruction input and output the instruction even if the instructionin change for 2nd clock cycle
            if (freeze) begin
                //if ((pc != temp) & (pc != (temp+1'b1))) begin
                    freeze <= 1'b0;  //at this time oopcode, baadd, poprand still same as last cycle
                //end
            end

            else if (~freeze) begin //if freeze is 0, proceed as normal
                oopcode <= instructionin[7:6]; 
                BB_addr <= instructionin[2:0];
                if	(opcode == 2'b11) begin
                    ooprand <= instructionin[5:3];
                    if (ooprand == 3'b111) begin
                        freeze <= 1'b1;
                        //temp = pc;
                    end
                end
                else begin
                    AA_addr = instructionin[5:3];
                end
            end 
            count <= 1'b1;
        end
        else begin
            count <= 1'b0;
        end

    end
    assign instructionout= temp;
    assign  freeza = freeze;
	assign	opcode = oopcode;
	assign	B_addr = BB_addr;
    assign	A_addr = AA_addr;
	assign 	oprand = ooprand;
	// work on ctrl unit
endmodule

module control(
	input	[1:0]	opcode,
	input	[2:0]	oprand,
	
	input	N, Z, freeza,
	output we_bar, re_bar, ram_en_bar, wb_en, br,
    output [2:0] sel
);


    reg re_barr = 1'b1, we_barr = 1'b1, ram_en_barr = 1'b1, wbb_en = 1'b0, brr = 1'b0; /* initialized signals to 0 */
    reg [2:0] sell;
 	always @(*) begin
        brr = 1'b0; //reset branch to 0
		if	(opcode == 2'b00) begin //MOVE: RA -> RB
			re_barr = 1'b1;
			we_barr = 1'b1;
			ram_en_barr = 1'b1;
			wbb_en = 1'b1;
            sell = 3'b000;
		end
		else if	(opcode == 2'b01) begin //LOAD: M[RA] -> RB
			re_barr = 1'b0;
			we_barr = 1'b1;	
			ram_en_barr = 1'b0;
			wbb_en = 1'b1;
            sell = 3'b001;
		end
		else if (opcode == 2'b10) begin //STORE: RB -> M[RA]
			re_barr = 1'b1;
			we_barr = 1'b0;
			ram_en_barr = 1'b0;
			//MAddrr = ra; /* fix me */
			//MDataa = rb;
			wbb_en = 1'b0;
		end
		else if (opcode == 2'b11) begin //ALU
			re_barr = 1'b1;
			we_barr = 1'b1;
			wbb_en = 1'b1;
			ram_en_barr = 1'b1;
            case (oprand)
                3'b000: begin // and
                    wbb_en = 1'b1;
                    sell = 3'b010;
                end
                3'b001: begin // add
                    wbb_en = 1'b1;
                    sell = 3'b011;
                end
                3'b010: begin //sub
                    wbb_en = 1'b1;
                    sell = 3'b100;
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
                    sell = 3'b101;
                    brr = 1;
                end

                3'b111: begin //ldi
                    if (freeza) begin
                        wbb_en = 0;
                    end
                    else  begin
                        wbb_en = 1'b1;
                        sell = 3'b110;
                    end
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

endmodule

module pc(
	input rst,
	input	br, clk, 
    input [7:0] A, B,
	output [7:0] pc_out,
    output [7:0] pc_to_wb  //write back pc+1 to wb
	

);
	reg [7:0] pctemp, outt, pc_to_wbb, pc_c;
	reg [7:0] pc = 8'b00000000; /* initialized to 0 */
    reg count;
	always @(*) begin
		if (rst == 0) begin
			pc = 8'b00000000;
		end
		if (br == 1'b0) begin /* deleted semi colon here */
			pc_c = pc + 1'b1; //increment
		end
		else begin
			pc_c = B;
	    end
    end
	
	always @(posedge clk) begin
        if (count == 1'b0) begin
            pc <= pc_c;
            pc_to_wbb <= pc+1'b1;
            outt <= pc;
            count <= 1'b1;
        end
        else begin
            outt <= A;
            count <= 1'b0;
        end
	end
    assign pc_to_wb = pc_to_wbb;
    assign pc_out = outt;
	//assign pc_to_wb = pctemp;
endmodule