// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I
    );
    //==== I/O Declaration ========================
    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;

    //==== Reg/Wire Declaration ===================
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    reg   [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg
    // ID
    wire signed [31:0] immediate; //imm's offset value
    // PC
    wire [1:0] pcNextState; //0: normal 1: pc+imm 2: x1+imm
    // normal PC (PC+4)
	wire [31:0] pcPlus4;
	// Branch/JAL/AUIPC  (PC + immediate)
	wire [31:0] pcPlusImm;
    // JALR (x1+immediate)
    wire [31:0] x1PlusImm;
    // Control (regWrite already claimed)
    wire Branch;
    wire MemRead;
    wire MemtoReg;
    wire [1:0] ALUOp; //-> connect to ALU control
	wire MemWrite;
	wire ALUSrc;
    // Mux decide ALU 的 input
    wire [31:0] InputTwo;
    // ALU control
    wire doMul;
	wire [1:0] ALUMode;
    // ALU
    wire [31:0] ALUOut;
    wire ZeroOut;
    // mulDiv
    wire [63:0] MULOut;
    wire mulReady;
    // ALU & MUL chose one outcome
    wire [31:0] Final;
    // BEQ
    wire doBranch;
    
    
    assign mem_addr_I = PC;
	assign rs1 = mem_rdata_I[19:15];
	assign rs2 = mem_rdata_I[24:20];
	assign rd = mem_rdata_I[11:7];
    //==== Submodule Connection ===================
    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//

    // Todo: other submodules

    // ID
	IMM_GEN imm_Gen(.Instruction(mem_rdata_I), .Immediate(immediate), .pcNextState(pcNextState)); //check instruction and pc_nxt
    // PC related
    PC_ADDER normalpc(.DataOne(32'd4), .DataTwo(PC), .Outcome(pcPlus4));
    PC_ADDER pcimm(.DataOne(PC), .DataTwo(immediate), .Outcome(pcPlusImm));
    PC_ADDER x1imm(.DataOne(rs1_data), .DataTwo(immediate), .Outcome(x1PlusImm));
    // Control
    CONTROL Control(.Opcode(mem_rdata_I[6:0]), .Branch(Branch), .MemRead(MemRead), .MemtoReg(MemtoReg), .ALUOp(ALUOp), .MemWrite(MemWrite), .ALUSrc(ALUSrc), .regWrite(regWrite));
    assign mem_wen_D = MemWrite;
    // EX
    // ALU control
	ALUControl ALUControl(.ALUOp(ALUOp), .Instruction(PC), .ALUMode(ALUMode),.doMul(doMul));
    MUX aluInput(.OptionOne(rs2_data), .OptionTwo(immediate), .SelectWire(ALUSrc), .Outcome(InputTwo));     //ALUSrc 0: rs2_data, 1: immediate
    ALU ALU(.InputOne(rs1_data), .InputTwo(InputTwo), .Mode(ALUMode), .ALUOut(ALUOut), .ZeroOut(Zreo));
    mulDiv mulDiv(.clk(clk), .rst_n(rst_n), .valid(doMul), .ready(mulReady), .in_A(rs1_data), .in_B(InputTwo), .out(MULOut));
    MUX aluOutDetermine(.OptionOne(ALUOut), .OptionTwo(MULOut[63:32]), .SelectWire(doMul), .Outcome(Final));  // doMul 0: ALUOut, 1: MULOut

    // BEQ
    AND branchDetect(.InputOne(Branch), .InputTwo(ZeroOut), .Outcome(doBranch));

    //==== Combinational Part =====================
    // Todo: any combinational/sequential circuit
    always @(pcPlus4 or pcPlusImm or x1PlusImm or PC or pcNextState)begin
        // if MUL stall
        if(doMul == 1'b1) begin
            if(mulReady == 1'b1)begin
                PC_nxt = pcPlus4;
            end
            else begin
                PC_nxt = PC;
            end
        end
        // rest
        else begin
            // noemal PC
            if (pcNextState == 2'd0)begin
                PC_nxt = pcPlus4;
            end
            // might be AUIPC/JAL/BEQ
            else if(pcNextState == 2'd1)begin
                if (doBranch == 1'd1) begin
                    PC_nxt = pcPlusImm;
                end
                else begin
                    PC_nxt = pcPlus4;
                end
            end
            // JALR
            else if (pcNextState == 2'd2)begin
                PC_nxt = x1PlusImm;
            end
            else begin
                PC_nxt = PC;
            end
        end 
    end

    // ME
    MUX PostALU(.OptionOne(0), .OptionTwo(rs2_data), .SelectWire(MemWrite), .Outcome(mem_wdata_D));     // MemWrite 0: dont write, 1: new memory data
	MUX DataAddr(.OptionOne(0), .OptionTwo(rs1_data + immediate), .SelectWire(MemWrite), .Outcome(mem_addr_D));   // MemWrite 0: dont write, 1: new memory address

	// WB
	MUX WB(.OptionOne(mem_rdata_D), .OptionTwo(Final), .SelectWire(MemtoReg), .Outcome(rd_data));   // MemtoReg 0: memory data, 1: Final output,


    //==== Sequential Part ========================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00400000; // Do not modify this value!!!
            
        end
        else begin
            PC <= PC_nxt;
        end
    end

endmodule

module PC_ADDER(DataOne, DataTwo, Outcome);
    input [31:0] DataOne;
    input [31:0] DataTwo;
    output [31:0] Outcome;
    reg signed [31:0] Outcome;

    always @(DataOne or DataTwo)begin
        Outcome = DataOne + DataTwo;
    end
endmodule

module CONTROL(Opcode, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, regWrite);
    input [6:0] Opcode;
	output Branch, MemRead, MemtoReg, MemWrite, ALUSrc, regWrite;
	output [1:0] ALUOp;
	reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, regWrite;
	reg [1:0] ALUOp;
    // ALUOp -> 00: auipc/sw/lw/J-type, 01: beq, 10: R-type, 11: addi/slti
    // ALUArc -> 0: reg, 1: immediate
    always@(*) begin
		case(Opcode)
            // ALIPC
            7'b0010111: begin
				//auipc
				Branch = 0;
				MemRead = 0;
				MemtoReg = 0;
				ALUOp = 0;      
				MemWrite = 0;
				ALUSrc = 1;
				regWrite = 1; 
			end
			// JAL
            7'b1101111: begin
				Branch = 0;
				MemRead = 0;
				MemtoReg = 0;
				ALUOp = 0;
				MemWrite = 0;
				ALUSrc = 1;
				regWrite = 1;       // rd 存 pc+4
			end
            // JALR
			7'b1100111: begin
				Branch = 0;
				MemRead = 0;
				MemtoReg = 0;
				ALUOp = 0;
				MemWrite = 0;
				ALUSrc = 1;
				regWrite = 1;       // rd 存 pc+4 
			end
            // BEQ
			7'b1100011: begin
				Branch = 1;
				MemRead = 0;
				MemtoReg = 0;
				ALUOp = 1;
				MemWrite = 0;
				ALUSrc = 0;
				regWrite = 0;
			end
            // LW
			7'b0000011: begin
				Branch = 0;
				MemRead = 1;
				MemtoReg = 1;
				ALUOp = 0;
				MemWrite = 0;
				ALUSrc = 1;
				regWrite = 1;
			end
            // SW
			7'b0100011: begin
				Branch = 0;
				MemRead = 0;
				MemtoReg = 0;
				ALUOp = 0;
				MemWrite = 1;
				ALUSrc = 1;
				regWrite = 0;
			end
            // ADDI & SLTI
			7'b0010011: begin
				Branch = 0;
				MemRead = 0;
				MemtoReg = 0;
				ALUOp = 3;
				MemWrite = 0;
				ALUSrc = 1;
				regWrite = 1;
			end
            // R-type (ADD, SUB, XOR, MUL)
			7'b0110011: begin
				Branch = 0;
				MemRead = 0;
				MemtoReg = 0;
				ALUOp = 2;
				MemWrite = 0;
				ALUSrc = 0;
				regWrite = 1;
			end
			default: begin
				Branch = 0;
				MemRead = 0;
				MemtoReg = 0;
				ALUOp = 0;
				MemWrite = 0;
				ALUSrc = 0;
				regWrite = 0;
			end
		endcase
	end
endmodule
// wen(regWrite),.a1(rs1),.a2(rs2),.aw(rd),.d(rd_data),.q1(rs1_data),.q2(rs2_data);   
module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth

    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0; // zero: hard-wired zero
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'h7fffeffc; // sp: stack pointer
                    32'd3: mem[i] <= 32'h10008000; // gp: global pointer
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end
    end
endmodule

module IMM_GEN(Instruction, Immediate, pcNextState);
    input [31:0] Instruction;  //mem_rdata_I (32-bit instruction)
    output [31:0] Immediate;
    output [1:0] pcNextState; 
    reg signed [31:0] Immediate;
    reg [1:0] pcNextState;
    //when new IC comein
    
    always @(Instruction) begin
        Immediate[31:0] = 32'b0;
        // AUIPC
        if(Instruction[6:0] == 7'b0010111) begin
            Immediate[11:0] = 12'b0;
            Immediate[31:12] = Instruction[31:12];
            pcNextState = 2'd1;
        end
        // JAL
        else if(Instruction[6:0] == 7'b1101111) begin
            Immediate[0] = 1'b0;
            Immediate[10:1] = Instruction[30:21];
            Immediate[11] = Instruction[20];
            Immediate[19:12] = Instruction[19:12];
            Immediate[20] = Instruction[31];
            //signed extension
            if(Immediate[20] == 1'b1) begin
                Immediate[31:21] = 11'b1;
            end
            else if(Immediate[20] == 1'b0) begin
                Immediate[31:21] = 11'b0;
            end
            pcNextState = 2'd1;
        end
        // JALR
        else if(Instruction[6:0] == 7'b1100111) begin
            Immediate[11:0] = Instruction[31:20];
            //signed extension
            if(Immediate[11] == 1'b1) begin
                Immediate[31:12] = 20'b1;
            end
            else if(Immediate[11] == 1'b0)begin
                Immediate[31:12] = 20'b0;
            end
            pcNextState = 2'd2;
        end
        // BEQ -> check opcode and funct3
        else if(Instruction[6:0] == 7'b1100011 && Instruction[14:12] == 3'b000) begin
            Immediate[0] = 1'b0;
            Immediate[4:1] = Instruction[11:8];
            Immediate[10:5] = Instruction[30:25];
            Immediate[11] = Instruction[7];
            Immediate[12] = Instruction[31];
            //signed extension
            if(Immediate[12] == 1'b1) begin
                Immediate[31:13] = 19'b1;
            end
            else if(Immediate[12] == 1'b0)begin
                Immediate[31:13] = 19'b0;
            end
            pcNextState = 2'd1;
        end
        // LW -> check opcode and funct3
        else if(Instruction[6:0] == 7'b0000011 && Instruction[14:12] == 3'b010) begin
            Immediate[11:0] = Instruction[31:20];
            //signed extension
            if(Immediate[11] == 1'b1) begin
                Immediate[31:12] = 20'b1;
            end
            else if(Immediate[11] == 1'b0)begin
                Immediate[31:12] = 20'b0;
            end
            pcNextState = 2'd0;
        end
        // SW -> check opcode and funct3
        else if(Instruction[6:0] == 7'b0100011 && Instruction[14:12] == 3'b010) begin
            Immediate[4:0] = Instruction[11:7];
            Immediate[11:5] = Instruction[31:25];
            //signed extension
            if(Immediate[11] == 1'b1) begin
                Immediate[31:12] = 20'b1;
            end
            else if(Immediate[11] == 1'b0)begin
                Immediate[31:12] = 20'b0;
            end
            pcNextState = 2'd0;
        end
        // ADDI -> check opcode and funct3
        else if(Instruction[6:0] == 7'b0010011 && Instruction[14:12] == 3'b000) begin
            Immediate[11:0] = Instruction[31:20];
            //signed extension
            if(Immediate[11] == 1'b1) begin
                Immediate[31:12] = 20'b1;
            end
            else if(Immediate[11] == 1'b0)begin
                Immediate[31:12] = 20'b0;
            end
            pcNextState = 2'd0;
        end
        // SLTI -> check opcode and funct3
        else if(Instruction[6:0] == 7'b0010011 && Instruction[14:12] == 3'b010) begin
            Immediate[11:0] = Instruction[31:20];
            //signed extension
            if(Immediate[11] == 1'b1) begin
                Immediate[31:12] = 20'b1;
            end
            else if(Immediate[11] == 1'b0)begin
                Immediate[31:12] = 20'b0;
            end
            pcNextState = 2'd0;
        end
        // ADD & SUB & XOR & MUL
        else if(Instruction[6:0] == 7'b0110011)begin
            Immediate[31:0] = 32'b0;
            pcNextState = 2'd0;
        end
        else begin
            Immediate[31:0] = 32'b0;
            pcNextState = 2'd0;
        end
    end
endmodule

module ALUControl(ALUOp, Instruction, ALUMode, doMul);  //depend on Control's wire ALUOp 
    // ALUOp -> 00: auipc/sw/lw/J-type, 01: beq, 10: R-type, 11: addi/slti
    // ALUMode -> 0: add, 1: sub, 2: mul, 3: xor
    input [1:0] ALUOp;
    input [31:0] Instruction;
    output [1:0] ALUMode; 
    output doMul;
    reg [1:0] ALUMode;
	reg doMul;
    always @(*)begin
        case(ALUOp)
            2'b00:begin 
                ALUMode = 0; //auipc/sw/lw/J-type
            end
            2'b01:begin
                ALUMode = 1; //sub
            end
            2'b10:begin
                case(Instruction[14:12])
                    3'b000: begin
                        if(Instruction[30] == 1) ALUMode = 1; //SUB
                        else begin
                            if(Instruction[25] == 1)ALUMode = 2; //MUL
                            else ALUMode = 0; //ADD
                        end
                    end
                    3'b100:begin
                        ALUMode = 3; // XOR
                    end
                    default: ALUMode = 0;
                endcase
            end
            2'b11:begin
                case(Instruction[14:12])
                    3'b000: ALUMode = 0;// ADDI
                    3'b010: ALUMode = 1;// SLTI
                    default: ALUMode = 0;
                endcase
            end
            default: ALUMode = 0;
        endcase
        // if is MUL go to mulDiv
        if(ALUMode == 2) doMul = 1;
        else doMul = 0;
    end
endmodule

module MUX(OptionOne, OptionTwo, SelectWire, Outcome);
    input [31:0] OptionOne;
    input [31:0] OptionTwo;
    input SelectWire;
    output [31:0] Outcome;
    reg [31:0] Outcome;

    always @(*)begin
        if(SelectWire == 1'b0) Outcome = OptionOne;
        else Outcome = OptionTwo;
    end
endmodule

module ALU(InputOne, InputTwo, Mode, ALUOut, ZeroOut);
    input [31:0] InputOne;
    input [31:0] InputTwo;
    input [1:0] Mode;
    output ZeroOut; // for BEQ
    output [31:0] ALUOut;
    
    reg ZeroOut;
    reg [31:0] tmp;

    parameter ADD = 2'd0;
    parameter SUB = 2'd1;
    parameter MUL  = 2'd2;
    parameter XOR  = 2'd3;

    assign ALUOut = tmp;

    always @(*) begin
        case(Mode)
            ADD: tmp = InputOne + InputTwo;
            SUB: tmp = InputOne - InputTwo;
            XOR: tmp = InputOne ^ InputTwo;
            default: tmp = 32'b0;
        endcase
        if(tmp == 32'b0) ZeroOut = 1;
        else ZeroOut = 0;
    end
endmodule

module mulDiv(clk, rst_n, valid, ready, in_A, in_B, out);
    // Todo: your HW2
    // Definition of ports
    input         clk, rst_n;
    input         valid;
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 2'd0;
    parameter MUL  = 2'd1;
    parameter OUT  = 2'd2;

    // Todo: Wire and reg if needed
    reg  [ 1:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;
      
    // Todo: Instatiate any primitives if needed
    reg  [ 0:0] msb;

    // Todo 5: Wire assignments    
    assign ready = (state == OUT) ;
    assign out = shreg ;

    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if(valid) begin
                    state_nxt = MUL;
                end
                else begin
                    state_nxt = IDLE;
                end
            end
            MUL :begin
                if(counter_nxt == 31)
                    state_nxt = OUT;
                else 
                    state_nxt = MUL;
            end
            OUT : state_nxt = IDLE;
            default :state_nxt = IDLE;
        endcase
    end

    // Todo 2: Counter
    always @(*) begin        
        case(state)
            MUL: begin
                counter = counter_nxt + 1 ;              
            end
            default :begin
                counter = 5'b0;
            end 
        endcase        
    end

    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            MUL : begin
                msb = shreg_nxt[0];
                if(msb)begin
                    alu_out = alu_in + shreg_nxt[63 : 32];
                end
                else begin
                    alu_out = {1'b0, shreg_nxt[63 : 32]};
                end    
            end
            default :begin
                msb = 1'b0;
                alu_out = alu_in;
            end
        endcase
    end
    
    // Todo 4: Shift register
    always @(*) begin
        alu_in = alu_in_nxt ;
        case(state)
            MUL : begin
                shreg = {alu_out, shreg_nxt[31 : 1]};
            end           
            OUT :begin
                shreg = shreg_nxt ;
            end
            default :shreg = 64'b0;
        endcase
    end

    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end
        else begin
            state <= state_nxt;
            case(state)
                IDLE :begin
                    counter_nxt <= counter ;
                    if (valid) begin
                        shreg_nxt <= {32'b0, in_A};
                    end
                    else begin
                        shreg_nxt <= 64'b0;     
                    end
                end
                MUL :begin
                    counter_nxt <= counter ;
                    shreg_nxt <= shreg ;
                end  
                default : begin
                    counter_nxt <= counter ;
                    shreg_nxt <= shreg_nxt ;
                end
            endcase
            
        end
    end   
endmodule

module AND(InputOne, InputTwo, Outcome);
    input InputOne;
    input InputTwo;
    output Outcome;
    reg Outcome;
    always @(InputOne or InputTwo) 
    begin
        Outcome = (InputOne && InputTwo) ;
    end
endmodule