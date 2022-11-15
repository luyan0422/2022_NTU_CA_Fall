module ALU(
    clk,
    rst_n,
    valid,
    ready,
    mode,
    in_A,
    in_B,
    out
);

    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input  [1:0]  mode; // mode: 0: mulu, 1: divu, 2: shift, 3: avg
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 3'd0;
    parameter MUL  = 3'd1;
    parameter DIV  = 3'd2;
    parameter SHIFT = 3'd3;
    parameter AVG = 3'd4;
    parameter OUT  = 3'd5;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
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
                    case(mode)
                        2'b00: state_nxt = MUL;
                        2'b01: state_nxt = DIV;
                        2'b10: state_nxt = SHIFT;
                        2'b11: state_nxt = AVG;
                    endcase
                end
                else 
                    state_nxt = IDLE;
            end
            MUL :begin
                if(counter_nxt == 31)
                    state_nxt = OUT;
                else 
                    state_nxt = MUL;
            end
            DIV :begin
                if(counter_nxt == 31)
                    state_nxt = OUT;
                else  
                    state_nxt = DIV;
            end
            SHIFT : state_nxt = OUT;
            AVG : state_nxt = OUT;
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
            DIV : begin
                counter = counter_nxt + 1;    
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
            DIV : begin                
                if({1'b0, shreg_nxt[62 : 31]} >= alu_in) begin
                    alu_out = {1'b0, shreg_nxt[62 : 31]} - alu_in;
                    msb = 1'b1;
                end
                else begin
                    alu_out = {1'b0, shreg_nxt[62 : 31]};
                    msb = 1'b0;
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
            DIV : begin                
                shreg = {alu_out[31:0], shreg_nxt[30 : 0], msb} ;   
            end
            SHIFT : begin
                shreg = {32'b0, (shreg_nxt[31:0] >> alu_in[2:0])};                
            end
            AVG :begin                
                shreg[63:32] = {32'b0} ;
                shreg[31:0] = shreg_nxt[31:0] + (({1'b0, alu_in} - shreg_nxt[31:0]) >> 1) ;
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
                DIV :begin
                    counter_nxt <= counter ;
                    shreg_nxt <= shreg ;
                end
                SHIFT : begin
                    shreg_nxt <= shreg ;           
                end
                AVG :begin                
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