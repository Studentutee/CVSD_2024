`include "fp_add32.v"
module core #( // DO NOT MODIFY INTERFACE!!!
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32
) ( 
    input wire i_clk,
    input wire i_rst_n,

    // Testbench IOs
    output reg [2:0] o_status, 
    output reg       o_status_valid,

    // Memory IOs
    output reg [ADDR_WIDTH-1:0] o_addr,
    output reg [DATA_WIDTH-1:0] o_wdata,
    output reg                  o_we,
    input wire [DATA_WIDTH-1:0] i_rdata
);

//---------------------------------------------------------------------
// Local parameters – **keep in sync with define.v**
//---------------------------------------------------------------------
localparam R_TYPE_SUCCESS = 3'd0,
           I_TYPE_SUCCESS = 3'd1,
           S_TYPE_SUCCESS = 3'd2,
           B_TYPE_SUCCESS = 3'd3,
           INVALID_TYPE   = 3'd4,
           EOF_TYPE       = 3'd5;

// FSM states (follow the TA hint) -----------------------------------
localparam S_IDLE     = 3'd0,
           S_FETCH    = 3'd1,
           S_DECODE   = 3'd2,
           S_EXECUTE  = 3'd3,
           S_WRITEBK  = 3'd4,
           S_NEXTPC   = 3'd5,
           S_HALT     = 3'd6;

//ALU CMD
localparam  ALU_ADD     = 5'd0,
            ALU_SUB     = 5'd1,
            ALU_ADDI    = 5'd2,
            ALU_LW      = 5'd3,
            ALU_SW      = 5'd4,
            ALU_BEQ     = 5'd5,
            ALU_BLT     = 5'd6,
            ALU_SLT     = 5'd7,
            ALU_SLL     = 5'd8,
            ALU_SRL     = 5'd9,
            ALU_FADD    = 5'd10, //a
            ALU_FSUB    = 5'd11, //b
            ALU_FLW     = 5'd12, //c
            ALU_FSW     = 5'd13, //d
            ALU_FCLASS  = 5'd14, //e
            ALU_FLT     = 5'd15, //f
            ALU_EOF     = 5'd16; //10

//FCLASS
localparam  Negative_infinite           = 32'd0,
            Negative_normal_number      = 32'd1,
            Negative_subnormal_number   = 32'd2,
            Negative_zero               = 32'd3,
            Positive_zero               = 32'd4,
            Positive_subnormal_number   = 32'd5,
            Positive_normal_number      = 32'd6,
            Positive_infinite           = 32'd7,
            NaN                         = 32'd8;
// ---------------------------------------------------------------------------
// Wires and Registers
// ---------------------------------------------------------------------------
// ---- Add your own wires and registers here if needed ---- //
reg [2:0]  cur_state, nxt_state;
reg [31:0] pc;                               // program counter


//register file
reg signed [31:0] int_reg [0:31];           // x0–x31 (x0 hard‑wired to 0)
reg        [31:0] fp_reg  [0:31];           // f0–f31 (IEEE‑754 single)

// Decoded instruction fields ----------------------------------------
reg [6:0]  opcode;
reg [4:0]  rd_fd, r1_f1, r2_f2;
reg [2:0]  funct3;
reg [6:0]  funct7;
reg [31:0] imm;   //需要這麼長？？                           // sign‑extended immediate

reg [4:0] alu_cmd; //ALU CMD
reg [4:0] alu_cmd_r; //ALU CMD Reg
reg [31:0] alu_result; //alu_result
reg [31:0] alu_result_r; //alu_result_reg
reg invalid_flag; 
reg [2:0] status_next; 
integer i;

reg [31:0] fp_in_a, fp_in_b, fp_out_sum;
reg fp_in_add_or_sub;
reg fp_out_invalid;

fp_add32_mod dut (
.in_a   (fp_in_a),
.in_b   (fp_in_b),
.add_sub(fp_in_add_or_sub),
.sum_o  (fp_out_sum),
.invalid_o(fp_out_invalid)
);

//---------------------------------------------------------------------
// Utility functions --------------------------------------------------
//---------------------------------------------------------------------
function automatic overflow_add;
    input [31:0] a, b, sum; //sum is adding result
    begin
        overflow_add = (~a[31] & ~b[31] &  sum[31]) | (a[31] & b[31] & ~sum[31]);
    end
endfunction
function automatic overflow_sub;      // 1 = overflow
    input [31:0] a, b, diff;          // diff = a - b
    begin
        overflow_sub = (a[31] ^ b[31]) & (a[31] ^ diff[31]);
    end
endfunction

//---------------------------------------------------------------------
// Asynchronous reset & FSM
//---------------------------------------------------------------------
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        cur_state       <= S_IDLE;
        pc              <= 32'b0;

        // simple loop to clear rest (power‑on‑reset only)
        // NOTE: synthesis will convert to reset logic.
        
        for (i = 0; i < 32; i = i + 1) begin
            int_reg[i] <= 32'b0;
            fp_reg [i] <= 32'b0;
        end
    end else begin
        cur_state      <= nxt_state;

        /*case (nxt_state) //decide what will output when next stage
            S_IDLE: begin
            end
            S_FETCH: begin
                
            end
            S_DECODE: begin
            end
            S_EXECUTE: begin
            end
            S_WRITEBK: begin
            end
            S_NEXTPC: begin
            end
            default: begin
                // No action needed for other states
            end
        endcase*/
        case (cur_state) //decide what will output when finish stage
            S_IDLE: begin
            end
            S_FETCH: begin
                invalid_flag <= 0;//改成在變更pc時就先檢查是否invalid ((pc >> 2) > 32'd1023); //invalid_flag reset, and check if pc over instruction area
            end
            S_DECODE: begin
                rd_fd  <= i_rdata[11:7];
                r1_f1  <= i_rdata[19:15];
                r2_f2  <= i_rdata[24:20];
                if(alu_cmd == ALU_EOF) begin//EOF
                    status_next <= EOF_TYPE;
                end
                else if(alu_cmd == ALU_ADDI | alu_cmd == ALU_LW |  alu_cmd == ALU_FLW) begin//I
                    imm <= {{20{i_rdata[31]}},i_rdata[31:20]};
                    status_next <= I_TYPE_SUCCESS; 
                end
                else if(alu_cmd == ALU_SW | alu_cmd == ALU_FSW) begin //S
                    imm <= {{20{i_rdata[31]}},i_rdata[31:25],i_rdata[11:7]};
                    status_next <= S_TYPE_SUCCESS; 
                end
                else if(alu_cmd == ALU_BEQ | alu_cmd == ALU_BLT)begin //B
                    imm <= {{20{i_rdata[31]}},i_rdata[7],i_rdata[30:25],i_rdata[11:8],1'b0};
                    status_next <= B_TYPE_SUCCESS;
                end 
                else begin //R
                    status_next <= R_TYPE_SUCCESS;
                end

                alu_cmd_r <= alu_cmd;
            end
            S_EXECUTE: begin
                case(alu_cmd_r)
                    ALU_ADD: begin
                        alu_result_r <= alu_result;
                        invalid_flag <= overflow_add(int_reg[r1_f1],int_reg[r2_f2],alu_result);
                    end
                    ALU_SUB: begin
                        alu_result_r <= alu_result;
                        invalid_flag <= overflow_sub(int_reg[r1_f1],int_reg[r2_f2],alu_result);
                    end
                    ALU_ADDI: begin
                        alu_result_r <= alu_result;
                        invalid_flag <= overflow_add(int_reg[r1_f1],imm,alu_result);
                    end
                    ALU_LW: begin
                        invalid_flag <= (alu_result < 32'd4096);
                    end
                    ALU_SW: begin
                        alu_result_r <= alu_result;
                        invalid_flag <= (alu_result < 32'd4096);
                    end
                    ALU_BEQ, ALU_BLT:begin
                        alu_result_r <= alu_result;
                        invalid_flag <= (alu_result > 32'd4095);//((pc >> 2) > 32'd1023);
                    end
                    ALU_SLT, ALU_SLL, ALU_SRL: begin
                        alu_result_r <= alu_result;
                        invalid_flag <= 1'b0;
                    end
                    ALU_FADD, ALU_FSUB: begin
                        alu_result_r <= alu_result;
                        invalid_flag <= fp_out_invalid;
                    end
                    ALU_FLW: begin
                        invalid_flag <= (alu_result < 32'd4096);
                    end
                    ALU_FSW: begin
                        alu_result_r <= alu_result;
                        invalid_flag <= (alu_result < 32'd4096);
                    end
                    ALU_FCLASS: begin
                        alu_result_r <= alu_result;
                        invalid_flag <= 1'b0;
                    end
                    ALU_FLT: begin
                        if(alu_result[31] == 1'b0 & alu_result[30:0] != 31'b0)
                            alu_result_r <= 32'b1;
                        else
                            alu_result_r <= 32'b0;
                        invalid_flag <= fp_out_invalid;
                    end
                    default: begin
                    end
                endcase
            end
            S_WRITEBK: begin
                case(alu_cmd_r)
                    ALU_ADD, ALU_SUB, ALU_ADDI: begin
                        int_reg[rd_fd] <= alu_result_r;
                    end
                    ALU_LW: begin
                        int_reg[rd_fd] <= i_rdata;
                    end
                    ALU_SW: begin
                    end
                    ALU_BEQ, ALU_BLT: begin
                        pc <= alu_result_r;
                    end
                    ALU_SLT, ALU_SLL, ALU_SRL: begin
                        int_reg[rd_fd] <= alu_result_r;
                    end
                    ALU_FADD, ALU_FSUB: begin
                        fp_reg[rd_fd] <= alu_result_r;
                    end
                    ALU_FLW: begin
                        fp_reg[rd_fd] <= i_rdata;
                    end
                    ALU_FSW: begin
                    end
                    ALU_FCLASS, ALU_FLT: begin
                        int_reg[rd_fd] <= alu_result_r; //應該是寫回整數暫存器吧？作業沒講清楚
                    end 
                    default:begin
                    end
                endcase
            end
            S_NEXTPC: begin
                // Next PC is already set in the combinational logic
                pc <= pc + 32'd4;

            end
            default: begin
                // No action needed for other states
            end
        endcase
    end
end

always @(*) begin
    // Default assignments
    nxt_state       = cur_state;
    
    // output ot memory
    o_addr          = 32'b0;
    o_we            = 1'b0;
    o_wdata         = 32'b0;
    // output to testbench
    o_status        = 3'b0;
    o_status_valid  = 1'b0;

    alu_result      = 32'b0;

    opcode      = i_rdata[6:0];
    funct3      = i_rdata[14:12];
    funct7      = i_rdata[31:25];

    alu_cmd     = 5'b0;

    fp_in_a = 32'b0;
    fp_in_b = 32'b0;
    fp_in_add_or_sub = 1'b0;

    //這樣些雖然看起來比較簡潔？但如果define那邊改了呢？還有到時候合成的時候dc也會幫我優化吧？
    //用default的話，是不是會影響布林代數簡化(don't care 沒辦法靈活運用？)
    /*case (opcode)
        //add, sub, slt, sll, srl
        `OP_ADD, `OP_SUB, `OP_SLT, `OP_SLL, `OP_SRL:begin //R-type 7'b0110011
            case(funct3)
                `FUNCT3_ADD,`FUNCT3_SUB:    alu_cmd = (funct7==`FUNCT7_ADD) ?  4'd0 : 4'd1;
                `FUNCT3_SLT:    alu_cmd = 4'd7;
                `FUNCT3_SLL:    alu_cmd = 4'd8;
                default:        alu_cmd = 4'd9;
            endcase
        end
        //fadd, fsub, fclass, flt
        `OP_FADD, `OP_FSUB, `OP_FCLASS, `OP_FLT:begin //R-type 7'b1010011
            case(funct3)
                `FUNCT3_FADD:   alu_cmd = 4'd10;
                `FUNCT3_FSUB:   alu_cmd = 4'd11;
                `FUNCT3_FCLASS: alu_cmd = 4'd14;
                default:        alu_cmd = 4'd15;
            endcase
        end
        //addi, lw, flw
        `OP_ADDI:begin //I-type 7'b0010011
            alu_cmd = 4'd2;
        end
        `OP_LW:begin //I-type 7'b0000011
            alu_cmd = 4'd3;
        end   
        `OP_FLW:begin //I-type 7'b0000111
            alu_cmd = 4'd12;
        end
        //sw, fsw
        `OP_SW, `OP_FSW:begin //S-type
            case(funct3)
                `FUNCT3_SW:   alu_cmd = 4'd4;
                default:      alu_cmd = 4'd12;
            endcase
        end 
        //beq, blt
        `OP_BEQ, `OP_BLT:begin //B-type

        end 
        default:begin //EOF

        end
    endcase */

    case (cur_state)
        //-----------------------------------------------------------------
        S_IDLE: begin
            o_addr          = 32'b0;
            o_we            = 1'b0;
            o_wdata         = 32'b0;
            o_status_valid  = 1'b0;
            o_status        = 3'b0;
            if (1'b1) nxt_state = S_FETCH;
        end
        //-----------------------------------------------------------------
        S_FETCH: begin
            o_we      = 1'b0;
            o_addr    = pc;
            nxt_state = S_DECODE;
        end
        //-----------------------------------------------------------------
        S_DECODE: begin
            if(opcode == `OP_ADD & funct3 == `FUNCT3_ADD & funct7 == `FUNCT7_ADD) //R-type
                alu_cmd = ALU_ADD;
            else if(opcode == `OP_SUB & funct3 == `FUNCT3_SUB & funct7 == `FUNCT7_SUB) //R-type
                alu_cmd = ALU_SUB;
            else if(opcode == `OP_ADDI & funct3 == `FUNCT3_ADDI) //I-type
                alu_cmd = ALU_ADDI;
            else if(opcode == `OP_LW & funct3 == `FUNCT3_LW) //I-type
                alu_cmd = ALU_LW;
            else if(opcode == `OP_SW & funct3 == `FUNCT3_SW) //S-type
                alu_cmd = ALU_SW;
            else if(opcode == `OP_BEQ & funct3 == `FUNCT3_BEQ) //B-type
                alu_cmd = ALU_BEQ;
            else if(opcode == `OP_BLT & funct3 == `FUNCT3_BLT) //B-type
                alu_cmd = ALU_BLT;
            else if(opcode == `OP_SLT & funct3 == `FUNCT3_SLT & funct7 == `FUNCT7_SLT) //R-type
                alu_cmd = ALU_SLT;
            else if(opcode == `OP_SLL & funct3 == `FUNCT3_SLL & funct7 == `FUNCT7_SLL) //R-type
                alu_cmd = ALU_SLL;
            else if(opcode == `OP_SRL & funct3 == `FUNCT3_SRL & funct7 == `FUNCT7_SRL) //R-type
                alu_cmd = ALU_SRL;
            else if(opcode == `OP_FADD & funct3 == `FUNCT3_FADD & funct7 == `FUNCT7_FADD) //R-type
                alu_cmd = ALU_FADD;
            else if(opcode == `OP_FSUB & funct3 == `FUNCT3_FSUB & funct7 == `FUNCT7_FSUB) //R-type
                alu_cmd = ALU_FSUB;
            else if(opcode == `OP_FLW & funct3 == `FUNCT3_FLW) //I-type
                alu_cmd = ALU_FLW;
            else if(opcode == `OP_FSW & funct3 == `FUNCT3_FSW) //S-type
                alu_cmd = ALU_FSW;
            else if(opcode == `OP_FCLASS & funct3 == `FUNCT3_FCLASS & funct7 == `FUNCT7_FCLASS) //R-type
                alu_cmd = ALU_FCLASS;
            else if(opcode == `OP_FLT & funct3 == `FUNCT3_FLT & funct7 == `FUNCT7_FLT) //R-type
                alu_cmd = ALU_FLT;
            else //EOF
                alu_cmd = ALU_EOF;
            nxt_state = S_EXECUTE;
        end
        //-----------------------------------------------------------------
        S_EXECUTE: begin
            case(alu_cmd_r)
                ALU_ADD: begin
                    alu_result = int_reg[r1_f1] + int_reg[r2_f2];
                end
                ALU_SUB: begin
                    alu_result = int_reg[r1_f1] - int_reg[r2_f2];
                end
                ALU_ADDI: begin
                    alu_result = int_reg[r1_f1] + imm;
                end
                ALU_LW: begin
                    o_we       = 1'b0;
                    alu_result = int_reg[r1_f1] + imm;
                    o_addr     = alu_result;
                end
                ALU_SW: begin
                    alu_result = int_reg[r1_f1] + imm;
                end
                ALU_BEQ: begin
                    alu_result = (int_reg[r1_f1] == int_reg[r2_f2]) ? pc + imm : pc + 4; 
                end
                ALU_BLT: begin
                    alu_result = (int_reg[r1_f1] < int_reg[r2_f2]) ? pc + imm : pc + 4;
                end
                ALU_SLT: begin
                    alu_result = (int_reg[r1_f1] < int_reg[r2_f2]) ? 1'b1 : 1'b0;
                end
                ALU_SLL: begin
                    alu_result = int_reg[r1_f1] << int_reg[r2_f2];
                end
                ALU_SRL: begin
                    alu_result = int_reg[r1_f1] >> int_reg[r2_f2];
                end
                ALU_FADD: begin
                    fp_in_a = fp_reg[r1_f1];
                    fp_in_b = fp_reg[r2_f2];
                    fp_in_add_or_sub = 1'b0;
                    alu_result = fp_out_sum;
                end
                ALU_FSUB: begin
                    fp_in_a = fp_reg[r1_f1];
                    fp_in_b = fp_reg[r2_f2];
                    fp_in_add_or_sub = 1'b1;
                    alu_result = fp_out_sum;
                end
                ALU_FLW: begin
                    o_we       = 1'b0;
                    alu_result = int_reg[r1_f1] + imm;
                    o_addr     = alu_result;
                end
                ALU_FSW: begin
                    alu_result = int_reg[r1_f1] + imm;
                end
                ALU_FCLASS: begin
                    if(fp_reg[r1_f1][30:23] == 8'd255) begin//e=255
                        if(fp_reg[r1_f1][22:0] == 23'd0) begin//m=0
                            if(fp_reg[r1_f1][31] == 1'd0) //+INF
                                alu_result = Positive_infinite;
                            else
                                alu_result = Negative_infinite;                            
                        end
                        else //m != 0
                            alu_result = NaN;                           
                    end 
                    else if(fp_reg[r1_f1][30:23] == 8'd0) begin//e=0
                        if(fp_reg[r1_f1][22:0] == 23'd0) begin//m=0
                            if(fp_reg[r1_f1][31] == 1'd0) //+Zero
                                alu_result = Positive_zero;
                            else
                                alu_result = Negative_zero;                            
                        end
                        else begin//m!=0
                            if(fp_reg[r1_f1][31] == 1'd0)
                                alu_result = Positive_subnormal_number;
                            else
                                alu_result = Negative_subnormal_number;    
                        end
                    end
                    else begin // e !=255, e != 0
                        if(fp_reg[r1_f1][31] == 1'd0)
                                alu_result = Positive_normal_number;
                            else
                                alu_result = Negative_normal_number;  
                    end
                end
                ALU_FLT: begin
                    fp_in_a = fp_reg[r2_f2];
                    fp_in_b = fp_reg[r1_f1];
                    fp_in_add_or_sub = 1'b1;
                    alu_result = fp_out_sum;
                end
                default: begin

                end
            endcase
            nxt_state = S_WRITEBK;
        end
        //-----------------------------------------------------------------
        S_WRITEBK: begin
            nxt_state = S_NEXTPC;
            o_status_valid = 1'b1;
            o_status = status_next;

            if(invalid_flag) begin //invalid_flag will not happen as ALU_EOF, ALU_SW, ALU_BEQ...
                o_status = INVALID_TYPE;
            end
            else if(alu_cmd_r == ALU_EOF) begin
                nxt_state = S_HALT;
            end
            else if(alu_cmd_r == ALU_SW) begin //Write Back to Memory
                o_addr          = alu_result_r;
                o_we            = 1'b1;
                o_wdata         = int_reg[r2_f2];
            end
            else if(alu_cmd_r == ALU_BEQ | alu_cmd_r == ALU_BLT) begin //Write Back to Memory
                nxt_state = S_FETCH; //Jump over S_NEXTPC
            end
            else if(alu_cmd_r == ALU_FSW) begin //Write Back to Memory
                o_addr          = alu_result_r;
                o_we            = 1'b1;
                o_wdata         = fp_reg[r2_f2];
            end
            else begin
                
            end

            
        end
        //-----------------------------------------------------------------
        S_NEXTPC: begin
  
            nxt_state = S_FETCH;
        end
        //-----------------------------------------------------------------
        S_HALT: begin
            // Stay here – testbench will finish.
            nxt_state = S_HALT;
        end
    endcase
end

endmodule