module alu #(
  parameter INST_W = 4,
  parameter INT_W = 6,
  parameter FRAC_W = 10,
  parameter DATA_W = INT_W + FRAC_W
)(
  input  wire        i_clk,
  input  wire        i_rst_n,      // async, low active
  input  wire        i_in_valid,
  input  wire [INST_W-1:0]  i_inst,
  input  wire [DATA_W-1:0] i_data_a,
  input  wire [DATA_W-1:0] i_data_b,
  output reg         o_busy = 1'b0,
  output reg         o_out_valid,
  output reg  [DATA_W-1:0] o_data
  );
  //input reg
  reg [3:0]  inst_r;
  reg [15:0] a_r, b_r;
  wire [31:0] a_r_a_r = {a_r,a_r}; //not reg is very importent!!!
  wire signed [15:0] a_s = a_r; //a_r just a reg, unsigned
  wire signed [15:0] b_s = b_r;
  
  reg  signed [31:0] mul_tmp; //16bits x 16bits
  reg  signed [19:0] acc_mem [0:15]; //Inetermediate value are guaranteed not to exceed 20 bits(see ppt)
  reg  signed [19:0] acc_tmp;
  
  // Softplus instance (piece-wise)
  wire [15:0] softplus_out;
  reg signed [16:0] adder_out;
  reg signed [16:0] substractor_out;
  reg signed [31:0] round_bias;
  reg signed [21:0] mul32_rounding22_out;
  
  // combinational result
  reg signed [15:0] result_w;
  
  softplus_lin u_soft(.x(a_s), .y(softplus_out)); //softplus_lin module named u_soft
  reg o_out_valid_buffer;
 
//==============================================
//Reset and input (inst & data) reg
//==============================================
  integer i;
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
	inst_r <= 4'd0;
	a_r    <= 16'd0;
	b_r    <= 16'd0;
	// reset acc_memory
	if (!i_rst_n) for (i=0;i<16;i=i+1) acc_mem[i] <= 20'd0; //Has fixed by myself
    end 
    else if (i_in_valid && !o_busy) begin
	inst_r <= i_inst;
	a_r    <= i_data_a;
	b_r    <= i_data_b;
    end
end

//==============================================
//Finite State Machine for working state (waitng, executing or outputting)
//I found FSM is useless here
//==============================================
/*  localparam S_IDLE = 2'd0, //Idle—ready for new command
             S_EXE  = 2'd1, //Execute current instruction
             //S_OUT  = 2'd2; //Assert output valid for 1 cycle

  reg [1:0] state, nstate;//present state, nest state
  //reg [1:0] exe_cnt; //counter

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) state <= S_IDLE; //reset
    else          state <= nstate;
end

always @(*) begin
    case (state) // Next stste set
	S_IDLE: nstate = (i_in_valid && !o_busy) ? S_EXE : S_IDLE;
	S_EXE : nstate = (exe_cnt == latency(inst_r)) ? S_OUT : S_EXE;
	//S_OUT : nstate = S_IDLE;
	default: nstate = S_IDLE;
    endcase
end
*/
//===============================================
//exe time don't care
// latency table
/*function [1:0] latency;
    input [3:0] op;
    begin
	case (op)
	    4'b0010: latency = 2'd1; // mul extra 1 cycle
	    4'b0100: latency = 2'd2; // softplus 2 cycles
	    default: latency = 2'd0;
	endcase
    end
endfunction*/

// busy & exe_cnt
/*always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
	o_busy   <= 1'b0;
	exe_cnt  <= 2'd0;
    end else begin
	case (state)
	    S_IDLE: begin
		//o_busy  <= (i_in_valid)? 1'b1 : 1'b0;
		exe_cnt <= 2'd0;
	    end
	    S_EXE: begin
		//o_busy  <= 1'b1;
		exe_cnt <= exe_cnt + 1'b1;
	    end
	    S_OUT: begin
		//o_busy  <= 1'b0;
		exe_cnt <= 2'd0;
	    end
	endcase
    end
end*/
//===============================================

//=============================================
//Operation Unit
//=============================================

// saturate helper //For over-flow
function signed [15:0] sat_6_10;
    input signed [21:0] in22;     // 已 round 且 >>>10 對齊
    begin
        if      (in22 >  22'sd32767) sat_6_10 = 16'h7FFF; // +31.999
        else if (in22 <  -22'sd32768) sat_6_10 = 16'h8000; // –32.000
        else                             sat_6_10 = in22[15:0];
    end
endfunction

// Exec result
always @(*) begin
    case (inst_r)
	4'b0000: begin // add
	    adder_out = a_s + b_s; //17 bits = 16bits + 16bits;
	    result_w = sat_6_10(adder_out); //Although input just 17bits, it wil be automatci extended(?
	    end
	4'b0001: begin // sub
	    substractor_out = a_s - b_s;
	    result_w = sat_6_10(substractor_out); 
	    end
	4'b0010: begin                                             // mul
	    mul_tmp = a_s * b_s; // 32-bit
	    round_bias = mul_tmp >= 0 ? 32'sd512 : 32'sd511;
	    mul32_rounding22_out = (mul_tmp + round_bias) >>> 10; //+ mul_tmp[9] for rounding X //but if negative? 512 511 balance
	    result_w = sat_6_10(mul32_rounding22_out); 
	end
	4'b0011: begin                                             // acc  
	    //$display("acc_mem[%d]+b_s = %b + %b", a_r[3:0], acc_mem[a_r[3:0]], b_s);
	    acc_tmp = acc_mem[a_r[3:0]] + b_s; //lenth different??
	    //$display("acc_tmp: %b",acc_tmp);
	    result_w = sat_6_10(acc_tmp);
	    //$display("Result: %b",result_w);
	    //acc_mem[a_r[3:0]] = acc_tmp; loop!!!!
	end
	4'b0100: result_w = softplus_out;                           // softplus
	4'b0101: result_w = a_r ^ b_r;                              // xor
	4'b0110: result_w = a_s >>> b_r[4:0];                       // sar
	4'b0111: result_w = a_r_a_r[(31 - b_r[4:0]) -: 16]; // ror //check the grammar
	4'b1000: begin                                             // clz
	    result_w = 16'd16; //avoid Latch
	    for (i=15;i>=0;i=i-1)
		if (a_r[i]==1'b1) begin
		  result_w = 15-i;
		  i = -1;
		end
	end
	4'b1001: begin                                             // reverse-match-4
	    result_w = 16'd0;
	    for (i=0;i<13;i=i+1)
		result_w[i] = (a_r[i +:4] == b_r[15-i -:4]);
	end
	default: result_w = 16'd0;
    endcase
end
//=============================================================
// Output (ins:0011 data fresh)
//=============================================================

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
	o_out_valid_buffer <= 1'b0;
	o_out_valid <= 1'b0;
	o_data      <= 16'd0;
    end 
    else begin
	o_out_valid_buffer <= 1'b1;
	o_out_valid <= o_out_valid_buffer;
	o_data <= result_w;
	// write-back for ACC
	if (inst_r==4'b0011) acc_mem[a_r[3:0]] <= acc_tmp;
    end
end

endmodule
 
