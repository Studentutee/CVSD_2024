`timescale 1ns/100ps
`define PERIOD 10.0
`define MAX_CYCLE 100000
`define RST_DELAY 2.0 

`ifdef I0
  `define IDATA "./pattern/INST0_I.dat"
  `define ODATA "./pattern/INST0_O.dat"
  `define PAT_LEN 40
`elsif I1
  `define IDATA "./pattern/INST1_I.dat"
  `define ODATA "./pattern/INST1_O.dat"
  `define PAT_LEN 40
`elsif I2
  `define IDATA "./pattern/INST2_I.dat"
  `define ODATA "./pattern/INST2_O.dat"
  `define PAT_LEN 40
`elsif I3
  `define IDATA "./pattern/INST3_I.dat"
  `define ODATA "./pattern/INST3_O.dat"
  `define PAT_LEN 40
`elsif I4
  `define IDATA "./pattern/INST4_I.dat"
  `define ODATA "./pattern/INST4_O.dat"
  `define PAT_LEN 40
`elsif I5
  `define IDATA "./pattern/INST5_I.dat"
  `define ODATA "./pattern/INST5_O.dat"
  `define PAT_LEN 40
`elsif I6
  `define IDATA "./pattern/INST6_I.dat"
  `define ODATA "./pattern/INST6_O.dat"
  `define PAT_LEN 40
`elsif I7
  `define IDATA "./pattern/INST7_I.dat"
  `define ODATA "./pattern/INST7_O.dat"
  `define PAT_LEN 40
`elsif I8
  `define IDATA "./pattern/INST8_I.dat"
  `define ODATA "./pattern/INST8_O.dat"
  `define PAT_LEN 40
`elsif I9
  `define IDATA "./pattern/INST9_I.dat"
  `define ODATA "./pattern/INST9_O.dat"
  `define PAT_LEN 40
`endif

module test_alu;
  parameter INST_W = 4;
  parameter INT_W = 6;
  parameter FRAC_W = 10;
  parameter DATA_W = INT_W + FRAC_W;

  reg        i_clk;
  reg        i_rst_n;      // async, low active
  reg        i_in_valid;
  reg [INST_W-1:0]  i_inst;
  reg [DATA_W-1:0] i_data_a;
  reg [DATA_W-1:0] i_data_b;
  wire         o_busy;
  wire         o_out_valid;
  wire  [DATA_W-1:0] o_data;

  reg [35:0] question[0:`PAT_LEN -1];
  reg [15:0] answer[0:`PAT_LEN -1];
  reg [35:0] fetch;
  integer outfile, error_counter, i, j;
alu u1(.*);

always #10 i_clk=~i_clk;

initial begin
  $dumpfile("wave.vcd");     // 1. 指定輸出的 VCD 檔名
  $dumpvars(0, test_alu);    // 2. 指定從哪個 module 開始記錄（0 表示全部階層）
  outfile=$fopen("testbench_report.txt");          //open one file for writing
  if(!outfile) begin
    $display("Can not write file!");
    $finish;
  end
  $readmemb(`IDATA,question);
  $readmemb(`ODATA,answer);
  
  error_counter = 0;
  
  i_clk = 1'b0;i_rst_n = 1'b1;i_in_valid = 0;i_data_a = 0;i_data_b = 0;
  #5 i_rst_n=1'b0;
  #5 i_rst_n=1'b1;
end

//Input data
initial begin 
  for(i = 0; i < `PAT_LEN; i++)begin
    #20
  i_in_valid = 1;
  fetch = question[i];
  i_inst = fetch[35:32];i_data_a = fetch[31:16];i_data_b = fetch[15:0];
  end
  #20
  i_in_valid = 0;
end

//Read output
initial begin
  #40
  for(j = 0; j < `PAT_LEN; j++)begin
      #20
      //$display("%b",answer[j]);
      if(o_data !== answer[j]) 
      begin
	//$display("Time = %0f ns", $realtime);
	$display("%d", j+1);
	$fdisplay(outfile,"Inst: %b, %b @ %b should be %b. But your output is %b.",question[j][35:32], question[j][31:16], question[j][15:0],answer[j],o_data);
	error_counter=error_counter+1;
      end
  end
  if(!error_counter)
       $display("\nCongratulations!! Your Verilog Code is correct!!\n");
     else
       $display("\nYour Verilog Code has %d errors. \nPlease read alu_out.txt for details.\n",error_counter);

  #10 $finish;
end
endmodule