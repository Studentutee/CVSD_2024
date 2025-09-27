`timescale 1ns/1ps
`define CYCLE       5.0     // CLK period.
`define HCYCLE      (`CYCLE/2)
`define MAX_CYCLE   10000000
`define RST_DELAY   2


`ifdef tb1
    `define INFILE "../00_TESTBED/PATTERN/indata1.dat"
    `define OPFILE "../00_TESTBED/PATTERN/opmode1.dat"
    `define GOLDEN "../00_TESTBED/PATTERN/golden1.dat"
`elsif tb2
    `define INFILE "../00_TESTBED/PATTERN/indata2.dat"
    `define OPFILE "../00_TESTBED/PATTERN/opmode2.dat"
    `define GOLDEN "../00_TESTBED/PATTERN/golden2.dat"
`elsif tb3
    `define INFILE "../00_TESTBED/PATTERN/indata3.dat"
    `define OPFILE "../00_TESTBED/PATTERN/opmode3.dat"
    `define GOLDEN "../00_TESTBED/PATTERN/golden3.dat"
`elsif tb4
    `define INFILE "../00_TESTBED/PATTERN/indata4.dat"
    `define OPFILE "../00_TESTBED/PATTERN/opmode4.dat"
    `define GOLDEN "../00_TESTBED/PATTERN/golden4.dat"
`else
    `define INFILE "../00_TESTBED/PATTERN/indata0.dat"
    `define OPFILE "../00_TESTBED/PATTERN/opmode0.dat"
    `define GOLDEN "../00_TESTBED/PATTERN/golden0.dat"
`endif

// Modify your sdf file name
`define SDFFILE "../02_SYN/Netlist/core_syn.sdf"


module testbed;

reg        clk, rst_n;
reg        op_valid = 0;
reg [ 3:0] op_mode = 0;
wire        op_ready;
reg        in_valid = 0;
reg [ 7:0] in_data = 0;
wire        in_ready;
wire        out_valid;
wire [13:0] out_data;

reg  [ 7:0] indata_mem [0:2047];
reg  [ 3:0] opmode_mem [0:1023];
reg  [13:0] golden_mem [0:4095];


// ==============================================
// TODO: Declare regs and wires you need
// ==============================================
reg [10:0]  indata_mem_cnt = 0;
reg [9:0]   opmode_mem_cnt = 0;
reg [11:0]  golden_mem_cnt = 0;
integer     i = 0;
integer     j = 0;
integer     err_cnt = 0;
integer     read_mode_cnt = 0;


// For gate-level simulation only
`ifdef SDF
    initial $sdf_annotate(`SDFFILE, u_core);
    initial #1 $display("SDF File %s were used for this simulation.", `SDFFILE);
`endif

// Write out waveform file
initial begin
    $fsdbDumpfile("core.fsdb");
    $fsdbDumpvars(0, "+mda");
    //$fsdbDumpMDA(1, testbed.u_core.GEN_SRAM[0].u_sram.mem);
end


core u_core (
	.i_clk       (clk),
	.i_rst_n     (rst_n),
	.i_op_valid  (op_valid),
	.i_op_mode   (op_mode),
    .o_op_ready  (op_ready),
	.i_in_valid  (in_valid),
	.i_in_data   (in_data),
	.o_in_ready  (in_ready),
	.o_out_valid (out_valid),
	.o_out_data  (out_data)
);

// Read in test pattern and golden pattern
initial $readmemb(`INFILE, indata_mem);
initial $readmemb(`OPFILE, opmode_mem);
initial $readmemb(`GOLDEN, golden_mem);

// Clock generation
initial clk = 1'b0;
always begin #(`CYCLE/2) clk = ~clk; end

// Reset generation
initial begin
    rst_n = 1; # (               0.25 * `CYCLE);
    rst_n = 0; # ((`RST_DELAY - 0.25) * `CYCLE);
    rst_n = 1; # (         `MAX_CYCLE * `CYCLE);
    $display("Error! Runtime exceeded!");
    $finish;
end


// ==============================================
// TODO: Check pattern after process finish
// ==============================================
always @(negedge clk)begin
    if(rst_n == 1)begin
        if(op_ready == 1)begin
            while(op_ready) @(negedge clk);
            op_valid = 1;
            op_mode = opmode_mem[opmode_mem_cnt];
            opmode_mem_cnt = opmode_mem_cnt + 1;
            @(negedge clk);
            op_valid = 0;
            if(op_mode == 0)begin
                in_valid = 1;
                while(i<2048) begin
                    in_data = indata_mem[i];                   
                    @(negedge clk);
                    if(in_ready == 1) i = i + 1;//indata == 0的話要重傳一次
                end
                in_valid = 0;
            end
            if(read_mode_cnt > 1) begin
                finish_display;
                $finish;
            end
        end
        
    end
end

always @(negedge clk) begin
    if(out_valid == 1)begin
        if(out_data != golden_mem[j])begin
            err_cnt = err_cnt + 1;
        end
    end
end

    task finish_display;
	    begin
	        if(err_cnt == 0) begin
	            $display("⠄⠄⠄⠄⠄⠄⠄⠈⠉⠁⠈⠉⠉⠙⠿⣿⣿⣿⣿⣿");    
	            $display("⠄⠄⠄⠄⠄⠄⠄⠄⣀⣀⣀⠄⠄⠄⠄⠄⠹⣿⣿⣿");    
	            $display("⠄⠄⠄⠄⠄⢐⣲⣿⣿⣯⠭⠉⠙⠲⣄⡀⠄⠈⢿⣿");    
	            $display("⠐⠄⠄⠰⠒⠚⢩⣉⠼⡟⠙⠛⠿⡟⣤⡳⡀⠄⠄⢻");    
	            $display("⠄⠄⢀⣀⣀⣢⣶⣿⣦⣭⣤⣭⣵⣶⣿⣿⣏⠄⠄⣿");    
	            $display("⠄⣼⣿⣿⣿⡉⣿⣀⣽⣸⣿⣿⣿⣿⣿⣿⣿⡆⣀⣿");    
	            $display("⢠⣿⣿⣿⠿⠟⠛⠻⢿⠿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣼");    
	            $display("⠄⣿⣿⣿⡆⠄⠄⠄⠄⠳⡈⣿⣿⣿⣿⣿⣿⣿⣿⣿");    
	            $display("⠄⢹⣿⣿⡇⠄⠄⠄⠄⢀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿");    
	            $display("⠄⠄⢿⣿⣷⣨⣽⣭⢁⣡⣿⣿⠟⣩⣿⣿⣿⠿⠿⠟");    
	            $display("⠄⠄⠈⡍⠻⣿⣿⣿⣿⠟⠋⢁⣼⠿⠋⠉⠄⠄⠄⠄");    
	            $display("⠄⠄⠄⠈⠴⢬⣙⣛⡥⠴⠂⠄⠄⠄⠄⠄⠄⠄⠄⠄");
	            $display("-----------------------------------------------");
		        $display("||             Simulation Correct!           ||");
		        $display("-----------------------------------------------\n");
	        end
	        else begin
	            $display("⠄⠄⠄⠄⠄⠄⠄⠄⠄⢀⡤⠒⢁⣨⣴⣶⣿⣿⣿⣷⣦⣥⡂⠄⠄⠄⠄⠄⠄⠄⠄⠄⠄");    
	            $display("⠄⠄⠄⠄⠄⠄⠄⢀⠜⠁⠄⢰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⠠⠄⠄⠄⠄⠄⠄⠄⠄");    
	            $display("⠄⠄⠄⠄⠄⠄⠄⡈⠄⠄⠄⢺⣿⣿⣿⣿⣛⢛⣛⣽⣿⣿⣿⣿⠇⠁⠄⠄⠄⠄⠄⠄⠄");    
	            $display("⠄⠄⠄⠄⠄⠄⠄⡇⠄⠄⠄⣽⣿⣿⣿⣿⣿⡿⠿⢿⣻⣿⣿⣧⣦⡄⡀⠄⠄⠄⠄⠄⠄");    
	            $display("⠄⠄⠄⠄⠄⠄⠄⡧⠄⣀⠄⣸⣿⣿⣿⣿⣶⣶⣲⣾⣿⣿⣿⣤⣤⣘⡇⠄⠄⠄⠄⠄⠄");    
	            $display("⠄⠄⠄⠄⠄⠄⠸⣾⡗⠛⣷⣿⣿⣿⣿⣿⣿⣿⣿⢿⣿⠿⣿⣿⣿⣿⡇⠄⠄⠄⠄⠄⠄");    
	            $display("⠄⠄⠄⠄⠄⠄⠄⢻⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣶⣤⣌⣿⣿⡇⠄⠄⠄⠄⠄⠄");    
	            $display("⠄⠄⠄⠄⠄⠄⠄⠄⠙⡟⠛⢻⣿⣿⣿⣿⣿⣿⠋⠉⠄⠄⠄⠄⢸⣿⠁⠄⠄⠄⠄⠄⠄");    
	            $display("⠄⠄⠄⠄⠄⠄⠄⠄⠄⢸⣴⠄⠉⢿⣿⣿⣿⣿⣿⣆⠙⠛⠄⣰⣿⠇⠄⠄⠄⠄⠄⠄⠄");    
	            $display("⠄⠄⠄⠄⠄⠄⢀⠠⠊⠰⠿⠦⠄⠄⠈⠛⠻⠿⢿⣿⢿⣷⣾⣟⠋⠄⠄⠄⠄⠄⠄⠄⠄");    
	            $display("⠄⠄⠿⠿⠿⠿⠿⠿⣿⠿⠄⠄⠄⢸⣇⣀⣶⣼⣟⣴⣂⠄⠄⢱⡶⣶⠻⡟⣿⢸⡟⣷⠄");
	            $display("⠄⠄⢰⡶⠶⢶⡆⠄⣿⠄⠄⠄⢰⣿⡏⢫⡽⢿⡿⠿⣭⠄⠄⢸⡇⣿⢸⡿⣿⢸⣧⡏⠄");
	            $display("⠄⠄⢸⣧⣤⣼⡇⠄⣿⠄⠄⠄⠈⢸⡇⣰⡿⣶⣷⢾⡷⠄⠄⢸⣧⣿⢾⡷⣿⢸⡇⣿⠄");    
	            $display("⠄⠄⠘⠃⠄⠄⣤⣤⣿⠄⠄⠄⠄⢸⡇⣩⡿⠓⠿⢾⡷⠄⠄⠘⠃⢀⡾⢁⣿⢸⡟⠋⠄");
	            $display("-----------------------------------------------");
		        $display("||      %d   Simulation Wrong!             ||", err_cnt);
		        $display("-----------------------------------------------\n");
	        end
	        
	        $finish;
	    end
	endtask
endmodule
