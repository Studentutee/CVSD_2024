module core #(
    parameter   SRAM_SIZE = 512, //word, remember to check SRAM inctance
                SRAM_SIZE_BITS = 9, //log_2(SRAM_SIZE)
                SRAM_COUNT = 4, //Number of SRAM
                SRAM_COUNT_BITS = 2 //log_2(SRAM_COUNT)
)(
    input  wire        i_clk,
    input  wire        i_rst_n,            // async active-low
    input  wire        i_op_valid,         // one-cycle pulse carrying op
    input  wire [3:0]  i_op_mode,          // opcode
    input  wire        i_in_valid,         // pixel input valid (LOAD only)
    input  wire [7:0]  i_in_data,          // pixel data (unsigned)
    output reg         o_op_ready,         // pulse when ready for next op
    output reg         o_in_ready,         // ready to accept input (LOAD)
    output reg         o_out_valid,        // streaming output valid
    output reg signed [13:0] o_out_data          // signed result / zero-extended pixel
);

	// -----------------------------
	// Parameters & constants
	// -----------------------------
	localparam 	IMG_W   = 8,
				IMG_H   = 8,
				IMG_C   = 32,
				IMG_PIX = IMG_W*IMG_H*IMG_C; // 2048

	// Depth encoding: 0->8, 1->16, 2->32
	localparam 	DEPTH_8  = 2'd0,
				DEPTH_16 = 2'd1,
				DEPTH_32 = 2'd2;

	// Opcodes
	localparam 	OP_LOAD    = 4'b0000,
				OP_ORG_R   = 4'b0001,
				OP_ORG_L   = 4'b0010,
				OP_ORG_U   = 4'b0011,
				OP_ORG_D   = 4'b0100,
				OP_SCALE_D = 4'b0101,
				OP_SCALE_U = 4'b0110,
				OP_DISPLAY = 4'b0111,
				OP_CONV    = 4'b1000,
				OP_MEDIAN  = 4'b1001,
				OP_SOBEL   = 4'b1010;

    // -----------------------------
    // Global state / registers
    // -----------------------------
    typedef enum logic [4:0] { //enum: defines a set of named constants (enumeration).
        S_RESET     = 5'd0, //localparam S_RESET    = 5'd0;
        S_START     = 5'd1,
        S_O_OP_READY= 5'd2,
        S_WAIT_OP   = 5'd3, //localparam S_WAIT_OP  = 5'd1;
        S_LOAD      = 5'd4,
        S_ORG_R     = 5'd5,
        S_ORG_L     = 5'd6,
        S_ORG_U     = 5'd7,
        S_ORG_D     = 5'd8,
        S_SCALE_D   = 5'd9,
        S_SCALE_U   = 5'd10,
        S_DISPLAY   = 5'd11,
        S_CONV      = 5'd12,
    
        S_MED_PREP  = 5'd18,
        S_MED_SORT  = 5'd19,
        S_SOBEL_ACC = 5'd20,
        S_SOBEL_NMS = 5'd21,
        S_STREAM    = 5'd22,
        S_PULSE_RDY = 5'd23
    } state_t;

    state_t state;
    state_t state_next;

    // 宣告一個 3×3 的常數矩陣
    localparam [2:0] GAUSS_KERNEL [2:0][2:0] = '{
        '{3'd0, 3'd1, 3'd0},  // row 0: (ky=0)
        '{3'd1, 3'd2, 3'd1},  // row 1: (ky=1)
        '{3'd0, 3'd1, 3'd0}   // row 2: (ky=2)
    };

    // -----------------------------
    // Negege-sampled inputs (spec p.6)
    // -----------------------------
    reg        i_op_valid_r;
    reg [3:0]  i_op_mode_r;
    reg        i_in_valid_r;
    reg [7:0]  i_in_data_r;
    //reg         o_op_ready_r;         // pulse when ready for next op
    //reg         o_in_ready_r;         // ready to accept input (LOAD)
    //reg         o_out_valid_r;        // streaming output valid
    //reg signed [13:0] o_out_data_r;   

    // Origin & depth
    reg [2:0] origin_x, origin_y; // 0..7, but window requires <=6
    reg [2:0] origin_x_r, origin_y_r;
    reg [1:0] depth_sel;           // DEPTH_8/16/32 (default 32)
    reg [1:0] depth_sel_r;

    // Helper: decode numeric depth
    function automatic [4:0] depth_value(input [1:0] dsel);
        case (dsel)
            DEPTH_8:  depth_value = 5'd7;
            DEPTH_16: depth_value = 5'd15;
            default:  depth_value = 5'd31;
        endcase
    endfunction

    // -----------------------------
    // Banked image memory: SRAM_COUNT × (SRAM_SIZE × 8)
    // Index mapping: addr = (c*64) + (y*8 + x)
    // bank = addr[10:8], off = addr[7:0]
    // -----------------------------

    wire [7:0] sram_q [SRAM_COUNT-1:0]; // Data Outputs (Q[0] = LSB)
    reg        sram_cen; // Chip Enable
    reg  [SRAM_COUNT-1:0] sram_wen;
    reg  [8:0] sram_a [SRAM_COUNT-1:0]; //Addresses (A[0] = LSB)
    reg  [7:0] sram_d [SRAM_COUNT-1:0]; // Data Inputs (D[0] = LSB)
    reg  [SRAM_COUNT_BITS-1:0] sram_select; // Which SRAM to use for current op
    reg  [SRAM_COUNT_BITS-1:0] sram_select_r;
    reg  [8:0] sram_addr; // Address within selected SRAM
    reg        sram_out_valid;
    reg        sram_out_valid_r;

    genvar gi;
    generate
    for (gi=0; gi<SRAM_COUNT; gi=gi+1) begin: GEN_SRAM
        sram_512x8 u_sram (
            .CLK(i_clk),
            .CEN(sram_cen),
            .WEN(sram_wen[gi]),
            .A  (sram_a[gi]),
            .D  (sram_d[gi]),
            .Q  (sram_q[gi])
        );
    end
    endgenerate

    // Address helper, change x,y,c to raster-scan order
    function automatic [10:0] RS_order2mem_addr(input [10:0] RS_order);
        RS_order2mem_addr = {RS_order[5+SRAM_COUNT_BITS:6], RS_order[10:6+SRAM_COUNT_BITS], RS_order[5:0]}; //10~9 select Mem, 8~0 select Mem_address
    endfunction
    function automatic [10:0] xyc2mem_addr(input [2:0] x, input [2:0] y, input [4:0] c);
        xyc2mem_addr = {c[SRAM_COUNT_BITS-1:0], c[4:SRAM_COUNT_BITS], y*6'd8 + x};//10~9 select Mem, 8~0 select Mem_address
    endfunction

    // -----------------------------
    // LOAD engine
    // -----------------------------
    reg [10:0] load_cnt; // 0..2047
    reg [10:0] load_cnt_r;

    // -----------------------------
    // DISPLAY streamer counters (channel-major after 2×2 raster)
    // -----------------------------
    reg [4:0]  disp_c;   // up to 32, 控制正在輸出的 channel
    reg [4:0]  disp_c_r;
    reg [1:0]  disp_yx;  // 0:(0,0) 1:(1,0) 2:(0,1) 3:(1,1), 控制在該 channel 裡的 2×2 小 tile 座標
    reg [1:0]  disp_yx_r;
    reg        disp_fsm; 
    reg        disp_fsm_r;


    // -----------------------------
    // CONV accumulator (process 1 output pixel at a time → 4 outputs)
    // -----------------------------
    reg [1:0]   conv_yx;      // which of 4 outputs we are accumulating  
    reg [1:0]   conv_yx_r;
    reg [1:0]   conv_ky, conv_kx; // 0..2 for kernel loops
    reg [1:0]   conv_ky_r, conv_kx_r;
    reg         conv_kernel_done;
    reg [12:0]  conv_acc_4x4 [3:0][3:0];
    reg [12:0]  conv_acc_4x4_r [3:0][3:0];
    reg [16:0]  conv_acc;
    reg [16:0]  conv_acc_r;
    reg [1:0]   conv_fsm;
    reg [1:0]   conv_fsm_r;
    reg [3:0]   conv_scan_point_x;
    reg [3:0]   conv_scan_point_x_r;
    reg [3:0]   conv_scan_point_y;
    reg [3:0]   conv_scan_point_y_r;
    reg [1:0]   conv_x_4x4_cnt;
    reg [1:0]   conv_x_4x4_cnt_r;
    reg [1:0]   conv_x_4x4_cnt_r_delay1;
    reg [1:0]   conv_y_4x4_cnt;
    reg [1:0]   conv_y_4x4_cnt_r;
    reg [1:0]   conv_y_4x4_cnt_r_delay1;
    reg [4:0]   conv_c_4x4_cnt;       // channel index for accumulation
    reg [4:0]   conv_c_4x4_cnt_r;

    //integer i;
    //integer j;

    // Next-state (combinational) for simple stream datapath
    always @(*) begin
            state_next = state;
            o_op_ready = 1'b0;
            o_in_ready = 1'b0;
            sram_cen = 1'b0; // default disable
            sram_wen = 4'b1111; // default read
            sram_select = sram_select_r;
            sram_out_valid = sram_out_valid_r;
            sram_addr = 9'd0;
            
            load_cnt   = load_cnt_r;
            origin_x   = origin_x_r;
            origin_y   = origin_y_r;

            depth_sel  = depth_sel_r;

            disp_fsm   = disp_fsm_r;
            disp_yx    = disp_yx_r;
            disp_c     = disp_c_r;

            conv_fsm    = conv_fsm_r;
            conv_yx     = conv_yx_r;
            conv_ky     = conv_ky_r;
            conv_kx     = conv_kx_r;
            conv_acc    = conv_acc_r;
            conv_scan_point_x = conv_scan_point_x_r;
            conv_scan_point_y = conv_scan_point_y_r;
            conv_x_4x4_cnt = conv_x_4x4_cnt_r;
            conv_y_4x4_cnt = conv_y_4x4_cnt_r;
            conv_c_4x4_cnt = conv_c_4x4_cnt_r;
            conv_acc_4x4 = conv_acc_4x4_r;
            conv_kernel_done = 1'b0;

            for (int i = 0; i < SRAM_COUNT; i = i + 1) begin
                sram_a[i] = 9'd0;   // 嘗試清零
                sram_d[i] = 8'd0; // 嘗試清零
            end
        case (state)
            S_RESET: begin
                state_next = S_START;
            end
            S_START: begin
                state_next = S_O_OP_READY;
            end
            S_O_OP_READY: begin
                o_op_ready = 1'b1;
                state_next = S_WAIT_OP;
            end
            S_WAIT_OP: begin
                if (i_op_valid_r) begin                
                    unique case (i_op_mode_r)
                        OP_LOAD: begin
                            // begin loading 2048 bytes
                            state_next = S_LOAD;
                        end
                        OP_ORG_R : begin
                            state_next = S_ORG_R;
                        end
                        OP_ORG_L : begin
                            state_next = S_ORG_L;
                        end
                        OP_ORG_U : begin
                            state_next = S_ORG_U;
                        end
                        OP_ORG_D : begin
                            state_next = S_ORG_D;
                        end
                        OP_SCALE_D: begin
                            state_next = S_SCALE_D;
                        end
                        OP_SCALE_U: begin
                            state_next = S_SCALE_U;
                        end
                        OP_DISPLAY: begin
                            //disp_c  <= 6'd0;
                            //disp_yx <= 2'd0;
                            state_next = S_DISPLAY;
                        end
                        OP_CONV: begin
                            //conv_yx  <= 2'd0; conv_c_4x4_cnt <= 6'd0; conv_ky<=2'd0; conv_kx<=2'd0; conv_acc<=24'd0;
                            state_next = S_CONV;
                        end
                        OP_MEDIAN: begin
                            //med_ch <= 2'd0; med_xy <= 2'd0;
                            state_next = S_MED_PREP;
                        end
                        OP_SOBEL: begin
                            //sob_ch <= 2'd0; sob_xy <= 2'd0;
                            state_next = S_SOBEL_ACC; // compute sram_center first
                        end
                        default: begin
                            // ignore undefined opcodes
                            //o_op_ready <= 1'b1; // next op please
                        end
                    endcase
                end
            end
            S_LOAD: begin
                o_in_ready = 1'b1;
                
                {sram_select, sram_addr} = RS_order2mem_addr(load_cnt_r);
                sram_wen[sram_select] = 1'b0; // write enable
                sram_a[sram_select]   = sram_addr;
                sram_d[sram_select]   = i_in_data_r;
                if (i_in_valid_r) begin
                    if (load_cnt_r == IMG_PIX-1) begin                                                     
                        state_next  = S_O_OP_READY;
                        load_cnt = 11'd0;
                    end else begin
                        load_cnt  = load_cnt_r + 11'd1;
                    end
                end

            end
            S_ORG_R: begin
                if (origin_x_r < 3'd6) origin_x = origin_x_r + 3'd1;
                state_next = S_O_OP_READY;
            end
            S_ORG_L: begin
                if (origin_x_r > 3'd0) origin_x = origin_x_r - 3'd1;
                state_next = S_O_OP_READY;
            end
            S_ORG_U: begin
                if (origin_y_r > 3'd0) origin_y = origin_y_r - 3'd1;
                state_next = S_O_OP_READY;
            end
            S_ORG_D: begin
                if (origin_y_r < 3'd6) origin_y = origin_y_r + 3'd1;
                state_next = S_O_OP_READY;
            end
            S_SCALE_D: begin
                if (depth_sel_r > DEPTH_8) depth_sel = depth_sel_r - 2'd1;
                state_next = S_O_OP_READY;
            end
            S_SCALE_U: begin
                if (depth_sel_r < DEPTH_32) depth_sel = depth_sel_r + 2'd1;
                state_next = S_O_OP_READY;
            end
            S_DISPLAY: begin
                //{sram_select, sram_addr} = xyc2mem_addr(origin_x_r + disp_yx_r[0], origin_y_r + disp_yx_r[1], disp_c_r);
                if (disp_yx_r == 2'd0) begin
                    {sram_select, sram_addr} = xyc2mem_addr(origin_x_r + 3'd0, origin_y_r + 3'd0, disp_c_r);
                end else if (disp_yx_r == 2'd1) begin
                    {sram_select, sram_addr} = xyc2mem_addr(origin_x_r + 3'd1, origin_y_r + 3'd0, disp_c_r);
                end else if (disp_yx_r == 2'd2) begin
                    {sram_select, sram_addr} = xyc2mem_addr(origin_x_r + 3'd0, origin_y_r + 3'd1, disp_c_r);
                end else begin
                    {sram_select, sram_addr} = xyc2mem_addr(origin_x_r + 3'd1, origin_y_r + 3'd1, disp_c_r);
                end
                //sram_wen[sram_select] = 1'b1; // read
                sram_a[sram_select]   = sram_addr;

                case (disp_fsm_r)
                    1'd0: begin
                        sram_out_valid = 1'b1;
                        disp_yx = disp_yx_r + 2'd1;
                        if (disp_yx_r == 2'd3) begin
                            if (disp_c_r < depth_value(depth_sel_r)) begin
                                disp_c = disp_c_r + 5'd1;
                            end else begin
                                disp_fsm = 1'd1;
                                disp_c = 5'd0;
                            end
                        end
                    end
                    1'd1: begin
                        sram_out_valid = 1'b0;
                        if (!sram_out_valid_r) begin
                            disp_fsm = 2'd0;
                            state_next = S_O_OP_READY;
                        end
                    end
                endcase
            end
            S_CONV: begin
                //sram_addr寫在這裡的話會被刷新成0
                case (conv_fsm_r)
                    2'd0: begin
                        //reset//
                        conv_kernel_done = 1'b0;
                        conv_scan_point_x = origin_x - 4'd1; //if origin_x=000(unsigned), conv_scan_point_x <= 0000 - 0001 = 1111(unsigned)
                        conv_scan_point_y = origin_y - 4'd1;
                        conv_x_4x4_cnt = 2'd0;
                        conv_y_4x4_cnt = 2'd0;
                        conv_c_4x4_cnt = 5'd0;
                        conv_yx = 2'd0;           
                        conv_ky = 2'd0;
                        conv_kx = 2'd0;
                        conv_acc = 24'd0;
                        conv_acc = 17'd0;
                        for (int i=0; i<4; i=i+1) begin
                            for (int j=0; j<4; j=j+1) begin
                                conv_acc_4x4[i][j] = 'd0;//(SRAM_COUNT_BITS+8)'d0;
                            end
                        end

                        // //first read//
                        // //刻意使用沒有_r的conv_scan_point_x，提前在這一輪read sram
                        // {sram_select, sram_addr} = xyc2mem_addr(conv_scan_point_x[2:0], conv_scan_point_y[2:0], conv_c_4x4_cnt);
                        // if(conv_scan_point_x > 3'd7 || conv_scan_point_y > 3'd7)begin //zeropadding
                        //     //這一輪本來就會reset，所以不需要conv_acc_4x4[conv_y_4x4_cnt][conv_x_4x4_cnt] = 'd0;//(SRAM_COUNT_BITS+8)'d0;
                        //     sram_out_valid = 1'b0;
                        //     conv_x_4x4_cnt = conv_x_4x4_cnt_r + 2'd1;
                        // end else begin
                        //     conv_c_4x4_cnt = conv_c_4x4_cnt_r + 5'd4;
                        //     sram_out_valid = 1'b1;
                        // end  
                        conv_fsm = 2'd1;
                    end
                    2'd1: begin
                        {sram_select, sram_addr} = xyc2mem_addr(conv_scan_point_x_r[2:0], conv_scan_point_y_r[2:0], conv_c_4x4_cnt_r);
                        sram_a[0]   = sram_addr;
                        sram_a[1]   = sram_addr;
                        sram_a[2]   = sram_addr;
                        sram_a[3]   = sram_addr;

                        //因為判別zero padding的需要，conv_scan_point_x為4bits的長度
                        if(conv_scan_point_x_r > 3'd7 || conv_scan_point_y_r > 3'd7)begin  //zero padding
                            conv_acc_4x4[conv_y_4x4_cnt_r][conv_x_4x4_cnt_r] = 'd0; //(SRAM_COUNT_BITS+8)'d0;
                            sram_out_valid = 1'b0;
                            //直接換行x++ or y++
                            conv_x_4x4_cnt = conv_x_4x4_cnt_r + 2'd1; //會自己循環
                            if(conv_x_4x4_cnt_r == 2'd3)begin //x這一輪是3，下一輪是0
                                conv_scan_point_x = conv_scan_point_x_r - 3'd3;
                                conv_y_4x4_cnt = conv_y_4x4_cnt_r + 2'd1; //會自己循環
                                conv_scan_point_y = conv_scan_point_y_r + 3'd1;
                                if(conv_y_4x4_cnt_r == 2'd3) begin
                                    //sram_out_valid = 1'b0;
                                    conv_fsm = 2'd2; //finish
                                end
                            end else begin
                                conv_scan_point_x = conv_scan_point_x_r + 3'd1;
                            end
                        end else begin //not zero padding
                            sram_out_valid = 1'b1;
                            if(conv_c_4x4_cnt_r + 5'd3 == depth_value(depth_sel)) begin//到底
                                conv_c_4x4_cnt = 5'd0;

                                conv_x_4x4_cnt = conv_x_4x4_cnt_r + 2'd1;//會自己循環
                                if(conv_x_4x4_cnt_r == 2'd3)begin//x這一輪是3，下一輪是0
                                    conv_scan_point_x = conv_scan_point_x_r - 3'd3;
                                    conv_y_4x4_cnt = conv_y_4x4_cnt_r + 2'd1;//會自己循環
                                    conv_scan_point_y = conv_scan_point_y_r + 3'd1;
                                    if(conv_y_4x4_cnt_r == 2'd3) begin
                                        //sram_out_valid = 1'b0;
                                        conv_fsm = 2'd2; //finish
                                    end
                                end else begin
                                    conv_scan_point_x = conv_scan_point_x_r + 3'd1;
                                end
                            end else begin //還沒到底
                                conv_c_4x4_cnt = conv_c_4x4_cnt_r + 5'd4;
                            end
                        end  
                        ///////////之前寫成讀取conv_x_4x4_cnt_r_delay1、conv_x_4x4_cnt_r_delay1
                        if(sram_out_valid_r) begin //save to acc, and conv_x_4x4 ++
                            conv_acc_4x4[conv_y_4x4_cnt_r_delay1][conv_x_4x4_cnt_r_delay1] = conv_acc_4x4_r[conv_y_4x4_cnt_r_delay1][conv_x_4x4_cnt_r_delay1] + (sram_q[0] + sram_q[1] + sram_q[2] + sram_q[3]);
                        end   
                    end
                    2'd2: begin
                        if(sram_out_valid_r) begin //the last memory output save to acc
                            conv_acc_4x4[conv_y_4x4_cnt_r_delay1][conv_x_4x4_cnt_r_delay1] = conv_acc_4x4_r[conv_y_4x4_cnt_r_delay1][conv_x_4x4_cnt_r_delay1] + (sram_q[0] + sram_q[1] + sram_q[2] + sram_q[3]);
                            sram_out_valid = 1'b0;
                        end

                        if (conv_kx_r == 2'd0 && conv_ky_r == 2'd0) begin
                            conv_acc = conv_acc_4x4_r[conv_ky_r+conv_yx_r[1]][conv_kx_r+conv_yx_r[0]] << GAUSS_KERNEL[conv_ky_r][conv_kx_r];//conv_ky+conv_yx[1]之後可以優化？？
                        end else begin
                            conv_acc = conv_acc_r + (conv_acc_4x4_r[conv_ky_r + conv_yx_r[1]][conv_kx_r + conv_yx_r[0]] << GAUSS_KERNEL[conv_ky_r][conv_kx_r]);//conv_ky+conv_yx[1]之後可以優化？？
                        end
                        
                        if (conv_kx_r == 2'd2) begin
                            conv_kx = 2'd0;
                            if (conv_ky_r == 2'd2) begin
                                conv_ky = 2'd0;
                                conv_kernel_done = 1'b1; // this kernel done
                                conv_yx = conv_yx_r + 2'd1; 
                                if(conv_yx_r == 2'd3) begin //all done
                                    conv_fsm = 2'd3; //all done
                                end
                            end else begin
                                conv_ky = conv_ky_r + 2'd1;
                            end
                        end else begin
                            conv_kx = conv_kx_r + 2'd1;
                            conv_kernel_done = 1'b0;
                        end                      
                    end
                    2'd3: begin
                        conv_kernel_done = 1'b0;
                        conv_fsm = 2'd0; //reset
                        state_next = S_O_OP_READY;
                    end
                endcase
            end
            default: begin

            end
        endcase
    end

    // Sequential main FSM
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_RESET;
            i_op_valid_r <= 1'b0;
            i_op_mode_r  <= 4'd0;
            i_in_valid_r <= 1'b0;
            i_in_data_r  <= 8'd0;
            //o_op_ready_r  <= 1'b0;
            //o_in_ready_r  <= 1'b0;
            o_out_valid <= 1'b0;
            o_out_data  <= 14'sb0;
            load_cnt_r  <= 11'd0;

            sram_select_r <= 2'd0;
            sram_out_valid_r <= 1'b0;
            origin_x_r    <= 3'd0;
            origin_y_r    <= 3'd0;
            depth_sel_r <= DEPTH_32;


            disp_c_r      <= 5'd0;
            disp_yx_r     <= 2'd0;
            disp_fsm_r    <= 1'd0;

            conv_fsm_r    <= 2'd0;
            // conv_scan_point_x <= 4'd0;
            // conv_scan_point_y <= 4'd0;
            // conv_x_4x4_cnt <= 3'd0;
            // conv_y_4x4_cnt <= 3'd0;
            // conv_c_4x4_cnt <= 5'd0;
            
            
            // med_xy      <= 2'd0;
            // med_ch      <= 2'd0;
            // sob_xy      <= 2'd0;
            // sob_ch      <= 2'd0;
        end else begin
            state <= state_next;
            i_op_valid_r <= i_op_valid;
            i_op_mode_r  <= i_op_mode;
            i_in_data_r <= i_in_data;
            i_in_valid_r <= i_in_valid;
            case (state)
                S_RESET: begin
                    
                end
                S_START: begin
                    
                end
                S_O_OP_READY: begin
                    
                end
                S_WAIT_OP: begin
                    
                end

                S_LOAD: begin
                    // Accept data only when both i_in_valid_r & o_in_ready; write to memory

                    load_cnt_r    <= load_cnt;                                          
                end
                // ================= ORIGIN SHIFT =================
                S_ORG_R: begin
                    origin_x_r <= origin_x;
                end
                S_ORG_L: begin
                    origin_x_r <= origin_x;
                end
                S_ORG_U: begin
                    origin_y_r <= origin_y;
                end
                S_ORG_D: begin
                    origin_y_r <= origin_y;
                end
                // ================= SCALE DEPTH =================
                S_SCALE_D: begin
                    depth_sel_r <= depth_sel;
                end  
                S_SCALE_U: begin
                    depth_sel_r <= depth_sel;
                end                      

                // ================= DISPLAY STREAM =================
                S_DISPLAY: begin
                    sram_out_valid_r <= sram_out_valid; 
                    o_out_valid <= sram_out_valid_r;
                    o_out_data <= sram_q[sram_select_r];

                    sram_select_r <= sram_select;
                    disp_yx_r <= disp_yx;
                    disp_c_r <= disp_c;
                    disp_fsm_r <= disp_fsm;
                end

                // ================= CONVOLUTION (sequential MAC) =================
                S_CONV: begin
                    // if (conv_fsm_r == 2'd0) begin //all done
                    conv_acc_4x4_r <= conv_acc_4x4;
                    // end else begin
                    //     conv_acc_4x4_r[conv_x_4x4_cnt_r_delay1][conv_x_4x4_cnt_r_delay1] <= conv_acc_4x4[conv_x_4x4_cnt_r_delay1][conv_x_4x4_cnt_r_delay1];
                    // end
                    conv_scan_point_x_r <= conv_scan_point_x;
                    conv_scan_point_y_r <= conv_scan_point_y;
                    conv_x_4x4_cnt_r <= conv_x_4x4_cnt;
                    conv_x_4x4_cnt_r_delay1 <= conv_x_4x4_cnt_r;
                    conv_y_4x4_cnt_r <= conv_y_4x4_cnt;
                    conv_y_4x4_cnt_r_delay1 <= conv_y_4x4_cnt_r;
                    conv_c_4x4_cnt_r <= conv_c_4x4_cnt;
                
                    conv_fsm_r <= conv_fsm;
                    
                    sram_out_valid_r <= sram_out_valid;      

                    conv_acc_r <= conv_acc;
                    conv_yx_r <= conv_yx;
                    conv_kx_r <= conv_kx;
                    conv_ky_r <= conv_ky;
                    o_out_data <= {1'b0, conv_acc[16:4] + conv_acc[3]}; // add rounding bit};
                    o_out_valid <= conv_kernel_done; 
                end

                // ================= MEDIAN FILTER (first 4 channels) =================
                S_MED_PREP: begin
                    /*
                    // Load 3×3 window into med_win[] for (med_ch, med_xy)
                    integer bx, by, dx, dy, idx;
                    bx = origin_x + (med_xy[0] ? 1 : 0);
                    by = origin_y + (med_xy[1] ? 1 : 0);
                    idx = 0;
                    for (dy=-1; dy<=1; dy=dy+1) begin
                        for (dx=-1; dx<=1; dx=dx+1) begin
                            med_win[idx] <= rd_px(bx+dx, by+dy, med_ch);
                            idx = idx + 1;
                        end
                    end
                    state <= S_MED_SORT;
                    */
                end

                S_MED_SORT: begin
                    /*
                    median9_sort();
                    // Output median (element 4)
                    o_out_valid <= 1'b1;
                    o_out_data  <= {6'd0, med_win[4]};
                    // Advance to next spatial pos, then channel
                    if (med_xy == 2'd3) begin
                        med_xy <= 2'd0;
                        if (med_ch == 2'd3) begin
                            o_op_ready <= 1'b1; state <= S_WAIT_OP;
                        end else begin
                            med_ch <= med_ch + 2'd1; state <= S_MED_PREP;
                        end
                    end else begin
                        med_xy <= med_xy + 2'd1; state <= S_MED_PREP;
                    end
                    */
                end

                // ================= SOBEL + NMS (first 4 channels) =================
                S_SOBEL_ACC: begin
                    /*
                    // Compute sram_center gradient & direction
                    integer bx, by; bx = origin_x + (sob_xy[0] ? 1 : 0); by = origin_y + (sob_xy[1] ? 1 : 0);
                    sobel_vec(bx, by, sob_ch, gx_c, gy_c, g_c);
                    g_dir <= sobel_dir(gx_c, gy_c);
                    state <= S_SOBEL_NMS;
                    */
                end

                S_SOBEL_NMS: begin
                    /*
                    integer bx, by; integer nx1, ny1, nx2, ny2;
                    bx = origin_x + (sob_xy[0] ? 1 : 0);
                    by = origin_y + (sob_xy[1] ? 1 : 0);
                    // Neighbor coords along quantized dir
                    unique case (g_dir)
                        DIR_0:   begin nx1=bx-1; ny1=by;   nx2=bx+1; ny2=by;   end
                        DIR_90:  begin nx1=bx;   ny1=by-1; nx2=bx;   ny2=by+1; end
                        DIR_45:  begin nx1=bx-1; ny1=by-1; nx2=bx+1; ny2=by+1; end
                        default: begin nx1=bx-1; ny1=by+1; nx2=bx+1; ny2=by-1; end
                    endcase
                    g_n1 = sobel_mag(nx1, ny1, sob_ch);
                    g_n2 = sobel_mag(nx2, ny2, sob_ch);

                    // Suppress if sram_center < any neighbor
                    if (g_c < g_n1 || g_c < g_n2) begin
                        o_out_data  <= 14'sd0;
                    end else begin
                        o_out_data  <= g_c[13:0];
                    end
                    o_out_valid <= 1'b1;

                    // Advance ordering: spatial 2×2 then channel 0..3
                    if (sob_xy == 2'd3) begin
                        sob_xy <= 2'd0;
                        if (sob_ch == 2'd3) begin
                            o_op_ready <= 1'b1; state <= S_WAIT_OP;
                        end else begin
                            sob_ch <= sob_ch + 2'd1; state <= S_SOBEL_ACC;
                        end
                    end else begin
                        sob_xy <= sob_xy + 2'd1; state <= S_SOBEL_ACC;
                    end*/
                end

                default: begin
                    //state <= S_WAIT_OP;
                end
            endcase
        end
    end

endmodule
