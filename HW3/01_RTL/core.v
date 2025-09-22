`timescale 1ns/1ps

module core #(
    parameter   SRAM_SIZE = 512, //word, remember to check SRAM inctance
                SRAM_SIZE_BITS = 9, //log_2(SRAM_SIZE)
                SRAM_COUNT = 4, //Number of SRAM
                SRAM_COUNT_BITS = 2; //log_2(SRAM_COUNT)
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

    // 宣告一個 3×3 的常數矩陣
    localparam [2:0] GAUSS_KERNEL [0:2][0:2] = '{
        '{3'd1, 3'd2, 3'd1},  // row 0: (ky=0)
        '{3'd2, 3'd4, 3'd2},  // row 1: (ky=1)
        '{3'd1, 3'd2, 3'd1}   // row 2: (ky=2)
    };

    // -----------------------------
    // Negege-sampled inputs (spec p.6)
    // -----------------------------
    //reg        i_op_valid_r;
    //reg [3:0]  i_op_mode_r;
    //reg        i_in_valid_r;
    //reg [7:0]  i_in_data_r;
    reg         o_op_ready_r;         // pulse when ready for next op
    reg         o_in_ready_r;         // ready to accept input (LOAD)
    reg         o_out_valid_r;        // streaming output valid
    reg signed [13:0] o_out_data_r;   

    // -----------------------------
    // Global state / registers
    // -----------------------------
    typedef enum logic [4:0] { //enum：定義一組有名字的常數（列舉）。
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

        S_DISPLAY   = 5'd5,
        S_CONV_ACC  = 5'd6,
        S_CONV_DONE = 5'd7,
        S_MED_PREP  = 5'd8,
        S_MED_SORT  = 5'd9,
        S_SOBEL_ACC = 5'd10,
        S_SOBEL_NMS = 5'd11,
        S_STREAM    = 5'd12,
        S_PULSE_RDY = 5'd13
    } state_t;

    state_t state;// state_n; //logic [4:0] state, state_n;

    // Origin & depth
    reg [2:0] origin_x, origin_y; // 0..7, but window requires <=6
    reg [1:0] depth_sel;           // DEPTH_8/16/32 (default 32)

    // Helper: decode numeric depth
    function automatic [5:0] depth_value(input [1:0] dsel);
        case (dsel)
            DEPTH_8:  depth_value = 6'd8;
            DEPTH_16: depth_value = 6'd16;
            default:  depth_value = 6'd32;
        endcase
    endfunction

    // -----------------------------
    // Banked image memory: SRAM_COUNT × (SRAM_SIZE × 8)
    // Index mapping: addr = (c*64) + (y*8 + x)
    // bank = addr[10:8], off = addr[7:0]
    // -----------------------------

    wire [8:0] sram_q [0:SRAM_COUNT-1]; // Data Outputs (Q[0] = LSB)
    reg        sram_cen; // Chip Enable
    reg        sram_wen [0:SRAM_COUNT-1];
    reg  [7:0] sram_a [0:SRAM_COUNT-1]; //Addresses (A[0] = LSB)
    reg  [7:0] sram_d [0:SRAM_COUNT-1]; // Data Inputs (D[0] = LSB)
    reg  [SRAM_COUNT_BITS-1:0] sram_select; // Which SRAM to use for current op
    reg  [SRAM_SIZE-1:0] sram_addr; // Address within selected SRAM

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

/*functon本身應該不會用到，但條件式之後可以用
    // Safe pixel read with zero padding (out-of-bound → 0)
    function automatic [7:0] rd_px(input integer xc, input integer yc, input integer ch);
        if (xc < 0 || xc >= IMG_W || yc < 0 || yc >= IMG_H || ch < 0 || ch >= IMG_C) begin
            rd_px = 8'd0;
        end else begin
            rd_px = rd_mem(xc[2:0], yc[2:0], ch[5:0]);
        end
    endfunction
*/

    // -----------------------------
    // LOAD engine
    // -----------------------------
    reg [10:0] load_cnt; // 0..2047
/*
    // -----------------------------
    // DISPLAY streamer counters (channel-major after 2×2 raster)
    // -----------------------------
    reg [5:0]  disp_c;   // up to 32, 控制正在輸出的 channel
    reg [1:0]  disp_xy;  // 0:(0,0) 1:(1,0) 2:(0,1) 3:(1,1), 控制在該 channel 裡的 2×2 小 tile 座標
    reg [13:0] stream_dat; //存放準備輸出的資料
    reg        stream_vld; //標示當前輸出資料是否有效

    // -----------------------------
    // CONV accumulator (process 1 output pixel at a time → 4 outputs)
    // -----------------------------
    reg [1:0]  conv_xy;      // which of 4 outputs we are accumulating
    reg [5:0]  conv_c;       // channel index for accumulation
    reg [1:0]  conv_ky, conv_kx; // 0..2 for kernel loops
    //reg [23:0] conv_acc;     // wide accumulator (enough for 32*4080 ≈ 130560)
    reg        conv_do_add;
*/
/*     // -----------------------------
    // MEDIAN buffers & sorter (median of 9)
    // -----------------------------
    reg [1:0]  med_xy;       // 0..3
    reg [1:0]  med_ch;       // 0..3 (first 4 channels)
    reg [7:0]  med_win [0:8];// 3×3 window values
    integer    mi, mj;

   // Selection sort to get median index 4 after sorting assram_cending
    task automatic median9_sort;
        reg [7:0] tmp;
        integer i,j,imin;
        begin
            for (i=0;i<9;i=i+1) begin
                imin = i;
                for (j=i+1;j<9;j=j+1) begin
                    if (med_win[j] < med_win[imin]) imin = j;
                end
                tmp = med_win[i];
                med_win[i] = med_win[imin];
                med_win[imin] = tmp;
            end
        end
    endtask
*/
/*
    // -----------------------------
    // SOBEL + NMS engine (per channel & per 2×2 pos)
    // -----------------------------
    reg [1:0]  sob_xy;       // 0..3
    reg [1:0]  sob_ch;       // 0..3

    // Compute |x|
    function automatic [15:0] abs16(input signed [15:0] v);
        abs16 = (v[15]) ? (~v + 16'sd1) : v;
    endfunction

    // Sobel gradients and L1 magnitude for (x0,y0,ch)
    function automatic [15:0] sobel_mag(input integer x0, input integer y0, input integer ch);
        integer dy, dx; integer xi, yi; reg signed [15:0] gx, gy; reg [7:0] p;
        begin
            gx = 16'sd0; gy = 16'sd0;
            for (dy=-1; dy<=1; dy=dy+1) begin
                for (dx=-1; dx<=1; dx=dx+1) begin
                    xi = x0 + dx; yi = y0 + dy; p = rd_px(xi, yi, ch);
                    // Gx kernel: [-1 0 1; -2 0 2; -1 0 1]
                    case ({dy,dx})
                        {-1,-1}: gx = gx - p;    {-1,0}: ;           {-1,1}: gx = gx + p;
                        {0,-1}:  gx = gx - (p<<1);{0,0}: ;            {0,1}:  gx = gx + (p<<1);
                        {1,-1}:  gx = gx - p;    {1,0}: ;             {1,1}:  gx = gx + p;
                        default: ;
                    endcase
                    // Gy kernel: [ 1  2  1; 0 0 0; -1 -2 -1]
                    case ({dy,dx})
                        {-1,-1}: gy = gy + p;    {-1,0}: gy = gy + (p<<1); {-1,1}: gy = gy + p;
                        {0,-1}:  ;                {0,0}: ;                 {0,1}:  ;
                        {1,-1}:  gy = gy - p;    {1,0}:  gy = gy - (p<<1); {1,1}:  gy = gy - p;
                        default: ;
                    endcase
                end
            end
            sobel_mag = abs16(gx) + abs16(gy); // L1 magnitude
        end
    endfunction

    // Direction quantization using tan approximations (p.30)
    typedef enum logic [1:0] {DIR_0=2'd0, DIR_45=2'd1, DIR_90=2'd2, DIR_135=2'd3} dir_t;

    function automatic dir_t sobel_dir(input signed [15:0] gx_in, input signed [15:0] gy_in);
        reg [15:0] ax, ay; reg [15:0] t22, t67; dir_t d;
        begin
            ax = abs16(gx_in); ay = abs16(gy_in);
            if (ax == 16'd0) begin
                d = DIR_90;
            end else if (ay == 16'd0) begin
                d = DIR_0;
            end else begin
                // tan22.5 ≈ 2^-2 + 2^-3 + 2^-5 + 2^-7
                t22 = (ax>>2) + (ax>>3) + (ax>>5) + (ax>>7);
                // tan67.5 ≈ 2 + 2^-2 + 2^-3 + 2^-5 + 2^-7
                t67 = (ax<<1) + (ax>>2) + (ax>>3) + (ax>>5) + (ax>>7);
                if (ay <= t22) d = DIR_0;
                else if (ay >= t67) d = DIR_90;
                else begin
                    // middle band → 45° or 135° by signs of gx,gy
                    d = (gx_in[15] ^ gy_in[15]) ? DIR_135 : DIR_45;
                end
            end
            sobel_dir = d;
        end
    endfunction

    // For NMS we need both gx,gy at the sram_center; compute alongside mag
    function automatic void sobel_vec(
        input integer x0, input integer y0, input integer ch,
        output signed [15:0] gx, output signed [15:0] gy, output [15:0] g
    );
        integer dy, dx; integer xi, yi; reg [7:0] p;
        begin
            gx = 16'sd0; gy = 16'sd0;
            for (dy=-1; dy<=1; dy=dy+1) begin
                for (dx=-1; dx<=1; dx=dx+1) begin
                    xi = x0 + dx; yi = y0 + dy; p = rd_px(xi, yi, ch);
                    case ({dy,dx})
                        {-1,-1}: begin gx = gx - p;             gy = gy + p;             end
                        {-1,0}:  begin                           gy = gy + (p<<1);       end
                        {-1,1}:  begin gx = gx + p;             gy = gy + p;             end
                        {0,-1}:  begin gx = gx - (p<<1);                               end
                        {0,0}:   begin                                               end
                        {0,1}:   begin gx = gx + (p<<1);                               end
                        {1,-1}:  begin gx = gx - p;             gy = gy - p;           end
                        {1,0}:   begin                           gy = gy - (p<<1);     end
                        {1,1}:   begin gx = gx + p;             gy = gy - p;           end
                    endcase
                end
            end
            g = abs16(gx) + abs16(gy);
        end
    endfunction
*/
    // -----------------------------
    // FSM & control
    // -----------------------------
    //reg [13:0] out_data_next;
    //reg        out_valid_next;

    // Temp regs for sobel sram_center/neighbor mags
    //reg  signed [15:0] gx_c, gy_c; reg [15:0] g_c;
    //reg  [15:0] g_n1, g_n2;
    //dir_t g_dir;


    // Next-state (combinational) for simple stream datapath
    always @(*) begin
        //stream_vld   = 1'b0;
        //stream_dat   = 14'sd0;
            o_op_ready = 1'b0;
            o_in_ready = 1'b0;
            sram_cen = 1'b1; // default disable
            sram_wen = 4'b1; // default read
        case (state)
            S_RESET: begin

            end
            S_START: begin

            end
            S_O_OP_READY: begin
                o_op_ready = 1'b1;
            end
            S_WAIT_OP: begin

            end
            S_LOAD: begin
                o_in_ready = 1'b0;
                sram_cen = 1'b1;
                {sram_select, sram_addr} = RS_order2mem_addr(load_cnt);
                    sram_wen[sram_select] = 1'b0; // write enable
                    sram_a[sram_select]   = sram_addr;
                    sram_d[sram_select]   = i_in_data;            
            end
            S_ORG_R: begin

            end
            S_ORG_L: begin

            end
            S_ORG_U: begin

            end
            S_ORG_D: begin

            end
            S_SCALE_D: begin

            end
            S_SCALE_U: begin

            end
            // S_DISPLAY: begin
            //     // Output order: for ch=0..depth-1: (0,0),(1,0),(0,1),(1,1)
            //     stream_vld = 1'b1;
            //     unique case (disp_xy)
            //         2'd0: stream_dat = {6'd0, rd_mem(origin_x+0, origin_y+0, disp_c)};
            //         2'd1: stream_dat = {6'd0, rd_mem(origin_x+1, origin_y+0, disp_c)};
            //         2'd2: stream_dat = {6'd0, rd_mem(origin_x+0, origin_y+1, disp_c)};
            //         default: stream_dat = {6'd0, rd_mem(origin_x+1, origin_y+1, disp_c)};
            //     endcase
            // end
            // default: begin
            //     stream_vld = 1'b0;
            //     stream_dat = 14'sd0;
            // end
        endcase
    end

    // Sequential main FSM
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_RESET;
            //i_op_valid_r <= 1'b0;
            //i_op_mode_r  <= 4'd0;
            //i_in_valid_r <= 1'b0;
            //i_in_data_r  <= 8'd0;
            o_op_ready_r  <= 1'b0;
            o_in_ready_r  <= 1'b0;
            o_out_valid_r <= 1'b0;
            o_out_data_r  <= 14'sd0;

            origin_x    <= 3'd0;
            origin_y    <= 3'd0;
            depth_sel   <= DEPTH_32; // default 32
            // load_cnt    <= 12'd0;
            // disp_c      <= 6'd0;
            // disp_xy     <= 2'd0;
            // conv_xy     <= 2'd0;
            // conv_c      <= 6'd0;
            // conv_ky     <= 2'd0;
            // conv_kx     <= 2'd0;
            // conv_acc    <= 24'd0;
            // med_xy      <= 2'd0;
            // med_ch      <= 2'd0;
            // sob_xy      <= 2'd0;
            // sob_ch      <= 2'd0;
        end else begin
            case (state)
                S_RESET: begin
                    state <= S_START;
                end
                S_START: begin
                    state <= S_O_OP_READY;
                end
                S_O_OP_READY: begin
                    state <= S_WAIT_OP;
                end
                S_WAIT_OP: begin
                    if (i_op_valid) begin
                        unique case (i_op_mode)
                            OP_LOAD: begin
                                // begin loading 2048 bytes
                                load_cnt   <= 11'd0;
                                state      <= S_LOAD;
                            end
                            OP_ORG_R : begin
                                state <= S_ORG_R;
                            end
                            OP_ORG_L : begin
                                state <= S_ORG_L;
                            end
                            OP_ORG_U : begin
                                state <= S_ORG_U;
                            end
                            OP_ORG_D : begin
                                state <= S_ORG_D;
                            end
                            OP_SCALE_D: begin
                                state <= S_SCALE_D;
                            end
                            OP_SCALE_U: begin
                                state <= S_SCALE_U;
                            end
                            OP_DISPLAY: begin
                                //disp_c  <= 6'd0;
                                //disp_xy <= 2'd0;
                                state   <= S_DISPLAY;
                            end
                            OP_CONV: begin
                                //conv_xy  <= 2'd0; conv_c <= 6'd0; conv_ky<=2'd0; conv_kx<=2'd0; conv_acc<=24'd0;
                                state    <= S_CONV_ACC;
                            end
                            OP_MEDIAN: begin
                                //med_ch <= 2'd0; med_xy <= 2'd0;
                                state  <= S_MED_PREP;
                            end
                            OP_SOBEL: begin
                                //sob_ch <= 2'd0; sob_xy <= 2'd0;
                                state  <= S_SOBEL_ACC; // compute sram_center first
                            end
                            default: begin
                                // ignore undefined opcodes
                                //o_op_ready <= 1'b1; // next op please
                            end
                        endcase
                    end
                end

                S_LOAD: begin
                    // Accept data only when both i_in_valid_r & o_in_ready; write to memory
                    if (i_in_valid) begin
                        load_cnt  <= load_cnt + 11'd1;
                        if (load_cnt == IMG_PIX-1) begin                                                       
                            state      <= S_O_OP_READY;
                        end
                    end
                end
                // ================= ORIGIN SHIFT =================
                S_ORG_R: begin
                    if (origin_x < 3'd6) origin_x <= origin_x + 3'd1;
                    state <= S_O_OP_READY;
                end
                S_ORG_L: begin
                    if (origin_x > 3'd0) origin_x <= origin_x - 3'd1;
                    state <= S_O_OP_READY;
                end
                S_ORG_U: begin
                    if (origin_y > 3'd0) origin_y <= origin_y - 3'd1;
                    state <= S_O_OP_READY;
                end
                S_ORG_D: begin
                    if (origin_y < 3'd6) origin_y <= origin_y + 3'd1;
                    state <= S_O_OP_READY;
                end
                // ================= SCALE DEPTH =================
                S_SCALE_D: begin
                    if (depth_sel > DEPTH_8) depth_sel <= depth_sel - 2'd1;
                    state <= S_O_OP_READY;
                end
                S_SCALE_U: begin
                    if (depth_sel < DEPTH_32) depth_sel <= depth_sel + 2'd1;
                    state <= S_O_OP_READY;
                end

                // ================= DISPLAY STREAM =================
                S_DISPLAY: begin
                    o_out_valid <= stream_vld;
                    o_out_data  <= stream_dat;
                    if (stream_vld) begin
                        if (disp_xy == 2'd3) begin
                            disp_xy <= 2'd0;
                            if (disp_c + 6'd1 < depth_value(depth_sel)) begin
                                disp_c <= disp_c + 6'd1;
                            end else begin
                                // done
                                o_out_valid <= 1'b0;
                                o_op_ready  <= 1'b1;
                                state       <= S_WAIT_OP;
                            end
                        end else begin
                            disp_xy <= disp_xy + 2'd1;
                        end
                    end
                end

                // ================= CONVOLUTION (sequential MAC) =================
                S_CONV_ACC: begin
                    // Accumulate: conv_acc += k_gauss * pixel across (ky,kx,c)
                    // Compute current window coord for this conv_xy
                    integer base_x, base_y, ch;
                    integer wx, wy; reg [7:0] px; reg [2:0] w;
                    base_x = origin_x + (conv_xy[0] ? 1 : 0);
                    base_y = origin_y + (conv_xy[1] ? 1 : 0);
                    wx = base_x + (conv_kx - 2'd1); // -1,0,1
                    wy = base_y + (conv_ky - 2'd1);
                    ch = conv_c;
                    px = rd_px(wx, wy, ch);
                    w  = k_gauss(conv_ky, conv_kx);
                    conv_acc <= conv_acc + (px * w); // implicit zero extend

                    // Advance inner loops: kx -> ky -> c
                    if (conv_kx == 2'd2) begin
                        conv_kx <= 2'd0;
                        if (conv_ky == 2'd2) begin
                            conv_ky <= 2'd0;
                            if (conv_c + 6'd1 < depth_value(depth_sel)) begin
                                conv_c <= conv_c + 6'd1;
                            end else begin
                                // Finish this pixel: round to nearest (÷16)
                                o_out_valid <= 1'b1;
                                o_out_data  <= (conv_acc + 24'd8) >>> 4; // 14-bit is enough
                                // Prepare for next output pixel
                                conv_acc <= 24'd0; conv_c <= 6'd0;
                                if (conv_xy == 2'd3) begin
                                    state <= S_CONV_DONE;
                                end else begin
                                    conv_xy <= conv_xy + 2'd1;
                                end
                            end
                        end else begin
                            conv_ky <= conv_ky + 2'd1;
                        end
                    end else begin
                        conv_kx <= conv_kx + 2'd1;
                    end
                end

                S_CONV_DONE: begin
                    o_op_ready <= 1'b1; state <= S_WAIT_OP;
                end

                // ================= MEDIAN FILTER (first 4 channels) =================
                S_MED_PREP: begin
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
                end

                S_MED_SORT: begin
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
                end

                // ================= SOBEL + NMS (first 4 channels) =================
                S_SOBEL_ACC: begin
                    // Compute sram_center gradient & direction
                    integer bx, by; bx = origin_x + (sob_xy[0] ? 1 : 0); by = origin_y + (sob_xy[1] ? 1 : 0);
                    sobel_vec(bx, by, sob_ch, gx_c, gy_c, g_c);
                    g_dir <= sobel_dir(gx_c, gy_c);
                    state <= S_SOBEL_NMS;
                end

                S_SOBEL_NMS: begin
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
                    end
                end

                default: begin
                    state <= S_WAIT_OP;
                end
            endcase
        end
    end

endmodule
