`timescale 1ns/1ps

module fp_add32_mod (
// -------------------------------------------------------------
//  IEEE-754 single-precision FP add/sub  (Round-to-Nearest-Even)
// -------------------------------------------------------------
//   in_a, in_b : 32-bit operands
//   add_sub    : 1=add, 0=sub  (若先在 decode 取反 b.sign，這裡永遠填 1)
//   sum_o      : IEEE-754 結果
//   invalid_o  : 1 = NaN / ±Inf  (交給 FSM 判定 o_status = INVALID)
// -------------------------------------------------------------
    input  logic [31:0] in_a,
    input  logic [31:0] in_b,
    input  logic        add_sub,
    output logic [31:0] sum_o,
    output logic        invalid_o
);

    localparam EXP_MAX = 8'hFF;

    // -------- 1. 拆解欄位與例外狀態 ----------------------------
    logic sa, sb;
    logic [7:0] ea, eb;
    logic [23:0] ma, mb;          // 隱含位元 + 23-bit mantissa
    logic a_inf, b_inf, a_nan, b_nan;//, a_zero, b_zero;

    logic [7:0]  e_big,  e_sml;
    logic [23:0] m_big,  m_sml;
    logic        s_big;//,  s_sml;
    logic [7:0]  exp_diff;

    //25改成47
    logic [47:0] m_sml_shift; // 24-bit(含 隱含位元 + Guard bit) + RS(2 bits)
    logic [47:0] tmp; // 24-bit(含Guard bit) + RS(24 bits)，位移超過25位後將不影響
    //logic [47:0] tmp_shift;

    logic [48:0] mant_sum;        // 多 1 bit 以防 overflow

    logic [7:0]  exp_n;  //改成另外給一個overflow flag？
    // logic overflow_flag = 1'b0;
    logic [48:0] man_n;
    logic [5:0] mant_LeftMove;

    integer i;

    logic guard;
    logic round;
    logic stick;
    logic [7:0]  exp_rnd;

    logic [24:0] man_rnd;  // 去掉 RS(含隱藏位元)
    always @(*)begin
    sa = in_a[31];  sb = add_sub ^ in_b[31]; //減法的話，負負得正
    ea = in_a[30:23];  eb = in_b[30:23];
    ma = (ea==0) ? {1'b0,  in_a[22:0]} : {1'b1, in_a[22:0]};
    mb = (eb==0) ? {1'b0,  in_b[22:0]} : {1'b1, in_b[22:0]};

    a_inf  = (ea==EXP_MAX) & (in_a[22:0]==0);
    b_inf  = (eb==EXP_MAX) & (in_b[22:0]==0);
    a_nan  = (ea==EXP_MAX) & (in_a[22:0]!=0);
    b_nan  = (eb==EXP_MAX) & (in_b[22:0]!=0);
    //a_zero = (ea==0) & (in_a[22:0]==0);
    //b_zero = (eb==0) & (in_b[22:0]==0);


    // -------- 3. 讓 A 是指數較大的那個 -------------------------
    
    if (ea > eb) begin //之後會使用加減法器是sa, sb來判斷 //s_big的作用純粹是用來判別sum_o的正負
        e_big = ea;  m_big = ma;  s_big = sa;
        e_sml = eb;  m_sml = mb;  //s_sml = sb;
    end else if (ea < eb) begin
        e_big = eb;  m_big = mb;  s_big = sb;
        e_sml = ea;  m_sml = ma;  //s_sml = sa;
    end else if (ma > mb) begin //ea = eb
        e_big = ea;  m_big = ma;  s_big = sa;
        e_sml = eb;  m_sml = mb;  //s_sml = sb;
    end else if (ma < mb) begin
        e_big = eb;  m_big = mb;  s_big = sb;
        e_sml = ea;  m_sml = ma;  //s_sml = sa;
    end else if (sa & sb) begin
        e_big = ea;  m_big = ma;  s_big = 1'b1; //-0 -0
        e_sml = ea;  m_sml = ma;  //s_sml = 1'b1;
    end else begin
        e_big = ea;  m_big = ma;  s_big = 1'b0;
        e_sml = ea;  m_sml = ma;  //s_sml = sa;
    end

    //因為不會有2^-127
    if (e_big == 8'b0)begin
        e_big = 8'b1; //2^-126
    end
    if (e_sml == 8'b0)begin
        e_sml = 8'b1; //2^-126
    end
    exp_diff = e_big - e_sml;

    // -------- 4. 對齊較小數的 mantissa (含 Sticky) --------------
    
    tmp = {m_sml, 24'b0};
    //tmp_shift = (tmp >> exp_diff);
    //sticky = |tmp_shift[22:0];
    m_sml_shift = (tmp >> exp_diff);//tmp_shift;//{tmp_shift[47:23],sticky};    // sticky 塞到最末位
   
    // -------- 5. 加／減 mantissa --------------------------------
    
    if (sa == sb) begin
        // 同號：相加
        mant_sum = {m_big, 24'b0} + m_sml_shift;
    end else begin
        // 異號：相減（確保 big ≥ sml，因此結果非負）
        mant_sum = {m_big, 24'b0} - m_sml_shift;
    end

    // -------- 6. 正規化 (normalize) -----------------------------
    //例外情況，如果for迴圈找不到1
    exp_n = 8'b0;
    man_n = 49'b0;

    for(i = 48; i >= 0; i--)begin
        if(mant_sum[i] == 1'b1)begin
            mant_LeftMove = 6'd48 - i;//最高位元變成多給的那一個位元
            //$display("mant_LeftMove:%d = 26 -%d",mant_LeftMove,i);
            if(mant_LeftMove == 6'b0)begin //manti不移位，但因為最高位有多給一個位元，表示exp要+1
                if(e_big == EXP_MAX)begin //避免exp上溢位
                    //overflow_flag = 1'b1; //好像也不需要這個
                    exp_n = EXP_MAX;
                end
                else begin //沒溢位
                    exp_n = e_big + 8'b1;
                    man_n = mant_sum; //不用移位
                end
            end
            else begin
                //因為改成以多給的進位位元為做高位元，故exp+1，但因為指數最小是2^-126，不會有2^-127，因此00000001其實已經不能再借
                //if((e_big + 9'b1) < mant_LeftMove)begin //下溢位
                if(e_big < mant_LeftMove)begin //下溢位
                    exp_n = 8'b0;
                    // man_n = mant_sum << (e_big + 8'b1);
                    man_n = mant_sum << e_big;
                end else begin
                    exp_n = (e_big - mant_LeftMove) + 8'b1;//先做減法才不用考慮溢位 //先unsigned extendtion再做補數？？
                    man_n = mant_sum << mant_LeftMove;
                end
            end
            break;
            //$display("unbreak!!");
        end
    end

    // -------- 7. Guard / Round / Sticky -------------------------
    // Round-to-Nearest-Even (RNE)
    guard = man_n[25];
    round = man_n[24];
    stick = |man_n[23:0];
    exp_rnd = exp_n;
    man_rnd = man_n[48:25];
    if ( (round & stick) | (guard & round) ) begin
        if(man_n[48:25] == 24'hffffff) begin //進位後會進位
            man_rnd = 24'h800000; //
            if(exp_n == EXP_MAX)begin //上溢位
                exp_rnd = EXP_MAX;
            end
            else begin
                exp_rnd   = exp_n + 1;
            end
        end
        else begin
            man_rnd = man_n[48:25] + 1'b1;
        end
    end

    // -------- 8. 組回 IEEE-754 ----------------------------------
    if (exp_rnd == EXP_MAX) begin
        // overflow → ±Inf
        sum_o     = {s_big, 8'hFF, 23'd0};
        invalid_o = 1'b1;          // 雖然 spec 可算 overflow，這題列為 invalid
    /*end else if (exp_n == 8'b0) begin
        // subnormal
        sum_o     = {s_big, 8'd0, man_rnd[22:0]};
        invalid_o = 1'b0;*/    
    end else begin
        sum_o     = {s_big, exp_rnd, man_rnd[22:0]};
        invalid_o = 1'b0;
    end
        // -------- 2. 特殊數值優先回傳 ------------------------------
    if (a_nan | b_nan) begin
        // NaN 或 (+Inf) + (-Inf) → NaN
        sum_o     = 32'h7FC0_0000;    // canonical quiet-NaN
        invalid_o = 1'b1;
    end 
    else if (a_inf & b_inf & (sa^sb))begin
        sum_o     = 32'hFFC0_0000;    // canonical quiet-NaN
        invalid_o = 1'b1;
    end
    else if (a_inf | b_inf) begin
        // ±Inf 與有限值相加 → ±Inf
        sum_o     = a_inf ? in_a : {sb, eb, mb[22:0]};//留意，mb是含隱藏位元的24bits
        invalid_o = 1'b1;
    end
    /*
    else if (a_zero & b_zero) begin
        // 0 + (-0) 規定回傳 +0（tie to even）
        sum_o     = 32'h0000_0000;
        invalid_o = 1'b0;
    end*/

end
endmodule