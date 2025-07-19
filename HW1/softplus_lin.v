module softplus_lin (
    input  wire signed [15:0] x,   // Q6.10
    output reg  signed [15:0] y    // Q6.10
);
    reg signed [33:0] num;         // 最長 16x17bit = 33bits
    reg signed [33:0] biased;
    reg [4:0] shift;               // 移位位數（16 或 17）

    always @(*) begin
        if (x >= 16'sd2048) begin                      // x ≥ 2
            y = x;                                     // y = x
        end else if (x >= 16'sd0) begin                // (2x+2)/3
            num    = ((x <<< 1) + 16'sd2048) * 17'sd21845; // Q0.16
            biased = num + 17'sd32768;
            shift  = 5'd16;
            y      = biased >>> shift;
        end else if (x >= -16'sd1024) begin            // (x+2)/3
            num    = (x + 16'sd2048) * 17'sd21845;
            biased = num + 17'sd32768;
            shift  = 5'd16;
            y      = biased >>> shift;
        end else if (x >= -16'sd2048) begin            // (2x+5)/9
            num    = ((x <<< 1) + 16'sd5120) * 18'sd14564; // Q0.17
            biased = num + 18'sd65536;
            shift  = 5'd17;
            y      = biased >>> shift;
        end else if (x >= -16'sd3072) begin            // (x+3)/9
            num    = (x + 16'sd3072) * 18'sd14564;
            biased = num + 18'sd65536;
            shift  = 5'd17;
            y      = biased >>> shift;
        end else begin
            y = 16'sd0;
        end
    end
endmodule
