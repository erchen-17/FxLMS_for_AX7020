module audio_fifo_simple #(
    parameter DATA_WIDTH = 24,
    parameter DEPTH = 16,                // FIFO 深度（必须是 2^N）
    parameter ADDR_WIDTH = 4              // log2(DEPTH)
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // ===== 写端（ADC） =====
    input  wire                  wr_en,   // adc_valid
    input  wire [DATA_WIDTH-1:0] wr_data,

    // ===== 读端（DAC） =====
    input  wire                  rd_en,   // dac 需要新采样
    output reg  [DATA_WIDTH-1:0] rd_data,

    output wire                  empty,
    output wire                  full
);

    // =========================================================
    // FIFO 存储
    // =========================================================
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    reg [ADDR_WIDTH:0] wr_ptr;
    reg [ADDR_WIDTH:0] rd_ptr;

    // =========================================================
    // 空 / 满判断
    // =========================================================
    assign empty = (wr_ptr == rd_ptr);
    assign full  = (wr_ptr[ADDR_WIDTH]     != rd_ptr[ADDR_WIDTH]) &&
                   (wr_ptr[ADDR_WIDTH-1:0] == rd_ptr[ADDR_WIDTH-1:0]);

    // =========================================================
    // 写 FIFO（ADC → FIFO）
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
        end else if (wr_en && !full) begin
            mem[wr_ptr[ADDR_WIDTH-1:0]] <= wr_data;
            wr_ptr <= wr_ptr + 1'b1;
        end
    end

    // =========================================================
    // 读 FIFO（FIFO → DAC）
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr  <= 0;
            rd_data <= 0;
        end else if (rd_en && !empty) begin
            rd_data <= mem[rd_ptr[ADDR_WIDTH-1:0]];
            rd_ptr  <= rd_ptr + 1'b1;
        end
    end

endmodule