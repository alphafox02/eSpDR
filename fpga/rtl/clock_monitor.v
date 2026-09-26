// Forwarded reference measurement in 1 ms windows of management time.
module clock_monitor #(
    parameter WINDOW_CYCLES = 120000,
    parameter MIN_EDGES = 19000,
    parameter MAX_EDGES = 21000
) (
    input wire source_clk,
    input wire clk,
    input wire reset,
    output reg present = 0,
    output reg [31:0] frequency_hz = 0
);
    reg [23:0] count = 0, gray = 0;
    wire [23:0] next_count = count + 1'b1;
    always @(posedge source_clk) begin
        count <= next_count;
        gray <= (next_count >> 1) ^ next_count;
    end
    (* ASYNC_REG = "TRUE" *) reg [23:0] gray_meta = 0, gray_sync = 0;
    wire [23:0] sampled;
    genvar i;
    generate for (i=0;i<24;i=i+1) begin : decode_gray
        assign sampled[i] = ^gray_sync[23:i];
    end endgenerate
    reg [$clog2(WINDOW_CYCLES)-1:0] ticks = 0;
    reg [23:0] binary_sample = 0, previous = 0, elapsed = 0;
    reg publish = 0;
    always @(posedge clk) begin
        gray_meta <= gray;
        gray_sync <= gray_meta;
        binary_sample <= sampled;
        publish <= 0;
        if (reset) begin
            ticks <= 0;
            previous <= binary_sample;
            present <= 0;
            frequency_hz <= 0;
        end else if (ticks == WINDOW_CYCLES-1) begin
            ticks <= 0;
            previous <= binary_sample;
            elapsed <= binary_sample - previous;
            publish <= 1;
        end else ticks <= ticks + 1'b1;
        if (publish && !reset) begin
            present <= elapsed >= MIN_EDGES && elapsed <= MAX_EDGES;
            frequency_hz <= elapsed * 1000;
        end
    end
endmodule
