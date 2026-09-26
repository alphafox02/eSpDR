// Management/DDR reference use the Au oscillator. Both acquisition clocks
// follow the ESP's forwarded 20 MHz, preserving the exact 240:120 ratio.
module clocks (
    input wire oscillator,
    input wire reference,
    input wire source_reset,
    input wire [8:0] target,
    output wire management_clk,
    output wire reference_clk,
    output wire sample_clk,
    output wire process_clk,
    output wire delay_clk,
    output wire board_locked,
    output wire locked,
    output wire phase_ready,
    output wire [8:0] phase
);
    wire control_clk, board_feedback, board_feedback_buffered, cmanage, c200;
    BUFG control_buffer (.I(oscillator), .O(control_clk));
    BUFG reference_buffer (.I(reference), .O(reference_clk));
    BUFG management_buffer (.I(cmanage), .O(management_clk));
    BUFG delay_buffer (.I(c200), .O(delay_clk));
    BUFG board_feedback_buffer (.I(board_feedback), .O(board_feedback_buffered));
    MMCME2_BASE #(.CLKIN1_PERIOD(10.0), .CLKFBOUT_MULT_F(12.0),
                   .CLKOUT0_DIVIDE_F(10.0), .CLKOUT1_DIVIDE(6)) board_mmcm (
        .CLKIN1(control_clk), .CLKFBIN(board_feedback_buffered), .CLKFBOUT(board_feedback),
        .CLKOUT0(cmanage), .CLKOUT1(c200), .LOCKED(board_locked), .RST(1'b0), .PWRDWN(1'b0));

    wire feedback, feedback_buffered, c240, c120, phase_done;
    BUFG feedback_buffer (.I(feedback), .O(feedback_buffered));
    BUFG sample_buffer (.I(c240), .O(sample_clk));
    BUFG process_buffer (.I(c120), .O(process_clk));
    (* ASYNC_REG = "TRUE" *) reg [8:0] target_meta = 0, target_sync = 0;
    (* ASYNC_REG = "TRUE" *) reg [1:0] lock_sync = 0;
    reg [8:0] current = 0;
    reg phase_enable = 0, increment = 0, waiting = 0;
    always @(posedge control_clk) begin
        target_meta <= target;
        target_sync <= target_meta;
        lock_sync <= {lock_sync[0], locked};
        phase_enable <= 0;
        if (!lock_sync[1]) begin
            current <= 0;
            waiting <= 0;
        end else if (waiting) begin
            if (phase_done) begin
                current <= increment ? current + 1'b1 : current - 1'b1;
                waiting <= 0;
            end
        end else if (current != target_sync) begin
            phase_enable <= 1;
            increment <= current < target_sync;
            waiting <= 1;
        end
    end
    MMCME2_ADV #(
        .CLKIN1_PERIOD(50.0), .CLKFBOUT_MULT_F(48.0),
        .CLKOUT0_DIVIDE_F(4.0), .CLKOUT0_USE_FINE_PS("TRUE"), .CLKOUT1_DIVIDE(8)
    ) mmcm (
        .CLKIN1(reference_clk), .CLKIN2(1'b0), .CLKINSEL(1'b1),
        .CLKFBIN(feedback_buffered), .CLKFBOUT(feedback),
        .CLKOUT0(c240), .CLKOUT1(c120), .LOCKED(locked), .RST(source_reset), .PWRDWN(1'b0),
        .PSCLK(control_clk), .PSEN(phase_enable), .PSINCDEC(increment), .PSDONE(phase_done),
        .DADDR(7'b0), .DCLK(1'b0), .DEN(1'b0), .DI(16'b0), .DWE(1'b0));

    wire settled = lock_sync[1] && !waiting && current == target_sync && target_sync == target_meta;
    (* ASYNC_REG = "TRUE" *) reg [9:0] status_meta = 0, status_sync = 0;
    always @(posedge management_clk) begin
        status_meta <= {settled, current};
        status_sync <= status_meta;
    end
    assign phase_ready = status_sync[9] && status_sync[8:0] == target;
    assign phase = status_sync[8:0];
endmodule
