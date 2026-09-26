`include "protocol.vh"
// ESP32-S3 IQ link receiver and USB streamer for Alchitry Au + Ft (+ Br).
//
//   link (16 lines, 240 MHz sampling) -> two lane receivers -> stream.v
//   (reorder, lossless encoder, 256 MiB DDR3 ring) -> FT600 USB 3.
//
// Control is over the FT2232 UART (1 Mbaud, protocol/control.h). The FPGA
// stays alive independently of the ESP-forwarded acquisition reference.
module iqstream_top (
    input  wire        clk100,
    input  wire        rst_n,
    input  wire        uart_rx,
    output wire        uart_tx,

    input  wire [15:0] link,
    input  wire        esp_clock,

    input  wire        ft_clk,
    input  wire        ft_txe,
    output wire        ft_wr,
    output wire        ft_rd,
    output wire        ft_oe,
    inout  wire [15:0] ft_data,
    inout  wire [1:0]  ft_be,

    inout  wire [15:0] ddr3_dq,
    inout  wire [1:0]  ddr3_dqs_n,
    inout  wire [1:0]  ddr3_dqs_p,
    output wire [13:0] ddr3_addr,
    output wire [2:0]  ddr3_ba,
    output wire        ddr3_ras_n,
    output wire        ddr3_cas_n,
    output wire        ddr3_we_n,
    output wire        ddr3_reset_n,
    output wire [0:0]  ddr3_ck_p,
    output wire [0:0]  ddr3_ck_n,
    output wire [0:0]  ddr3_cke,
    output wire [0:0]  ddr3_cs_n,
    output wire [0:0]  ddr3_odt,
    output wire [1:0]  ddr3_dm
);
    localparam UART_DIVIDER = 120;      // 120 MHz / 1 Mbaud

    assign ft_rd = 1'b1;                // write-only FT600 channel
    assign ft_oe = 1'b1;

    // ---- clocks and reset ----------------------------------------------------------------
    // Link sampling phase at start-up, in MMCM steps; measured for the
    // documented wiring. `iqstream calibrate` measures it for other builds.
    localparam DEFAULT_PHASE = 21;

    wire oscillator, reference, reference_clk, management_clk, sample_clk, clk, delay_clk;
    wire board_locked, source_locked, phase_ready, clock_present, reset, source_reset;
    wire [31:0] clock_hz;
    wire [8:0] phase;
    reg [8:0] phase_target = DEFAULT_PHASE;
    IBUF oscillator_buffer (.I(clk100), .O(oscillator));
    IBUF reference_buffer (.I(esp_clock), .O(reference));
    clocks clocks (
        .oscillator(oscillator), .reference(reference), .source_reset(source_reset),
        .target(phase_target), .sample_clk(sample_clk), .process_clk(clk),
        .management_clk(management_clk), .reference_clk(reference_clk), .delay_clk(delay_clk),
        .board_locked(board_locked), .locked(source_locked), .phase_ready(phase_ready), .phase(phase));

    (* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync = 0;
    reg [5:0] boot = 0;
    always @(posedge management_clk) begin
        reset_sync <= {reset_sync[0], rst_n && board_locked};
        if (!reset_sync[1]) boot <= 0;
        else if (!(&boot)) boot <= boot + 1'b1;
    end
    assign reset = !(&boot);
    clock_monitor source_monitor (.source_clk(reference_clk), .clk(management_clk), .reset(reset),
        .present(clock_present), .frequency_hz(clock_hz));
    (* ASYNC_REG = "TRUE" *) reg [1:0] source_lock_sync = 0;
    always @(posedge management_clk) source_lock_sync <= {source_lock_sync[0], source_locked};
    // A LOCKED drop does not itself guarantee the MMCM's fine-phase state
    // resets. Explicitly reset after loss so the phase counter restarts at 0.
    reg [9:0] source_reset_count = 1023;
    reg previously_locked = 0;
    assign source_reset = reset || source_reset_count != 0;
    always @(posedge management_clk) begin
        if (reset || !clock_present) begin
            source_reset_count <= 1023;
            previously_locked <= 0;
        end else if (source_reset_count != 0) source_reset_count <= source_reset_count - 1'b1;
        else if (source_lock_sync[1]) previously_locked <= 1;
        else if (previously_locked) begin
            source_reset_count <= 1023;
            previously_locked <= 0;
        end
    end
    wire source_ready = clock_present && source_lock_sync[1] && !source_reset;
    reg clock_fault = 0;
    reg [31:0] clock_fault_detail = 0;
    (* ASYNC_REG = "TRUE" *) reg [1:0] reference_level_sync = 0;
    reg [127:0] reference_history = 0, fault_history = 0;
    always @(posedge management_clk) begin
        reference_level_sync <= {reference_level_sync[0], reference};
        reference_history <= {reference_history[126:0], reference_level_sync[1]};
    end

    wire [383:0] wave_report, reported_wave;
    clock_wave_monitor waveform (.clk(delay_clk), .reference(reference),
        .clear(clear), .active(stream_open), .report(wave_report));
    snapshot #(.WIDTH(384)) waveform_report (.source_clk(delay_clk), .source_value(wave_report),
        .clk(management_clk), .value(reported_wave));

    // Run controls are held levels synchronized into the acquisition domain.
    // Clear lasts sixteen management clocks; capture commands/replies take far
    // longer than the synchronizer latency. Clock loss never clears DDR/FIFOs.
    reg arm = 0, disarm = 0, armed = 0, stream_started = 0, stream_open = 0;
    reg [4:0] clear_count = 15;
    reg clear = 1;
    always @(posedge management_clk) begin
        if (reset || arm) clear_count <= 15;
        else if (clear_count != 0) clear_count <= clear_count - 1'b1;
        clear <= reset || clear_count != 0;
        if (reset || disarm) armed <= 0;
        else if (arm) armed <= source_ready;
        else if (!source_ready || clock_fault) armed <= 0;
        if (reset) stream_started <= 0;
        else if (arm) stream_started <= 1;
        if (reset || arm) begin
            clock_fault <= 0;
            clock_fault_detail <= 0;
            fault_history <= 0;
        end else if (stream_open && !source_ready && !clock_fault) begin
            clock_fault <= 1;
            clock_fault_detail <= {!source_lock_sync[1], !clock_present, clock_hz[29:0]};
            fault_history <= reference_history;
        end
    end
    (* ASYNC_REG = "TRUE" *) reg [3:0] acquisition_meta = 4'b1000, acquisition_sync = 4'b1000;
    always @(posedge clk) begin
        acquisition_meta <= {clear, armed, stream_started, clock_fault};
        acquisition_sync <= acquisition_meta;
    end
    wire acq_clear = acquisition_sync[3];
    wire acq_run = acquisition_sync[2] && !acquisition_sync[0];
    wire acq_flush = acquisition_sync[1] && !acquisition_sync[2] && !acquisition_sync[0];

    // ---- link receive ------------------------------------------------------------------------------
    reg [15:0] late_target = 0;
    (* ASYNC_REG = "TRUE" *) reg [15:0] late_meta = 0, late_sync = 0;
    always @(posedge clk) begin late_meta <= late_target; late_sync <= late_meta; end
    reg [79:0] link_taps = {5'd0, 5'd0, 5'd5, 5'd4, 5'd2, 5'd5, 5'd2, 5'd5,
                            5'd2, 5'd3, 5'd3, 5'd1, 5'd3, 5'd4, 5'd7, 5'd3};
    wire [255:0] marker_bit_errors [0:1];
    wire [511:0] reported_marker_errors;
    snapshot #(.WIDTH(512)) marker_report (.source_clk(clk),
        .source_value({marker_bit_errors[1],marker_bit_errors[0]}),
        .clk(management_clk), .value(reported_marker_errors));
    wire        deskew_ready, pair_valid;
    wire [63:0] sample_pair;
    wire [31:0] sample_overflow;
    link_input link_input (
        .link(link), .sample_clk(sample_clk), .delay_clk(delay_clk), .clk(clk),
        .reset(reset), .clear(acq_clear), .run(acq_run), .tap_settings(link_taps), .deskew_ready(deskew_ready),
        .pair_valid(pair_valid), .pair(sample_pair), .overflow(sample_overflow));

    wire [1:0]  byte_valid, lane_valid, lane_last;
    wire [7:0]  byte_value [0:1];
    wire [15:0] byte_index [0:1];
    wire [31:0] lane_sequence [0:1];
    wire [1:0]  unit_valid;
    wire [13:0] unit_first [0:1], unit_count [0:1];
    wire [3:0]  head_pairs [0:1], tail_pairs [0:1];
    wire [10:0] total_groups [0:1];
    wire [19:0] lane_pair [0:1];
    wire [31:0] units [0:1], framing_errors [0:1], unpack_errors [0:1];
    wire [31:0] checksum_errors [0:1], end_marker_errors [0:1];
    genvar lane;
    generate
        for (lane = 0; lane < 2; lane = lane + 1) begin : lanes
            link_lane #(.LANE(lane)) receive (
                .clk(clk), .reset(acq_clear), .run(acq_run), .data_late(late_sync[lane*8+:8]), .pair_valid(pair_valid),
                .samples({sample_pair[48 + lane * 8 +: 8], sample_pair[32 + lane * 8 +: 8],
                          sample_pair[16 + lane * 8 +: 8], sample_pair[lane * 8 +: 8]}),
                .byte_valid(byte_valid[lane]), .byte_value(byte_value[lane]),
                .byte_index(byte_index[lane]), .sequence_word(lane_sequence[lane]),
                .unit_valid(unit_valid[lane]), .unit_first(unit_first[lane]),
                .unit_count(unit_count[lane]),
                .head_pairs(head_pairs[lane]), .tail_pairs(tail_pairs[lane]),
                .total_groups(total_groups[lane]), .units(units[lane]),
                .framing_errors(framing_errors[lane]), .checksum_errors(checksum_errors[lane]),
                .end_marker_errors(end_marker_errors[lane]), .marker_bit_errors(marker_bit_errors[lane]));
            link_unpack unpack (
                .clk(clk), .reset(acq_clear), .byte_valid(byte_valid[lane]),
                .byte_value(byte_value[lane]), .byte_index(byte_index[lane]),
                .head_pairs(head_pairs[lane]), .tail_pairs(tail_pairs[lane]),
                .total_groups(total_groups[lane]), .pair_valid(lane_valid[lane]),
                .pair(lane_pair[lane]), .last(lane_last[lane]), .errors(unpack_errors[lane]));
        end
    endgenerate

    // ---- DDR3 --------------------------------------------------------------------------------------------
    reg [7:0] ddr_reset_count = 255;
    always @(posedge management_clk)
        if (reset) ddr_reset_count <= 255;
        else if (ddr_reset_count != 0) ddr_reset_count <= ddr_reset_count - 1'b1;

    wire         ui_clk, ui_reset, ddr_calibrated;
    wire [27:0]  app_addr;
    wire [2:0]   app_cmd;
    wire         app_en, app_wdf_end, app_wdf_wren, app_rdy, app_wdf_rdy, app_rd_data_valid;
    wire [127:0] app_wdf_data, app_rd_data;
    wire [15:0]  app_wdf_mask;
    ddr_memory ddr (
        .oscillator(oscillator), .reference_200(delay_clk), .reset(reset || ddr_reset_count != 0),
        .ui_clk(ui_clk), .ui_reset(ui_reset), .calibrated(ddr_calibrated),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_end(app_wdf_end), .app_wdf_wren(app_wdf_wren),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid),
        .ddr3_dq(ddr3_dq), .ddr3_dqs_n(ddr3_dqs_n), .ddr3_dqs_p(ddr3_dqs_p),
        .ddr3_addr(ddr3_addr), .ddr3_ba(ddr3_ba), .ddr3_ras_n(ddr3_ras_n),
        .ddr3_cas_n(ddr3_cas_n), .ddr3_we_n(ddr3_we_n), .ddr3_reset_n(ddr3_reset_n),
        .ddr3_ck_p(ddr3_ck_p), .ddr3_ck_n(ddr3_ck_n), .ddr3_cke(ddr3_cke),
        .ddr3_cs_n(ddr3_cs_n), .ddr3_odt(ddr3_odt), .ddr3_dm(ddr3_dm));

    (* ASYNC_REG = "TRUE" *) reg [1:0] calibrated_sync = 0;
    always @(posedge management_clk) calibrated_sync <= {calibrated_sync[0], ddr_calibrated};
    wire memory_ready = calibrated_sync[1];

    // ---- stream -----------------------------------------------------------------------------------------------
    wire [63:0] stream_pairs, lost_pairs;
    wire [31:0] stream_crc, records, reorder_overflow0, reorder_overflow1, lost_units, discarded_units;
    wire [31:0] ring_input_overflow, ring_output_overflow, ring_errors, usb_underruns, usb_max_stall;
    wire [14:0] reorder_peak0, reorder_peak1;
    wire [24:0] ring_used, ring_peak;
    wire        stream_ended;
    stream stream (
        .clk(clk), .reset(acq_clear), .management_clk(management_clk),
        .clear_stream(clear), .flush(acq_flush),
        .valid(lane_valid), .last(lane_last), .pair0(lane_pair[0]), .pair1(lane_pair[1]),
        .unit_valid(unit_valid), .unit_sequence0(lane_sequence[0]),
        .unit_sequence1(lane_sequence[1]), .unit_first0(unit_first[0]),
        .unit_first1(unit_first[1]), .unit_count0(unit_count[0]), .unit_count1(unit_count[1]),
        .ui_clk(ui_clk), .ui_reset(ui_reset), .calibrated(ddr_calibrated),
        .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_end(app_wdf_end), .app_wdf_wren(app_wdf_wren),
        .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data),
        .app_rd_data_valid(app_rd_data_valid),
        .ft_clk(ft_clk), .ft_txe(ft_txe), .ft_data(ft_data), .ft_be(ft_be), .ft_wr(ft_wr),
        .pairs(stream_pairs), .lost_pairs(lost_pairs), .lost_units(lost_units),
        .discarded_units(discarded_units),
        .stream_crc(stream_crc), .records(records), .ended(stream_ended),
        .reorder_overflow0(reorder_overflow0), .reorder_overflow1(reorder_overflow1),
        .reorder_peak0(reorder_peak0),
        .reorder_peak1(reorder_peak1), .ring_input_overflow(ring_input_overflow),
        .ring_output_overflow(ring_output_overflow), .ring_errors(ring_errors),
        .ring_used(ring_used), .ring_peak(ring_peak), .usb_underruns(usb_underruns),
        .usb_max_stall(usb_max_stall));

    // Coherent acquisition snapshots reach the always-running management domain.
    function automatic [31:0] source_statistic(input [15:0] index);
        case (index)
            `FPGA_STAT_PAIRS_LO: source_statistic = stream_pairs[31:0];
            `FPGA_STAT_PAIRS_HI: source_statistic = stream_pairs[63:32];
            `FPGA_STAT_STREAM_CRC: source_statistic = stream_crc;
            `FPGA_STAT_RECORDS: source_statistic = records;
            `FPGA_STAT_UNITS0: source_statistic = units[0];
            `FPGA_STAT_UNITS1: source_statistic = units[1];
            `FPGA_STAT_FRAMING0: source_statistic = framing_errors[0] + unpack_errors[0];
            `FPGA_STAT_FRAMING1: source_statistic = framing_errors[1] + unpack_errors[1];
            `FPGA_STAT_CHECKSUM0: source_statistic = checksum_errors[0];
            `FPGA_STAT_CHECKSUM1: source_statistic = checksum_errors[1];
            `FPGA_STAT_END_MARK0: source_statistic = end_marker_errors[0];
            `FPGA_STAT_END_MARK1: source_statistic = end_marker_errors[1];
            `FPGA_STAT_SAMPLE_OVERFLOW: source_statistic = sample_overflow;
            `FPGA_STAT_REORDER_OVERFLOW0: source_statistic = reorder_overflow0;
            `FPGA_STAT_REORDER_OVERFLOW1: source_statistic = reorder_overflow1;
            `FPGA_STAT_LOST_UNITS: source_statistic = lost_units;
            `FPGA_STAT_RING_INPUT_OVERFLOW: source_statistic = ring_input_overflow;
            `FPGA_STAT_REORDER_PEAK0: source_statistic = {17'b0, reorder_peak0};
            `FPGA_STAT_REORDER_PEAK1: source_statistic = {17'b0, reorder_peak1};
            `FPGA_STAT_LOST_PAIRS_LO: source_statistic = lost_pairs[31:0];
            `FPGA_STAT_LOST_PAIRS_HI: source_statistic = lost_pairs[63:32];
            `FPGA_STAT_DISCARDED_UNITS: source_statistic = discarded_units;
            default: source_statistic = 0;
        endcase
    endfunction

    wire [32*`FPGA_STAT_COUNT+1:0] acquisition_status;
    wire [32*`FPGA_STAT_COUNT+1:0] reported_status;
    genvar stat;
    generate for (stat=0;stat<`FPGA_STAT_COUNT;stat=stat+1) begin : status_words
        assign acquisition_status[stat*32+:32] = source_statistic(16'(stat));
    end endgenerate
    assign acquisition_status[32*`FPGA_STAT_COUNT+:2] = {deskew_ready, stream_ended};
    snapshot #(.WIDTH(32*`FPGA_STAT_COUNT+2)) acquisition_report (
        .source_clk(clk), .source_value(acquisition_status), .clk(management_clk), .value(reported_status));
    wire reported_ended = reported_status[32*`FPGA_STAT_COUNT];
    wire reported_deskew = reported_status[32*`FPGA_STAT_COUNT+1];
    wire [31:0] flags = (armed ? `FPGA_FLAG_ARMED : 0) |
                        (memory_ready ? `FPGA_FLAG_DDR_READY : 0) |
                        (reported_deskew && source_ready ? `FPGA_FLAG_DESKEW_READY : 0) |
                        (phase_ready && source_ready ? `FPGA_FLAG_PHASE_READY : 0) |
                        (source_ready ? `FPGA_FLAG_REFERENCE_ON : 0) |
                        (reported_ended && !clock_fault ? `FPGA_FLAG_STREAM_ENDED : 0) |
                        (stream_open ? `FPGA_FLAG_STREAM_OPEN : 0) |
                        (clock_fault ? `FPGA_FLAG_CLOCK_FAULT : 0);
    function automatic [31:0] statistic(input [15:0] index);
        case (index)
            `FPGA_STAT_FLAGS: statistic = flags;
            `FPGA_STAT_RING_OUTPUT_OVERFLOW: statistic = ring_output_overflow;
            `FPGA_STAT_RING_ERRORS: statistic = ring_errors;
            `FPGA_STAT_RING_USED: statistic = {7'b0, ring_used};
            `FPGA_STAT_RING_PEAK: statistic = {7'b0, ring_peak};
            `FPGA_STAT_USB_UNDERRUNS: statistic = usb_underruns;
            `FPGA_STAT_USB_MAX_STALL: statistic = usb_max_stall;
            `FPGA_STAT_PHASE: statistic = {23'b0, phase};
            `FPGA_STAT_CLOCK_HZ: statistic = clock_hz;
            `FPGA_STAT_CLOCK_FAULT_DETAIL: statistic = clock_fault_detail;
            `FPGA_STAT_CLOCK_ANOMALIES: statistic = reported_wave[31:0];
            `FPGA_STAT_CLOCK_WIDTHS: statistic = reported_wave[63:32];
            default: statistic = index < `FPGA_STAT_COUNT ? reported_status[index*32+:32] : 0;
        endcase
    endfunction

    // ---- control ------------------------------------------------------------------------------------------------
    wire        request_valid;
    wire [7:0]  op;
    wire [15:0] arg;
    reg  [7:0]  status = 0;
    wire [31:0] value = op == `CTL_INFO ? 32'(`CTL_FPGA_FIRMWARE_ID)
                      : op == `CTL_STATUS ? statistic(arg)
                      : op == `FPGA_CLOCK_DEBUG ? (arg == 0 ? reference_history[31:0] : arg <= 4 ? fault_history[(arg-1)*32+:32] : reported_wave[(arg-5)*32+:32])
                      : op == `FPGA_GET_DELAY ? {27'b0,link_taps[arg*5+:5]}
                      : op == `FPGA_MARKER_ERRORS ? reported_marker_errors[arg*32+:32]
                      : op == `FPGA_GET_LATE_DATA ? {16'b0,late_target}
                      : flags;
    control_port #(.DIVIDER(UART_DIVIDER)) control (
        .clk(management_clk), .reset(reset), .rx(uart_rx), .tx(uart_tx), .request_valid(request_valid),
        .op(op), .arg(arg), .status(status), .value(value));

    always @(posedge management_clk) begin
        arm <= 0;
        disarm <= 0;
        if (reset) begin
            phase_target <= DEFAULT_PHASE;
            stream_open <= 0;
        end else if (request_valid) begin
            status <= `CTL_OK;
            case (op)
                `CTL_INFO: if (arg != 0) status <= `CTL_BAD_ARGUMENT;
                `CTL_SAFE: disarm <= 1;
                `CTL_STATUS: if (arg >= `FPGA_STAT_COUNT) status <= `CTL_BAD_ARGUMENT;
                `FPGA_CLOCK_DEBUG: if (arg > 16) status <= `CTL_BAD_ARGUMENT;
                `FPGA_SET_DELAY: if (arg >= 512) status <= `CTL_BAD_ARGUMENT;
                    else if (stream_open) status <= `CTL_BUSY;
                    else link_taps[arg[8:5]*5+:5] <= arg[4:0];
                `FPGA_GET_DELAY,`FPGA_MARKER_ERRORS: if (arg >= 16) status <= `CTL_BAD_ARGUMENT;
                `FPGA_SET_LATE_DATA: if (stream_open) status <= `CTL_BUSY;
                    else late_target <= arg;
                `FPGA_GET_LATE_DATA: if (arg != 0) status <= `CTL_BAD_ARGUMENT;
                `FPGA_REFERENCE: status <= `CTL_BAD_ARGUMENT; // no FPGA output clock
                `FPGA_ARM:
                    if (stream_open) status <= `CTL_BUSY;
                    else if (memory_ready && reported_deskew && phase_ready && source_ready) begin
                        arm <= 1;
                        stream_open <= 1;
                    end else status <= `CTL_NOT_READY;
                `FPGA_STOP: disarm <= 1;
                `FPGA_RELEASE: begin
                    disarm <= 1;
                    stream_open <= 0;
                end
                `FPGA_PHASE:
                    if (arg >= `FPGA_PHASE_STEPS) status <= `CTL_BAD_ARGUMENT;
                    else if (stream_open) status <= `CTL_BUSY;
                    else phase_target <= arg[8:0];
                default: status <= `CTL_UNKNOWN_OP;
            endcase
        end
    end
endmodule
