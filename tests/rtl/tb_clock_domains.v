`timescale 1ns/1ps
module IBUF(input I,output O);assign O=I;endmodule
// Functional clock-domain/control test; clock generators, sampling pads and
// MIG are stubbed. Real codec/DDR-ring RTL is tested separately by tb_stream.
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
 reg cm=0,cs=0,cp=0,cd=0,lock_loss=0;
 always #4.166667 cm=~cm;
 always #2.083333 cs=~cs;
 always #4.166667 cp=~cp;
 always #2.5 cd=~cd;
 assign management_clk=cm, reference_clk=reference;
 assign sample_clk=source_reset?1'b0:cs;
 assign process_clk=source_reset?1'b0:cp;
 assign delay_clk=cd,board_locked=1,locked=!source_reset&&!lock_loss;
 assign phase_ready=!source_reset,phase=target;
endmodule
module link_input #(
    // Tap values (78 ps each) for link lines 15..0, measured with the ESP
    // driving 10 mA: lines whose data ends early are delayed so that every
    // line's data stays valid as long after the marker edge as possible.
    parameter [79:0] TAPS = {5'd0, 5'd0, 5'd5, 5'd4, 5'd2, 5'd5, 5'd2, 5'd5,
                             5'd2, 5'd3, 5'd3, 5'd1, 5'd3, 5'd4, 5'd7, 5'd3}
) (
    input  wire [15:0] link,
    input  wire        sample_clk,      // 240 MHz
    input  wire        delay_clk,       // 200 MHz IDELAY reference
    input  wire        clk,             // 120 MHz
    input  wire        reset,           // clk domain
    input  wire        clear,           // clk domain: clears the capture path
    input  wire [79:0] tap_settings,
    input  wire        run,             // clk domain
    output wire        deskew_ready,    // clk domain
    output wire        pair_valid,
    // Per 120 MHz clock: {falling, rising} samples of the later period above
    // those of the earlier one; each sample is the 16 lines.
    output wire [63:0] pair,
    output wire [31:0] overflow         // clk domain: capture FIFO overflows
);
 assign deskew_ready=!reset,pair_valid=0,pair=0,overflow=0;
endmodule
module ddr_memory (
    input  wire         oscillator,     // 100 MHz, from an IBUF
    input  wire         reference_200,
    input  wire         reset,
    output wire         ui_clk,
    output wire         ui_reset,
    output wire         calibrated,

    input  wire [27:0]  app_addr,
    input  wire [2:0]   app_cmd,
    input  wire         app_en,
    input  wire [127:0] app_wdf_data,
    input  wire [15:0]  app_wdf_mask,
    input  wire         app_wdf_end,
    input  wire         app_wdf_wren,
    output wire         app_rdy,
    output wire         app_wdf_rdy,
    output wire [127:0] app_rd_data,
    output wire         app_rd_data_valid,

    inout  wire [15:0]  ddr3_dq,
    inout  wire [1:0]   ddr3_dqs_n,
    inout  wire [1:0]   ddr3_dqs_p,
    output wire [13:0]  ddr3_addr,
    output wire [2:0]   ddr3_ba,
    output wire         ddr3_ras_n,
    output wire         ddr3_cas_n,
    output wire         ddr3_we_n,
    output wire         ddr3_reset_n,
    output wire [0:0]   ddr3_ck_p,
    output wire [0:0]   ddr3_ck_n,
    output wire [0:0]   ddr3_cke,
    output wire [0:0]   ddr3_cs_n,
    output wire [0:0]   ddr3_odt,
    output wire [1:0]   ddr3_dm
);
 reg c=0;always #6 c=~c;
 assign ui_clk=c,ui_reset=reset,calibrated=!reset;
 assign app_rdy=1,app_wdf_rdy=1,app_rd_data=0,app_rd_data_valid=0;
endmodule
module tb;
 reg osc=0,refclk=0,ft=0,rst_n=0,reference_enabled=1;
 always #5 osc=~osc;
 always #25 refclk=~refclk;
 always #5 ft=~ft;
 wire esp_clock=reference_enabled&&refclk;
 iqstream_top dut(.clk100(osc),.esp_clock(esp_clock),.rst_n(rst_n),.uart_rx(1'b1),.link(16'b0),.ft_clk(ft),.ft_txe(1'b1));
 reg request=0;reg[7:0]op=0;reg[15:0]arg=0;
 reg [127:0] saved_history;
 initial begin force dut.request_valid=request;force dut.op=op;force dut.arg=arg;end
 task command(input[7:0]what,input[7:0]expected_status=0);
  @(negedge dut.management_clk);op=what;request=1;
  @(negedge dut.management_clk);request=0;
  repeat(40)@(negedge dut.management_clk);
  if(dut.status!=expected_status)$fatal(1,"command %d status %d",what,dut.status);
 endtask
 initial begin
  #100;rst_n=1;#3000000;
  if((dut.flags&32'h1e)!=32'h1e)$fatal(1,"not ready %h",dut.flags);
  if(dut.clock_hz<19990000||dut.clock_hz>20010000)$fatal(1,"frequency %d",dut.clock_hz);
  arg=(3<<5)|12;command(22);
  arg=3;command(23);
  if(dut.value!=12)$fatal(1,"delay setting/readback");
  arg=16;command(23,2);
  arg=16'ha55a;command(25);
  arg=0;command(26);
  if(dut.value!=16'ha55a||dut.late_sync!=16'ha55a)$fatal(1,"later-sample setting/readback");
  command(25);
  command(17);
  if(!dut.armed||!dut.acq_run||!dut.stream_open)$fatal(1,"not armed");
  arg=(3<<5)|5;command(22,3);
  arg=16'hffff;command(25,3);
  if(dut.link_taps[15+:5]!=12||dut.late_target!=0)$fatal(1,"settings changed during capture");
  arg=0;
  // A PLL lock drop with a healthy incoming reference must force an MMCM
  // reset, preserve DDR, and remain a failed capture after automatic relock.
  dut.clocks.lock_loss=1;#500;
  if(!dut.clock_fault||!dut.source_reset||dut.armed||!dut.clock_present)
   $fatal(1,"PLL-only loss was not contained/reset");
  if(dut.clock_fault_detail[31:30]!=2'b10)
   $fatal(1,"PLL-only fault reason %h",dut.clock_fault_detail);
  if(!dut.memory_ready||dut.clear||dut.stream.ring_reset)
   $fatal(1,"PLL-only loss cleared DDR");
  saved_history=dut.fault_history;
  if(saved_history==0||saved_history=={128{1'b1}})
   $fatal(1,"missing toggling reference history at PLL fault");
  dut.clocks.lock_loss=0;#20000;
  if(!dut.source_ready||!dut.clock_fault||dut.fault_history!=saved_history)
   $fatal(1,"PLL-only recovery lost fault evidence or did not relock");
  command(20);command(17);
  if(!dut.armed||!dut.acq_run||dut.clock_fault||dut.fault_history!=0)
   $fatal(1,"cannot re-arm after PLL-only loss");
  reference_enabled=0;#2500000;
  if(!dut.clock_fault||dut.armed||!dut.memory_ready||dut.clear)$fatal(1,"clock fault containment");
  if(dut.stream.clear_sync[1]||dut.stream.ring_reset)$fatal(1,"clock loss cleared DDR");
  command(1);
  reference_enabled=1;#3000000;
  if(!dut.clock_fault||!dut.source_ready)$fatal(1,"fault not sticky or no relock");
  command(20);command(17);
  if(!dut.armed||!dut.acq_run||dut.clock_fault||dut.acq_clear)$fatal(1,"cannot re-arm after loss");
  $display("PASS source clock: frequency, PLL-only loss/reset/history, absent-clock management, DDR preservation, sticky fault, relock and re-arm");$finish;
 end
endmodule
