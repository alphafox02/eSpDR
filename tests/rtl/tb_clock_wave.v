`timescale 1ns/1ps
module tb;
 reg clk=0,reference=0,clear=1,active=0,inject=0;
 always #2.5 clk=~clk;
 always begin #25;reference=~reference;end
 wire pin=reference^inject;
 wire[383:0] report;
 clock_wave_monitor dut(.clk(clk),.reference(pin),.clear(clear),.active(active),.report(report));
 reg[255:0] frozen;
 initial begin
  #100;clear=0;active=1;#2000;
  if(report[31:0]!=0||report[63:32]!=32'h05050505)$fatal(1,"healthy clock %h",report[63:0]);
  @(posedge reference);#8;inject=1;#10;inject=0;#500;
  if(report[31:0]==0||report[127:96]==0||report[95:64]==0)$fatal(1,"missing anomaly %h",report[127:0]);
  frozen=report[383:128];#2000;
  if(report[383:128]!=frozen)$fatal(1,"first anomaly overwritten");
  clear=1;#100;clear=0;#500;
  if(report[31:0]!=0||report[383:64]!=0)$fatal(1,"clear did not erase evidence");
  $display("PASS pulse monitor: healthy20 MHz, 10 ns glitch, latched first history, explicit clear");$finish;
 end
endmodule
