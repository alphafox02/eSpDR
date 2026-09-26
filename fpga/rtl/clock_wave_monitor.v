// Coarse pulse-width diagnostic using the independent 200 MHz Au clock.
// Expected input is 20 MHz, 50% duty. This cannot measure sub-5 ns jitter.
module clock_wave_monitor(
 input wire clk, reference, clear, active,
 output wire [383:0] report
);
 (* ASYNC_REG = "TRUE" *) reg [1:0] pin_sync=0, clear_sync=3, active_sync=0;
 always @(posedge clk) begin
  pin_sync <= {pin_sync[0],reference};
  clear_sync <= {clear_sync[0],clear};
  active_sync <= {active_sync[0],active};
 end
 reg previous=0,started=0;
 reg [7:0] length=0, min_high=255,max_high=0,min_low=255,max_low=0;
 reg [31:0] anomalies=0,ticks=0,first_tick=0,first_width=0;
 reg [255:0] history=0,first_history=0;
 wire [7:0] width=length==255?8'd255:length+1'b1;
 assign report={first_history,first_width,first_tick,max_high,min_high,max_low,min_low,anomalies};
 always @(posedge clk) begin
  history <= {history[254:0],pin_sync[1]};
  previous <= pin_sync[1];
  if(clear_sync[1]) begin
   started<=0;length<=0;min_high<=255;max_high<=0;min_low<=255;max_low<=0;
   anomalies<=0;ticks<=0;first_tick<=0;first_width<=0;first_history<=0;
  end else if(active_sync[1]) begin
   ticks<=ticks+1'b1;
   if(pin_sync[1]!=previous) begin
    length<=0;started<=1;
    if(started) begin
     if(previous) begin
      if(width<min_high)min_high<=width;
      if(width>max_high)max_high<=width;
     end else begin
      if(width<min_low)min_low<=width;
      if(width>max_low)max_low<=width;
     end
     if(width<3||width>7) begin
      anomalies<=anomalies+1'b1;
      if(anomalies==0) begin
       first_tick<=ticks;first_width<={23'b0,previous,width};
       first_history<={history[254:0],pin_sync[1]};
      end
     end
    end
   end else if(length!=255)length<=length+1'b1;
  end else begin started<=0;length<=0;end
 end
endmodule
