`timescale 1ns/1ps
module jamming_waveform(input clk, output reg [13:0] sawtooth, output reg check_led, output reg toggle);

localparam sawtoothPeriod=16384;

initial begin sawtooth=14'b10000000000000; check_led=1'b0; toggle=1'b0; end
always@(posedge clk)
begin
if(sawtooth==(sawtoothPeriod-1)) sawtooth <=8192; 
else begin 
sawtooth <= sawtooth+1'b1; check_led=~check_led; toggle=~toggle; end 
end

endmodule
