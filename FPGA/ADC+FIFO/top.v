`timescale 1ns / 1ps

module top(
input [11:0] datain, input clk, rst_n, output clk_adc, clk_fifo, clk_test, full_o, empty_o, wren_i, rden_i, output reg  [11:0] dataout
    );
    
// Debug 신호 선언
(* mark_debug = "true" *) wire [11:0] debug_datain = datain;
(* mark_debug = "true" *) wire [11:0] debug_dataout=dataout;
(* mark_debug = "true" *) wire debug_full_o = full_o;
(* mark_debug = "true" *) wire debug_empty_o = empty_o;    
(* mark_debug = "true" *) wire debug_wren_i = wren_i;   
(* mark_debug = "true" *) wire debug_rden_i = rden_i;   

    
  fifo f1(
   .clk(clk_fifo), //20MHz
   .clk_delay(clk_delay), //8MHz 
   .rst_n(rst_n),
   .wdata_i(debug_datain),
   .full_o(debug_full_o),
   .empty_o(debug_empty_o),
   .rdata_o(debug_dataout),
   .rden_i(debug_rden_i),
   .wren_i(debug_wren_i)
  );
  
  
  clk_wiz_0 instance_name
   (
    // Clock out port4
    .clk_test(clk_test),
    .clk_fifo(clk_fifo), 
    .clk_adc(clk_adc),    
    .clk_delay(clk_delay),
    // Clock in ports 
    .clk_in1(clk)      // input clk_in1 //입력받는 신호 clk을 clk_in1에 넣어서 
    //근데 clocking wizard에서 clk_in1에 시스템 클럭(125MHz)할당해놓음.
);

endmodule
