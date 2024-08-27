`timescale 1ns / 1ps

module top(
input [11:0] datain, 
input clk, rst_n, 
output clk_adc, clk_fifo, clk_ILA, clk_SPI, full_o, empty_o, wren_i, rden_i, 
output o_TX_Ready_, o_SPI_Clk_,
output dataout
);

wire [11:0] fifo_out_spi_in;
 
(* mark_debug = "true" *) wire debug_full_o = full_o;
(* mark_debug = "true" *) wire debug_empty_o = empty_o; 
(* mark_debug = "true" *) wire debug_wren_i = wren_i;   
(* mark_debug = "true" *) wire debug_rden_i = rden_i;   

    
  fifo f1(
   .clk(clk_fifo), //20MHz
   .clk_delay(clk_delay), //8MHz 
   .rst_n(rst_n),
   .wdata_i(datain),
   .full_o(debug_full_o),
   .empty_o(debug_empty_o),
   .rdata_o(fifo_out_spi_in),
   .rden_i(debug_rden_i),
   .wren_i(debug_wren_i)
  );
  
   SPI_Master s1(
    .i_Rst_L(rst_n),
    .i_Clk(clk_SPI),
    .i_TX_Byte(fifo_out_spi_in),
    //.i_TX_DV(i_TX_DV_),
    .o_TX_Ready(o_TX_Ready_),
    //.o_RX_DV(o_RX_DV_),
    .o_SPI_Clk(o_SPI_Clk_),
    //.i_SPI_MISO(i_SPI_MISO_),
    .o_SPI_MOSI(dataout)
    );
  
  
  clk_wiz_0 c1
   (
    // Clock out port4
    .clk_SPI(clk_SPI),
    .clk_ILA(clk_ILA),
    .clk_fifo(clk_fifo), 
    .clk_adc(clk_adc),    
    .clk_delay(clk_delay),
    // Clock in ports 
    .clk_in1(clk)      // input clk_in1 //입력받는 신호 clk을 clk_in1에 넣어서 
    //근데 clocking wizard에서 clk_in1에 시스템 클럭(125MHz)할당해놓음.
);



endmodule
