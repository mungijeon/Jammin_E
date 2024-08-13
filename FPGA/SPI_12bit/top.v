`timescale 1ns / 1ps

module top(input clk, i_Rst_L_, i_TX_DV_, i_SPI_MISO_,  output clk_SPI, clk_ILA, o_TX_Ready_, o_RX_DV_, o_SPI_Clk_, o_SPI_MOSI_);
    clk_wiz_0 instance1(
    .clk_SPI(clk_SPI), //12MHz
    .clk_ILA(clk_ILA),
    .clk_in1(clk)
    );
    
    SPI_Master instance2(
    .i_Rst_L(i_Rst_L_),
    .i_Clk(clk_SPI),
     //지금 당장은 쓸 일 없음 (i_TX_Byte)
    .i_TX_DV(i_TX_DV_),
    .o_TX_Ready(o_TX_Ready_),
    .o_RX_DV(o_RX_DV_),
    .o_SPI_Clk(o_SPI_Clk_),
    .i_SPI_MISO(i_SPI_MISO_),
    .o_SPI_MOSI(o_SPI_MOSI_)
    );
    
    
endmodule
