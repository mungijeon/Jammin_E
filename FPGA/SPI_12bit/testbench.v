module SPI_Master_TB ();
  
  parameter SPI_MODE = 0; // CPOL = 1, CPHA = 1
  parameter CLKS_PER_HALF_BIT = 4;  // 6.25 MHz SCK의 반주기에 fpga clk이 4번 들어감. 
  parameter MAIN_CLK_DELAY = 2;  // 25 MHz

  reg r_Rst_L     = 1'b0;  
  wire w_SPI_Clk;
  reg r_Clk       = 1'b0;
  wire w_SPI_MOSI;

  // Master Specific
  reg [11:0] r_Master_TX_Byte = 0;
  reg r_Master_TX_DV = 1'b0;
  wire w_Master_TX_Ready;
  wire r_Master_RX_DV;
  wire [7:0] r_Master_RX_Byte;

  // Clock Generators:
  always #(MAIN_CLK_DELAY) r_Clk = ~r_Clk;

  // Instantiate UUT
  SPI_Master 
  #(.SPI_MODE(SPI_MODE),
    .CLKS_PER_HALF_BIT(CLKS_PER_HALF_BIT)) SPI_Master_UUT
  (
   // Control/Data Signals,
   .i_Rst_L(r_Rst_L),     // FPGA Reset
   .i_Clk(r_Clk),         // FPGA Clock
   
   // TX (MOSI) Signals
   .i_TX_Byte(r_Master_TX_Byte),     // Byte to transmit on MOSI
   .i_TX_DV(r_Master_TX_DV),         // Data Valid Pulse with i_TX_Byte
   .o_TX_Ready(w_Master_TX_Ready),   // Transmit Ready for Byte
   
   // RX (MISO) Signals
   .o_RX_DV(r_Master_RX_DV),       // Data Valid pulse (1 clock cycle)
     // Byte received on MISO

   // SPI Interface
   .o_SPI_Clk(w_SPI_Clk),
   .i_SPI_MISO(w_SPI_MOSI),
   .o_SPI_MOSI(w_SPI_MOSI)
   );
  
  initial
    begin
      
      repeat(10) @(posedge r_Clk);
      r_Rst_L  = 1'b0;
      repeat(10) @(posedge r_Clk);
      r_Rst_L          = 1'b1;
      
      #100 $finish();      
    end // initial begin

endmodule // SPI_Slave
