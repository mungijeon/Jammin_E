
module SPI_Master
  #(parameter SPI_MODE = 0,
    parameter CLKS_PER_HALF_BIT = 1) //SPI Clock 주기의 반에 i_clk이 얼마나 들어가냐 
  (
   // Control/Data Signals,
   input        i_Rst_L,     // FPGA Reset 
   input        i_Clk,       // FPGA Clock -fpga reference clock 
   
   // TX (MOSI) Signals
   input [11:0]  i_TX_Byte,        // Byte to transmit on MOSI 
   input        i_TX_DV,          // Data Valid Pulse with i_TX_Byte 
   output reg   o_TX_Ready,       // Transmit Ready for next byte
   
   //MISO 안쓰니까 필요 없을듯
   // RX (MISO) Signals
   output reg       o_RX_DV,     // Data Valid pulse (1 clock cycle)
   //output reg [7:0] o_RX_Byte,   // Byte received on MISO
   

   // SPI Interface
   output reg o_SPI_Clk,
   input      i_SPI_MISO, 
   output reg o_SPI_MOSI 
   );

  // SPI Interface (All Runs at SPI Clock Domain)
  wire w_CPOL;     // Clock polarity
  wire w_CPHA;     // Clock phase

  reg [$clog2(CLKS_PER_HALF_BIT*2)-1:0] r_SPI_Clk_Count; 
  reg r_SPI_Clk;
  reg [4:0] r_SPI_Clk_Edges;
  reg r_Leading_Edge;
  reg r_Trailing_Edge;
  reg       r_TX_DV;
  reg [11:0] r_TX_Byte;

  reg [2:0] r_RX_Bit_Count;
  reg [3:0] r_TX_Bit_Count;

   reg [11:0] lut[0:15]; //for LUT 
  // CPOL: Clock Polarity
  // CPOL=0 means clock idles at 0, leading edge is rising edge.
  // CPOL=1 means clock idles at 1, leading edge is falling edge.
  assign w_CPOL  = (SPI_MODE == 2) | (SPI_MODE == 3);

  // CPHA: Clock Phase
  // CPHA=0 means the "out" side changes the data on trailing edge of clock
  //              the "in" side captures data on leading edge of clock
  // CPHA=1 means the "out" side changes the data on leading edge of clock
  //              the "in" side captures data on the trailing edge of clock
  assign w_CPHA  = (SPI_MODE == 1) | (SPI_MODE == 3);
  integer j; 
  integer i;
    initial begin
    for (i = 0; i < 16; i = i + 1) begin // 클럭 상관없이 데이터 넣기
        case (i)
            0:  lut[i] = 11'b11111111111;
            1:  lut[i] = 11'b00011111000;
            2:  lut[i] = 11'b11111000000;
            3:  lut[i] = 11'b10101010101;
            4:  lut[i] = 11'b11010011011;
            5:  lut[i] = 11'b11101000100;
            6:  lut[i] = 11'b11100011011;
            7:  lut[i] = 11'b10011010110;
            8:  lut[i] = 11'b10000011111;
            9:  lut[i] = 11'b11100110110;
            10: lut[i] = 11'b01110111011;
            11: lut[i] = 11'b10110100100;
            12: lut[i] = 11'b00110110100;
            13: lut[i] = 11'b11000011111;
            14: lut[i] = 11'b11110011011;
            15: lut[i] = 11'b10001111111;
            default: lut[i] = 11'b00000000000; // 기본값 설정
        endcase
    end
end


  // Purpose: Generate SPI Clock correct number of times when DV pulse comes
  //내가 원하는 SPI_clk(SCK) 만들어주는 부분
  always @(posedge i_Clk or negedge i_Rst_L)
  begin
    if (~i_Rst_L) //active-low로 동작하는 reset
    begin //초기
      o_TX_Ready      <= 1'b0;
      r_SPI_Clk_Edges <= 0;
      r_Leading_Edge  <= 1'b0;
      r_Trailing_Edge <= 1'b0;
      r_SPI_Clk       <= w_CPOL; // assign default state to idle state, w_CPOL 0이면 clk값 0부터 시작, 1이면 clk값 1부터 시작
      r_SPI_Clk_Count <= 0;
    end
    else
    begin

      // Default assignments
      r_Leading_Edge  <= 1'b0;
      r_Trailing_Edge <= 1'b0;
      
      if(r_SPI_Clk_Edges==0)
      begin
      r_SPI_Clk_Edges=24;
      o_TX_Ready <= 1'b1;
      end
      else if (r_SPI_Clk_Edges > 0)
      begin
        o_TX_Ready <= 1'b0;
        
        if (r_SPI_Clk_Count == CLKS_PER_HALF_BIT*2-1) 
        begin
          r_SPI_Clk_Edges <= r_SPI_Clk_Edges - 1'b1;
          r_Trailing_Edge <= 1'b1; //trailing: 하강
          r_SPI_Clk_Count <= 0;
          r_SPI_Clk       <= ~r_SPI_Clk;
        end
        else if (r_SPI_Clk_Count == CLKS_PER_HALF_BIT-1) //3
        begin
          r_SPI_Clk_Edges <= r_SPI_Clk_Edges - 1'b1;
          r_Leading_Edge  <= 1'b1; //leading: 상승
          r_SPI_Clk_Count <= r_SPI_Clk_Count + 1'b1;
          r_SPI_Clk       <= ~r_SPI_Clk;
        end
        else
        begin
          r_SPI_Clk_Count <= r_SPI_Clk_Count + 1'b1;
        end
      end
      //  
      
      else if(r_SPI_Clk_Edges==24)
      begin
        o_TX_Ready      <= 1'b0;
        r_SPI_Clk_Edges <= 24;  // Total # edges in one byte ALWAYS 16
      end
      
  end // always @ (posedge i_Clk or negedge i_Rst_L)
//여기까지 내가 원하는 SCK 생성 파트
  end

  // Purpose: Register i_TX_Byte when Data Valid is pulsed.
  // Keeps local storage of byte in case higher level module changes the data
  always@(posedge o_TX_Ready or negedge i_Rst_L)
  begin
    if (~i_Rst_L)
    begin
      r_TX_Byte <= 12'h00;
      r_TX_DV   <= 1'b0;
      j<=0;
    end
    else
    begin
        r_TX_Byte = lut[j];
        j=j+1;
        if(j==16) j=0;
     end
 end   
    

  // Purpose: Generate MOSI data
  // Works with both CPHA=0 and CPHA=1
  always@(posedge i_Clk or negedge i_Rst_L)
  begin
    if (~i_Rst_L)
    begin
      o_SPI_MOSI     <= 1'b0;
      r_TX_Bit_Count <= 4'b1011; // send MSb first(11) 
    end
    else
    begin
      // If ready is high, reset bit counts to default
      if (o_TX_Ready)
      begin
        r_TX_Bit_Count <= 4'b1011; 
      end
      // Catch the case where we start transaction and CPHA = 0
      else if ((r_Leading_Edge & w_CPHA) | (r_Trailing_Edge & ~w_CPHA)) 
      begin
        r_TX_Bit_Count <= r_TX_Bit_Count - 1'b1;
        o_SPI_MOSI     <= r_TX_Byte[r_TX_Bit_Count]; 
      end
    end
  end

  
  // Purpose: Add clock delay to signals for alignment.
  always@(posedge i_Clk or negedge i_Rst_L)
  begin
    if (~i_Rst_L)
    begin
      o_SPI_Clk  <= w_CPOL;
    end
    else
      begin
        o_SPI_Clk <= r_SPI_Clk;
      end // else: !if(~i_Rst_L)
  end // always @ (posedge i_Clk or negedge i_Rst_L)
  

endmodule // SPI_Master
