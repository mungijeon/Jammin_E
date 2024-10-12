module SPI_tx
  #(parameter SPI_MODE = 0,
    parameter CLKS_PER_HALF_BIT = 1) //SPI Clock �ֱ��� �ݿ� i_clk�� �󸶳� ���� 
  (
   // Control/Data Signals,
   input        i_Rst_L,     // FPGA Reset //reset ��ȣ
   input        i_Clk,       // FPGA Clock -fpga reference clock ���ϴµ�
   
   // TX (MOSI) Signals
   input [13:0]  i_TX_Byte,        // Byte to transmit on MOSI -MOSI������ ������ 8��Ʈ ������
   output reg   o_TX_Ready,       // Transmit Ready for next byte -���� ����Ʈ�� ���� �غ� �Ǿ���(����Ʈ�� ��Ʈ�� �ٲٱ� ���ؼ� �ʿ�?) 
   
   // SPI Interface
   output reg o_SPI_Clk, //SCK ������ֱ� ���Ѱ��ε�
   output reg o_SPI_MOSI, //�̰� ���� �ʿ��� ���ϵ�-slave�� �����ϱ� ���� ��
   output reg cs
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
  reg [13:0] r_TX_Byte;
  reg Tx_finish;

  reg [2:0] r_RX_Bit_Count;
  reg [3:0] r_TX_Bit_Count;
  
   reg transmitting;

  // i_TX_Byte의 이전 상태를 저장하여 변화 감지
  reg [13:0] prev_TX_Byte=14'b0;


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

/*
  // Purpose: Generate SPI Clock correct number of times when DV pulse comes
  //���� ���ϴ� SPI_clk(SCK) ������ִ� �κ�
  always @(posedge i_Clk or negedge i_Rst_L)
  begin
    if (~i_Rst_L) //active-low�� �����ϴ� reset
    begin //�ʱ�
      o_TX_Ready      <= 1'b0;
      r_SPI_Clk_Edges <= 0;
      r_Leading_Edge  <= 1'b0;
      r_Trailing_Edge <= 1'b0;
      r_SPI_Clk       <= w_CPOL; // assign default state to idle state, w_CPOL 0�̸� clk�� 0���� ����, 1�̸� clk�� 1���� ����
      r_SPI_Clk_Count <= 0;
      o_SPI_Clk<=0;
      cs<='b0;
    end
    else
    begin
      if(r_SPI_Clk_Edges==0)
      begin
      r_SPI_Clk_Edges<=24;
      o_TX_Ready <= 1'b1;
      cs<='b0;
      end
      else if (r_SPI_Clk_Edges > 0)
      begin
        o_TX_Ready <= 1'b0;
        cs<=1'b1;
        r_SPI_Clk_Edges <= r_SPI_Clk_Edges - 1'b1;
        
        if (r_SPI_Clk_Count == CLKS_PER_HALF_BIT*2-1)
        begin
          r_SPI_Clk_Edges <= r_SPI_Clk_Edges - 1'b1;
          r_Trailing_Edge <= 1'b1;
          r_SPI_Clk_Count <= 0;
          r_SPI_Clk       <= ~r_SPI_Clk;
        end
        else if (r_SPI_Clk_Count == CLKS_PER_HALF_BIT-1)
        begin
          r_SPI_Clk_Edges <= r_SPI_Clk_Edges - 1'b1;
          r_Leading_Edge  <= 1'b1;
          r_SPI_Clk_Count <= r_SPI_Clk_Count + 1'b1;
          r_SPI_Clk       <= ~r_SPI_Clk;
        end
        else
        begin
          r_SPI_Clk_Count <= r_SPI_Clk_Count + 1'b1;
        end
      end  
      end
       
  end // always @ (posedge i_Clk or negedge i_Rst_L)
//������� ���� ���ϴ� SCK ���� ��Ʈ

*/
  // Purpose: Register i_TX_Byte when Data Valid is pulsed.
  // Keeps local storage of byte in case higher level module changes the data
  /*always@(negedge o_TX_Ready or negedge i_Rst_L)
  begin
    if (~i_Rst_L)
    begin
      r_TX_Byte <= 12'h00;
      r_TX_DV   <= 1'b0;
    end
    else
    begin
        r_TX_Byte <= i_TX_Byte;
     end
 end   
    */
always @(posedge i_Clk or negedge i_Rst_L) begin
    if (~i_Rst_L) begin
      o_TX_Ready <= 1'b1;        // Initially ready to send data
      transmitting <= 1'b0;
      r_TX_Bit_Count <= 4'd0;
      o_SPI_MOSI <= 1'b0;        // MOSI 초기화
      cs <= 1'b1;                // Chip select inactive (high)
      prev_TX_Byte <= 14'd0;     // 이전 TX 데이터를 초기화
    end else begin
      // i_TX_Byte가 변경되면 즉시 전송 시작
      if (i_TX_Byte != prev_TX_Byte) begin
        r_TX_Byte <= i_TX_Byte;    // 전송할 데이터를 저장
        o_SPI_MOSI <= i_TX_Byte[13]; // MSB 비트를 즉시 MOSI로 전송
        r_TX_Bit_Count <= 4'd12;   // 다음 비트를 준비 (MSB는 이미 전송)
        o_TX_Ready <= 1'b0;        // 전송 중이므로 Ready 비활성화
        transmitting <= 1'b1;      // 전송 상태로 전환
        cs <= 1'b0;                // Chip select 활성화 (low)
        prev_TX_Byte <= i_TX_Byte; // prev_TX_Byte를 업데이트
      end else if (transmitting) begin
        // 전송 중이면 매 클럭마다 다음 비트를 MOSI로 전송
        o_SPI_MOSI <= r_TX_Byte[r_TX_Bit_Count]; // 다음 비트 전송
        if (r_TX_Bit_Count == 0) begin
          // 모든 비트 전송 완료
          transmitting <= 1'b0;    // 전송 완료 상태로 전환
          o_TX_Ready <= 1'b1;      // 다음 데이터를 받을 준비 완료
          cs <= 1'b1;              // Chip select 비활성화 (high)
        end else begin
          r_TX_Bit_Count <= r_TX_Bit_Count - 1'b1; // 비트 카운터 감소
        end
      end
    end
  end

/*
always @(posedge i_Clk or negedge i_Rst_L)  //i_Clk=Clk_SPI
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
*/
endmodule // SPI_Master
