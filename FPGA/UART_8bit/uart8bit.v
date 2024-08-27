`timescale 1ns / 1ps
// simple uart_tx by tomato (moore version) //

module uart_tx (
	input clk,
	input rst_n,
	output reg tx_output,
	output reg clk_inside
);

	parameter			IDLE_ST= 4'd0,
					START_ST= 4'd1,
					D0_ST= 4'd2,
					D1_ST= 4'd3,
					D2_ST= 4'd4,
					D3_ST= 4'd5,
					D4_ST= 4'd6,
					D5_ST= 4'd7,
					D6_ST= 4'd8,
					D7_ST= 4'd9,
					IDLE_ST2= 4'd10,
					IDLE_ST3= 4'd11,
					IDLE_ST4= 4'd12,
					//D11_ST= 4'd13,
					STOP_ST= 4'd13;
reg tx_output=1'b0;
	reg [3:0] tx_state;
	reg [7:0] tx_data=8'b0;
	reg [31:0] clk_count;
   reg [7:0] lut[0:5];
   reg clk_inside=1'b0;
   reg [31:0] IDLE_count='b0; 

integer j=1'b0; 
  integer i;
    initial begin
    for (i = 0; i < 6; i = i + 1) begin // 클럭 상관없이 데이터 넣기
        case (i)
            0:  lut[i] = 8'b01000001; //A
            1:  lut[i] = 8'b01000010; //B
            2:  lut[i] = 8'b01000011; //C
            3:  lut[i] = 8'b01000100; //D
            4:  lut[i] = 8'b01000101; //E
            5:  lut[i] = 8'b01000110; //F
            /*6:  lut[i] = 11'b11100011011; //1819
            7:  lut[i] = 11'b10011010110; //1238
            8:  lut[i] = 11'b10011010110; //1238
            9:  lut[i] = 11'b11100110110; //1846
            10: lut[i] = 11'b01110111011; //955
            11: lut[i] = 11'b10110100100; //1444
            12: lut[i] = 11'b00110110100; //436
            13: lut[i] = 11'b11000011111; //1567
            14: lut[i] = 11'b11110011011; //1947
            15: lut[i] = 11'b10001111111; //1151*/
            default: lut[i] = 8'b00000000; // 기본값 설정
        endcase
    end
end
	//OUTPUT LOGIC
	
	always @(posedge clk_inside) begin
		case(tx_state) 
			IDLE_ST  :  tx_output <= 1; 
			IDLE_ST2:  tx_output <= 1; 
			IDLE_ST3:  tx_output <= 1; 
			IDLE_ST4:  tx_output <= 1; 
			START_ST :  tx_output <= 0; 
			D0_ST		:  tx_output <= tx_data[0]; 
			D1_ST		:  tx_output <= tx_data[1]; 
			D2_ST		: tx_output <= tx_data[2]; 
			D3_ST		: tx_output <= tx_data[3];
			D4_ST		:  tx_output <= tx_data[4]; 
			D5_ST		:  tx_output <= tx_data[5]; 
			D6_ST		:  tx_output <= tx_data[6];
		   	D7_ST		:  tx_output <= tx_data[7]; 
		   	//D8_ST		:  tx_output <= tx_data[3]; 
		   	//D9_ST		:  tx_output <= tx_data[2]; 
		   	//D10_ST		: tx_output <= tx_data[1]; 
		   	//D11_ST		:  tx_output <= tx_data[0]; 
			STOP_ST  :  begin tx_output <= 1; tx_data<=lut[j]; j<=j+1; 
			if(j==5)  j<='b0; 
			end
			default  : tx_output <= 1; 
		endcase
	end
	
	//STATE REGISTER
	
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			tx_state <= IDLE_ST;
			clk_count <= 0;
			
		end	
		else begin
	       if(clk_count==543) begin clk_inside<=~clk_inside; clk_count <= clk_count + 1; ; end
		   else if(clk_count == 1086) begin //시뮬레이션 확인만 하고 434로 바꿔야함. 
				clk_count <= 0;
				clk_inside<=~clk_inside;
				case(tx_state) 
						IDLE_ST  :	begin  tx_state <= IDLE_ST2; end
						IDLE_ST2  :	 tx_state <= IDLE_ST3;
						IDLE_ST3  :	 tx_state <= IDLE_ST4;
						IDLE_ST4  :	 tx_state <= START_ST;
						START_ST : 	begin tx_state <= D0_ST; end
						D0_ST		:begin tx_state <= D1_ST;  end
						D1_ST		:begin tx_state <= D2_ST;  end
						D2_ST		:begin tx_state <= D3_ST;  end
						D3_ST		:begin tx_state <= D4_ST;  end
						D4_ST		:begin tx_state <= D5_ST; end
						D5_ST		:begin tx_state <= D6_ST;  end
						D6_ST		:begin tx_state <= D7_ST;  end
						D7_ST		:begin tx_state <= STOP_ST;  end
						//D8_ST		:begin tx_state <= D9_ST;  end
						//D9_ST		:begin tx_state <= D10_ST;  end
						//D10_ST		:begin tx_state <= D11_ST;  end
						//D11_ST		:begin tx_state <= STOP_ST;  end
						STOP_ST  :begin tx_state <= IDLE_ST;  end
						default  :begin tx_state <= IDLE_ST; end
					endcase
				end
			else begin clk_count <= clk_count + 1;
			
			 end
		end 
	end
endmodule
