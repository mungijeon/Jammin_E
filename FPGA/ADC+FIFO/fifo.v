module fifo #(
    parameter DATA_WIDTH = 12, //ADC출력이 12비트니까 12로 함. 
    parameter FIFO_DEPTH = 256    //코드를 reusable하게 하기 위해 2의 승수 형태로 
    )(
    input                       clk,
    input                       clk_delay, 
    input                       rst_n,
    input   [DATA_WIDTH-1:0]    wdata_i, //ADC로 들어오는 데이터
    output  reg [DATA_WIDTH-1:0]    rdata_o,  //나가는 데이터
    output  reg                    full_o,
    output  reg                   empty_o,
    output reg wren_i,
    output reg rden_i
    );
    integer i;
    localparam FIFO_DEPTH_LG2 = $clog2(FIFO_DEPTH);
    
    (* keep = "true" *) reg [FIFO_DEPTH_LG2:0] wrptr; //데이터 8칸이라면 데이터 표시 3bit + full/empty 표시 1bit
    (* keep = "true" *) reg [FIFO_DEPTH_LG2:0] rdptr; //wrptr와 같은 비트수  
    //wptr와 rptr 모두 동일하면 FIFO에 아무것도 없는 empty 
    //wptr이 증가하면 wptr과 rptr의 하위 3비트가 다름. full도 empty도 아님. 
    //FIFO가 가득차면 wptr과 rptr의 MSB만 다르고 하위비트는 일치(full) 
    //full에서 데이터를 읽으면 rptr이 증가하고 
    //계속해서 데이터를 읽으면 wptr과 rptr의 비트가 모두 같아지는 경우(empty)
    
    (* keep = "true" *) reg wren_i; //최적화로 사라지는거 방지하기 위한 keep 
    (* dont_touch = "true" *) (* keep = "true" *) reg rden_i;
    (* keep = "true" *) reg [DATA_WIDTH-1:0] data_buffer;
    
      // mem(This part is replace by any other memroy & inteface)
    (* keep = "true" *) reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1]; // 12비트짜리 8개(12비트 데이터가 한번에 왔다갔다, 12비트를 저장할 수 있는게 8칸) 
    

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        wren_i <= 1;
        rden_i <= 0;
    end else begin
        wren_i <= !full_o;  // FIFO가 가득 차 있지 않다면 쓰기 활성화
        rden_i <= !empty_o; // FIFO가 비어 있지 않다면 읽기 활성화
    end
end


 
  
    always @(posedge clk, negedge rst_n) begin
        if (!rst_n) 
        begin
            rdptr <= {(FIFO_DEPTH_LG2+1){1'b0}}; //0으로 초기화
            wrptr <= {(FIFO_DEPTH_LG2+1){1'b0}}; //0으로 초기화 
            for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
                mem[i] = 0; // 모든 셀을 0으로 초기화
            end
        end
        else if(wrptr==255 & rdptr!=0) 
        begin 
            mem[wrptr] <= wdata_i; //마지막 데이터 쓰고 
            rdata_o<=mem[rdptr];
            //if(rdptr==0) begin wrptr<=255; rdptr<=rdptr+1; end //사실상 이부분은 밑에 full_o로 인해서 자동으로 구현되는데
            //else begin 
            wrptr<='b0; 
            rdptr<=rdptr+1; // end //이 부분이 밑에서 구현하는 부분이 없다 
        end 
        else if(rdptr==255) 
        begin 
            mem[wrptr] <= wdata_i;
            rdata_o<=mem[rdptr];
            rdptr<='b0; 
            wrptr<=wrptr+1;
            //mem[rdptr]<=0;
        end
        else begin 
            case({full_o, empty_o})
                2'b00: //read, write 모두 가능
                begin
                    mem[wrptr] <= wdata_i; //쓰면서
                    rdata_o<=mem[rdptr]; //읽으면서
                    rdptr <= rdptr + 'd1;
                    wrptr <= wrptr + 'd1;
                end
                2'b01: //비어있으니까 write만 가능
                begin 
                    mem[wrptr] <= wdata_i; //한 클럭에 12비트씩 이동  //write
                    wrptr <= wrptr + 'd1;
                end 
                2'b10: //가득차있으니까 read만 가능  
                begin 
                    rdata_o<=mem[rdptr];
                    rdptr <= rdptr+ 'd1;
                    mem[rdptr]<=0; 
                end
                
                default: data_buffer<='b0;
            endcase
        end
    end

    
    // Full condition update
always @(posedge clk_delay, negedge rst_n) begin
    if (!rst_n) begin
        full_o <= 0;
    end else if ((wrptr==255) & (rdptr==0)) begin //무조건 이게 맞음 
        full_o <= 1;
    end else begin
        full_o <= 0;
    end
end

// Empty condition update
always @(posedge clk_delay, negedge rst_n) begin
    if (!rst_n) begin
        empty_o <= 1;
    end else if (!wrptr & !rdptr) begin
        empty_o <= 1; //문제점: write 255까지 다 하고 read는 하나도 안한 상태면 empty가 아닌데
    end else begin
        empty_o <= 0;
    end
end
    
  
endmodule
