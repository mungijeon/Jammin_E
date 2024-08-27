`timescale 1ns / 1ps

module uart_tx (
    input clk,
    input rst_n,
    output reg tx_output
);

// UART configuration parameters
parameter   IDLE_ST = 4'd0,
            START_FRAME_ST = 4'd1,
            DATA_ST = 4'd2,
            CHECKSUM_ST = 4'd3,
            STOP_FRAME_ST = 4'd4;

reg [4:0] state = IDLE_ST;
reg [11:0] lut[0:5]; // Look-Up Table for data
reg [7:0] data_high;   // High part of the data
reg [3:0] data_low;    // Low part of the data (4 bits)
reg [7:0] checksum = 0;
reg [3:0] bit_idx = 0;
integer data_index = 0;
reg [15:0] clk_count = 0;
reg uart_clk = 0; // Derived UART clock

// Initialize lookup table
initial begin
    lut[0] = 12'b010000010110;
    lut[1] = 12'b010000010110;
    lut[2] = 12'b010000010110;
    lut[3] = 12'b010000010110;
    lut[4] = 12'b010000010110;
    lut[5] = 12'b010000010110;
end

// Clock divider for UART transmission
always @(posedge clk) begin
    if (clk_count >= 543) begin // Generate a clock tick every ~8us for 115200bps
        clk_count <= 0;
        uart_clk <= ~uart_clk;
    end else begin
        clk_count <= clk_count + 1;
    end
end

// State machine for UART transmission
always @(posedge uart_clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE_ST;
        tx_output <= 1'b1; // IDLE state high
        data_index <= 0;
        checksum <= 0;
        bit_idx <= 0;
    end else begin
        case (state)
            IDLE_ST: begin
                data_high <= lut[data_index][11:4]; // High 8 bits
                data_low <= lut[data_index][3:0];   // Low 4 bits
                checksum <= 0; // Reset checksum at the start of transmission
                state <= START_FRAME_ST;
            end
            START_FRAME_ST: begin
                tx_output <= 0; // Start bit
                state <= DATA_ST;
                bit_idx <= 0; // Reset bit index for data bits
            end
            DATA_ST: begin
                if (bit_idx < 8) begin
                    tx_output <= data_high[bit_idx];
                    checksum <= checksum ^ data_high[bit_idx];
                    bit_idx <= bit_idx + 1;
                end else if (bit_idx < 12) begin
                    tx_output <= data_low[bit_idx - 8];
                    checksum <= checksum ^ data_low[bit_idx - 8];
                    bit_idx <= bit_idx + 1;
                end else begin
                    bit_idx <= 0;
                    state <= CHECKSUM_ST;
                end
            end
            CHECKSUM_ST: begin
                if (bit_idx < 8) begin
                    tx_output <= checksum[bit_idx];
                    bit_idx <= bit_idx + 1;
                end else begin
                    state <= STOP_FRAME_ST;
                    bit_idx <= 0; // Reset bit index for stop bit
                end
            end
            STOP_FRAME_ST: begin
                tx_output <= 1; // Stop bit
                state <= IDLE_ST;
                data_index <= (data_index + 1) % 6; // Cycle through LUT
            end
        endcase
    end
end

endmodule
