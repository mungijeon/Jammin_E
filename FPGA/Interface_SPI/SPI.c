#include <stdio.h>
#include <stdint.h>
#include <Windows.h>
#include "ftd2xx.h"
#include "libMPSSE_spi.h"

#define SPI_DEVICE_BUFFER_SIZE 1

void print_and_quit(const char* message) {
    printf("%s\n", message);
    exit(1);
}

void print_binary(uint16_t value, int bits) {
    for (int i = bits - 1; i >= 0; i--) {
        printf("%d", (value >> i) & 1);
    }
    printf("\n");
}

int main(int argc, char** argv) {
    FT_STATUS status;
    FT_HANDLE handle;

    // Initialize the MPSSE library
    Init_libMPSSE();

    // Open the FTDI device
    status = SPI_OpenChannel(0, &handle);
    if (status != FT_OK) {
        print_and_quit("Error while opening the MPSSE channel.");
    }

    // Configure the SPI channel
    ChannelConfig channelConfig;
    channelConfig.ClockRate = 1; // 6 MHz clock rate
    channelConfig.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_DBUS4 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
    channelConfig.LatencyTimer = 75;

    status = SPI_InitChannel(handle, &channelConfig);
    if (status != FT_OK) {
        print_and_quit("Error while initializing the MPSSE channel.");
    }

    uint8_t rx_buffer[SPI_DEVICE_BUFFER_SIZE] = {}; // Buffer to store one bit of received data
    uint32_t sizeToTransfer = 1; // Read 1 bit at a time
    uint32_t sizeTransferred;

    uint16_t data12bit = 0; // 12-bit data storage
    uint8_t bit_count = 0;  // Counter to track how many bits have been read

    // Infinite loop to read data continuously
    while (1) {
        // Read one bit from the FPGA
        status = SPI_Read(handle, rx_buffer, sizeToTransfer, (LPDWORD)&sizeTransferred, SPI_TRANSFER_OPTIONS_SIZE_IN_BITS);
        if (status != FT_OK) {
            print_and_quit("Error while reading data from the FPGA.");
        }

        // Shift data into the 12-bit variable
        data12bit = (data12bit << 1) | (rx_buffer[0] & 0x01);
        bit_count++;


        printf("bit: %d", rx_buffer[0] & 0x01);
        printf("\n");
        printf("count: %d", bit_count);
        printf("\n");
        // Check if we've collected 12 bits
        if (bit_count == 12) {
            // Print the 12-bit value in both binary and decimal formats
            print_binary(data12bit, 12);      // Binary format
            printf("Decimal: %u\n", data12bit);  // Decimal format

            bit_count = 0;  // Reset the bit counter
            data12bit = 0;  // Clear the 12-bit storage for the next set of bits
        }

        Sleep(100); // Add a small delay to avoid flooding the output
    }

    // Close the FTDI device
    SPI_CloseChannel(handle);
    Cleanup_libMPSSE();

    return 0;
}
