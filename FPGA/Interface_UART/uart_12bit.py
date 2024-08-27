import serial

def open_serial_connection(port, baud_rate=115200):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        return None

def calculate_checksum(data):
    checksum = 0
    for byte in data:
        checksum += byte
    return checksum & 0xFF  # 8비트 체크섬

def read_and_process_data(ser):
    if ser is not None:
        print(f"Listening on {ser.port}...")
        try:
            while True:
                # 데이터와 체크섬을 포함하여 3바이트 읽기
                data = ser.read(3)  # 2바이트 데이터 + 1바이트 체크섬
                if len(data) == 3:
                    high_byte = data[0]
                    low_byte = data[1]
                    received_checksum = data[2]
                    
                    full_data = (high_byte << 4) | (low_byte & 0x0F)
                    calculated_checksum = calculate_checksum(data[:2])
                    
                    # 체크섬 검증
                    if received_checksum == calculated_checksum:
                        print(f"Received valid data: {bin(full_data)[2:].zfill(12)}")
                    else:
                        print("Checksum mismatch.")
                else:
                    print("Not enough data received.")
        except KeyboardInterrupt:
            print("Stopped by user.")
        finally:
            ser.close()
            print("Serial connection closed.")
    else:
        print("Serial port not opened.")

def main():
    port = "COM5"
    baud_rate = 115200
    ser = open_serial_connection(port, baud_rate)
    read_and_process_data(ser)

if __name__ == "__main__":
    main()
