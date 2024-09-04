import serial
import sys

def open_serial_connection(port, baud_rate=115200):
    """시리얼 연결을 엽니다."""
    try:
        ser = serial.Serial(port, baud_rate, timeout=0)
        ser.flushInput()  # 입력 버퍼 초기화
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        return None

def byte_to_bits(byte_data):
    """바이트 데이터를 비트 스트링으로 변환합니다."""
    return f'{byte_data:08b}'

def read_from_serial(ser):
    """시리얼 데이터를 읽고 파일에 저장합니다."""
    if ser and ser.isOpen():
        print(f"Listening on {ser.port}...")
        data_count = 0
        file_index = 1
        bits_to_save = []
        try:
            while True:
                if ser.inWaiting() > 0:
                    data = ser.read(1)  # 한 바이트씩 읽기
                    if data:
                        bit_string = byte_to_bits(ord(data))
                        bits_to_save.append(bit_string)
                        data_count += 1

                        # 10000개 데이터가 모일 때마다 파일에 저장
                        if data_count >= 10000:
                            with open(f"signal_data{file_index}.txt", 'w') as file:
                                file.write("\n".join(bits_to_save))
                            print(f"Saved 10000 bits to data_{file_index}.txt")
                            bits_to_save = []  # 리스트 초기화
                            file_index += 1
                            data_count = 0

        except KeyboardInterrupt:
            print("\nInterrupt received, exiting...")
            if input("Do you really want to exit? (y/n): ").lower() == 'y':
                return
            else:
                read_from_serial(ser)  # 사용자가 종료를 원하지 않으면 계속 수신

        finally:
            ser.close()
            print(f"Closed connection on {ser.port}.")
    else:
        print("Failed to open serial port or port already closed.")

def main():
    port = "COM5"
    baud_rate = 115200
    ser = open_serial_connection(port, baud_rate)
    if ser is not None:
        read_from_serial(ser)

if __name__ == "__main__":
    main()
