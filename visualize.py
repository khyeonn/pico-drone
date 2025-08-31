import serial

with serial.Serial('COM3', 115200, timeout=1) as ser:
    print(ser.readline())