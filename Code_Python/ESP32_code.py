import serial
import time

serial_port = 'COM12'  


ser = serial.Serial(serial_port, 115200, timeout=1)

if ser.isOpen():
    print(f"Connected to {serial_port}")

    time.sleep(2)


    ser.write(b"AT+CWJAP=\" faraj_xA\",\"sawqst231\"\r\n")
    time.sleep(5)  


    ser.write(b"AT+CIFSR\r\n")
    time.sleep(2)


    response = ser.readlines()
    for line in response:
        line = line.decode().strip()
        if line.startswith("STAIP"):
            ip_address = line.split(": ")[1]
            print(f"Connected to WiFi. IP Address: {ip_address}")

    ser.close()
else:
    print(f"Failed to open {serial_port}")
