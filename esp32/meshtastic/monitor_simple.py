#!/usr/bin/env python3
import serial
import time
import sys

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
baud = 115200

print(f"Opening {port} at {baud} baud...")
ser = serial.Serial(port, baud, timeout=1)

# Reset the board
print("Resetting board...")
ser.setDTR(False)
ser.setRTS(True)
time.sleep(0.1)
ser.setRTS(False)
time.sleep(0.5)

print("Reading output for 30 seconds...")
start = time.time()
while time.time() - start < 30:
    if ser.in_waiting:
        data = ser.read(ser.in_waiting)
        print(data.decode('utf-8', errors='ignore'), end='')
    time.sleep(0.01)

ser.close()
print("\nDone")
