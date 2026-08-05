import sys
import time

import serial

port = sys.argv[1]
cmd = sys.argv[2]

ser = serial.Serial(port, 115200, timeout=1)
time.sleep(0.3)
ser.reset_input_buffer()
ser.write((cmd + "\n").encode())
time.sleep(0.5)
while ser.in_waiting:
    print(ser.readline().decode(errors="replace").rstrip())
ser.close()
