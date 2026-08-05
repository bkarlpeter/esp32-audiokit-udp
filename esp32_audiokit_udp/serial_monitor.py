"""Reads both boards' serial output concurrently and prefixes each line
with the COM port and a timestamp, so master/slave behaviour can be
compared side by side."""
import sys
import threading
import time

import serial

PORTS = ["COM3", "COM4"]
BAUD = 115200
DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


def reader(port: str, stop_at: float):
    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except Exception as e:
        print(f"[{port}] OPEN FAILED: {e}")
        return
    while time.time() < stop_at:
        try:
            line = ser.readline()
        except Exception as e:
            print(f"[{port}] READ ERROR: {e}")
            break
        if line:
            text = line.decode(errors="replace").rstrip()
            print(f"{time.time():.2f} [{port}] {text}")
    ser.close()


def main():
    stop_at = time.time() + DURATION_S
    threads = [threading.Thread(target=reader, args=(p, stop_at)) for p in PORTS]
    for t in threads:
        t.start()
    for t in threads:
        t.join()


if __name__ == "__main__":
    main()
