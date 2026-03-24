"""
Motor 3 Position Monitor
========================
Polls the STM32 over USB serial, sends 'R' every 200 ms, and displays
the full J3 status line (pos, setpoint, velocity, current, erpm).

Usage:
    python motor3_monitor.py

Change COM_PORT below to match your STM32's serial port.
"""

import serial
import time
import sys

COM_PORT  = "COM5"    # ← change to your STM32 port
BAUD_RATE = 115200
POLL_HZ   = 5         # how often to send R (5 Hz is plenty)

def main():
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(0.5)
        ser.reset_input_buffer()
        print(f"Connected: {COM_PORT} @ {BAUD_RATE}\n")
        print("Spin motor 3 by hand and watch the values change.")
        print("Press Ctrl+C to stop.\n")
        print(f"  {'pos (rev)':>12}  {'sp (rev)':>10}  {'vel (r/s)':>10}  {'I (A)':>8}  {'erpm':>10}")
        print("  " + "─" * 60)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    interval = 1.0 / POLL_HZ

    try:
        while True:
            t = time.time()

            ser.write(b"R\n")

            # Read all lines that arrive within the poll window
            deadline = time.time() + interval
            while time.time() < deadline:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # ArmController_PrintStatus format:
                # "J3  pos:+0.123rev  sp:+0.123rev  vel:+0.123 r/s  I:+1.23A  erpm:+  1234"
                if line.startswith("J3") and "pos:" in line:
                    try:
                        pos  = float(line.split("pos:")[1].split("rev")[0])
                        sp   = float(line.split("sp:")[1].split("rev")[0])
                        vel  = float(line.split("vel:")[1].split("r/s")[0])
                        cur  = float(line.split("I:")[1].split("A")[0])
                        erpm = int(  line.split("erpm:")[1].split()[0])
                        print(f"  {pos:>+12.5f}  {sp:>+10.5f}  {vel:>+10.5f}  "
                              f"{cur:>+8.3f}  {erpm:>10d}")
                    except (IndexError, ValueError):
                        print(f"  parse error: {line}")

    except KeyboardInterrupt:
        print("\nStopped.")
        ser.close()

if __name__ == "__main__":
    main()
