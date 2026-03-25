"""
Motor 3 Position Monitor
========================
Zeroes J3 gains so no current is commanded, then keeps the STM32
watchdog alive with J 0 0 0 0 0 at 10 Hz so the control loop runs
and integrates hall-sensor ERPM into position.

Polls 'R' at 5 Hz and displays J3: pos / vel / erpm in real-time.
Spin motor 3 by hand and watch pos accumulate.

Change COM_PORT to match your STM32 port.
Do NOT run xbox_Controller.py at the same time.

On exit, restores J3 gains to kp=2.0 kv=2.5.
"""

import serial
import time
import sys

COM_PORT  = "COM5"    # ← change to your STM32 port
BAUD_RATE = 115200


def send(ser, msg):
    ser.write(msg if isinstance(msg, bytes) else msg.encode())


def parse_j3(line):
    """Parse a J3 status line. Returns (pos, vel, erpm) or None."""
    try:
        pos  = float(line.split("pos:")[1].split("rev")[0])
        vel  = float(line.split("vel:")[1].split("r")[0])
        erpm = int(  line.split("erpm:")[1].split()[0])
        return pos, vel, erpm
    except (IndexError, ValueError):
        return None


def main():
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.05)
        time.sleep(0.5)
        ser.reset_input_buffer()
        print(f"Connected to {COM_PORT}")
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    # Zero J3 gains — no current will be commanded while monitoring
    send(ser, b"P 3 0.0 0.0\n")
    time.sleep(0.1)
    ser.reset_input_buffer()

    print("J3 gains zeroed — no current will be applied.")
    print("Spin motor 3 by hand and watch pos accumulate.")
    print("Press Ctrl+C to stop and restore gains.\n")
    print(f"  {'time':>8}  {'pos (rev)':>12}  {'vel (rev/s)':>12}  {'erpm':>10}")
    print("  " + "─" * 50)

    j_interval  = 0.1    # 10 Hz J command to keep watchdog alive
    r_interval  = 0.2    # 5 Hz status poll
    last_j = 0.0
    last_r = 0.0
    t_start = time.time()

    try:
        while True:
            now = time.time()

            # Keep watchdog alive
            if now - last_j >= j_interval:
                send(ser, b"J 0 0 0 0 0\n")
                last_j = now

            # Poll status
            if now - last_r >= r_interval:
                send(ser, b"R\n")
                last_r = now

            # Read whatever is in the buffer
            line = ser.readline().decode("utf-8", errors="ignore").rstrip()
            if line.startswith("J3") and "pos:" in line:
                result = parse_j3(line)
                if result:
                    pos, vel, erpm = result
                    elapsed = now - t_start
                    print(f"  {elapsed:8.1f}  {pos:>+12.5f}  {vel:>+12.5f}  {erpm:>10d}")

    except KeyboardInterrupt:
        print("\nRestoring J3 gains (kp=2.0 kv=2.5)...")
        send(ser, b"P 3 2.0 2.5\n")
        time.sleep(0.1)
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
