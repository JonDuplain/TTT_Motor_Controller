"""
Motor 3 Monitor
===============
Sends 'R' to the STM32 over serial and prints everything that comes back.
Use this to diagnose whether the STM32 is responding at all before
worrying about individual values.

Change COM_PORT to match your STM32 serial port.
Do NOT run xbox_Controller.py at the same time — only one program can use the port.
"""

import serial
import time
import sys

COM_PORT  = "COM5"    # ← change to your STM32 port (check Device Manager)
BAUD_RATE = 115200

def main():
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.3)
        time.sleep(0.5)
        ser.reset_input_buffer()
        print(f"Connected to {COM_PORT}")
        print("Sending R every second. Everything received is shown below.")
        print("Press Ctrl+C to stop.\n")
    except serial.SerialException as e:
        print(f"ERROR: Could not open {COM_PORT}: {e}")
        print("Check Device Manager for the correct COM port number.")
        sys.exit(1)

    try:
        while True:
            ser.write(b"R\n")
            print(f"--- sent R ({time.strftime('%H:%M:%S')}) ---")

            # Read everything that arrives within 1 second
            deadline = time.time() + 1.0
            got_anything = False
            while time.time() < deadline:
                line = ser.readline().decode("utf-8", errors="ignore").rstrip()
                if line:
                    print(f"  {repr(line)}")
                    got_anything = True

            if not got_anything:
                print("  (nothing received)")

    except KeyboardInterrupt:
        print("\nStopped.")
        ser.close()

if __name__ == "__main__":
    main()
