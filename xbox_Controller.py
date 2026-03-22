"""
Xbox Controller → UART → STM32 → 5x VESC Motor Control (Current Mode)
======================================================================

Controller mapping:
  Left  Stick X  → Motor 1  (-MAX_AMPS to +MAX_AMPS)
  Left  Stick Y  → Motor 2  (push forward = positive current)
  Right Stick X  → Motor 3
  Right Stick Y  → Motor 4  (push forward = positive current)
  Left  Trigger  → Motor 5 forward  (0 to MAX_AMPS)
  Right Trigger  → Motor 5 reverse  (0 to -MAX_AMPS)
  B button       → Stop ALL motors (handbrake)

Per-motor max amps defaults set in MAX_AMPS_DEFAULT (all 5A).
Override at runtime with:  max <1-5> <amps>   e.g. "max 5 8.0"

Requirements:
    python -m pip install pygame pyserial

Usage:
    python xbox_controller.py
"""

import pygame
import serial
import time
import sys
import threading
import queue

# ── Configuration ──────────────────────────────────────────────────────────────
COM_PORT      = "COM5"    # ← change to your STM32 COM port
BAUD_RATE     = 115200
POLL_HZ       = 50        # controller read rate (Hz)
DEADZONE      = 0.08      # stick deadzone (0.0 - 1.0)
CHANGE_THRESH = 0.05      # minimum amps change before sending a new command
KEEPALIVE_MS  = 400       # resend current if no change for this long (ms)
NUM_MOTORS    = 5

# Per-motor max current in Amps (±). All start at 5A.
# Edit these defaults, or change at runtime with: max <1-5> <amps>
MAX_AMPS_DEFAULT = [2.0, 5.0, 20.0, 5.0, 5.0]
# ───────────────────────────────────────────────────────────────────────────────

# Xbox axis/button indices (pygame)
AXIS_LEFT_X   = 0
AXIS_LEFT_Y   = 1
AXIS_RIGHT_X  = 2
AXIS_RIGHT_Y  = 3
AXIS_LT       = 4   # Left  Trigger: -1.0 (released) to +1.0 (full)
AXIS_RT       = 5   # Right Trigger: -1.0 (released) to +1.0 (full)
BTN_B         = 1

# ── Shared state ───────────────────────────────────────────────────────────────
motor_status = ["No data"] * NUM_MOTORS
status_lock  = threading.Lock()
max_amps     = list(MAX_AMPS_DEFAULT)   # mutable at runtime


def uart_reader(ser):
    """Background thread: reads VESC status lines from STM32."""
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            # Expected: "M1 | RPM:   412  Curr:  3.2A  Duty:+0.100"
            if line.startswith("M") and "|" in line:
                try:
                    motor_num = int(line[1]) - 1
                    data_part = line.split("|", 1)[1].strip()
                    if 0 <= motor_num < NUM_MOTORS:
                        with status_lock:
                            motor_status[motor_num] = data_part
                except (ValueError, IndexError):
                    pass
        except Exception:
            break


def apply_deadzone(value, deadzone):
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def trigger_to_ratio(axis_value):
    """Convert trigger axis (-1.0 released, +1.0 full press) to 0.0 - 1.0."""
    return (axis_value + 1.0) / 2.0


def print_display(currents, stopped=False):
    """Overwrite the terminal display block in place."""
    lines = NUM_MOTORS + 2
    print(f"\033[{lines}A", end="")
    print("─" * 64)
    for i in range(NUM_MOTORS):
        direction = "FWD" if currents[i] > 0.05 else ("REV" if currents[i] < -0.05 else "BRK")
        with status_lock:
            status = motor_status[i]
        print(f"  M{i+1} [{direction}] cmd:{currents[i]:+5.2f}A  max:{max_amps[i]:.1f}A | {status:<30}")
    if stopped:
        print("  *** STOP ALL (handbrake) ***" + " " * 36)
    else:
        print(" " * 64)


def print_help():
    print("\nRuntime commands (type in this terminal):")
    print("  max <1-5> <amps>   set per-motor max current, e.g. 'max 5 8.0'")
    print("  quit               stop all motors and exit")
    print("  B button on controller = stop all (handbrake)\n")


def handle_runtime_input(line, ser):
    """Process commands typed in the Python terminal at runtime."""
    parts = line.strip().split()
    if not parts:
        return

    if parts[0].lower() == "max" and len(parts) == 3:
        try:
            idx = int(parts[1]) - 1
            val = float(parts[2])
            if 0 <= idx < NUM_MOTORS and 0.0 <= val <= 100.0:
                max_amps[idx] = val
                print(f"  Motor {idx+1} max current set to {val:.1f}A")
            else:
                print("  ERR: motor must be 1-5, amps must be 0.0-100.0")
        except ValueError:
            print("  ERR: usage: max <1-5> <amps>")

    elif parts[0].lower() == "quit":
        ser.write(b"sa\r\n")
        time.sleep(0.1)
        ser.close()
        pygame.quit()
        sys.exit(0)

    else:
        print("  Unknown command. Type 'max <1-5> <amps>' or 'quit'")


def main():
    # ── pygame init ────────────────────────────────────────────────────────────
    pygame.init()
    pygame.display.init()
    pygame.joystick.init()

    for _ in range(20):
        pygame.event.pump()
        if pygame.joystick.get_count() > 0:
            break
        time.sleep(0.1)

    if pygame.joystick.get_count() == 0:
        print("ERROR: No controller found.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller: {joystick.get_name()}")

    # ── serial init ────────────────────────────────────────────────────────────
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(0.5)
        ser.reset_input_buffer()
        print(f"Serial:     {COM_PORT} @ {BAUD_RATE}")
    except serial.SerialException as e:
        print(f"ERROR: Could not open {COM_PORT}: {e}")
        sys.exit(1)

    # ── background threads ─────────────────────────────────────────────────────
    threading.Thread(target=uart_reader, args=(ser,), daemon=True).start()

    cmd_queue = queue.Queue()
    def stdin_reader():
        while True:
            try:
                line = sys.stdin.readline()
                if line:
                    cmd_queue.put(line)
            except Exception:
                break
    threading.Thread(target=stdin_reader, daemon=True).start()

    print_help()
    print(f"Controlling {NUM_MOTORS} motors | B = Stop all | Ctrl+C = Exit\n")

    # Print initial blank display block so in-place overwrite works
    print("─" * 64)
    for i in range(NUM_MOTORS):
        print(f"  M{i+1} [BRK] cmd: +0.00A  max:{max_amps[i]:.1f}A | No data                    ")
    print(" " * 64)

    interval      = 1.0 / POLL_HZ
    sent_currents = [0.0] * NUM_MOTORS
    last_send     = [0]   * NUM_MOTORS

    def send_motor(idx, amps):
        """Send current command for one motor (STM32 protocol: c <1-5> <amps>)."""
        ser.write(f"c {idx+1} {amps:.2f}\r\n".encode())
        sent_currents[idx] = amps
        last_send[idx]     = int(time.time() * 1000)

    def stop_all():
        ser.write(b"sa\r\n")
        for i in range(NUM_MOTORS):
            sent_currents[i] = 0.0
            last_send[i]     = int(time.time() * 1000)

    try:
        while True:
            loop_start = time.time()
            pygame.event.pump()

            # Check for typed runtime commands
            try:
                line = cmd_queue.get_nowait()
                handle_runtime_input(line, ser)
            except queue.Empty:
                pass

            if joystick.get_button(BTN_B):
                stop_all()
                print_display(sent_currents, stopped=True)

            else:
                lx = apply_deadzone(joystick.get_axis(AXIS_LEFT_X),  DEADZONE)
                ly = apply_deadzone(joystick.get_axis(AXIS_LEFT_Y),  DEADZONE)
                rx = apply_deadzone(joystick.get_axis(AXIS_RIGHT_X), DEADZONE)
                ry = apply_deadzone(joystick.get_axis(AXIS_RIGHT_Y), DEADZONE)

                # Triggers: LT = forward, RT = reverse for motor 5
                lt = trigger_to_ratio(joystick.get_axis(AXIS_LT))
                rt = trigger_to_ratio(joystick.get_axis(AXIS_RT))
                m5_ratio = lt - rt   # +1.0 full forward, -1.0 full reverse
                if abs(m5_ratio) < DEADZONE:
                    m5_ratio = 0.0

                desired = [
                    round( lx       * max_amps[0], 2),   # Left  X  → Motor 1
                    round(-ly       * max_amps[1], 2),   # Left  Y  → Motor 2
                    round( rx       * max_amps[2], 2),   # Right X  → Motor 3
                    round(-ry       * max_amps[3], 2),   # Right Y  → Motor 4
                    round( m5_ratio * max_amps[4], 2),   # Triggers → Motor 5
                ]

                now_ms = int(time.time() * 1000)
                for i in range(NUM_MOTORS):
                    if abs(desired[i]) < CHANGE_THRESH:
                        # Motor idle — let STM32 handle handbrake, don't interfere
                        if sent_currents[i] != 0.0:
                            # Just returned to zero — send one stop immediately
                            send_motor(i, 0.0)
                    else:
                        changed   = abs(desired[i] - sent_currents[i]) >= CHANGE_THRESH
                        timed_out = (now_ms - last_send[i]) >= KEEPALIVE_MS
                        if changed or timed_out:
                            send_motor(i, desired[i])

                print_display(sent_currents)

            elapsed = time.time() - loop_start
            sleep   = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n\nStopping all motors...")
        stop_all()
        time.sleep(0.1)
        ser.close()
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()