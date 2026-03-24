"""
Xbox Controller → UART → STM32 → 5-joint Robotic Arm (Position PD + Gravity Comp)
====================================================================================

Controller mapping:
  Left  Stick X  → Joint 1  (base yaw)
  Left  Stick Y  → Joint 2  (shoulder)    push forward = positive direction
  Right Stick X  → Joint 3  (elbow)
  Right Stick Y  → Joint 4  (wrist pitch) push forward = positive direction
  Left Trigger   → Joint 5 forward   (wrist roll)
  Right Trigger  → Joint 5 reverse
  B button       → Emergency stop (held = stay stopped, release = resume)

The STM32 runs a 100 Hz position PD + gravity feedforward loop.
This script streams joystick axes at 50 Hz; the STM32 watchdog brakes
all joints if no update arrives within 250 ms.

Runtime commands (type in this terminal):
  G n kg          Set gravity gain for joint n (1-5),  e.g. "G 2 10.0"
  P n kp kv       Set PD gains for joint n,            e.g. "P 2 8.0 3.0"
  H               Home — zero position estimates at current pose
  E               Emergency stop
  R               Request status printout from STM32
  quit            Stop arm and exit

Requirements:
    python -m pip install pygame pyserial
"""

import pygame
import serial
import time
import sys
import threading
import queue

# ── Configuration ──────────────────────────────────────────────────────────────
COM_PORT   = "COM5"   # ← change to your STM32 COM port
BAUD_RATE  = 115200
POLL_HZ    = 50       # joystick stream rate (Hz) — must be > 4 Hz for watchdog
DEADZONE   = 0.08     # stick deadzone applied on the PC side
# ───────────────────────────────────────────────────────────────────────────────

NUM_JOINTS = 5

# Xbox axis/button indices (pygame)
AXIS_LEFT_X  = 0
AXIS_LEFT_Y  = 1
AXIS_RIGHT_X = 2
AXIS_RIGHT_Y = 3
AXIS_LT      = 4   # Left  Trigger: -1.0 (released) → +1.0 (full)
AXIS_RT      = 5   # Right Trigger: -1.0 (released) → +1.0 (full)
BTN_B        = 1

# ── Shared state ───────────────────────────────────────────────────────────────
joint_status = ["--"] * NUM_JOINTS   # last status line received per joint
status_lock  = threading.Lock()
last_msg     = ""                    # last non-status line from STM32 (shown in display)
estop_active = False


def uart_reader(ser):
    """Background thread: read lines from STM32 and update joint_status."""
    global last_msg
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            # Status line format from ArmController_PrintStatus():
            # "J1  pos:+0.123rev  sp:+0.123rev  vel:+0.123 r/s  I:+1.23A  erpm:+1234"
            if line.startswith("J") and "pos:" in line:
                try:
                    idx = int(line[1]) - 1
                    if 0 <= idx < NUM_JOINTS:
                        with status_lock:
                            joint_status[idx] = line[2:].strip()
                except (ValueError, IndexError):
                    pass
            else:
                # OK HOME, OK ESTOP, ERR ..., help text, welcome message, etc.
                with status_lock:
                    last_msg = line[:70]
        except Exception:
            break


def apply_deadzone(value, deadzone):
    """Rescale so that the deadzone region maps cleanly to 0."""
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def trigger_to_ratio(axis_value):
    """Convert trigger axis (-1.0 released, +1.0 full) to 0.0–1.0."""
    return (axis_value + 1.0) / 2.0


def send_joystick(ser, axes):
    """Send the J command: J j1 j2 j3 j4 j5\n"""
    msg = "J " + " ".join(f"{v:.3f}" for v in axes) + "\n"
    ser.write(msg.encode())


def print_display(axes, stopped):
    """Overwrite the in-place terminal display."""
    lines = NUM_JOINTS + 3
    print(f"\033[{lines}A", end="")
    print("─" * 72)
    labels = ["J1 Base  Yaw  (LX)", "J2 Shoulder  (LY)",
              "J3 Elbow     (RX)", "J4 WristPitch(RY)", "J5 WristRoll (TR)"]
    for i in range(NUM_JOINTS):
        bar_len = int(abs(axes[i]) * 20)
        bar = ("█" * bar_len).ljust(20)
        direction = "→" if axes[i] > 0.01 else ("←" if axes[i] < -0.01 else "·")
        with status_lock:
            status = joint_status[i]
        print(f"  {labels[i]}  {direction} {axes[i]:+.3f}  [{bar}]  {status[:28]:<28}")
    if stopped:
        print("  *** EMERGENCY STOP — release B and move stick to resume ***" + " " * 10)
    else:
        print(" " * 72)
    with status_lock:
        msg = last_msg
    print(f"  STM32: {msg:<64}")


def print_help():
    print("\nRuntime commands:")
    print("  G n kg       Gravity gain joint n (1-5),  e.g. G 2 10.0")
    print("  P n kp kv    PD gains joint n,            e.g. P 2 8.0 3.0")
    print("  H            Home (zero position estimates at current pose)")
    print("  E            Emergency stop")
    print("  R            Request joint status from STM32")
    print("  quit         Stop arm and exit")
    print("  B button     Emergency stop (hold)\n")


def handle_runtime_input(line, ser):
    parts = line.strip().split()
    if not parts:
        return

    cmd = parts[0].upper()

    if cmd == "G" and len(parts) == 3:
        try:
            n  = int(parts[1])
            kg = float(parts[2])
            ser.write(f"G {n} {kg:.3f}\n".encode())
        except ValueError:
            print("  ERR: usage: G <1-5> <kg>")

    elif cmd == "P" and len(parts) == 4:
        try:
            n  = int(parts[1])
            kp = float(parts[2])
            kv = float(parts[3])
            ser.write(f"P {n} {kp:.3f} {kv:.3f}\n".encode())
        except ValueError:
            print("  ERR: usage: P <1-5> <kp> <kv>")

    elif cmd == "H":
        ser.write(b"H\n")

    elif cmd == "E":
        ser.write(b"E\n")

    elif cmd == "R":
        ser.write(b"R\n")

    elif cmd == "QUIT":
        ser.write(b"E\n")
        time.sleep(0.1)
        ser.close()
        pygame.quit()
        sys.exit(0)

    else:
        print("  Unknown command. Type H for home, G/P for gains, R for status, quit to exit.")


def main():
    global estop_active

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
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.05)
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
    print(f"Streaming to {NUM_JOINTS} joints at {POLL_HZ} Hz | B = E-stop | Ctrl+C = Exit\n")

    # Print initial blank display block so in-place overwrite works
    # Must match lines = NUM_JOINTS + 3 in print_display
    print("─" * 72)
    for i in range(NUM_JOINTS):
        print(" " * 72)
    print(" " * 72)
    print(" " * 72)

    interval      = 1.0 / POLL_HZ
    status_timer  = 0.0   # periodic R poll
    axes          = [0.0] * NUM_JOINTS

    try:
        while True:
            loop_start = time.time()
            pygame.event.pump()

            # Check for typed runtime commands
            try:
                cmd_line = cmd_queue.get_nowait()
                handle_runtime_input(cmd_line, ser)
            except queue.Empty:
                pass

            b_pressed = joystick.get_button(BTN_B)

            if b_pressed:
                estop_active = True
                axes = [0.0] * NUM_JOINTS
                ser.write(b"E\n")
                print_display(axes, stopped=True)
            else:
                estop_active = False

                lx = apply_deadzone(joystick.get_axis(AXIS_LEFT_X),  DEADZONE)
                ly = apply_deadzone(joystick.get_axis(AXIS_LEFT_Y),  DEADZONE)
                rx = apply_deadzone(joystick.get_axis(AXIS_RIGHT_X), DEADZONE)
                ry = apply_deadzone(joystick.get_axis(AXIS_RIGHT_Y), DEADZONE)

                lt = trigger_to_ratio(joystick.get_axis(AXIS_LT))
                rt = trigger_to_ratio(joystick.get_axis(AXIS_RT))
                j5 = lt - rt
                if abs(j5) < DEADZONE:
                    j5 = 0.0

                axes = [
                     lx,   # J1 – base yaw
                    -ly,   # J2 – shoulder   (push forward = positive)
                     rx,   # J3 – elbow
                    -ry,   # J4 – wrist pitch (push forward = positive)
                     j5,   # J5 – wrist roll
                ]

                # Always send every cycle — the STM32 watchdog brakes in 250 ms
                # if this stream stops, so we must never skip a cycle.
                send_joystick(ser, axes)
                print_display(axes, stopped=False)

                # Periodically request status from STM32 (every 2 s)
                status_timer += interval
                if status_timer >= 2.0:
                    status_timer = 0.0
                    ser.write(b"R\n")

            elapsed = time.time() - loop_start
            sleep   = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n\nStopping arm...")
        ser.write(b"E\n")
        time.sleep(0.1)
        ser.close()
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
