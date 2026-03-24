"""
VESC CAN Bus Monitor — Motor 3
==============================
Listens directly on the CAN bus for STATUS_1 and STATUS_5 frames from VESC ID 3
and displays ERPM, velocity, current, and tachometer position in real-time.

This script is completely independent of the STM32 — use it to verify that the
VESC is actually broadcasting frames before debugging the STM32 firmware.

HARDWARE REQUIRED
-----------------
A USB-to-CAN adapter connected to the CAN bus (CANH / CANL).
Common cheap options:
  • CANable v1/v2 (slcan or candlelight firmware)  — ~$20
  • PEAK PCAN-USB                                  — ~$300
  • Any SLCAN-compatible adapter

The adapter must be on the same physical bus as the VESC.
Make sure 120 Ω termination is present at both ends of the bus.

INSTALL
-------
    python -m pip install python-can

USAGE (pick the line that matches your adapter)
-----------------------------------------------
    # CANable / SLCAN adapter on Windows
    python vesc_can_monitor.py --interface slcan --channel COM3

    # CANable / SLCAN adapter on Linux
    python vesc_can_monitor.py --interface slcan --channel /dev/ttyUSB0

    # CANable with candlelight firmware (shows up as native CAN on Linux)
    python vesc_can_monitor.py --interface socketcan --channel can0

    # PEAK PCAN-USB on Windows
    python vesc_can_monitor.py --interface pcan --channel PCAN_USBBUS1

    # PEAK PCAN-USB on Linux (after loading pcan driver)
    python vesc_can_monitor.py --interface socketcan --channel can0

CONFIGURATION
-------------
Edit the constants below to match your motor if needed.
"""

import argparse
import struct
import time
import sys

# ── Motor 3 configuration ───────────────────────────────────────────────────
VESC_ID     = 3      # must match App Settings > VESC ID in VESC Tool
POLE_PAIRS  = 7      # 6374 motor: 14 poles = 7 pole pairs
GEAR_RATIO  = 0.02   # output shaft revolutions per motor revolution (1/50 for 50:1)
# ────────────────────────────────────────────────────────────────────────────

# VESC CANSimple extended frame IDs
# ExtId = (cmd_id << 8) | vesc_id
STATUS_1_EXTID = (9  << 8) | VESC_ID   # ERPM, current, duty
STATUS_5_EXTID = (27 << 8) | VESC_ID   # tachometer, abs_tachometer


def parse_status1(data):
    """STATUS_1: int32 ERPM, int16 current (/10 = A), int16 duty (/1000)"""
    erpm, current_raw, duty_raw = struct.unpack(">ihh", data[:8])
    return erpm, current_raw / 10.0, duty_raw / 1000.0


def parse_status5(data):
    """STATUS_5: int32 tachometer, int32 abs_tachometer"""
    tach, tach_abs = struct.unpack(">ii", data[:8])
    return tach, tach_abs


def tach_to_position(tach, pole_pairs, gear_ratio):
    """
    Convert VESC tachometer steps to output-shaft position.
    VESC tachometer increments by (pole_pairs * 6) per motor revolution.
    """
    motor_revs  = tach / (pole_pairs * 6)
    output_revs = motor_revs * gear_ratio
    output_deg  = output_revs * 360.0
    return motor_revs, output_revs, output_deg


def main():
    parser = argparse.ArgumentParser(
        description="Monitor VESC ID 3 CAN frames (STATUS_1 + STATUS_5)")
    parser.add_argument("--interface", default="slcan",
                        help="python-can interface type  (default: slcan)")
    parser.add_argument("--channel",   default="COM3",
                        help="CAN adapter channel/port   (default: COM3)")
    parser.add_argument("--bitrate",   default=500000, type=int,
                        help="CAN bus bitrate in bit/s   (default: 500000)")
    args = parser.parse_args()

    try:
        import can
    except ImportError:
        print("ERROR: python-can not installed.  Run:  python -m pip install python-can")
        sys.exit(1)

    print(f"Connecting: interface={args.interface}  channel={args.channel}"
          f"  bitrate={args.bitrate}")
    print(f"Monitoring VESC ID {VESC_ID}  |  "
          f"STATUS_1 ExtId=0x{STATUS_1_EXTID:04X}  "
          f"STATUS_5 ExtId=0x{STATUS_5_EXTID:04X}")
    print(f"pole_pairs={POLE_PAIRS}  gear_ratio={GEAR_RATIO}\n")

    try:
        bus = can.interface.Bus(interface=args.interface,
                                channel=args.channel,
                                bitrate=args.bitrate)
    except Exception as e:
        print(f"ERROR: Could not open CAN bus: {e}")
        print("\nCheck:")
        print("  • Adapter is plugged in and recognised by the OS")
        print("  • --channel matches the correct port (e.g. COM3, /dev/ttyUSB0, can0)")
        print("  • No other software (VESC Tool) has the adapter open")
        sys.exit(1)

    print("Waiting for frames from VESC 3... (Ctrl+C to stop)\n")
    print(f"{'Time':>8}  {'Frame':8}  {'ERPM':>10}  {'Vel(rev/s)':>11}  "
          f"{'Current(A)':>11}  {'Duty':>7}  {'MotorRev':>10}  {'OutRev':>9}  {'OutDeg':>9}")
    print("─" * 100)

    tach_home      = None
    last_status1   = None
    last_status5   = None
    frame_count    = 0
    t_start        = time.time()

    try:
        while True:
            msg = bus.recv(timeout=0.5)

            if msg is None:
                elapsed = time.time() - t_start
                if frame_count == 0:
                    print(f"  No frames received in {elapsed:.1f}s — "
                          f"check VESC Tool Status rate > 0, CAN wiring, termination")
                continue

            if not msg.is_extended_id:
                continue  # VESC uses 29-bit extended IDs

            elapsed = time.time() - t_start

            if msg.arbitration_id == STATUS_1_EXTID and len(msg.data) >= 8:
                erpm, current, duty = parse_status1(msg.data)
                vel_rps = (erpm / POLE_PAIRS / 60.0) * GEAR_RATIO
                last_status1 = (erpm, vel_rps, current, duty)
                frame_count += 1
                print(f"{elapsed:8.2f}  {'STATUS_1':8}  {erpm:>10d}  {vel_rps:>+11.5f}  "
                      f"{current:>+11.3f}  {duty:>+7.4f}  "
                      f"{'':>10}  {'':>9}  {'':>9}")

            elif msg.arbitration_id == STATUS_5_EXTID and len(msg.data) >= 8:
                tach, tach_abs = parse_status5(msg.data)

                if tach_home is None:
                    tach_home = tach
                    print(f"  [tachometer zeroed at {tach} — spin motor to see position change]")

                rel_tach                = tach - tach_home
                motor_revs, out_revs, out_deg = tach_to_position(
                    rel_tach, POLE_PAIRS, GEAR_RATIO)
                last_status5 = (rel_tach, motor_revs, out_revs, out_deg)
                frame_count += 1
                print(f"{elapsed:8.2f}  {'STATUS_5':8}  {'':>10}  {'':>11}  "
                      f"{'':>11}  {'':>7}  "
                      f"{motor_revs:>+10.4f}  {out_revs:>+9.5f}  {out_deg:>+9.3f}°")

    except KeyboardInterrupt:
        print(f"\n\nStopped.  {frame_count} frames received in {time.time()-t_start:.1f}s")
        if last_status1:
            erpm, vel, cur, duty = last_status1
            print(f"  Last STATUS_1: erpm={erpm:+d}  vel={vel:+.4f} rev/s  "
                  f"I={cur:+.2f}A  duty={duty:+.4f}")
        if last_status5:
            rel, mrev, orev, odeg = last_status5
            print(f"  Last STATUS_5: tach_rel={rel:+d}  motor={mrev:+.4f}rev  "
                  f"output={orev:+.5f}rev  ({odeg:+.2f}°)")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
