# TTT Motor Controller

5-joint robotic arm controller using an STM32F767ZI, five VESC motor controllers
over CAN bus, and an Xbox controller connected via USB to a host PC.

---

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | STM32F767ZI (Nucleo-144 or custom board) |
| Motor controllers | 5× VESC (any model supporting CANSimple) |
| CAN bus | 500 kbit/s, STM32 PD0 (RX) / PD1 (TX) |
| Host serial | USART3 — PD8 (TX) / PD9 (RX) — 115200 8N1 |
| Controller | Xbox (USB) connected to host PC |
| Host PC | Runs `xbox_Controller.py` (Python 3, Windows/Linux/macOS) |

Each VESC must have its **VESC ID** set to match the table in `vesc_can.h`
(defaults: Motor 1 = ID 1, Motor 2 = ID 2, … Motor 5 = ID 5).
Set this in VESC Tool under *App Settings → General → VESC ID*.

---

## Control Flow

### End-to-end path

```
Xbox controller (USB)
        │  pygame reads axes at 50 Hz
        ▼
xbox_Controller.py  (host PC)
        │  Applies deadzone + rescale → normalised axis [-1.0, +1.0]
        │  Sends "J j1 j2 j3 j4 j5\n" over USB-serial at 50 Hz
        ▼
STM32F767ZI  USART3 RX (interrupt-driven)
        │  Accumulates bytes → sets cmd_ready flag
        │  Main loop calls Process_Command() → updates joy_axes[]
        │
        │  100 Hz control loop (polled via HAL_GetTick)
        │  calls ArmController_Update(joy_axes, dt)
        │
        ▼
arm_controller.c  — per joint, every 10 ms:
    1.  Slew-rate limit raw joystick axis
            joy_slewed approaches raw_joy at ±1.5 full-scale/sec
            → prevents current spikes from sudden stick deflections
    2.  Apply deadband (±5%)
    3.  Derive desired output-shaft velocity
            v_des = joy_slewed × v_max   [rev/s]
    4.  Update position setpoint
            • joystick active  → pos_sp += v_des × dt
                                  pos_sp clamped to [pos_min, pos_max]
            • joystick idle    → pos_sp frozen (position hold)
            • on joystick engage (first cycle out of deadband):
                                  pos_sp snapped to pos_est
                                  (prevents lurch from accumulated drift)
    5.  State estimation
            ERPM from VESC STATUS_1 CAN frame (arriving via interrupt)
            → output velocity:  vel_est = (ERPM / pole_pairs / 60) × gear_ratio
            → low-pass filter (α = 0.25, ~4 Hz bandwidth)
            → integrate to position:  pos_est += vel_est × dt
    6.  PD + gravity feedforward
            e_pos  = pos_sp  − pos_est
            e_vel  = v_des   − vel_est
            I_grav = kg × cos(pos_est × 2π)   ← counteracts gravity at any angle
            I_cmd  = kp × e_pos + kv × e_vel + I_grav
            I_cmd  clamped to ±i_max
        │
        ▼
vesc_can.c
        │  VESC_SetCurrent(motor, I_cmd)
        │  Encodes as CANSimple SET_CURRENT frame (cmd_id=1)
        │  Big-endian int32: amps × 1000
        ▼
VESC motor controller  (FOC current loop, ~20–40 kHz)
        │  Drives motor phase currents to match I_cmd
        │  Broadcasts STATUS_1 frame every ~50 ms (configurable in VESC Tool)
        ▼  (CAN RX interrupt → VESC_ProcessRx → updates _status[].erpm)
back to step 5 ↑
```

### Why this approach works for a heavy arm

| Problem with raw current/duty commands | How this design solves it |
|----------------------------------------|--------------------------|
| Arm falls when stick is released — no holding torque | **Position hold:** when joystick is idle the controller actively applies current to maintain `pos_sp` against gravity |
| Arm fights gravity — constant large current needed | **Gravity feedforward:** `kg × cos(θ)` adds just enough bias current at every angle to offset gravitational torque, so PD gains only need to correct small errors |
| Jitter / overshoot on stick engage | **Slew-rate limiter:** joystick value ramps at 1.5/sec, preventing the `kv × (v_des − vel_est)` term from spiking |
| Shooting past target and falling back | **Velocity damping:** the `kv × e_vel` term brakes the joint as it approaches setpoint, keeping motion smooth |
| PC comms glitch → arm falls | **250 ms watchdog:** STM32 calls `ArmController_EStop()` → `VESC_BrakeAll()` if no J command arrives |

### Safety layers (in order of priority)

1. **VESC hardware limits** — set absolute current, duty, and ERPM limits in
   VESC Tool. These cannot be overridden by CAN commands.
2. **STM32 `i_max` per joint** — `arm_controller.c` clamps `I_cmd` to `±i_max`
   before any CAN frame is sent.
3. **Soft position limits** — `pos_min`/`pos_max` clamp the setpoint integrator,
   preventing the arm from being commanded beyond configured travel bounds.
4. **Slew-rate limiter** — caps the rate of velocity change, preventing
   large instantaneous current demands.
5. **250 ms watchdog** — triggers `VESC_BrakeAll()` (handbrake current) if the
   PC serial stream stops for any reason.
6. **Emergency stop (B button / `E` command)** — immediately calls
   `ArmController_EStop()` (freezes setpoints, brakes all motors) and keeps
   the watchdog triggered until the J stream resumes.
7. **`VESC_BrakeAll()` on startup** — arm is braked before any control loop
   tick runs.

---

## Software Setup

### Requirements

```
python -m pip install pygame pyserial
```

### Running

1. Flash the STM32 firmware via STM32CubeIDE (build in `Debug` config).
2. Connect the Xbox controller to the PC via USB.
3. Connect the STM32 USART3 to the PC (USB-serial adapter or Nucleo virtual COM port).
4. Edit `COM_PORT` in `xbox_Controller.py` to match the device.
5. Run:
   ```
   python xbox_Controller.py
   ```

### Controller mapping

| Input | Joint | Action |
|-------|-------|--------|
| Left stick X | J1 – Base yaw | Left / right rotation |
| Left stick Y | J2 – Shoulder | Raise / lower upper arm |
| Right stick X | J3 – Elbow | Extend / retract forearm |
| Right stick Y | J4 – Wrist pitch | Tilt wrist up / down |
| LT − RT triggers | J5 – Wrist roll | Rotate wrist |
| **B button** | All | **Emergency stop** (hold to keep stopped) |

---

## Commissioning and Tuning

Before the arm moves a centimetre, set `gear_ratio` and `pole_pairs` correctly
in the config table at the top of `arm_controller.c`. Wrong values mean wrong
position estimates and wrong velocity scaling.

**Tune in this order, one joint at a time, with the arm at a safe pose:**

### Step 1 — Confirm ERPM direction

Send `R` in the Python terminal. Move a joint by hand and watch the
`erpm` column. Verify the sign is what you expect. If not, swap two motor
phase wires or invert the motor in VESC Tool.

### Step 2 — Set gravity gain `kg`

Set `kp = 0`, `kv = 0` for the joint under test. Then command via Python:
```
G 2 0
```
Let go of the stick. The arm will fall. Increase `kg`:
```
G 2 5
G 2 8
G 2 12
```
Stop when the arm holds a horizontal pose without drifting up or down.
This value cancels gravity so the PD gains only have to handle small errors.

### Step 3 — Set position gain `kp`

Re-enable with a small kp:
```
P 2 3.0 0.0
```
Nudge the joint by hand. It should spring back. Increase until it snaps
back crisply without oscillating. Typical range: 4–10 A/rev.

### Step 4 — Set velocity gain `kv`

```
P 2 8.0 1.0
```
Move the joint with the stick and release. A good `kv` makes it stop
smoothly without overshoot. Typical range: 1–4 A/(rev/s).

### Step 5 — Set speed limit `v_max`

The default `v_max` values are conservative (43–90 °/s). Increase only
after the arm is stable:
```
python xbox_Controller.py  # then observe motion at defaults first
```
Edit `cfg[].v_max` in `arm_controller.c` and reflash.

### Step 6 — Set soft travel limits `pos_min`/`pos_max`

Home the arm (`H` command) at a known safe pose, then move each joint to
its physical limits and record the `pos` value from `R`. Set `pos_min`
and `pos_max` slightly inside those limits in `arm_controller.c`.

---

## Serial Protocol Reference

All commands are ASCII, newline-terminated (`\n`), 115200 baud.

| Command | Direction | Description |
|---------|-----------|-------------|
| `J j1 j2 j3 j4 j5` | PC → STM32 | Joystick axes, each −1.0…+1.0. Send at ≥ 10 Hz (watchdog = 250 ms). |
| `G n kg` | PC → STM32 | Set gravity gain for joint *n* (1-based). |
| `P n kp kv` | PC → STM32 | Set PD gains for joint *n*. |
| `H` | PC → STM32 | Zero all position estimates (home). |
| `E` | PC → STM32 | Emergency stop. |
| `R` | PC → STM32 | Request joint status printout. |
| `?` | PC → STM32 | Print command help. |
| `J1 pos:… sp:… vel:… I:… erpm:…` | STM32 → PC | One line per joint, printed in response to `R`. |

---

## Building

Open `TTT_Controller/TTT_Controller.cproject` in STM32CubeIDE and build
the `Debug` configuration (Ctrl+B).  Alternatively from the command line:

```sh
cd TTT_Controller/Debug
make -j4
```

Toolchain: GNU Tools for STM32 13.3.rel1 (`arm-none-eabi-gcc`).
