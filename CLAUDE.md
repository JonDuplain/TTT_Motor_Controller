# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F767ZI-based CAN bus controller for 5x VESC motor controllers. The firmware receives UART commands over USART3 (115200 baud) and translates them into VESC CANSimple protocol frames sent over CAN1 at 500 kbit/s.

## Build Environment

This project is built with **STM32CubeIDE** (Eclipse-based). The IDE manages compilation via the generated makefiles in `TTT_Controller/Debug/`.

**Toolchain:** GNU Tools for STM32 (arm-none-eabi-gcc 13.3.rel1)

To build from the command line (from `TTT_Controller/Debug/`):
```sh
make -j4
```

To flash, use STM32CubeIDE's built-in debug/flash or ST-Link Utility targeting the `.elf` in `TTT_Controller/Debug/`.

**Compiler flags (Debug config):**
- Target: `-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard`
- Standard: `-std=gnu11`
- Defines: `-DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx`
- Specs: `--specs=nano.specs`

## Architecture

### Source files (all application code is in `TTT_Controller/Core/Src/`)

| File | Role |
|------|------|
| `main.c` | Peripheral init (CAN1, USART3, GPIO, MPU), main loop, UART command parser |
| `vesc_can.c` | VESC CANSimple protocol driver — TX commands and RX status parsing |
| `vesc_can.h` | Public API, VESC IDs, command IDs, `VescStatus_t` struct |
| `stm32f7xx_it.c` | ISR handlers (USART3, CAN1 IRQs routed through HAL callbacks) |
| `stm32f7xx_hal_msp.c` | HAL MSP (GPIO pin mux for CAN1: PD0/PD1, UART3: PD8/PD9) |

### CAN / VESC protocol

- Extended 29-bit CAN ID format: `cmd_id[15:8] | vesc_id[7:0]`
- VESC IDs 1–5 map to `MOTOR_1`–`MOTOR_5` (must match VESC Tool "App Settings > VESC ID")
- Key TX command IDs: `SET_DUTY(0)`, `SET_CURRENT(1)`, `SET_CURRENT_BRAKE(2)`, `SET_RPM(3)`, `SET_CURRENT_HANDBRAKE(12)`
- RX: `STATUS_1(9)` frames carry ERPM (int32), current (int16 ÷10 → A), duty (int16 ÷1000)
- All TX values are big-endian 4-byte integers; current is amps × 1000, duty is × 100000
- `VESC_BRAKE_CURRENT` (15 A via handbrake cmd 12) is applied when a motor is idle — adjust in `vesc_can.h` if motors drift or run warm at standstill

### Main loop behavior

- **Keep-alive refresh every 200 ms:** re-sends the last current command or brake to all motors (VESC requires periodic commands or it will cut output)
- **Auto status print every 500 ms** when any motor is running (via UART)
- **UART command interface** (interrupt-driven, newline-terminated):
  - `c <1-5> <A>` — set current on one motor
  - `ca <A>` — set current on all motors
  - `s <1-5>` — brake one motor
  - `sa` — brake all motors
  - `r` — print status
  - `?` — help

### CAN ISR flow

`CAN1_RX0_IRQn` → `HAL_CAN_RxFifo0MsgPendingCallback()` (in `main.c`) → `VESC_ProcessRx()` (drains FIFO, updates `_status[]`)

### STM32CubeIDE / code-generation notes

CubeMX-generated sections are fenced with `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` comments. Only code inside these fences survives regeneration. Do not place application logic outside these blocks.

The `.ioc` file (`TTT_Controller/TTT_Controller.ioc`) is the CubeMX project file — editing it in CubeMX and regenerating will overwrite non-fenced code.

### Linker scripts

- `STM32F767ZITX_FLASH.ld` — normal flash-boot configuration
- `STM32F767ZITX_RAM.ld` — RAM-execute configuration (for fast debug iteration)
