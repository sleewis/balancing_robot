# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Self-balancing robot (Segway-style) on an ESP32, with planned Bluetooth XBOX controller support.

## Compiling

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 "C:/Users/sleew/Onedrive/documenten/arduino/balancing_robot"
```

The MKS-ESP32FOC V2.0 is not a named board in arduino-cli; use the generic `esp32:esp32:esp32` FQBN. Uploading is done via the Arduino IDE or GitHub Desktop workflow — not from the CLI.

## Hardware

- **Board:** MKS-ESP32FOC V2.0 (ESP32)
- **Motors:** 2× GM4108H-120T BLDC (11 pole pairs, 11.1 Ω phase resistance, ~20 V)
- **Encoders:** 2× AS5600 magnetic encoder (I²C 0x36), one per bus
- **IMU:** BNO055 (I²C 0x28, IMUPLUS mode — accel + gyro fusion, no magnetometer)
- **I²C buses:**
  - `I2C_1` (Wire0, GPIO 19/18): motor 1 encoder + BNO055
  - `I2C_2` (Wire1, GPIO 23/5): motor 2 encoder

### Motor GPIO pins

| | U (PWM) | V (PWM) | W (PWM) | EN | IA (ADC) | IB (ADC) |
|---|---|---|---|---|---|---|
| motor1 | 33 | 32 | 25 | 12 | 39 | 36 |
| motor2 | 26 | 27 | 14 | 12 | 34 | 35 |

Motor 2 is mounted mirrored; its velocity and drive voltage sign are inverted in firmware (`motor2.loop(-voltage)`).

## Architecture

The firmware runs two FreeRTOS tasks pinned to separate cores:

| Task | Core | Rate | Responsibility |
|------|------|------|----------------|
| `fastTask` | 1 | 1000 Hz (1 ms) | Motor FOC loop, cascade PID, IMU read (÷10 = 100 Hz) |
| `slowTask` | 0 | 50 Hz (20 ms) | Serial telemetry output, live PID tuning, (future) Bluetooth |

Tasks share state via `gShared` (`SharedState` struct) guarded by `xSharedMutex`. The fast task always takes the mutex non-blocking (timeout 0) — it never waits, to avoid jitter in the motor loop.

### Cascade PID

```
tilt [°] ──► tiltPID ──► velTarget [rad/s] ──► velocityPID ──► iqTarget [A] ──► FOC
```

- **tiltPID** (outer loop): tilt error → desired wheel velocity. Safety: reset + stop if `|tilt| > 30°` or IMU not ready.
- **velocityPID** (inner loop): velocity error → q-axis current command (Ampère, not Volt).
- Motor 2 is mounted mirrored; its velocity sign is inverted before averaging.

### Motor / FOC (`Motor.h/.cpp`)

Each motor runs full Field-Oriented Control internally:
- Clarke + Park transforms on phase currents (Ia, Ib measured via shunt ADC)
- Two PI controllers: Id → 0 A (flux), Iq → iqTarget (torque)
- Inverse Park + inverse Clarke → SVPWM duty cycles at 20 kHz
- AS5600 encoder read over I²C on every `loop()` call for rotor angle
- Parallel alignment at startup: both motors `alignStart()` simultaneously, single 2 s delay, then both `alignFinish()`.

### Modules

| File | Purpose |
|------|---------|
| `balancing_robot.ino` | Entry point; task creation, shared state, PID instances, I²C init |
| `Motor.h/.cpp` | BLDC motor driver: FOC, AS5600 encoder, current sensing, overcurrent protection |
| `IMU.h/.cpp` | BNO055 driver (raw I²C, IMUPLUS mode); exposes `getTilt()` in degrees |
| `PID.h/.cpp` | Generic PID with anti-windup (integrator clamping) and derivative-on-error |
| `SharedState.h` | `SharedState` struct + extern declarations for the mutex and global state |

## Live PID Tuning

Send commands over Serial (115200 baud) — no newline needed, `Serial.readStringUntil('\n')`:

| Command | Effect |
|---------|--------|
| `p<val>` | tilt PID Kp |
| `i<val>` | tilt PID Ki |
| `d<val>` | tilt PID Kd |
| `vp<val>` | velocity PID Kp |
| `vi<val>` | velocity PID Ki |
| `vd<val>` | velocity PID Kd |
| `t<val>` | target tilt [°] |

Example: `p3.5` sets tilt Kp to 3.5.

## Key Constants to Know

- `TILT_LIMIT_DEG 30.0f` — safety cutoff angle (Motor.h defines motor-side limits)
- `VOLTAGE_LIMIT 7.0f` V — max phase voltage for FOC (conservative for 4S LiPo)
- `VELOCITY_ALPHA 0.80f` — LP filter coefficient for wheel velocity
- `MAX_CURRENT_A 6.0f` — overcurrent threshold (sum of |Ia|+|Ib|+|Ic|)
- `FOC_KP 0.5f`, `FOC_KI 50.0f` — inner current-loop PI gains

## Pending / TODOs

- Bluetooth XBOX controller input in `slowTask` (stubs present as `TODO` comments)
- `targetYaw` for differential steering (stub in `SharedState`)
