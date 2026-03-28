# Balancing Robot

## Overview

This project implements a two‑wheel self‑balancing robot using an ESP32. The control system is split into fast and slow tasks running on separate CPU cores to ensure deterministic motor control while still allowing debugging and live tuning.

Main goals of the project:

- Maintain upright balance using a tilt controller
- Drive two BLDC motors with magnetic encoders
- Use an IMU for tilt estimation
- Allow live PID tuning during testing

The architecture is intentionally modular so that sensors, motor drivers, and control algorithms can be improved independently.

---

# System Architecture

The ESP32 runs two FreeRTOS tasks:

## Fast Task (Core 0 – 500 Hz)
Responsible for real‑time control.

Execution cycle:

1. Read target tilt from shared state
2. Update IMU (100 Hz decimated from loop)
3. Run balance PID
4. Safety check (tilt limit)
5. Update motor voltages
6. Publish telemetry
7. Wait for next control tick

This loop must never block.

## Slow Task (Core 1 – ~50 Hz)
Handles non‑critical work:

- Serial debugging
- PID live tuning
- Controller input (future)
- Telemetry output

Communication between tasks uses a mutex protected shared structure.

---

# Hardware

## Controller

MKS-ESP32FOC V2.0 (ESP32-gebaseerd motorstuurboard)

## Sensors

### IMU

BNO055

Used for tilt measurement using Euler pitch.

Verbonden op I2C bus 0 (GPIO 19/18), gedeeld met de encoder van motor 1.
Adres BNO055: 0x28 — geen conflict met AS5600 (0x36).

### Motor Encoders

AS5600 magnetic encoders.

Each motor has its own I2C bus to avoid address conflicts.

## Motors

Two BLDC motors with three phase drivers.

Motor phases are driven using ESP32 LEDC PWM outputs.

---

# Software Components

## Motor

Responsible for:

- Reading magnetic encoder
- Generating three‑phase sinusoidal voltage
- Rotor alignment at startup

Currently the motor control is open‑loop voltage based.

## IMU

Minimal BNO055 driver configured in **IMUPLUS mode**.

Only pitch is used for balancing.

## PID Controller

Generic PID implementation with:

- anti‑windup
- runtime gain adjustment

## SharedState

Small structure used to exchange data between the fast and slow cores.

---

# Control Strategy

The robot uses a single tilt controller:

PID( tilt_error ) → motor voltage

Where:

```
error = measuredTilt - targetTilt
```

Motor 2 is mounted mirrored so the sign of its voltage is inverted.

---

# Safety

Motor output is disabled if:

- IMU is not ready
- tilt angle exceeds safety threshold

The PID integrator is reset during safety shutdown to avoid wind‑up.

---

# Tuning PID

PID gains can be tuned live via serial:

```
p1.5
i0.1
d0.05
```

Recommended tuning order:

1. Start with **P only**
2. Add **D** to damp oscillations
3. Add **I** only if robot slowly drifts

---

# Future Improvements

## Motor Control Improvements

The current motor implementation is intentionally simple and should be improved.

### 1. Closed Loop Velocity Control

Currently motors are driven by direct phase voltage.

Future improvement:

Use encoder feedback to regulate **wheel velocity**.

This creates a cascade controller:

```
Tilt PID → desired wheel velocity
Velocity PID → motor torque / voltage
```

Benefits:

- smoother motion
- less sensitivity to load changes
- improved balance stability

### 2. Field‑Oriented Control (FOC) — stroom-regulatie

The motor driver already generates sinusoidal phase voltages based on encoder angle.
This is the foundation of FOC (open‑loop).

What is currently missing is the **inner current control loop**:

- Clarke transform (phase currents → αβ)
- Park transform (αβ → dq, field-aligned)
- PI current regulators on d and q axis
- Inverse Park + Space Vector Modulation

The hardware already measures phase currents (AS5600 shunts).

Benefits once implemented:

- torque control instead of voltage control
- higher efficiency
- smoother response at low speeds
- protection against overcurrent

### 3. Current Limiting

Current is already measured but not yet used.

Future use:

- motor protection
- torque control
- fault detection

### 4. Encoder Failure Handling

Encoder read currently waits until data is available.

Future improvement:

- timeout detection
- motor shutdown on sensor loss

---

# Possible Sensor Improvements

The BNO055 provides fused Euler angles which may introduce latency.

Future improvement could include:

- raw gyro + accelerometer fusion
- complementary or Kalman filter

Lower latency improves balancing performance.

---

# Planned Features

- Bluetooth controller input
- speed control
- steering control
- telemetry streaming
- autonomous stabilization modes

---

# Project Status

Early prototype.

Core architecture and control loop implemented.

Currently focused on achieving stable balancing before expanding functionality.

