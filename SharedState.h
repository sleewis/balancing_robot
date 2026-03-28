
#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ─────────────────────────────────────────────────────────────────────────────
// SharedState — data uitgewisseld tussen de slow core (1) en de fast core (0)
//
// Schrijven: altijd via xSharedMutex
// Lezen:     fast core probeert non-blocking (timeout 0); als de mutex bezet
//            is, gebruikt hij de vorige waarde.
// ─────────────────────────────────────────────────────────────────────────────
struct SharedState {

  // ── Targets van slow → fast ──────────────────────────────────────────────
  float targetVoltage1;   // [V] voltage-commando voor motor 1
  float targetVoltage2;   // [V] voltage-commando voor motor 2

  // Toekomstige PID-targets (uncomment wanneer PID wordt toegevoegd):
  // float targetTilt;    // [rad] gewenste kanteling voor balanceer-PID
  // float targetSpeed;   // [m/s] gewenste rijsnelheid
  // float targetYaw;     // [rad/s] gewenste draaisnelheid (XBOX joystick)

  // ── Telemetrie van fast → slow (read-only voor slow core) ────────────────
  float currentAngle1;    // [rad] gemeten mechanische hoek motor 1
  float currentAngle2;    // [rad] gemeten mechanische hoek motor 2
};

// Globale instanties — gedefinieerd in balancing_robot.ino
extern SemaphoreHandle_t xSharedMutex;
extern SharedState gShared;

#endif // SHARED_STATE_H
