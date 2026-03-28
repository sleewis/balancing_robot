
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
//            is, gebruikt hij de vorige waarde — motorloop blokkeert nooit.
// ─────────────────────────────────────────────────────────────────────────────
struct SharedState {

  // ── Targets van slow → fast ──────────────────────────────────────────────

  // Gewenste kanteling [°] — 0 = rechtop staand
  // Slow core past dit aan op basis van XBOX-joystick (voor/achter rijden)
  float targetTilt;

  // Toekomstige uitbreidingen:
  // float targetYaw;     // [°/s] draaisnelheid (XBOX rechter joystick)

  // ── Telemetrie van fast → slow (alleen-lezen voor slow core) ─────────────

  float currentTilt;    // [°]  gemeten kanteling (BNO055 pitch)
  float currentAngle1;  // [rad] mechanische hoek motor 1 (AS5600)
  float currentAngle2;  // [rad] mechanische hoek motor 2 (AS5600)
  float pidOutput;      // [V]  laatste PID-uitgang (debug)
};

// Globale instanties — gedefinieerd in balancing_robot.ino
extern SemaphoreHandle_t xSharedMutex;
extern SharedState gShared;

#endif // SHARED_STATE_H
