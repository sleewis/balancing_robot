
// ═════════════════════════════════════════════════════════════════════════════
//  Balancing Robot — Sjoerd Leewis
//
//  Geschreven in samenwerking met Claude (Anthropic).
//  Claude heeft geholpen met de architectuur, motoraansturing, PID-cascade
//  en alle andere code in dit project.
// ═════════════════════════════════════════════════════════════════════════════

#include "Motor.h"
#include "IMU.h"
#include "PID.h"
#include "SharedState.h"

// ─── Veiligheidsdrempel ───────────────────────────────────────────────────────
// Als de kanteling groter is dan deze hoek, worden beide PIDs gereset en
// de motoren uitgeschakeld om integrator-windup te voorkomen.
#define TILT_LIMIT_DEG  30.0f

// ─── I2C-bussen ───────────────────────────────────────────────────────────────
// I2C_1 (Wire0, GPIO19/18): AS5600 motor 1 (0x36) + BNO055 IMU (0x28)
// I2C_2 (Wire1, GPIO23/5):  AS5600 motor 2 (0x36)
TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);

// ─── Motoren ──────────────────────────────────────────────────────────────────
Motor motor1(33, 32, 25, 12, 39, 36, &I2C_1, 11);
Motor motor2(26, 27, 14, 12, 34, 35, &I2C_2, 11);

// ─── IMU ──────────────────────────────────────────────────────────────────────
IMU imu(&I2C_1);  // BNO055 op I2C_1, adres 0x28

// ─── CASCADE REGELAAR ─────────────────────────────────────────────────────────
//
//  Buitenste lus (tilt PID) — langzame fout → gewenste snelheid:
//    fout     = gemeten kanteling [°] − gewenste kanteling [°]
//    uitgang  = gewenste wielsnelheid [rad/s]
//
//  Binnenste lus (velocity PID) — snelheidsfout → motorspanning:
//    fout     = gewenste snelheid [rad/s] − gemeten snelheid [rad/s]
//    uitgang  = motorspanning [V]
//
// ── Afsteladvies ─────────────────────────────────────────────────────────────
//  1. Begin met velocity PID alleen (tiltPID.Kp = 0), geef een vaste
//     targetVelocity van 0 rad/s en controleer of de wielen stilhouden.
//     Stem vp en vi af tot de snelheid stabiel op 0 blijft.
//
//  2. Zet daarna tiltPID.Kp op een kleine waarde (bijv. 0.5).
//     De robot probeert nu te balanceren door de snelheid bij te sturen.
//     Verhoog Kp tot de robot goed reageert op kanteling.
//
//  3. Voeg tiltPID.Kd toe om oscillaties te dempen.
//     Voeg tiltPID.Ki pas als laatste toe voor langzame drift.
//
// Serieel afstellen: zie slowTask hieronder.
// ─────────────────────────────────────────────────────────────────────────────

// Buitenste lus: tilt [°] → gewenste snelheid [rad/s]
// Uitgangslimieten bepalen de maximale rijsnelheid — niet te hoog instellen!
PID tiltPID(
  /*Kp*/  2.0f,    // [rad/s per °]  begin hier
  /*Ki*/  0.0f,
  /*Kd*/  0.05f,
  /*min*/ -5.0f,   // [rad/s] max achteruit
  /*max*/  5.0f    // [rad/s] max vooruit
);

// Binnenste lus: snelheidsfout [rad/s] → motorspanning [V]
// Stem deze PID eerst af voordat je de tilt PID aanzet.
PID velocityPID(
  /*Kp*/  0.5f,    // [V per rad/s]
  /*Ki*/  0.2f,    // integrator corrigeert slippen en lading
  /*Kd*/  0.0f,    // afgeleide van snelheid is te ruis-gevoelig
  /*min*/ -10.0f,  // [V] = VOLTAGE_LIMIT
  /*max*/  10.0f
);

// ─── Gedeelde toestand & mutex ────────────────────────────────────────────────
SemaphoreHandle_t xSharedMutex;
SharedState gShared = {
  .targetTilt      = 0.0f,
  .currentTilt     = 0.0f,
  .currentAngle1   = 0.0f,
  .currentAngle2   = 0.0f,
  .currentVelocity = 0.0f,
  .targetVelocity  = 0.0f,
  .pidOutput       = 0.0f,
  .avgCurrent1     = 0.0f,
  .avgCurrent2     = 0.0f,
  .motor1Ok        = false,
  .motor2Ok        = false,
};


// ═════════════════════════════════════════════════════════════════════════════
//  FAST TASK — Core 0  (500 Hz)
//
//  Volgorde per cyclus:
//    1. Lees targetTilt van slow core (non-blocking mutex)
//    2. Lees IMU elke 5 cycli → 100 Hz
//    3. Buitenste lus: tilt PID → gewenste snelheid
//    4. Veiligheidscheck: reset & stop bij te grote kanteling
//    5. Binnenste lus: velocity PID → motorspanning
//    6. Stuur motoren aan
//    7. Schrijf telemetrie terug naar gShared
//    8. Wacht op volgend 2ms-slot
// ═════════════════════════════════════════════════════════════════════════════
void fastTask(void *pvParameters) {
  motor1.begin();
  motor2.begin();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(2);  // 2 ms = 500 Hz

  const float DT       = 0.002f;   // [s] tijdstap
  float targetTilt     = 0.0f;     // lokale kopie van gShared.targetTilt
  uint8_t imuDiv       = 0;        // teller voor IMU-decimering (÷5 = 100 Hz)

  while (true) {
    // ── 1) Lees target van slow core ─────────────────────────────────────────
    if (xSemaphoreTake(xSharedMutex, 0) == pdTRUE) {
      targetTilt = gShared.targetTilt;
      xSemaphoreGive(xSharedMutex);
    }

    // ── 2) Lees IMU elke 5e cyclus (100 Hz) ──────────────────────────────────
    if (++imuDiv >= 5) {
      imuDiv = 0;
      imu.update();
    }
    float tilt = imu.getTilt();

    // ── 3) Buitenste lus: tilt PID → gewenste wielsnelheid ───────────────────
    // Bij positieve kanteling (voorover) → positieve doelsnelheid (rijdt vooruit)
    // zodat de wielen de massa achterop lopen en de robot rechtop trekt.
    float tiltError     = tilt - targetTilt;
    float velTarget     = tiltPID.update(tiltError, DT);

    // ── 4) Veiligheidscheck ───────────────────────────────────────────────────
    bool safe = (fabsf(tilt) < TILT_LIMIT_DEG) && imu.isReady();
    if (!safe) {
      velTarget = 0.0f;
      tiltPID.reset();
      velocityPID.reset();
    }

    // ── 5) Binnenste lus: velocity PID → motorspanning ───────────────────────
    // Gemiddelde voorwaartse wielsnelheid: motor 2 is gespiegeld → teken omdraaien.
    float avgVelocity = (motor1.getVelocity() - motor2.getVelocity()) * 0.5f;
    float velError    = velTarget - avgVelocity;
    float voltage     = velocityPID.update(velError, DT);

    // ── 6) Motoraansturing ────────────────────────────────────────────────────
    // Motor.loop() stopt automatisch bij encoder-fout of overstroom.
    motor1.loop( voltage);
    motor2.loop(-voltage);

    // ── 7) Telemetrie → slow core (non-blocking) ──────────────────────────────
    if (xSemaphoreTake(xSharedMutex, 0) == pdTRUE) {
      gShared.currentTilt     = tilt;
      gShared.currentAngle1   = motor1.getAngle();
      gShared.currentAngle2   = motor2.getAngle();
      gShared.currentVelocity = avgVelocity;
      gShared.targetVelocity  = velTarget;
      gShared.pidOutput       = voltage;
      gShared.avgCurrent1     = motor1.getAvgCurrent();
      gShared.avgCurrent2     = motor2.getAvgCurrent();
      gShared.motor1Ok        = motor1.isOk();
      gShared.motor2Ok        = motor2.isOk();
      xSemaphoreGive(xSharedMutex);
    }

    // ── 8) Wacht op volgend 2ms-slot ──────────────────────────────────────────
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  SLOW TASK — Core 1  (~50 Hz)
//
//  Seriële live-tuning — stuur commando's via de seriële monitor:
//
//    Tilt PID:      p<waarde>   i<waarde>   d<waarde>
//    Velocity PID:  vp<waarde>  vi<waarde>  vd<waarde>
//    Target tilt:   t<waarde>   (handmatig instellen in graden)
//
//  Voorbeelden:  "p2.0"   "vp0.5"   "t3.0"
// ═════════════════════════════════════════════════════════════════════════════
void slowTask(void *pvParameters) {
  while (true) {
    // ── Lees telemetrie van fast core ─────────────────────────────────────────
    float tilt, vel, velTgt, pidOut, cur1, cur2;
    bool  ok1, ok2;
    if (xSemaphoreTake(xSharedMutex, portMAX_DELAY) == pdTRUE) {
      tilt   = gShared.currentTilt;
      vel    = gShared.currentVelocity;
      velTgt = gShared.targetVelocity;
      pidOut = gShared.pidOutput;
      cur1   = gShared.avgCurrent1;
      cur2   = gShared.avgCurrent2;
      ok1    = gShared.motor1Ok;
      ok2    = gShared.motor2Ok;
      xSemaphoreGive(xSharedMutex);
    }

    // ── Seriële debug-output ──────────────────────────────────────────────────
    Serial.printf(
      "[slow] tilt=%6.2f°  vel=%6.2f→%5.2f rad/s  out=%6.3fV  I1=%5.2fA  I2=%5.2fA  M1=%s  M2=%s\n",
      tilt, vel, velTgt, pidOut, cur1, cur2,
      ok1 ? "OK" : "FOUT",
      ok2 ? "OK" : "FOUT"
    );

    // ── Seriële live-tuning ───────────────────────────────────────────────────
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

      if (cmd.length() >= 2) {
        // Velocity PID: commando begint met 'v' gevolgd door p/i/d
        if (cmd.charAt(0) == 'v' && cmd.length() >= 3) {
          char  key = cmd.charAt(1);
          float val = cmd.substring(2).toFloat();
          if      (key == 'p') { velocityPID.Kp = val; Serial.printf("velocity Kp = %.4f\n", val); }
          else if (key == 'i') { velocityPID.Ki = val; Serial.printf("velocity Ki = %.4f\n", val); }
          else if (key == 'd') { velocityPID.Kd = val; Serial.printf("velocity Kd = %.4f\n", val); }
        } else {
          // Tilt PID of target
          char  key = cmd.charAt(0);
          float val = cmd.substring(1).toFloat();
          if      (key == 'p') { tiltPID.Kp = val; Serial.printf("tilt Kp = %.4f\n", val); }
          else if (key == 'i') { tiltPID.Ki = val; Serial.printf("tilt Ki = %.4f\n", val); }
          else if (key == 'd') { tiltPID.Kd = val; Serial.printf("tilt Kd = %.4f\n", val); }
          else if (key == 't') {
            if (xSemaphoreTake(xSharedMutex, portMAX_DELAY) == pdTRUE) {
              gShared.targetTilt = val;
              xSemaphoreGive(xSharedMutex);
            }
            Serial.printf("targetTilt = %.2f°\n", val);
          }
        }
      }
    }

    // TODO: lees XBOX-controller via Bluetooth
    // TODO: gShared.targetTilt = joystickY * MAX_TILT_DEG;

    vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // I2C initialiseren vóór de tasks starten
  I2C_1.begin(19, 18, 400000);  // motor 1 encoder + BNO055
  I2C_2.begin(23,  5, 400000);  // motor 2 encoder

  // BNO055 initialiseren
  if (!imu.begin()) {
    Serial.println("[ERROR] BNO055 niet gevonden! Controleer bedrading op I2C_1 (GPIO19/18).");
  } else {
    Serial.println("[OK] BNO055 gevonden.");
  }

  xSharedMutex = xSemaphoreCreateMutex();

  // Fast task: Core 0, hogere prioriteit zodat de motorloop nooit blokkeert
  xTaskCreatePinnedToCore(fastTask, "FastTask", 4096, NULL, 2, NULL, 0);
  // Slow task: Core 1, lagere prioriteit
  xTaskCreatePinnedToCore(slowTask, "SlowTask", 4096, NULL, 1, NULL, 1);
}


// ═════════════════════════════════════════════════════════════════════════════
//  LOOP — niet gebruikt; beide cores draaien via FreeRTOS-tasks
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  vTaskDelete(NULL);
}
