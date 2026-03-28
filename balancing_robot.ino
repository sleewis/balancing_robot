
#include "Motor.h"
#include "IMU.h"
#include "PID.h"
#include "SharedState.h"

// ─── Veiligheidsdrempel ───────────────────────────────────────────────────────
// Als de kanteling groter is dan deze hoek, worden de motoren uitgeschakeld
// en wordt de PID gereset om integrator-windup te voorkomen.
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

// ─── PID ──────────────────────────────────────────────────────────────────────
// Gains zijn PLACEHOLDER-waarden — moeten worden afgestemd!
// Begin met alleen Kp (Ki=0, Kd=0) en verhoog voorzichtig.
//
//  Kp  te laag  → robot valt langzaam; te hoog → oscillaties
//  Kd          → dempt oscillaties; te hoog → versterkt sensor-ruis
//  Ki          → corrigeert mechanisch onevenwicht; voeg pas toe als P+D stabiel is
//
// Uitgang [V]: positief = voorover rijden, negatief = achteruit
PID tiltPID(
  /*Kp*/  1.5f,
  /*Ki*/  0.0f,
  /*Kd*/  0.05f,
  /*min*/ -12.0f,
  /*max*/  12.0f
);

// ─── Gedeelde toestand & mutex ────────────────────────────────────────────────
SemaphoreHandle_t xSharedMutex;
SharedState gShared = {
  .targetTilt   = 0.0f,
  .currentTilt  = 0.0f,
  .currentAngle1 = 0.0f,
  .currentAngle2 = 0.0f,
  .pidOutput    = 0.0f,
};


// ═════════════════════════════════════════════════════════════════════════════
//  FAST TASK — Core 0  (500 Hz)
//
//  Volgorde per cyclus:
//    1. Lees targetTilt van slow core (non-blocking mutex)
//    2. Lees IMU elke 5 cycli  → 100 Hz (BNO055 fusion-snelheid)
//    3. Voer PID uit
//    4. Veiligheidscheck: snijd stroom bij te grote kanteling
//    5. Stuur motoren aan
//    6. Schrijf telemetrie terug naar gShared (non-blocking)
//    7. Wacht op volgend 2ms-slot
// ═════════════════════════════════════════════════════════════════════════════
void fastTask(void *pvParameters) {
  motor1.begin();
  motor2.begin();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(2);  // 2 ms = 500 Hz

  const float DT = 0.002f;                // [s] tijdstap voor PID
  float targetTilt = 0.0f;               // lokale kopie van gShared.targetTilt
  uint8_t imuDiv = 0;                    // teller voor IMU-decimering (÷5)

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

    // ── 3) Balanceer-PID ─────────────────────────────────────────────────────
    // fout = huidige kanteling − gewenste kanteling
    // Positieve fout (voorover) → positief voltage → motoren rijden vooruit
    float error  = tilt - targetTilt;
    float voltage = tiltPID.update(error, DT);

    // ── 4) Veiligheidscheck ───────────────────────────────────────────────────
    bool safe = (fabsf(tilt) < TILT_LIMIT_DEG) && imu.isReady();
    if (!safe) {
      voltage = 0.0f;
      tiltPID.reset();  // voorkom integrator-windup na noodstop
    }

    // ── 5) Motoraansturing ────────────────────────────────────────────────────
    // Motor 2 is gespiegeld gemonteerd → omgekeerd teken
    // Pas het teken van motor2 aan als de robot de verkeerde kant op rijdt
    motor1.loop( voltage);
    motor2.loop(-voltage);

    // ── 6) Telemetrie → slow core (non-blocking) ──────────────────────────────
    if (xSemaphoreTake(xSharedMutex, 0) == pdTRUE) {
      gShared.currentTilt   = tilt;
      gShared.currentAngle1 = motor1.getAngle();
      gShared.currentAngle2 = motor2.getAngle();
      gShared.pidOutput     = voltage;
      xSemaphoreGive(xSharedMutex);
    }

    // ── 7) Wacht op volgend 2ms-slot ──────────────────────────────────────────
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  SLOW TASK — Core 1  (~50 Hz)
//
//  Verantwoordelijk voor:
//    - PID live-tuning via serieel (type: "p1.5 i0.0 d0.05")
//    - Debug-output
//    - TODO: BT / XBOX-controller → targetTilt aanpassen
// ═════════════════════════════════════════════════════════════════════════════
void slowTask(void *pvParameters) {
  while (true) {
    // ── Lees telemetrie van fast core ─────────────────────────────────────────
    float tilt, pidOut;
    if (xSemaphoreTake(xSharedMutex, portMAX_DELAY) == pdTRUE) {
      tilt   = gShared.currentTilt;
      pidOut = gShared.pidOutput;
      xSemaphoreGive(xSharedMutex);
    }

    // ── Seriële debug-output ──────────────────────────────────────────────────
    Serial.printf("[slow] tilt=%.2f°  pid=%.3fV\n", tilt, pidOut);

    // ── Seriële live-tuning van PID-gains ────────────────────────────────────
    // Stuur via serieel monitor: "p1.5"  "i0.1"  "d0.05"
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.length() >= 2) {
        char   key = cmd.charAt(0);
        float  val = cmd.substring(1).toFloat();
        if      (key == 'p') { tiltPID.Kp = val; Serial.printf("Kp = %.4f\n", val); }
        else if (key == 'i') { tiltPID.Ki = val; Serial.printf("Ki = %.4f\n", val); }
        else if (key == 'd') { tiltPID.Kd = val; Serial.printf("Kd = %.4f\n", val); }
        else if (key == 't') {
          // Overschrijf targetTilt handmatig: "t2.5"
          if (xSemaphoreTake(xSharedMutex, portMAX_DELAY) == pdTRUE) {
            gShared.targetTilt = val;
            xSemaphoreGive(xSharedMutex);
          }
          Serial.printf("targetTilt = %.2f°\n", val);
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
    // Doorgaan zonder IMU — fastTask controleert imu.isReady() en schakelt motoren uit
  } else {
    Serial.println("[OK] BNO055 gevonden.");
  }

  xSharedMutex = xSemaphoreCreateMutex();

  // Fast task: Core 0, hogere prioriteit zodat motorloop nooit blokkeert
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
