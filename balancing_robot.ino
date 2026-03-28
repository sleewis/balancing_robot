
#include "Motor.h"
#include "SharedState.h"

// ─── I2C-bussen ───────────────────────────────────────────────────────────────
TwoWire I2C_1 = TwoWire(0);   // motor 1  (SDA=19, SCL=18)
TwoWire I2C_2 = TwoWire(1);   // motor 2  (SDA=23, SCL=5)

// ─── Motoren ──────────────────────────────────────────────────────────────────
Motor motor1(33, 32, 25, 12, 39, 36, &I2C_1, 11);
Motor motor2(26, 27, 14, 12, 34, 35, &I2C_2, 11);

// ─── Gedeelde toestand & mutex (extern gedeclareerd in SharedState.h) ─────────
SemaphoreHandle_t xSharedMutex;
SharedState gShared = {
  .targetVoltage1 = 0.0f,
  .targetVoltage2 = 0.0f,
  .currentAngle1  = 0.0f,
  .currentAngle2  = 0.0f,
};


// ═════════════════════════════════════════════════════════════════════════════
//  FAST TASK — Core 0  (500 Hz)
//  Verantwoordelijk voor: AS5600-uitlezing, FOC-commutatie, toekomstige PIDs
// ═════════════════════════════════════════════════════════════════════════════
void fastTask(void *pvParameters) {
  motor1.begin();
  motor2.begin();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(2);  // 2 ms → 500 Hz

  float v1 = 0.0f, v2 = 0.0f;

  while (true) {
    // 1) Lees targets van slow core (non-blocking — gebruik vorige waarde bij bezette mutex)
    if (xSemaphoreTake(xSharedMutex, 0) == pdTRUE) {
      v1 = gShared.targetVoltage1;
      v2 = gShared.targetVoltage2;
      xSemaphoreGive(xSharedMutex);
    }

    // 2) Stuur motoren aan
    //    TODO: vervang door PID-output zodra balanceer-PID is toegevoegd
    motor1.loop(v1);
    motor2.loop(v2);

    // 3) Schrijf telemetrie terug (zelfde mutex-slot, non-blocking)
    if (xSemaphoreTake(xSharedMutex, 0) == pdTRUE) {
      gShared.currentAngle1 = motor1.getAngle();
      gShared.currentAngle2 = motor2.getAngle();
      xSemaphoreGive(xSharedMutex);
    }

    // 4) Wacht tot volgende 2ms-slot (precisie-timing via FreeRTOS)
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  SLOW TASK — Core 1  (~50 Hz)
//  Verantwoordelijk voor: IMU, Bluetooth/XBOX, serial debug, outer PID-loop
// ═════════════════════════════════════════════════════════════════════════════
void slowTask(void *pvParameters) {
  while (true) {
    // TODO: lees BNO055 (tilt-hoek)
    // TODO: lees XBOX-controller via Bluetooth
    // TODO: bereken outer PID (gewenste tilt → voltage-target)

    // ── Stuur nieuwe targets naar fast core ──────────────────────────────────
    if (xSemaphoreTake(xSharedMutex, portMAX_DELAY) == pdTRUE) {
      gShared.targetVoltage1 =  1.0f;   // placeholder — vervangen door PID-output
      gShared.targetVoltage2 = -0.2f;
      xSemaphoreGive(xSharedMutex);
    }

    // ── Debug-output ─────────────────────────────────────────────────────────
    float a1, a2;
    if (xSemaphoreTake(xSharedMutex, portMAX_DELAY) == pdTRUE) {
      a1 = gShared.currentAngle1;
      a2 = gShared.currentAngle2;
      xSemaphoreGive(xSharedMutex);
    }
    Serial.printf("[slow] angle1=%.3f  angle2=%.3f\n", a1, a2);

    vTaskDelay(pdMS_TO_TICKS(20));  // 20 ms → 50 Hz
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // I2C initialiseren vóór de tasks starten
  I2C_1.begin(19, 18, 400000);
  I2C_2.begin(23,  5, 400000);

  // Mutex aanmaken
  xSharedMutex = xSemaphoreCreateMutex();

  // Tasks aanmaken en pinnen aan hun core
  //   prioriteit fast > slow zodat de motorloop nooit wordt geblokkeerd
  xTaskCreatePinnedToCore(fastTask, "FastTask", 4096, NULL, 2, NULL, 0);  // Core 0
  xTaskCreatePinnedToCore(slowTask, "SlowTask", 4096, NULL, 1, NULL, 1);  // Core 1
}


// ═════════════════════════════════════════════════════════════════════════════
//  LOOP — niet gebruikt; beide cores draaien via FreeRTOS-tasks
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  vTaskDelete(NULL);
}
