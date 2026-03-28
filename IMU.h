
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

// ─── BNO055 register-adressen ─────────────────────────────────────────────────
#define BNO055_CHIP_ID_REG  0x00   // moet 0xA0 teruggeven
#define BNO055_OPR_MODE     0x3D   // operation mode
#define BNO055_PWR_MODE     0x3E   // power mode
#define BNO055_SYS_TRIGGER  0x3F   // o.a. extern kristal inschakelen
#define BNO055_EUL_DATA     0x1A   // start Euler-registers (6 bytes: heading, roll, pitch)

// ─── Operation modes ──────────────────────────────────────────────────────────
#define BNO055_MODE_CONFIG   0x00
// IMUPLUS: accel + gyro fusion — geen magnetometer nodig, ideaal voor balanceerrobot
#define BNO055_MODE_IMUPLUS  0x08

// ─── Schaal ───────────────────────────────────────────────────────────────────
// BNO055 Euler-registers: 1 LSB = 1/16 graad (standaard degree-modus)
#define BNO055_EULER_SCALE   16.0f

// ─── Montage-instelling ───────────────────────────────────────────────────────
// Pas aan als de richting of as niet klopt na monteren.
// 0 = heading (yaw), 1 = roll, 2 = pitch
#define TILT_AXIS   2
#define TILT_SIGN   1.0f   // -1.0f als de richting omgekeerd is

// ─────────────────────────────────────────────────────────────────────────────
class IMU {
public:
  IMU(TwoWire *bus, uint8_t addr = 0x28);

  // Initialiseer BNO055 in IMUPLUS-modus; geeft false als chip niet reageert
  bool begin();

  // Lees nieuwe Euler-hoeken (I2C-transactie, ~0.3 ms @ 400 kHz)
  // Aanroepen op max. 100 Hz
  void update();

  // Laatste gemeten kanteling [°] (positief = voorover)
  float getTilt() const { return _tilt; }

  // True als begin() geslaagd is
  bool isReady() const { return _ready; }

private:
  TwoWire *_wire;
  uint8_t  _addr;
  float    _tilt;
  bool     _ready;

  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
};

#endif // IMU_H
