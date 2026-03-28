
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ─────────────────────────────────────────────────────────────────────────────
// IMU — wrapper om de Adafruit BNO055.
//
// Montage-aanname: de BNO055 is zo gemonteerd dat de Y-as van de Euler-hoeken
// de kanteling (pitch, voor-/achterover) weergeeft.
// Pas TILT_AXIS en TILT_SIGN aan als de richting of as afwijkt.
//
// I2C-bus: gedeeld met AS5600 motor 0 op I2C_1 (GPIO19/18).
// Adressen: BNO055 = 0x28, AS5600 = 0x36 → geen conflict.
// ─────────────────────────────────────────────────────────────────────────────

#define TILT_AXIS  1        // 0=X, 1=Y, 2=Z Euler-vector
#define TILT_SIGN  1.0f     // 1.0 of -1.0 afhankelijk van montagerichting

class IMU {
public:
  IMU(TwoWire *bus, uint8_t addr = 0x28);

  // Initialiseer BNO055; geeft false als het apparaat niet gevonden wordt
  bool begin();

  // Lees nieuwe Euler-hoeken uit de BNO055 (I2C-transactie)
  // Aanroepen op max. 100 Hz (BNO055 NDOF fusion rate)
  void update();

  // Geeft de laatste kanteling terug [°]
  // Positief = voorover (richting bepaald door TILT_SIGN)
  float getTilt() const { return _tilt; }

  // Geeft aan of de BNO055 succesvol is geïnitialiseerd
  bool isReady() const { return _ready; }

private:
  Adafruit_BNO055 _bno;
  float _tilt;
  bool  _ready;
};

#endif // IMU_H
