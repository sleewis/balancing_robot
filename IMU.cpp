
#include "IMU.h"

IMU::IMU(TwoWire *bus, uint8_t addr)
  : _wire(bus), _addr(addr), _tilt(0.0f), _ready(false)
{}

// ─── Hulpfuncties ─────────────────────────────────────────────────────────────

bool IMU::writeReg(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(val);
  return (_wire->endTransmission() == 0);
}

bool IMU::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;

  uint8_t received = _wire->requestFrom(_addr, len);
  if (received != len) return false;

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = _wire->read();
  }
  return true;
}

// ─── Initialisatie ────────────────────────────────────────────────────────────

bool IMU::begin() {
  // Controleer chip-ID
  uint8_t chipId = 0;
  if (!readRegs(BNO055_CHIP_ID_REG, &chipId, 1) || chipId != 0xA0) {
    return false;
  }

  // Reset naar config-modus
  writeReg(BNO055_OPR_MODE, BNO055_MODE_CONFIG);
  delay(25);  // BNO055 heeft ~19 ms nodig voor moduswisseling

  // Normaal vermogen
  writeReg(BNO055_PWR_MODE, 0x00);
  delay(10);

  // Extern kristal inschakelen (hogere nauwkeurigheid)
  // Na het inschakelen heeft de BNO055 ~650 ms nodig om het kristal te stabiliseren.
  // Te kort wachten leidt tot foutieve sensor-fusie of een vastgelopen initialisatie.
  writeReg(BNO055_SYS_TRIGGER, 0x80);
  delay(650);

  // Zet in IMUPLUS-modus (accel + gyro, geen magnetometer)
  writeReg(BNO055_OPR_MODE, BNO055_MODE_IMUPLUS);
  delay(25);  // BNO055 heeft ~7 ms nodig voor moduswisseling naar fusion

  _ready = true;
  return true;
}

// ─── Datauitlezing ────────────────────────────────────────────────────────────

void IMU::update() {
  if (!_ready) return;

  // Lees 6 bytes Euler-hoeken vanaf register 0x1A:
  //   bytes[0..1] = heading (yaw),  LSB eerst
  //   bytes[2..3] = roll,           LSB eerst
  //   bytes[4..5] = pitch,          LSB eerst
  // Schaal: 1 LSB = 1/16 graad
  uint8_t buf[6];
  if (!readRegs(BNO055_EUL_DATA, buf, 6)) return;

  int16_t raw[3];
  raw[0] = (int16_t)((buf[1] << 8) | buf[0]);  // heading
  raw[1] = (int16_t)((buf[3] << 8) | buf[2]);  // roll
  raw[2] = (int16_t)((buf[5] << 8) | buf[4]);  // pitch

  _tilt = TILT_SIGN * (raw[TILT_AXIS] / BNO055_EULER_SCALE);
}
