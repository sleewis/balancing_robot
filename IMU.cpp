
#include "IMU.h"

IMU::IMU(TwoWire *bus, uint8_t addr)
  : _bno(55, addr, bus), _tilt(0.0f), _ready(false)
{}

bool IMU::begin() {
  _ready = _bno.begin();
  if (_ready) {
    // Gebruik externe kristal voor hogere nauwkeurigheid
    _bno.setExtCrystalUse(true);
  }
  return _ready;
}

void IMU::update() {
  if (!_ready) return;

  imu::Vector<3> euler = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // euler[0] = heading (Z), euler[1] = pitch/roll (Y), euler[2] = roll/pitch (X)
  // Kies de juiste as via TILT_AXIS en pas richting aan via TILT_SIGN
  float raw = 0.0f;
  if      (TILT_AXIS == 0) raw = euler.x();
  else if (TILT_AXIS == 1) raw = euler.y();
  else                      raw = euler.z();

  _tilt = TILT_SIGN * raw;
}
