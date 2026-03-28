
#include "PID.h"

PID::PID(float kp, float ki, float kd, float outMin, float outMax)
  : Kp(kp), Ki(ki), Kd(kd),
    _outMin(outMin), _outMax(outMax),
    _integral(0.0f), _prevError(0.0f), _firstRun(true)
{}

float PID::update(float error, float dt) {
  // Eerste aanroep: geen afgeleide berekenen (zou spike geven)
  if (_firstRun) {
    _prevError = error;
    _firstRun  = false;
  }

  // Proportioneel
  float P = Kp * error;

  // Integraal
  _integral += error * dt;

  // Anti-windup: knip de integraal bij zodat Ki*I de uitgang niet al
  // alleen al verzadigt
  if (Ki != 0.0f) {
    _integral = constrain(_integral, _outMin / Ki, _outMax / Ki);
  }
  float I = Ki * _integral;

  // Afgeleide (over de fout; kan oscilleren bij sensor-ruis — verlaag Kd dan)
  float D = Kd * (error - _prevError) / dt;
  _prevError = error;

  return constrain(P + I + D, _outMin, _outMax);
}

void PID::reset() {
  _integral  = 0.0f;
  _prevError = 0.0f;
  _firstRun  = true;
}

void PID::setGains(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}
