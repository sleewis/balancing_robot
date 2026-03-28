

#include "Motor.h"

Motor::Motor(int u, int v, int w, int en,
             int ia, int ib,
             TwoWire *wbus,
             int poles)
{
  U = u; V = v; W = w;
  EN = en;
  IA_PIN = ia;
  IB_PIN = ib;
  wire = wbus;
  polePairs = poles;

  offset = 0;
  lastMechAngle = 0;
  avgIa = avgIb = avgIc = avgI = 0;

  shunt = 0.01;
  gain = 50.0;
  adcMid = 1845.0;
}

void Motor::begin() {
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  ledcAttach(U, PWM_FREQ, PWM_RES);
  ledcAttach(V, PWM_FREQ, PWM_RES);
  ledcAttach(W, PWM_FREQ, PWM_RES);

  alignRotor();
}

bool Motor::readSensor() {
  wire->beginTransmission(AS5600_ADDR);
  wire->write(0x0C);
  wire->endTransmission(false);

  wire->requestFrom(AS5600_ADDR, 2);
  if (wire->available() == 2) {
    uint16_t high = wire->read();
    uint16_t low  = wire->read();
    rawAngle = ((high << 8) | low) & 0x0FFF;
    return true;
  }
  return false;
}

float Motor::getMechanicalAngle() {
  while (!readSensor());
  return rawAngle * (TWO_PI / 4096.0f);
}

float Motor::readCurrent(int pin) {
  int raw = analogRead(pin);
  float voltage = (raw - adcMid) * (3.3 / 4095.0);
  return voltage / (shunt * gain);
}

void Motor::updateCurrent() {
  float Ia = readCurrent(IA_PIN);
  float Ib = readCurrent(IB_PIN);
  float Ic = -Ia - Ib;

  avgIa = 0.9 * avgIa + 0.1 * abs(Ia);
  avgIb = 0.9 * avgIb + 0.1 * abs(Ib);
  avgIc = 0.9 * avgIc + 0.1 * abs(Ic);

  avgI = avgIa + avgIb + avgIc;
}

void Motor::setPhaseVoltage(float angle, float voltage) {
  float a = sinf(angle) * voltage;
  float b = sinf(angle - 2.0943951f) * voltage;
  float c = sinf(angle - 4.1887902f) * voltage;

  int pwmA = (a * 0.5f + 0.5f) * PWM_MAX;
  int pwmB = (b * 0.5f + 0.5f) * PWM_MAX;
  int pwmC = (c * 0.5f + 0.5f) * PWM_MAX;

  pwmA = constrain(pwmA, 0, PWM_MAX);
  pwmB = constrain(pwmB, 0, PWM_MAX);
  pwmC = constrain(pwmC, 0, PWM_MAX);

  ledcWrite(U, pwmA);
  ledcWrite(V, pwmB);
  ledcWrite(W, pwmC);
}

void Motor::loop(float voltage) {
  float mech = getMechanicalAngle();
  lastMechAngle = mech;                              // opslaan voor getAngle()

  float elec = mech * polePairs + offset + PI / 2.0;
  setPhaseVoltage(elec, voltage);
}

void Motor::alignRotor() {
  float alignAngle = 0;

  setPhaseVoltage(alignAngle, 0.3);
  delay(3000);

  float mech = getMechanicalAngle();
  offset = alignAngle - mech * polePairs;

  Serial.print("Offset: ");
  Serial.println(offset);
}
