
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Wire.h>

#define PWM_FREQ 20000
#define PWM_RES 10
#define PWM_MAX 1023
#define AS5600_ADDR 0x36

class Motor {
public:
  Motor(int u, int v, int w, int en,
        int ia, int ib,
        TwoWire *wbus,
        int poles);

  void begin();
  void loop(float voltage);

  // Geeft de laatste gemeten mechanische hoek terug [rad]
  // Veilig op te roepen vanuit fastTask na loop()
  float getAngle() const { return lastMechAngle; }

private:
  int U, V, W;
  int EN;
  int IA_PIN, IB_PIN;

  TwoWire *wire;
  int polePairs;

  float offset;
  float lastMechAngle;
  uint16_t rawAngle;

  float avgIa, avgIb, avgIc, avgI;

  float shunt;
  float gain;
  float adcMid;

  bool readSensor();
  float getMechanicalAngle();
  float readCurrent(int pin);
  void updateCurrent();
  void setPhaseVoltage(float angle, float voltage);
  void alignRotor();
};

#endif