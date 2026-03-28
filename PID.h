
#ifndef PID_H
#define PID_H

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// Generieke PID-regelaar met anti-windup en afgestelde afgeleide.
//
// Gebruik:
//   PID pid(Kp, Ki, Kd, uitMinimum, uitMaximum);
//   float stuurSignaal = pid.update(fout, dt);   // aanroepen op vaste dt
//
// Anti-windup: de integraal wordt bijgeknipt zodat de uitgang [outMin, outMax]
//              niet overschreden wordt.
// Derivative:  berekend over de fout; overweeg Kd klein te houden bij lawaai.
// ─────────────────────────────────────────────────────────────────────────────
class PID {
public:
  float Kp, Ki, Kd;

  PID(float kp, float ki, float kd, float outMin, float outMax);

  // Bereken nieuwe PID-uitgang
  // error : gewenste waarde − gemeten waarde  [zelfde eenheid als Kp-gain]
  // dt    : tijdstap [s]
  float update(float error, float dt);

  // Reset integrator en vorige fout (gebruik na noodstop / inschakelen)
  void reset();

  // Aanpassen van gains tijdens runtime (voor live-tuning via serieel)
  void setGains(float kp, float ki, float kd);

private:
  float _outMin, _outMax;
  float _integral;
  float _prevError;
  bool  _firstRun;
};

#endif // PID_H
