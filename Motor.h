
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Wire.h>

// ─── PWM-instellingen ─────────────────────────────────────────────────────────
#define PWM_FREQ    20000   // [Hz]  dragerfrequentie boven menselijk gehoor
#define PWM_RES     10      // [bit] resolutie: 0–1023
#define PWM_MAX     1023

// ─── AS5600 encoder ───────────────────────────────────────────────────────────
#define AS5600_ADDR         0x36
#define ENCODER_TIMEOUT_MS  5     // [ms] max wachttijd per I2C-transactie

// ─── Spanningsnormalisatie ────────────────────────────────────────────────────
// De PID geeft een voltage-commando in Volt.
// VOLTAGE_LIMIT is de maximale fasespanning (≈ helft voedingsspanning).
// Bij 20 V voeding: VOLTAGE_LIMIT = 10 V.
// Aanpassen als de motor niet genoeg koppel levert of te snel verzadigt.
#define VOLTAGE_LIMIT   10.0f   // [V]

// ─── Uitlijnspanning bij opstarten ────────────────────────────────────────────
// Klein genoeg om de rotor zacht te verplaatsen, groot genoeg om aan te trekken.
#define ALIGN_VOLTAGE   3.0f    // [V]

// ─── Stroombeveiliging ────────────────────────────────────────────────────────
// Totale som van |Ia| + |Ib| + |Ic|.
// Controleer het datasheet van jouw motor voor de maximale fasestroom.
// GM4108H-120T: R_fase = 11.1 Ω, V_voeding ≈ 20 V → I_stall ≈ 1.8 A/fase
// Stel MAX_CURRENT_A in op ≈ 3× I_nominaal als vangnet.
#define MAX_CURRENT_A   6.0f    // [A] aanpassen aan motor

// ─────────────────────────────────────────────────────────────────────────────
class Motor {
public:
  Motor(int u, int v, int w, int en,
        int ia, int ib,
        TwoWire *wbus,
        int poles);

  // Initialiseer PWM-kanalen en voer rotor-uitlijning uit
  void begin();

  // Hoofd-regellus — aanroepen op vaste tijdstap (bijv. 500 Hz)
  // voltage: gewenste spanning [V], bereik [-VOLTAGE_LIMIT, +VOLTAGE_LIMIT]
  // Bij encoder-fout of overstroom wordt de motor veilig gestopt.
  void loop(float voltage);

  // ── Status & telemetrie ──────────────────────────────────────────────────
  // True als er geen encoder-fout en geen overstroom is
  bool  isOk()           const { return _encoderOk && !_overcurrent; }
  bool  hasEncoderError() const { return !_encoderOk; }
  bool  isOvercurrent()   const { return _overcurrent; }

  // Laatste gemeten mechanische hoek [rad]
  float getAngle()       const { return lastMechAngle; }

  // Gefilterde totale fasestroom [A] (EMA, tijdconstante ≈ 20 ms @ 500 Hz)
  float getAvgCurrent()  const { return avgI; }

  // Directe piekstroom [A] voor dit moment (gebruikt voor overstroom-detectie)
  float getPeakCurrent() const { return peakI; }

private:
  // GPIO-pinnen
  int U, V, W;    // PWM-uitgangen (fasespanningen)
  int EN;         // motor-enable
  int IA_PIN;     // ADC stroomsensor fase A
  int IB_PIN;     // ADC stroomsensor fase B

  TwoWire *wire;
  int polePairs;

  // Encoder & positie
  uint16_t rawAngle;       // ruwe AS5600-waarde (0–4095)
  float    lastMechAngle;  // laatste mechanische hoek [rad]
  float    offset;         // elektrische offset bepaald tijdens uitlijning [rad]

  // Stroommetingen
  float avgIa, avgIb, avgIc;  // EMA per fase [A]
  float avgI;                 // gefilterde totale stroom [A]
  float peakI;                // directe piekstroom [A]

  // Shunt-versterker parameters (MKS-ESP32FOC V2.0)
  // V_uit = (I × shunt × gain) + V_mid
  float shunt;    // [Ω]  shuntweerstand
  float gain;     // [-]  versterking van de opamp
  float adcMid;   // [ADC-ticks] nulpunt bij nulstroom (≈ 1845 @ 12-bit, 3.3 V)

  // Foutstatus
  bool _encoderOk;
  bool _overcurrent;

  // ── Private methoden ─────────────────────────────────────────────────────
  // Lees AS5600 via I2C; geeft false bij time-out of busfout
  bool readSensor();

  // Lees fasestroom op 'pin' [A]; negatief = terugstroom
  float readCurrent(int pin);

  // Meet alle fasestromen en update EMA-filters en peakI
  void updateCurrent();

  // Genereer sinusvormige three-phase spanning
  // angle  : elektrische hoek [rad]
  // voltage: amplitude [V], wordt genormaliseerd via VOLTAGE_LIMIT
  void setPhaseVoltage(float angle, float voltage);

  // Zet alle fases op 50% duty cycle (= nulspanning, vrij draaien)
  void brake();

  // Bepaal elektrische offset bij opstarten
  void alignRotor();
};

#endif // MOTOR_H
