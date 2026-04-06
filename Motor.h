
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
// De FOC-stroomregelaars geven een spanning-commando in Volt (Vd en Vq).
// VOLTAGE_LIMIT is de maximale fasespanning (≈ helft voedingsspanning).
// 4S LiPo: 13.2 V (leeg) – 16.8 V (vol) → maximale fasespanning ≈ Vbus/2 = 6.6–8.4 V.
// Conservatief instellen om clipping te voorkomen.
#define VOLTAGE_LIMIT   7.0f    // [V]

// ─── Uitlijnspanning bij opstarten ────────────────────────────────────────────
// Klein genoeg om de rotor zacht te verplaatsen, groot genoeg om aan te trekken.
#define ALIGN_VOLTAGE   3.0f    // [V]

// ─── Snelheidsberekening ──────────────────────────────────────────────────────
// Verwachte aanroepfrequentie van loop() — gebruikt voor hoeksnelheid [rad/s].
#define MOTOR_DT        0.002f   // [s]  1 / 500 Hz

// Low-pass filter coëfficiënt voor de snelheidsmeting (0 = geen filter, 1 = bevroren)
// Hogere waarde → soepeler maar trage respons; lagere waarde → sneller maar meer ruis.
#define VELOCITY_ALPHA  0.80f

// ─── Stroombeveiliging ────────────────────────────────────────────────────────
// Som van |Ia| + |Ib| + |Ic|. Stop motor bij overschrijding.
// GM4108H-120T: R_fase = 11.1 Ω — stel in op ≈ 3× nominale stroom als vangnet.
#define MAX_CURRENT_A   6.0f    // [A] aanpassen aan motor

// ─── FOC stroomregelaar (gesloten lus) ────────────────────────────────────────
//
// De motor wordt aangestuurd via Field-Oriented Control (FOC):
//
//   Buitenwereld (velocity PID)
//        │
//        ▼ iqTarget [A]  ← gewenste koppelstroom
//   ┌────────────────────────────────────────────┐
//   │  Park transform: Ia/Ib/Ic → Id, Iq        │
//   │  PI op Id (target = 0 A) → Vd             │
//   │  PI op Iq (target = iqTarget) → Vq        │
//   │  Inverse Park: Vd/Vq → Vα, Vβ            │
//   │  Inverse Clarke: Vα/Vβ → Va/Vb/Vc → PWM  │
//   └────────────────────────────────────────────┘
//
// Id-as (d = direct):  magnetiserende stroom — houd op 0 voor max koppel/amp.
// Iq-as (q = quadrature): koppelstroom — dit is het eigenlijke stuurcommando.
//
// Afsteltips:
//   - Begin met FOC_KP klein (bijv. 0.5) en controleer of de stroom stabiel volgt.
//   - Verhoog FOC_KI om de statische fout weg te werken (steady-state = 0 A fout).
//   - Als de stroom oscilleert: verlaag FOC_KP of FOC_KI.
//   - De motor heeft R = 11.1 Ω; voor 1 A steady-state moet Vq ≈ 11.1 V.
//     Omdat VOLTAGE_LIMIT = 7 V, is de max Iq in steady state ≈ 7/11.1 ≈ 0.63 A.
//
#define FOC_KP       0.5f   // [V/A]       proportionele gain stroomregelaar
#define FOC_KI      50.0f   // [V/(A·s)]   integrerende gain stroomregelaar
#define FOC_INT_MAX  6.0f   // [V]         maximale integratorbijdrage (anti-windup)

// ─────────────────────────────────────────────────────────────────────────────
class Motor {
public:
  Motor(int u, int v, int w, int en,
        int ia, int ib,
        TwoWire *wbus,
        int poles);

  // Initialiseer PWM-kanalen (zonder uitlijning)
  void begin();

  // Stap 1 van parallelle uitlijning: zet uitlijningsspanning aan (geen delay)
  void alignStart();

  // Stap 2 van parallelle uitlijning: lees encoder en bereken offset
  void alignFinish();

  // Hoofd-regellus — aanroepen op vaste tijdstap (bijv. 500 Hz)
  //
  // iqTarget: gewenste koppelstroom [A]
  //   positief = rijdt vooruit (afhankelijk van montagepolariteit)
  //   negatief = rijdt achteruit
  //   bereik: typisch ±1..2 A voor deze motor
  //
  // Intern: Clarke → Park → PI(Id,Iq) → inverse Park → inverse Clarke → PWM
  void loop(float iqTarget);

  // ── Status & telemetrie ──────────────────────────────────────────────────
  bool  isOk()            const { return _encoderOk && !_overcurrent; }
  bool  hasEncoderError() const { return !_encoderOk; }
  bool  isOvercurrent()   const { return _overcurrent; }

  // Laatste gemeten mechanische hoek [rad]
  float getAngle()        const { return lastMechAngle; }

  // Gefilterde mechanische hoeksnelheid [rad/s]
  float getVelocity()     const { return _velocity; }

  // Gefilterde totale fasestroom [A] (EMA)
  float getAvgCurrent()   const { return avgI; }

  // Directe piekstroom [A] (voor overstroom-detectie)
  float getPeakCurrent()  const { return peakI; }

  // Gemeten d-as stroom [A] — zou dicht bij 0 moeten blijven bij goede FOC
  float getId()           const { return _id; }

  // Gemeten q-as stroom [A] — dit is de werkelijke koppelstroom
  float getIq()           const { return _iq; }

private:
  // GPIO-pinnen
  int U, V, W;    // PWM-uitgangen (fasespanningen)
  int EN;         // motor-enable
  int IA_PIN;     // ADC stroomsensor fase A
  int IB_PIN;     // ADC stroomsensor fase B

  TwoWire *wire;
  int polePairs;

  // Encoder & positie
  uint16_t rawAngle;
  float    lastMechAngle;
  float    prevMechAngle;
  float    offset;        // elektrische offset [rad], bepaald bij uitlijning

  // Snelheid
  float    _velocity;

  // Stroommetingen (instantaan en gefilterd)
  float avgIa, avgIb, avgIc;  // EMA per fase [A]
  float avgI;                  // gefilterde totale stroom [A]
  float peakI;                 // directe piekstroom [A]

  // FOC toestandsvariabelen
  float _id;           // gemeten d-as stroom [A]  (magnetiserend, target = 0)
  float _iq;           // gemeten q-as stroom [A]  (koppel, target = iqTarget)
  float _id_integral;  // integratortoestand voor Id-PI regelaar
  float _iq_integral;  // integratortoestand voor Iq-PI regelaar

  // Shunt-versterker parameters (MKS-ESP32FOC V2.0)
  float shunt;    // [Ω]  shuntweerstand
  float gain;     // [-]  versterking van de opamp
  float adcMid;   // [ADC-ticks] nulpunt bij nulstroom

  // Foutstatus
  bool _encoderOk;
  bool _overcurrent;

  // ── Private methoden ─────────────────────────────────────────────────────
  bool  readSensor();
  float readCurrent(int pin);

  // Zet αβ-spanning om naar driefasige PWM-signalen
  // valpha, vbeta: statorspanning in het stilstaande referentiestelsel [V]
  void applyVoltageAlphaBeta(float valpha, float vbeta);

  // Noodstop: alle fases naar 50% duty = nulspanning, ook FOC-integratoren resetten
  void brake();
};

#endif // MOTOR_H
