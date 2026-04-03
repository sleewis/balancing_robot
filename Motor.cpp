
#include "Motor.h"

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
Motor::Motor(int u, int v, int w, int en,
             int ia, int ib,
             TwoWire *wbus,
             int poles)
{
  U = u; V = v; W = w;
  EN = en;
  IA_PIN = ia;
  IB_PIN = ib;
  wire      = wbus;
  polePairs = poles;

  rawAngle      = 0;
  lastMechAngle = 0.0f;
  prevMechAngle = 0.0f;
  offset        = 0.0f;
  _velocity     = 0.0f;

  avgIa = avgIb = avgIc = avgI = peakI = 0.0f;

  // FOC toestandsvariabelen beginnen op nul
  _id          = 0.0f;
  _iq          = 0.0f;
  _id_integral = 0.0f;
  _iq_integral = 0.0f;

  // Shunt-versterker van de MKS-ESP32FOC V2.0
  shunt  = 0.01f;    // 10 mΩ shuntweerstand
  gain   = 50.0f;    // opamp-versterking
  adcMid = 1845.0f;  // ADC-waarde bij nulstroom (ijken indien nodig)

  _encoderOk   = false;
  _overcurrent = false;
}


// ─────────────────────────────────────────────────────────────────────────────
// Initialisatie
// ─────────────────────────────────────────────────────────────────────────────
void Motor::begin() {
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  ledcAttach(U, PWM_FREQ, PWM_RES);
  ledcAttach(V, PWM_FREQ, PWM_RES);
  ledcAttach(W, PWM_FREQ, PWM_RES);
}


// ─────────────────────────────────────────────────────────────────────────────
// Encoder uitlezen (AS5600 via I2C)
// ─────────────────────────────────────────────────────────────────────────────
bool Motor::readSensor() {
  wire->beginTransmission(AS5600_ADDR);
  wire->write(0x0C);
  if (wire->endTransmission(false) != 0) return false;
  if (wire->requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return false;

  uint16_t high = wire->read();
  uint16_t low  = wire->read();
  rawAngle = ((high << 8) | low) & 0x0FFF;
  return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// Stroomsensor uitlezen
// ─────────────────────────────────────────────────────────────────────────────
float Motor::readCurrent(int pin) {
  int   raw     = analogRead(pin);
  float voltage = (raw - adcMid) * (3.3f / 4095.0f);
  // V = I × R_shunt × gain  →  I = V / (R_shunt × gain)
  return voltage / (shunt * gain);
}


// ─────────────────────────────────────────────────────────────────────────────
// Noodstop
// Alle fases naar 50% duty cycle = nulspanning, motor vrij draaiend.
// FOC-integratoren worden ook gereset om wind-up bij herstart te voorkomen.
// ─────────────────────────────────────────────────────────────────────────────
void Motor::brake() {
  int mid = PWM_MAX / 2;
  ledcWrite(U, mid);
  ledcWrite(V, mid);
  ledcWrite(W, mid);

  // Reset FOC-integratoren zodat er bij herstart geen opgebouwde spanning is
  _id_integral = 0.0f;
  _iq_integral = 0.0f;
}


// ─────────────────────────────────────────────────────────────────────────────
// αβ-spanning naar driefasige PWM (inverse Clarke + PWM-mapping)
//
// Dit is de laatste stap van de FOC-keten:
//
//   (Vα, Vβ)  →  inverse Clarke  →  (Va, Vb, Vc)  →  PWM
//
// Inverse Clarke-formules (amplitude-invariant):
//   Va =  Vα
//   Vb = -Vα/2 + Vβ × √3/2
//   Vc = -Vα/2 - Vβ × √3/2
//
// De driefasspanningen liggen in het bereik [-VOLTAGE_LIMIT, +VOLTAGE_LIMIT].
// Ze worden verschoven naar [0, 1] en geschaald naar PWM-resolutie:
//   pwm = (V / VOLTAGE_LIMIT × 0.5 + 0.5) × PWM_MAX
// ─────────────────────────────────────────────────────────────────────────────
void Motor::applyVoltageAlphaBeta(float valpha, float vbeta) {
  // Inverse Clarke: van het tweeassige αβ-stelsel terug naar drie fasen
  float Va =  valpha;
  float Vb = -0.5f * valpha + 0.8660254f * vbeta;  // 0.866 = √3/2
  float Vc = -0.5f * valpha - 0.8660254f * vbeta;

  // Normaliseer en converteer naar PWM-waarden
  int pwmA = (int)constrain((Va / VOLTAGE_LIMIT * 0.5f + 0.5f) * PWM_MAX, 0, PWM_MAX);
  int pwmB = (int)constrain((Vb / VOLTAGE_LIMIT * 0.5f + 0.5f) * PWM_MAX, 0, PWM_MAX);
  int pwmC = (int)constrain((Vc / VOLTAGE_LIMIT * 0.5f + 0.5f) * PWM_MAX, 0, PWM_MAX);

  ledcWrite(U, pwmA);
  ledcWrite(V, pwmB);
  ledcWrite(W, pwmC);
}


// ─────────────────────────────────────────────────────────────────────────────
// Hoofd-regellus — Field-Oriented Control (gesloten lus)
//
// Wordt aangeroepen op 500 Hz. Voert de volledige FOC-keten uit:
//
//   1. Encoder uitlezen → mechanische hoek θ_mech [rad]
//   2. Hoeksnelheid berekenen [rad/s]
//   3. Fasestromen meten: Ia, Ib  (Ic = -Ia - Ib via Kirchhoff)
//   4. Clarke-transform: (Ia, Ib, Ic) → (Iα, Iβ)
//      Converteert van het driefasige stelsel naar een stilstaand tweeassig stelsel.
//   5. Park-transform: (Iα, Iβ, θ_elec) → (Id, Iq)
//      Converteert naar een roterend stelsel dat met de rotor meedraait.
//      Id = magnetiserende stroom (moet 0 zijn)
//      Iq = koppelstroom (dit is wat we regelen)
//   6. PI-regelaar op Id (target = 0 A) → Vd
//   7. PI-regelaar op Iq (target = iqTarget) → Vq
//   8. Inverse Park: (Vd, Vq, θ_elec) → (Vα, Vβ)
//   9. Inverse Clarke + PWM: (Vα, Vβ) → (Va, Vb, Vc) → ledcWrite
//
// iqTarget: gewenste koppelstroom [A], positief = vooruit
// ─────────────────────────────────────────────────────────────────────────────
void Motor::loop(float iqTarget) {

  // ── Stap 1: encoder uitlezen ─────────────────────────────────────────────
  if (!readSensor()) {
    _encoderOk = false;
    brake();
    return;
  }
  _encoderOk = true;

  // Converteer ruwe encoderwaarde (0–4095) naar mechanische hoek [0, 2π]
  lastMechAngle = rawAngle * (TWO_PI / 4096.0f);

  // ── Stap 2: hoeksnelheid berekenen ───────────────────────────────────────
  float delta = lastMechAngle - prevMechAngle;
  // Corrigeer voor de overgang van 2π → 0 (wrap-around van de encoder)
  if      (delta >  PI) delta -= TWO_PI;
  else if (delta < -PI) delta += TWO_PI;

  float rawVelocity = delta / MOTOR_DT;
  _velocity = VELOCITY_ALPHA * _velocity + (1.0f - VELOCITY_ALPHA) * rawVelocity;
  prevMechAngle = lastMechAngle;

  // ── Stap 3: fasestromen meten ────────────────────────────────────────────
  // Fase A en B worden direct gemeten via de shunt-versterkers.
  // Fase C volgt uit de wet van Kirchhoff: Ia + Ib + Ic = 0.
  float Ia = readCurrent(IA_PIN);
  float Ib = readCurrent(IB_PIN);
  float Ic = -Ia - Ib;

  // Update EMA-filters voor telemetrie en overstroom-detectie
  avgIa = 0.9f * avgIa + 0.1f * fabsf(Ia);
  avgIb = 0.9f * avgIb + 0.1f * fabsf(Ib);
  avgIc = 0.9f * avgIc + 0.1f * fabsf(Ic);
  avgI  = avgIa + avgIb + avgIc;
  peakI = fabsf(Ia) + fabsf(Ib) + fabsf(Ic);

  if (peakI > MAX_CURRENT_A) {
    _overcurrent = true;
    brake();
    Serial.printf("[Motor] OVERSTROOM: %.2f A (limiet: %.1f A)\n",
                  peakI, (float)MAX_CURRENT_A);
    return;
  }
  _overcurrent = false;

  // ── Stap 4: Clarke-transform ─────────────────────────────────────────────
  //
  // Converteert de driefasige stroom (Ia, Ib, Ic) naar een tweeassig
  // stilstaand stelsel (Iα, Iβ). Dit maakt de wiskundige verwerking eenvoudiger.
  //
  // Formules (amplitude-invariant):
  //   Iα = Ia
  //   Iβ = (Ia + 2·Ib) / √3
  //
  // Visueel: stel je voor dat Ia langs de α-as valt, dan geeft de β-as de
  // component loodrecht daarop. Het resultaat is een roterende vector in het
  // αβ-vlak die synchroon met de rotor draait.
  //
  float Ialpha = Ia;
  float Ibeta  = (Ia + 2.0f * Ib) * 0.5773503f;  // 1/√3 = 0.5774

  // ── Stap 5: Park-transform ───────────────────────────────────────────────
  //
  // Converteert de roterende αβ-vector naar een stilstaand (d,q)-stelsel
  // door te roteren met de elektrische hoek van de rotor.
  //
  // Na deze transformatie zijn Id en Iq constante gelijkstromen (DC) als
  // de motor in steady-state draait — dit maakt PID-regeling eenvoudig.
  //
  // Formules:
  //   Id =  Iα·cos(θ) + Iβ·sin(θ)
  //   Iq = -Iα·sin(θ) + Iβ·cos(θ)
  //
  // θ = elektrische hoek van de rotor-fluxas.
  // Let op: GEEN HALF_PI hier — die correctie was nodig voor open-loop
  // (om spanning 90° voor de rotor te zetten), maar bij FOC handelen de
  // PI-regelaars dit automatisch af via de Vq-uitgang.
  //
  float theta = lastMechAngle * polePairs + offset;
  float cosT  = cosf(theta);
  float sinT  = sinf(theta);

  _id =  Ialpha * cosT + Ibeta * sinT;
  _iq = -Ialpha * sinT + Ibeta * cosT;

  // ── Stap 6: PI-regelaar op Id (target = 0 A) ─────────────────────────────
  //
  // De d-as stroom (magnetiserende stroom) moet nul zijn voor maximaal
  // koppel per ampère. Een afwijking van nul betekent dat er onnodig energie
  // in het magnetisch veld wordt gestopt in plaats van in koppel.
  //
  // Anti-windup: de integraal wordt bijgeknipt zodat de bijdrage (Ki × integraal)
  // nooit groter wordt dan FOC_INT_MAX Volt.
  //
  float id_error    = 0.0f - _id;
  _id_integral     += id_error * MOTOR_DT;
  _id_integral      = constrain(_id_integral,
                                -FOC_INT_MAX / FOC_KI,
                                 FOC_INT_MAX / FOC_KI);
  float Vd = FOC_KP * id_error + FOC_KI * _id_integral;
  Vd = constrain(Vd, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

  // ── Stap 7: PI-regelaar op Iq (target = iqTarget) ───────────────────────
  //
  // De q-as stroom (koppelstroom) is het eigenlijke stuurcommando.
  // Door Iq te regelen sturen we direct het koppel van de motor,
  // onafhankelijk van de snelheid of belasting.
  //
  // Dit is het grote voordeel ten opzichte van open-loop spanning-aansturing:
  //   - Bij open-loop verandert het koppel met de snelheid (back-EMF neemt toe)
  //   - Bij FOC regelen we direct de stroom → koppel blijft constant voor
  //     dezelfde iqTarget, ook bij hogere snelheden
  //
  float iq_error    = iqTarget - _iq;
  _iq_integral     += iq_error * MOTOR_DT;
  _iq_integral      = constrain(_iq_integral,
                                -FOC_INT_MAX / FOC_KI,
                                 FOC_INT_MAX / FOC_KI);
  float Vq = FOC_KP * iq_error + FOC_KI * _iq_integral;
  Vq = constrain(Vq, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

  // ── Stap 8: inverse Park-transform ──────────────────────────────────────
  //
  // Converteert (Vd, Vq) terug van het roterende dq-stelsel naar het
  // stilstaande αβ-stelsel. Dit is de omgekeerde rotatie van stap 5.
  //
  // Formules:
  //   Vα = Vd·cos(θ) - Vq·sin(θ)
  //   Vβ = Vd·sin(θ) + Vq·cos(θ)
  //
  float Valpha = Vd * cosT - Vq * sinT;
  float Vbeta  = Vd * sinT + Vq * cosT;

  // ── Stap 9: inverse Clarke + PWM ─────────────────────────────────────────
  // (zie applyVoltageAlphaBeta hieronder voor de uitleg)
  applyVoltageAlphaBeta(Valpha, Vbeta);
}


// ─────────────────────────────────────────────────────────────────────────────
// Rotor-uitlijning — twee stappen voor parallelle uitvoering
// ─────────────────────────────────────────────────────────────────────────────
void Motor::alignStart() {
  Serial.println("[Motor] Rotor-uitlijning starten...");
  // Trek de rotor naar elektrische hoek 0 door direct de fasespanning in te
  // stellen via de sinusvormige methode (hier gebruiken we α-β direct).
  // Vα = ALIGN_VOLTAGE, Vβ = 0 → trekt rotor naar d-as positie 0.
  applyVoltageAlphaBeta(ALIGN_VOLTAGE, 0.0f);
}

void Motor::alignFinish() {
  bool ok = false;
  for (int i = 0; i < 10 && !ok; i++) {
    ok = readSensor();
    if (!ok) delay(5);
  }

  if (!ok) {
    Serial.println("[Motor] FOUT: encoder niet bereikbaar tijdens uitlijning!");
    _encoderOk = false;
    brake();
    return;
  }

  // Bereken de offset: de elektrische hoek die overeenkomt met de meetpositie
  // is 0 (we hebben de rotor daarheen getrokken), dus:
  // offset = 0 - mech * polePairs
  float mech = rawAngle * (TWO_PI / 4096.0f);
  offset = 0.0f - mech * polePairs;

  lastMechAngle = mech;
  prevMechAngle = mech;
  _velocity     = 0.0f;

  _encoderOk = true;
  brake();

  Serial.printf("[Motor] Uitlijning klaar. Offset: %.4f rad\n", offset);
}
