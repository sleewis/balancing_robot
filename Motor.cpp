
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

  // Encoder & positie
  rawAngle      = 0;
  lastMechAngle = 0.0f;
  prevMechAngle = 0.0f;
  offset        = 0.0f;

  // Snelheid
  _velocity = 0.0f;

  // Stroommetingen
  avgIa = avgIb = avgIc = avgI = peakI = 0.0f;

  // Shunt-versterker van de MKS-ESP32FOC V2.0
  shunt  = 0.01f;    // 10 mΩ shuntweerstand
  gain   = 50.0f;    // opamp-versterking
  adcMid = 1845.0f;  // ADC-waarde bij nulstroom (ijken indien nodig)

  // Foutstatus
  _encoderOk   = false;
  _overcurrent = false;
}


// ─────────────────────────────────────────────────────────────────────────────
// Initialisatie
// ─────────────────────────────────────────────────────────────────────────────
void Motor::begin() {
  // Enable-pin: HIGH = motordriver actief
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  // Koppel de drie PWM-kanalen aan de fasen
  // LEDC op de ESP32 genereert hardware-PWM zonder CPU-overhead
  ledcAttach(U, PWM_FREQ, PWM_RES);
  ledcAttach(V, PWM_FREQ, PWM_RES);
  ledcAttach(W, PWM_FREQ, PWM_RES);

  // Uitlijning wordt extern aangestuurd via alignStart() + delay + alignFinish()
  // zodat beide motoren parallel uitgelijnd kunnen worden.
}


// ─────────────────────────────────────────────────────────────────────────────
// Encoder uitlezen (AS5600 via I2C)
// ─────────────────────────────────────────────────────────────────────────────
bool Motor::readSensor() {
  // Stuur leescommando naar het hoekregister (0x0C = RAW ANGLE hoog-byte)
  wire->beginTransmission(AS5600_ADDR);
  wire->write(0x0C);
  if (wire->endTransmission(false) != 0) {
    // I2C-busfout: apparaat reageert niet
    return false;
  }

  // Vraag 2 bytes op (hoog- en laagbyte van het 12-bit hoekregister)
  if (wire->requestFrom(AS5600_ADDR, (uint8_t)2) != 2) {
    // Onverwacht aantal bytes ontvangen
    return false;
  }

  uint16_t high = wire->read();
  uint16_t low  = wire->read();

  // Combineer tot 12-bit waarde (0–4095 = 0–360°)
  rawAngle = ((high << 8) | low) & 0x0FFF;
  return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// Stroommetingen
// ─────────────────────────────────────────────────────────────────────────────
float Motor::readCurrent(int pin) {
  // Lees ADC en bereken spanning ten opzichte van het nulpunt
  int   raw     = analogRead(pin);
  float voltage = (raw - adcMid) * (3.3f / 4095.0f);

  // V = I × R_shunt × gain  →  I = V / (R_shunt × gain)
  return voltage / (shunt * gain);
}

void Motor::updateCurrent() {
  float Ia = readCurrent(IA_PIN);
  float Ib = readCurrent(IB_PIN);

  // Fase C wordt niet gemeten: volgt uit Kirchhoff (Ia + Ib + Ic = 0)
  float Ic = -Ia - Ib;

  // EMA-filter per fase — tijdconstante ≈ (1/0.1) × 2 ms = 20 ms
  // Gebruikt voor energiebewaking en langzame overstroom-detectie
  avgIa = 0.9f * avgIa + 0.1f * fabsf(Ia);
  avgIb = 0.9f * avgIb + 0.1f * fabsf(Ib);
  avgIc = 0.9f * avgIc + 0.1f * fabsf(Ic);
  avgI  = avgIa + avgIb + avgIc;

  // Directe piekstroom voor snelle overstroom-detectie (geen filtering)
  peakI = fabsf(Ia) + fabsf(Ib) + fabsf(Ic);
}


// ─────────────────────────────────────────────────────────────────────────────
// Noodstop: alle fases naar 50% duty cycle = nulspanning, motor vrij draaiend
// ─────────────────────────────────────────────────────────────────────────────
void Motor::brake() {
  int mid = PWM_MAX / 2;
  ledcWrite(U, mid);
  ledcWrite(V, mid);
  ledcWrite(W, mid);
}


// ─────────────────────────────────────────────────────────────────────────────
// Sinusvormige three-phase spanning genereren (open-loop FOC basis)
//
// Principe:
//   De drie fasen krijgen sinusvormige spanningen met 120° faseverschil.
//   De amplitude wordt bepaald door 'voltage', genormaliseerd via VOLTAGE_LIMIT.
//   Het PWM-bereik 0–PWM_MAX stelt de spanning in van 0 V tot voedingsspanning.
//   De sinus wordt verschoven naar het bereik [0, 1] vóór omzetting naar PWM.
//
//   PWM = (sin(hoek) × norm × 0.5 + 0.5) × PWM_MAX
//
// angle   : elektrische rotor-hoek [rad]
// voltage : gewenste spanning [V], bereik [-VOLTAGE_LIMIT, +VOLTAGE_LIMIT]
// ─────────────────────────────────────────────────────────────────────────────
void Motor::setPhaseVoltage(float angle, float voltage) {
  // Normaliseer spanning naar [-1, 1] op basis van de ingestelde spanning-limiet
  float norm = constrain(voltage / VOLTAGE_LIMIT, -1.0f, 1.0f);

  // Bereken sinusvormige spanning per fase (120° = 2π/3 rad faseverschil)
  float a = sinf(angle)               * norm;
  float b = sinf(angle - 2.0943951f)  * norm;  // fase B: -120°
  float c = sinf(angle - 4.1887902f)  * norm;  // fase C: -240°

  // Verschuif van [-1, 1] naar [0, 1] en schaal naar PWM-resolutie
  int pwmA = (int)((a * 0.5f + 0.5f) * PWM_MAX);
  int pwmB = (int)((b * 0.5f + 0.5f) * PWM_MAX);
  int pwmC = (int)((c * 0.5f + 0.5f) * PWM_MAX);

  // Begrens om clipping door afrondingsfouten te voorkomen
  pwmA = constrain(pwmA, 0, PWM_MAX);
  pwmB = constrain(pwmB, 0, PWM_MAX);
  pwmC = constrain(pwmC, 0, PWM_MAX);

  ledcWrite(U, pwmA);
  ledcWrite(V, pwmB);
  ledcWrite(W, pwmC);
}


// ─────────────────────────────────────────────────────────────────────────────
// Hoofd-regellus — aanroepen op vaste tijdstap (bijv. 500 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void Motor::loop(float voltage) {
  // ── Stap 1: encoder uitlezen ─────────────────────────────────────────────
  if (!readSensor()) {
    // Encoder niet bereikbaar: motor veilig stoppen en fout registreren
    _encoderOk = false;
    brake();
    return;
  }
  _encoderOk = true;

  // Converteer ruwe encoder-waarde (0–4095) naar mechanische hoek [rad]
  lastMechAngle = rawAngle * (TWO_PI / 4096.0f);

  // ── Stap 1b: hoeksnelheid berekenen ──────────────────────────────────────
  // Differenteer de mechanische hoek over de vaste tijdstap MOTOR_DT.
  // De hoek loopt van 0 tot 2π en wrapat terug naar 0, dus corrigeren we
  // voor de overgang (bijv. van 6.2 naar 0.1 rad is geen grote sprong).
  float delta = lastMechAngle - prevMechAngle;
  if      (delta >  PI) delta -= TWO_PI;   // wrap van 2π → 0
  else if (delta < -PI) delta += TWO_PI;   // wrap van 0 → 2π

  float rawVelocity = delta / MOTOR_DT;

  // Low-pass filter: dempt ruis zonder de respons te traag te maken
  _velocity = VELOCITY_ALPHA * _velocity + (1.0f - VELOCITY_ALPHA) * rawVelocity;

  prevMechAngle = lastMechAngle;

  // ── Stap 2: stroommetingen bijwerken & overstroom controleren ────────────
  updateCurrent();

  if (peakI > MAX_CURRENT_A) {
    // Directe piekstroom overschrijdt limiet: motor direct stoppen
    _overcurrent = true;
    brake();
    Serial.printf("[Motor] OVERSTROOM: %.2f A (limiet: %.1f A)\n",
                  peakI, (float)MAX_CURRENT_A);
    return;
  }
  _overcurrent = false;

  // ── Stap 3: elektrische hoek berekenen ───────────────────────────────────
  // De elektrische hoek loopt polePairs maal sneller dan de mechanische hoek.
  // + PI/2 corrigeert de faseverschuiving van de sinusvormige aansturing:
  // zonder deze correctie zou de motor koppel leveren in plaats van positie-
  // houdende spanning (de magnetische veldvector staat dan loodrecht op de rotor).
  float elec = lastMechAngle * polePairs + offset + HALF_PI;

  // ── Stap 4: fasespanningen instellen ─────────────────────────────────────
  setPhaseVoltage(elec, voltage);
}


// ─────────────────────────────────────────────────────────────────────────────
// Rotor-uitlijning — twee stappen voor parallelle uitvoering
//
// Gebruik vanuit de task:
//   motor1.alignStart();   motor2.alignStart();
//   delay(2000);           // één gedeelde wachttijd voor beide motoren
//   motor1.alignFinish();  motor2.alignFinish();
//
// De AS5600 meet absolute mechanische positie, maar de elektrische nulhoek
// (fase A maximaal) is afhankelijk van de montage. De offset wordt bepaald
// door de rotor naar elektrische hoek 0 te trekken en de mechanische hoek
// te meten na het settlen.
// ─────────────────────────────────────────────────────────────────────────────

void Motor::alignStart() {
  Serial.println("[Motor] Rotor-uitlijning starten...");
  // Trek de rotor naar elektrische hoek 0 — rotor begint nu te bewegen
  setPhaseVoltage(0.0f, ALIGN_VOLTAGE);
  // Wachttijd zit in de aanroepende task (gedeeld met de andere motor)
}

void Motor::alignFinish() {
  // Probeer de encoder uit te lezen (max. 10 pogingen)
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

  // Bereken de offset: verschil tussen de aangestuurde elektrische hoek (0)
  // en de gemeten mechanische positie × polePairs
  float mech = rawAngle * (TWO_PI / 4096.0f);
  offset = 0.0f - mech * polePairs;

  // Initialiseer prevMechAngle zodat de eerste snelheidsmeting nul oplevert
  lastMechAngle = mech;
  prevMechAngle = mech;
  _velocity     = 0.0f;

  _encoderOk = true;
  brake();  // spanning weg na uitlijning

  Serial.printf("[Motor] Uitlijning klaar. Offset: %.4f rad\n", offset);
}
