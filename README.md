# Balancing Robot

## Overzicht

Dit project implementeert een zelfbalancerende robot op twee wielen met een ESP32. Het regelsysteem is opgesplitst in een snelle en een langzame task op aparte CPU-cores, zodat de motoraansturing deterministisch blijft terwijl debugging en live tuning toch mogelijk zijn.

Doelen van het project:

- Rechtop blijven staan met een tilt controller
- Twee BLDC-motoren aansturen met FOC (Field-Oriented Control)
- Een IMU gebruiken voor kantelmeting
- Live PID-tuning mogelijk maken tijdens het testen
- Rijden op ±2 m/s via Bluetooth XBOX-controller (gepland)

De architectuur is bewust modulair opgezet zodat sensoren, motoraansturing en regelalgoritmen onafhankelijk van elkaar verbeterd kunnen worden.

---

# Systeemarchitectuur

De ESP32 draait twee FreeRTOS-tasks:

## Fast Task (Core 1 – 500 Hz)
Verantwoordelijk voor real-time regeling.

Uitvoeringsvolgorde per cyclus:

1. Lees target tilt uit shared state
2. Update IMU (100 Hz, gedecimeerd vanuit de loop)
3. Voer balanceer-PID uit
4. Veiligheidscontrole (tilt-limiet en IMU-status)
5. Stuur motoren aan — motor.loop() voert volledige FOC-keten uit
6. Schrijf telemetrie terug naar shared state
7. Wacht op volgende control tick

Deze loop mag nooit blokkeren.

## Slow Task (Core 0 – ~50 Hz)
Behandelt niet-kritische taken:

- Seriële debugging en telemetrie-uitvoer
- Live PID-tuning via seriële monitor
- Controller-invoer (toekomstig)

Communicatie tussen de tasks verloopt via een mutex-beschermde shared structure.

---

# Hardware

## Controller

MKS-ESP32FOC V2.0 (ESP32-gebaseerd motorstuurboard)

## Voeding

4S LiPo — 14.8V nominaal (13.2V–16.8V), 1300mAh

## Motoren

2× GM4108H-120T BLDC gimbal motor, 11 poolparen

## Wielen

Diameter 83mm — omtrek ≈ 261mm — theoretische topsnelheid bij VOLTAGE_LIMIT: ~3.6 m/s

## Vermogensmeting

Adafruit INA228 (I²C 0x40, 85V/20-bit) — batterijspanning en stroomverbruik bewaking. Nog niet in code geïmplementeerd.

## Sensoren

### IMU

BNO055, geconfigureerd in IMUPLUS-modus (accelerometer + gyroscoop, geen magnetometer).

Verbonden op I2C bus 0 (GPIO 19/18), gedeeld met de encoder van motor 1.
Adres BNO055: 0x28 — geen conflict met AS5600 (0x36).

### Motor Encoders

AS5600 magnetische encoders, één per motor.

Elke motor heeft zijn eigen I2C-bus om adresconflicten te voorkomen.

| Motor | I2C bus | SDA  | SCL  |
|-------|---------|------|------|
| 1     | Wire0   | GP19 | GP18 |
| 2     | Wire1   | GP23 | GP5  |

## Motoren

GM4108H-120T BLDC gimbal motoren (11 poolparen), two-phase drivers.

De motorfases worden aangestuurd via ESP32 LEDC PWM-uitgangen (20 kHz, 10-bit resolutie).

---

# Softwarecomponenten

## Motor

Verantwoordelijk voor:

- Uitlezen van de magnetische encoder (AS5600 via I2C)
- Volledige FOC-keten: Clarke → Park → PI-stroomregelaars → inverse Park → inverse Clarke → PWM
- Rotor-uitlijning bij opstarten
- Stroommetingen via shunts (EMA-filter + directe piekwaarde)
- Interne beveiliging tegen encoder-fouten en overstroom

De interface accepteert een **koppelstroom-commando** (`iqTarget` in Ampère). Alle spanning-berekeningen en PWM-generatie verlopen intern.

Belangrijke constanten in `Motor.h`:

| Constante       | Standaard  | Omschrijving                                           |
|-----------------|------------|--------------------------------------------------------|
| `VOLTAGE_LIMIT` | 7.0 V      | Maximale fasespanning (≈ helft voedingsspanning 4S LiPo) |
| `ALIGN_VOLTAGE` | 3.0 V      | Spanning tijdens rotor-uitlijning bij opstarten        |
| `MAX_CURRENT_A` | 6.0 A      | Totale fasestroom waarbij motor wordt uitgeschakeld    |
| `FOC_KP`        | 0.5 V/A    | Proportionele gain van de dq-stroomregelaars           |
| `FOC_KI`        | 50 V/(A·s) | Integrerende gain van de dq-stroomregelaars            |
| `FOC_INT_MAX`   | 6.0 V      | Anti-windup limiet voor de FOC-integratoren            |

### FOC afstellen

- Begin met kleine `FOC_KP` (bijv. 0.3) en verhoog tot de stroom snel en stabiel het setpoint volgt.
- Verhoog `FOC_KI` om de statische fout naar nul te brengen.
- Als de stroom oscilleert: verlaag `FOC_KP` of `FOC_KI`.
- Controleer via de seriële monitor of `Id ≈ 0` — een afwijkende Id wijst op een verkeerde encoderoffset.

### Foutafhandeling

`motor.loop()` stopt de motor automatisch en zet een foutvlag als:

- De AS5600 encoder niet reageert binnen de I2C time-out
- De directe piekstroom `MAX_CURRENT_A` overschrijdt

De status is op te vragen via `motor.isOk()`, `motor.hasEncoderError()` en `motor.isOvercurrent()`.

## IMU

Minimale BNO055-driver zonder externe libraries — rechtstreeks via I2C-registers aangesproken.

Alleen de pitch-as wordt gebruikt voor het balanceren. Montage-afhankelijke instellingen (`TILT_AXIS`, `TILT_SIGN`) staan bovenaan `IMU.h`.

## PID Controller

Generieke PID-implementatie met:

- Anti-windup: integrator wordt bijgeknipt zodat de uitgang de limiet niet overschrijdt
- Aanpasbare gains tijdens runtime (via seriële monitor of toekomstige Bluetooth-interface)
- `reset()` methode voor gebruik bij veiligheidsuitschakeling

## SharedState

Centrale structure voor gegevensuitwisseling tussen de twee cores via een mutex.

| Veld           | Richting       | Omschrijving                              |
|----------------|----------------|-------------------------------------------|
| `targetTilt`   | slow → fast    | Gewenste kanteling [°]                    |
| `currentTilt`  | fast → slow    | Gemeten kanteling [°]                     |
| `pidOutput`    | fast → slow    | Laatste velocity PID-uitgang [A] (iqTarget) |
| `avgCurrent1`  | fast → slow    | Gefilterde stroom motor 1 [A]             |
| `avgCurrent2`  | fast → slow    | Gefilterde stroom motor 2 [A]             |
| `motor1Ok`     | fast → slow    | False bij encoder-fout of overstroom      |
| `motor2Ok`     | fast → slow    | False bij encoder-fout of overstroom      |

---

# Regelstrategie

De robot gebruikt een **cascade regelaar** met drie geneste lussen:

```
Tilt PID → gewenste wielsnelheid [rad/s] → Velocity PID → iqTarget [A] → FOC → PWM
```

**Buitenste lus — Tilt PID (500 Hz):**

```
fout    = gemeten kanteling [°] − gewenste kanteling [°]
uitgang = gewenste wielsnelheid [rad/s]  (begrensd op ±50 rad/s ≈ ±2 m/s)
```

Bij een positieve kanteling (voorover) stuurt de tilt PID een positieve doelsnelheid. De wielen rijden vooruit en lopen zo de massa achterop — de robot trekt zichzelf rechtop.

**Middelste lus — Velocity PID (500 Hz):**

```
fout    = gewenste snelheid [rad/s] − gemeten snelheid [rad/s]
uitgang = gewenste koppelstroom iqTarget [A]
```

De gemeten snelheid is het gemiddelde van beide motoren, gecorrigeerd voor de gespiegelde montage van motor 2.

**Binnenste lus — FOC stroomregelaar (500 Hz, intern in Motor.loop):**

```
Ia, Ib meten  →  Ic = −Ia − Ib  (Kirchhoff)
Clarke-transform:  (Ia, Ib, Ic) → (Iα, Iβ)   stilstaand tweeassig stelsel
Park-transform:    (Iα, Iβ, θ)  → (Id, Iq)   roterend stelsel (met rotor mee)
PI op Id (target = 0 A)   → Vd              onderdrukt magnetiseerstroom
PI op Iq (target = iqTarget) → Vq           regelt koppel direct
Inverse Park:  (Vd, Vq) → (Vα, Vβ)
Inverse Clarke: (Vα, Vβ) → (Va, Vb, Vc) → PWM
```

**Voordeel van FOC ten opzichte van open-loop spanning:**
Bij hogere snelheden bouwt de motor back-EMF op, waardoor een vaste spanning minder koppel levert. FOC compenseert dit automatisch: voor dezelfde `iqTarget` blijft het geleverde koppel constant, ongeacht de snelheid.

---

# Veiligheid

Er zijn drie lagen van beveiliging:

**1. Tilt-limiet (fast task)**
De PID-uitgang wordt op nul gezet en de integratoren gereset als de kanteling groter is dan `TILT_LIMIT_DEG` (standaard 30°) of als de IMU niet gereed is.

**2. Encoder-beveiliging (Motor class)**
Bij elke loop-cyclus wordt de I2C-transactie gecontroleerd. Reageert de AS5600 niet, dan wordt de motor via `brake()` gestopt en `_encoderOk = false` gezet.

**3. Overstroom-beveiliging (Motor class)**
De directe piekstroom (`peakI`) wordt elke cyclus berekend. Overschrijdt deze `MAX_CURRENT_A`, dan wordt de motor gestopt en een melding gestuurd via serieel.

Bij elke noodstop worden zowel de PID-integratoren als de FOC-integratoren gereset om wind-up te voorkomen.

---

# Seriële monitor

De slow task geeft elke 20 ms de volgende uitvoer:

```
[slow] tilt=  2.14°  vel= 0.12→ 0.43 rad/s  iq= 0.312A  I1= 0.84A  I2= 0.81A  M1=OK  M2=OK
```

| Veld   | Eenheid | Omschrijving                                      |
|--------|---------|---------------------------------------------------|
| tilt   | °       | Gemeten kanteling                                 |
| vel    | rad/s   | Gemeten → gewenste wielsnelheid                   |
| iq     | A       | Koppelstroom-commando (uitgang velocity PID)      |
| I1/I2  | A       | Gefilterde totale fasestroom per motor            |
| M1/M2  |         | Motorstatus (OK / FOUT)                           |

---

# PID Afstellen

PID-gains kunnen live worden aangepast via de seriële monitor zonder opnieuw te flashen.

**Tilt PID** (buitenste lus):

```
p5.0    → tilt Kp instellen
i0.0    → tilt Ki instellen
d0.05   → tilt Kd instellen
t3.0    → target tilt handmatig instellen [°]
```

**Velocity PID** (middelste lus — uitgang in Ampère):

```
vp0.1   → velocity Kp instellen  [A/(rad/s)]
vi0.05  → velocity Ki instellen  [A/rad]
vd0.0   → velocity Kd instellen
```

Aanbevolen volgorde:

1. Stem eerst de **FOC-gains** af (`FOC_KP`, `FOC_KI` in Motor.h): controleer via serieel of `Id ≈ 0` en `Iq` het setpoint snel volgt.
2. Stem daarna de **velocity PID** af: zet tilt PID gains op 0, geef `t0` en controleer of de wielen stilhouden bij verstoring. Stem `vp` en `vi` af.
3. Zet daarna `p` (tilt Kp) op een kleine waarde (bijv. 0.5) en verhoog tot de robot reageert op kanteling.
4. Voeg `d` (tilt Kd) toe om oscillaties te dempen.
5. Voeg `i` (tilt Ki) als laatste toe als de robot langzaam wegdrijft.

---

# Toekomstige verbeteringen

## Sensorverbeteringen

De BNO055 levert gefuseerde Euler-hoeken die enige vertraging kunnen introduceren. Mogelijke verbetering: ruwe gyro + accelerometer fusie met een complementair filter of Kalman filter voor lagere latency.

## Vermogensmeting

De Adafruit INA228 is aangesloten maar nog niet in de code opgenomen. Toekomstig gebruik: batterijspanning bewaken en VOLTAGE_LIMIT dynamisch aanpassen aan de laadtoestand.

---

# Geplande functies

- Bluetooth controller-invoer (XBOX)
- Stuurregeling
- Telemetrie streaming
- Autonome stabilisatiemodi

---

# Projectstatus

Actief in ontwikkeling.

Geïmplementeerd:
- Dual-core FreeRTOS architectuur (500 Hz fast task, 50 Hz slow task)
- Cascade regelaar: tilt PID → velocity PID → FOC stroomregelaar
- Gesloten-lus FOC met Clarke/Park-transforms en dq-PI-regelaars
- Encoder-foutafhandeling en overstroom-beveiliging
- Live PID-tuning via seriële monitor

Huidige focus: afstellen van FOC-gains en PID-cascade voor stabiel balanceren.
