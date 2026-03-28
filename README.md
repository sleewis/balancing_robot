# Balancing Robot

## Overzicht

Dit project implementeert een zelfbalancerende robot op twee wielen met een ESP32. Het regelsysteem is opgesplitst in een snelle en een langzame task op aparte CPU-cores, zodat de motoraansturing deterministisch blijft terwijl debugging en live tuning toch mogelijk zijn.

Doelen van het project:

- Rechtop blijven staan met een tilt controller
- Twee BLDC-motoren aansturen met magnetische encoders
- Een IMU gebruiken voor kantelmeting
- Live PID-tuning mogelijk maken tijdens het testen

De architectuur is bewust modulair opgezet zodat sensoren, motoraansturing en regelalgoritmen onafhankelijk van elkaar verbeterd kunnen worden.

---

# Systeemarchitectuur

De ESP32 draait twee FreeRTOS-tasks:

## Fast Task (Core 0 – 500 Hz)
Verantwoordelijk voor real-time regeling.

Uitvoeringsvolgorde per cyclus:

1. Lees target tilt uit shared state
2. Update IMU (100 Hz, gedecimeerd vanuit de loop)
3. Voer balanceer-PID uit
4. Veiligheidscontrole (tilt-limiet en IMU-status)
5. Stuur motoren aan — motor.loop() handelt encoder- en overstroom-fouten intern af
6. Schrijf telemetrie terug naar shared state
7. Wacht op volgende control tick

Deze loop mag nooit blokkeren.

## Slow Task (Core 1 – ~50 Hz)
Behandelt niet-kritische taken:

- Seriële debugging en telemetrie-uitvoer
- Live PID-tuning via seriële monitor
- Controller-invoer (toekomstig)

Communicatie tussen de tasks verloopt via een mutex-beschermde shared structure.

---

# Hardware

## Controller

MKS-ESP32FOC V2.0 (ESP32-gebaseerd motorstuurboard)

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

Twee BLDC-motoren met three-phase drivers.

De motorfases worden aangestuurd via ESP32 LEDC PWM-uitgangen (20 kHz, 10-bit resolutie).

---

# Softwarecomponenten

## Motor

Verantwoordelijk voor:

- Uitlezen van de magnetische encoder (AS5600 via I2C)
- Genereren van sinusvormige three-phase spanning (open-loop FOC basis)
- Rotor-uitlijning bij opstarten
- Stroommetingen via shunts (EMA-filter + directe piekwaarde)
- Interne beveiliging tegen encoder-fouten en overstroom

Belangrijke constanten in `Motor.h`:

| Constante       | Standaard | Omschrijving                                      |
|-----------------|-----------|---------------------------------------------------|
| `VOLTAGE_LIMIT` | 10.0 V    | Maximale fasespanning (≈ helft voedingsspanning)  |
| `ALIGN_VOLTAGE` | 3.0 V     | Spanning tijdens rotor-uitlijning bij opstarten   |
| `MAX_CURRENT_A` | 6.0 A     | Totale fasestroom waarbij motor wordt uitgeschakeld |

De PID-uitgang (in Volt) wordt door `VOLTAGE_LIMIT` gedeeld om te normaliseren naar het PWM-bereik. Pas `VOLTAGE_LIMIT` aan als de motor niet genoeg koppel levert of juist te snel verzadigt.

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

| Veld           | Richting       | Omschrijving                        |
|----------------|----------------|-------------------------------------|
| `targetTilt`   | slow → fast    | Gewenste kanteling [°]              |
| `currentTilt`  | fast → slow    | Gemeten kanteling [°]               |
| `pidOutput`    | fast → slow    | Laatste PID-uitgang [V]             |
| `avgCurrent1`  | fast → slow    | Gefilterde stroom motor 1 [A]       |
| `avgCurrent2`  | fast → slow    | Gefilterde stroom motor 2 [A]       |
| `motor1Ok`     | fast → slow    | False bij encoder-fout of overstroom |
| `motor2Ok`     | fast → slow    | False bij encoder-fout of overstroom |

---

# Regelstrategie

De robot gebruikt een **cascade regelaar** met twee geneste PID-lussen:

```
Tilt PID → gewenste wielsnelheid [rad/s] → Velocity PID → motorspanning [V]
```

**Buitenste lus — Tilt PID (500 Hz):**

```
fout = gemeten kanteling [°] − gewenste kanteling [°]
uitgang = gewenste wielsnelheid [rad/s]
```

Bij een positieve kanteling (voorover) stuurt de tilt PID een positieve doelsnelheid. De wielen rijden vooruit en lopen zo de massa achterop — de robot trekt zichzelf rechtop.

**Binnenste lus — Velocity PID (500 Hz):**

```
fout = gewenste snelheid [rad/s] − gemeten snelheid [rad/s]
uitgang = motorspanning [V]
```

De gemeten snelheid is het gemiddelde van beide motoren, gecorrigeerd voor de gespiegelde montage van motor 2.

Het voordeel ten opzichte van directe spanningsregeling: de binnenste lus corrigeert automatisch voor slippen, belastingwisselingen en andere verstoringen. De buitenste lus hoeft alleen de kanteling bij te sturen.

---

# Veiligheid

Er zijn drie lagen van beveiliging:

**1. Tilt-limiet (fast task)**
De PID-uitgang wordt op nul gezet en de integrator gereset als de kanteling groter is dan `TILT_LIMIT_DEG` (standaard 30°) of als de IMU niet gereed is.

**2. Encoder-beveiliging (Motor class)**
Bij elke loop-cyclus wordt de I2C-transactie gecontroleerd. Reageert de AS5600 niet, dan wordt de motor via `brake()` gestopt en `_encoderOk = false` gezet.

**3. Overstroom-beveiliging (Motor class)**
De directe piekstroom (`peakI`) wordt elke cyclus berekend. Overschrijdt deze `MAX_CURRENT_A`, dan wordt de motor gestopt en een melding gestuurd via serieel.

De PID-integrator wordt gereset bij elke veiligheidsuitschakeling om wind-up te voorkomen.

---

# Seriële monitor

De slow task geeft elke 20 ms de volgende uitvoer:

```
[slow] tilt=  2.14°  pid= 3.210V  I1= 0.84A  I2= 0.81A  M1=OK  M2=OK
```

---

# PID Afstellen

PID-gains kunnen live worden aangepast via de seriële monitor zonder opnieuw te flashen.

**Tilt PID** (buitenste lus):

```
p2.0    → tilt Kp instellen
i0.0    → tilt Ki instellen
d0.05   → tilt Kd instellen
t3.0    → target tilt handmatig instellen [°]
```

**Velocity PID** (binnenste lus):

```
vp0.5   → velocity Kp instellen
vi0.2   → velocity Ki instellen
vd0.0   → velocity Kd instellen
```

Aanbevolen volgorde:

1. Stem eerst de **velocity PID** af: zet tilt PID gains op 0, geef `t0` en controleer of de wielen stilhouden bij verstoring. Stem `vp` en `vi` af.
2. Zet daarna `p` (tilt Kp) op een kleine waarde (bijv. 0.5) en verhoog tot de robot reageert op kanteling.
3. Voeg `d` (tilt Kd) toe om oscillaties te dempen.
4. Voeg `i` (tilt Ki) als laatste toe als de robot langzaam wegdrijft.

De seriële monitor toont elke 20 ms:

```
[slow] tilt=  2.14°  vel= 0.12→ 0.43 rad/s  out= 3.210V  I1= 0.84A  I2= 0.81A  M1=OK  M2=OK
```

---

# Toekomstige verbeteringen

## Motoraansturing

### 1. Field-Oriented Control (FOC) — gesloten stroom-regelaar

De motordriver genereert al sinusvormige fasespanningen op basis van de encoder-hoek — dit is de open-loop basis van FOC. De stroommetingen worden momenteel alleen gebruikt voor overstroom-beveiliging, niet voor regeling.

De volgende stap is een **gesloten stroom-regelaar**:

- Clarke transform (fasestroom → αβ)
- Park transform (αβ → dq, veld-uitgelijnd)
- PI stroomregelaars op de d- en q-as
- Inverse Park + Space Vector Modulation

De hardware meet al fasestroom via de shunts en de infrastructuur in de code is aanwezig — de waarden zijn beschikbaar via `motor.getAvgCurrent()`.

Voordelen: koppelregeling in plaats van spanningsregeling, hogere efficiëntie, soepelere respons bij lage snelheden.

## Sensorverbeteringen

De BNO055 levert gefuseerde Euler-hoeken die enige vertraging kunnen introduceren. Mogelijke verbetering: ruwe gyro + accelerometer fusie met een complementair filter of Kalman filter voor lagere latency.

---

# Geplande functies

- Bluetooth controller-invoer (XBOX)
- Snelheidsregeling
- Stuurregeling
- Telemetrie streaming
- Autonome stabilisatiemodi

---

# Projectstatus

Vroeg prototype.

Kernarchitectuur, dual-core regellus, motoraansturing met foutafhandeling en cascade regelaar (tilt PID → velocity PID) zijn geïmplementeerd. Momenteel gericht op het afstellen van de PID-gains voor stabiel balanceren.
