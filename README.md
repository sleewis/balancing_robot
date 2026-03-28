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
4. Veiligheidscontrole (tilt-limiet)
5. Update motorspanningen
6. Schrijf telemetrie terug
7. Wacht op volgende control tick

Deze loop mag nooit blokkeren.

## Slow Task (Core 1 – ~50 Hz)
Behandelt niet-kritische taken:

- Seriële debugging
- Live PID-tuning
- Controller-invoer (toekomstig)
- Telemetrie-uitvoer

Communicatie tussen de tasks verloopt via een mutex-beschermde shared structure.

---

# Hardware

## Controller

MKS-ESP32FOC V2.0 (ESP32-gebaseerd motorstuurboard)

## Sensoren

### IMU

BNO055

Gebruikt voor kantelmeting via Euler pitch.

Verbonden op I2C bus 0 (GPIO 19/18), gedeeld met de encoder van motor 1.
Adres BNO055: 0x28 — geen conflict met AS5600 (0x36).

### Motor Encoders

AS5600 magnetische encoders.

Elke motor heeft zijn eigen I2C-bus om adresconflicten te voorkomen.

## Motoren

Twee BLDC-motoren met three-phase drivers.

De motorfases worden aangestuurd via ESP32 LEDC PWM-uitgangen.

---

# Softwarecomponenten

## Motor

Verantwoordelijk voor:

- Uitlezen van de magnetische encoder
- Genereren van sinusvormige three-phase spanning
- Rotor-uitlijning bij opstarten

De motoraansturing is momenteel open-loop op basis van spanning.

## IMU

Minimale BNO055-driver geconfigureerd in **IMUPLUS-modus**.

Alleen de pitch wordt gebruikt voor het balanceren.

## PID Controller

Generieke PID-implementatie met:

- anti-windup
- aanpasbare gains tijdens runtime

## SharedState

Kleine structure voor gegevensuitwisseling tussen de fast en slow core.

---

# Regelstrategie

De robot gebruikt één enkele tilt controller:

PID( tilt_error ) → motorspanning

Waarbij:

```
error = gemeten kanteling - gewenste kanteling
```

Motor 2 is gespiegeld gemonteerd, waardoor het teken van de spanning omgekeerd is.

---

# Veiligheid

De motoruitgang wordt uitgeschakeld als:

- de IMU niet gereed is
- de kanteling de veiligheidsdrempel overschrijdt

De PID-integrator wordt gereset bij een veiligheidsuitschakeling om wind-up te voorkomen.

---

# PID Afstellen

PID-gains kunnen live worden aangepast via de seriële monitor:

```
p1.5
i0.1
d0.05
```

Aanbevolen volgorde voor het afstellen:

1. Begin met alleen **P**
2. Voeg **D** toe om oscillaties te dempen
3. Voeg **I** pas toe als de robot langzaam wegdrijft

---

# Toekomstige verbeteringen

## Motoraansturing

De huidige motorimplementatie is bewust eenvoudig gehouden en kan worden verbeterd.

### 1. Closed-loop snelheidsregeling

Momenteel worden de motoren direct aangestuurd via fasespanning.

Toekomstige verbetering: gebruik encoder-feedback om de **wielsnelheid** te regelen.

Dit levert een cascade-regelaar op:

```
Tilt PID → gewenste wielsnelheid
Snelheid PID → motorkoppel / spanning
```

Voordelen:

- soepelere beweging
- minder gevoelig voor belastingwisselingen
- betere balanceerstabiliteit

### 2. Field-Oriented Control (FOC) — stroom-regulatie

De motordriver genereert al sinusvormige fasespanningen op basis van de encoder-hoek.
Dit is de basis van FOC (open-loop).

Wat nog ontbreekt is de **inner current control loop**:

- Clarke transform (fasestroom → αβ)
- Park transform (αβ → dq, veld-uitgelijnd)
- PI stroomregelaars op de d- en q-as
- Inverse Park + Space Vector Modulation

De hardware meet al fasestroom via de shunts.

Voordelen na implementatie:

- koppelregeling in plaats van spanningsregeling
- hogere efficiëntie
- soepelere respons bij lage snelheden
- beveiliging tegen overstroom

### 3. Stroombegrenzing

Stroom wordt al gemeten maar nog niet gebruikt.

Toekomstig gebruik:

- motorbeveiliging
- koppelregeling
- foutdetectie

### 4. Encoder-foutafhandeling

De encoder-uitlezing wacht momenteel totdat data beschikbaar is.

Toekomstige verbetering:

- timeout-detectie
- motoruitschakeling bij sensorverlies

---

# Mogelijke sensorverbeteringen

De BNO055 levert gefuseerde Euler-hoeken die enige vertraging kunnen introduceren.

Mogelijke toekomstige verbetering:

- ruwe gyro + accelerometer fusie
- complementair filter of Kalman filter

Lagere vertraging verbetert de balanseerprestaties.

---

# Geplande functies

- Bluetooth controller-invoer
- snelheidsregeling
- stuurregeling
- telemetrie streaming
- autonome stabilisatiemodi

---

# Projectstatus

Vroeg prototype.

Kernarchitectuur en regelloop zijn geïmplementeerd.

Momenteel gericht op het bereiken van stabiel balanceren voordat functionaliteit wordt uitgebreid.
