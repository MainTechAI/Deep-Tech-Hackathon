# Deep Tech Hackathon — Project 1

We are building **Project 1: Small Robot That Measures Distance to Obstacles** from
`Hackathon Projects - Component Breakdown.pdf` (the organisers' bill of materials; Projects 2 and 3
in that PDF are other teams' challenges).

Goal: an autonomous 2WD rover that continuously measures distance to obstacles and steers around
them.

## Hardware we get

| Component | Qty |
|---|---|
| Arduino Uno R4 WiFi | 1 |
| 2WD robot chassis kit | 1 |
| 28BYJ-48 5V stepper + ULN2003 driver | 2 sets |
| HC-SR04 ultrasonic sensor | 1 |
| SG-90 micro servo (180°) | 1 |
| 2×18650 holder in series (or laptop power, or AA) | 1 |
| Breadboard, Dupont jumper cables | 1–2, 1 pack |
| [Chassis](https://botland.store/chassis-for-robots/7283-chassis-rectangle-2wd-2-wheel-robot-chassis-with-dc-motor-drive-5904422335649.html) | 1 pack |

Specification

Voltage motor power supply: max. 6 V

Torque: 0.8 kg*cm (0.78 Nm)

Engine speed with no load: 90 +/- 10 rpm

Consumption of current of motor without load: 190 mA (max. 250 mA)

The current consumption of the engine with the stopped shaft 1 A

Wheels:

Diameter: 65 mm

Width: 30 mm

Plastic wheels with rubber tyres

Design size: 200 x 140 x 65 mm


A shared pool at the organiser bench has assorted resistors, LEDs, switches, sensor kits, spare
boards and charged 18650 cells.

## Constraints that shape the code

- **The board is an Uno R4 WiFi (Renesas RA4M1, ARM Cortex-M4), not an ATmega328P.** AVR-specific
  code and tooling (`avr-gcc`, `avrdude`, `DDRB`/`PORTB`, `TCCR0A`) does not apply. Build with
  `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi <sketch dir>`.
- **The assembled rover uses the chassis DC motors through an L298N**, not the 28BYJ-48 steppers
  from the PDF. Active firmware: `hardware/wayfinder_rover/`.
- **PWM on the R4 is only on D3, D5, D6, D9, D10, D11.** The L298N enable pins must be on those.
- **Motors keep running through blocking calls.** `WiFi.status()`, `WiFi.RSSI()` and HTTP all block,
  so the firmware makes no Wi-Fi calls while a move is running; keep it that way.
- **Space HC-SR04 pings at least 50 ms apart.** Faster, it hears late echoes of the previous ping
  and reports false short distances.
- No encoders or IMU: the pose is dead-reckoned from move timings, with forward travel corrected
  from the change in sonar reading when there is a steady echo ahead. Heading is uncorrected.
- Rover settings are tuned live from the console sliders (`/api/tuning`), not by re-uploading.
- The sonar is fixed facing forward; the rover turns its body to look around.

## Pin plan

| Part | Arduino pins |
|---|---|
| L298N left motor | ENA D5 (PWM), IN1 D9, IN2 D10 |
| L298N right motor | ENB D6 (PWM), IN3 D11, IN4 D12 |
| HC-SR04 TRIG / ECHO | D3 / D2 |

## Open questions

- `turn_rate` (degrees per second) has not been re-measured since the motor brake was added; the
  1 s spin test sets it.
- `SONAR_OFFSET_CM` (sonar face ahead of the turning centre) is an estimate.

## Working notes

- Development is on macOS (Apple Silicon); the board appears as `/dev/cu.usbmodem*`.
- Run the console with `python3 local_gateway_plain.py` (port 8000) and
  `python3 -m http.server 5173 --directory prototype`; the rover and this Mac must share the hotspot.
- `hardware/wayfinder_rover/secrets.h` is git-ignored (hotspot credentials, backend address, RSSI
  calibration). Without it the firmware builds with placeholder values and cannot join Wi-Fi.

## upload final version [here](https://github.com/LUTESDeepTechHackathon2026/Submissions)