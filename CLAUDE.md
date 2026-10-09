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
  code and tooling (`avr-gcc`, `avrdude`, `DDRB`/`PORTB`, `TCCR0A`) does not apply. Use the
  "Arduino UNO R4 Boards" core (PlatformIO board `uno_r4_wifi`).
- **Drive motors are steppers, not DC motors.** There is no L298N and no PWM speed control; each
  motor is four GPIO pins sequenced through a ULN2003. With AccelStepper the 28BYJ-48 pin order is
  IN1, IN3, IN2, IN4, in `HALF4WIRE` mode.
- **The robot is slow.** The 28BYJ-48 tops out around 15 RPM (about 1000 half-steps/s at 5 V), so
  roughly 5 cm/s on 65 mm wheels. Turns take seconds, not milliseconds.
- **Never block the loop.** Steppers only move while the code is stepping them, so no `delay()`
  and keep the `pulseIn` timeout short (about 6000 µs). Use `millis()` state machines.
- **Power:** 2×18650 in series is about 7.4 V, but the steppers and servo are 5 V parts. Battery
  goes to VIN; steppers and servo run from 5 V. USB power alone is likely to brown out.
- The SG-90 is presumably for sweeping the HC-SR04 left and right to pick the clearer direction.

## Pin plan

| Part | Arduino pins |
|---|---|
| Left ULN2003 IN1–IN4 | 2, 3, 4, 5 |
| Right ULN2003 IN1–IN4 | 6, 7, 8, 9 |
| Servo signal | 10 |
| HC-SR04 TRIG / ECHO | 11 / 12 |

## Open questions

- Whether the chassis kit's mounts and wheels fit the 28BYJ-48 shafts, or whether it ships with DC
  gear motors that we have no driver for.
- Whether the board's 5 V rail copes with two steppers plus the servo when fed from VIN.

## Working notes

- Development is on macOS (Apple Silicon); the board appears as `/dev/cu.usbmodem*`.
- Nothing has been run on real hardware yet; treat sketches as untested until they have.

## upload final version [here](https://github.com/LUTESDeepTechHackathon2026/Submissions)