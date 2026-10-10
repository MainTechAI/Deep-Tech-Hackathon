# Wayfinder — hardware test TODO

## Measured on the assembled rover

- Arduino UNO R4 WiFi connects to the phone hotspot and sends telemetry to the
  laptop backend.
- Laptop hotspot address: `172.20.10.2`; rover address observed: `172.20.10.3`.
- Fixed front HC-SR04 works. The current wiring in the active firmware is
  `TRIG D3`, `ECHO D2`.
- Wi-Fi RSSI calibration at 1 m: **-53.7 dBm**.
- Observed signal around 3 m: approximately **-60 dBm**.
- Observed signal close to the phone: approximately **-43 dBm**.
- The frontend command reaches the backend and the rover receives telemetry.
- A short backend `forward` test command was accepted while the front sonar
  reported clear space. The physical motor result was not confirmed and must
  be repeated with wheels raised.
- The latest explicit `Stop` command was confirmed by rover telemetry before
  this TODO was written.

## Status after the 10 October evening session

The spinning described below is resolved on the bench and floor. The causes
found were a broken D10 -> IN2 jumper (left wheel had no forward drive), enable
pins without PWM, timed moves overrunning during HTTP calls, and the firmware
not matching the gateway's JSON, so console commands were ignored.

Measured or observed on the floor: 30 cm in a 1 s forward run at duty 110; a
130 ms turn is about 20 degrees; left wheel at 98 % runs nearly straight.

Still open: the hunt can arc because it always turns right when the signal
drops; the wheel trim was tuned at the faster speed; the map is dead-reckoned
and drifts; the live tuning sliders have not been tried on the rover.

## Earlier blocker (kept for the record)

During an autonomous hunt the rover can spin very quickly, keep rotating, and
hit objects. This is not acceptable behaviour. The rover must stay stopped
until the motor direction and safe motion timings are validated.

The likely causes are:

1. One motor direction does not match the assumed `INVERT_LEFT` /
   `INVERT_RIGHT` configuration, so a commanded forward move becomes a spin.
2. The L298N enable pins were on D8/D13, which have no PWM on the UNO R4, so
   the motors ran at full power. **Changed, untested on the rover:** ENA is now
   D5 and ENB is D6, and the firmware drives them with `DRIVE_DUTY` /
   `TURN_DUTY`. The duties and the step and turn timings are unmeasured
   starting values.
3. Timed moves overran: the motors kept running while the loop was blocked in
   HTTP calls (up to about a second), so a 260 ms turn could last far longer.
   **Changed, untested on the rover:** the backend is now only contacted while
   the rover is stopped.
4. A single fixed forward sonar cannot tell which side is clear after an
   obstacle. Autonomous avoidance must use very small, validated turns.

## Safe recovery plan

1. Keep autonomous hunt disabled during the motor-direction test.
2. Reconnect USB and upload a safety-tuning firmware with short manual pulses:
   forward, backward, left, right, and stop.
3. Test with wheels raised first. Confirm that **forward drives both wheels in
   the physical forward direction**, not a spin. Change only the incorrect
   `INVERT_LEFT` or `INVERT_RIGHT` flag.
4. Test on the floor with one very short forward step. Reduce step, reverse,
   and turn durations until the rover cannot build dangerous speed.
5. Increase the minimum front clearance before autonomous movement and verify
   that `Stop` halts the rover immediately.
6. Re-enable autonomous Wi-Fi hunt only after the four manual directions and
   obstacle stop work correctly.

## Behaviour that is already intentional

- The rover stops as `TARGET_NEARBY` when the RSSI is stronger than its
  calibrated near-target threshold. It must be moved several metres away from
  the phone before a hunt can begin.
- Wi-Fi RSSI estimates direction only through changes after short moves. The
  displayed distance is rough and is not a true position measurement.
- The front sonar is a collision guard; it does not create a room map or know
  whether left or right is clear.

## Files involved

- `wayfinder_rover.ino` — active firmware.
- `secrets.h` — local ignored hotspot and calibration settings.
- `diagnostics/diagnostics.ino` — safe, no-motor Wi-Fi and sonar probe.
- `../movement_reference/movement_reference.ino` — original L298N movement
  reference.
- `../motor_bench/motor_bench.ino` — tethered serial test: one timed move per
  line at a chosen PWM duty, for checking direction and tuning duty and timing.

## Do not publish

`secrets.h` contains the hotspot credentials, laptop address and local Wi-Fi
calibration. It is intentionally ignored by Git and must never be committed or
uploaded.
