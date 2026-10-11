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

## Firmware rework (written after the floor session, not yet run on the rover)

- No Wi-Fi calls while a move runs; moves end with a motor brake.
- Sonar pings spaced 50 ms apart (false short readings came from firing them
  8 ms apart), and no pings during turns.
- Forward travel measured from the change in sonar reading when possible.
- Hunt decides from the mean of RSSI samples taken at a standstill, and turns
  by `hunt_turn_deg` (default 45) instead of 20 degrees.
- One HTTP round trip per report; hunt stops if the gateway is unreachable.
- Gateway: rover shown offline after 5 s of silence, unknown text no longer
  starts a hunt, stale map points removed.

To check on the floor, in this order:

1. Manual pad: each direction, Stop, and that forward is refused near a wall.
2. **1 s spin**: estimate the degrees turned and set the `turn_rate` slider to
   it. The brake changed how far a turn coasts, so the old value is suspect.
3. **1 s run** facing a wall about 1.5 m away: distance should still be near
   30 cm, and `odo_fixes` in the console should count up on forward steps.
4. Drive a square with the pad and see whether the trail closes on the map.
5. Hunt with the phone 3 m away.

Still open: heading is uncorrected, so the map drifts with every turn; the
wheel trim was tuned at the faster speed; `SONAR_OFFSET_CM` is a guess.

## Earlier blocker (kept for the record)

During an autonomous hunt the rover can spin very quickly, keep rotating, and
hit objects. This is not acceptable behaviour. The rover must stay stopped
until the motor direction and safe motion timings are validated.

The likely causes are:

1. One motor direction does not match the assumed `INVERT_LEFT` /
   `INVERT_RIGHT` configuration, so a commanded forward move becomes a spin.
2. The L298N enable pins were on D8/D13, which have no PWM on the UNO R4, so
   the motors ran at full power. **Changed, untested on the rover:** ENA is now
   D5 and ENB is D6, and the firmware drives them with `driveDuty` /
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

- The rover stops as `TARGET_NEARBY` when the signal reaches the `target_rssi`
  slider value (default -50 dBm, about 50 cm in the test room) at two stops in
  a row, or is well above it at one.
- Wi-Fi RSSI estimates direction only through changes after short moves. The
  displayed distance is rough and is not a true position measurement.
- The fixed front sonar guards against collisions and adds a point to the map
  for each echo; it cannot tell whether left or right is clear.

## Files involved

- `wayfinder_rover.ino` — active firmware.
- `secrets.h` — local ignored hotspot and calibration settings.
- `../movement_reference/movement_reference.ino` — original L298N movement
  reference.
- `../motor_bench/motor_bench.ino` — tethered serial test: one timed move per
  line at a chosen PWM duty, for checking direction and tuning duty and timing.

## Do not publish

`secrets.h` contains the hotspot credentials, laptop address and local Wi-Fi
calibration. It is intentionally ignored by Git and must never be committed or
uploaded.
