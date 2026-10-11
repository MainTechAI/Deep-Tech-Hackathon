# Wayfinder rover firmware

This is the active firmware for the physical rover. It uses a known Android or
iPhone hotspot as both the target signal and the local network for telemetry.
Bluetooth is not required.

## Wiring

| Module | UNO R4 WiFi pins |
| --- | --- |
| L298N left motor | ENA D5 (PWM), IN1 D9, IN2 D10 |
| L298N right motor | ENB D6 (PWM), IN3 D11, IN4 D12 |
| Fixed HC-SR04 | TRIG D3, ECHO D2 |

The L298N motor supply must be suitable for the motors. Join its ground to UNO
GND. Do not power motors from the UNO's 5 V pin. Start with wheels lifted and
keep a physical way to remove motor power during tuning.

## First network test

1. Copy `secrets.h.example` to `secrets.h` and enter the hotspot SSID/password.
2. Connect the laptop running `local_gateway_plain.py` to that same hotspot.
3. Find the laptop's hotspot IPv4 address and place it in
   `WAYFINDER_BACKEND_HOST`.
4. Run the backend, then upload `wayfinder_rover.ino`.
5. The console should show **Rover online**, the Wi-Fi signal and the sonar distance ahead.

The backend already listens on the local network; Windows Firewall may ask to
allow Python on private networks.

## What runs where

The system consists of three separate parts:

| Part | Purpose |
| --- | --- |
| UNO R4 WiFi | Reads the fixed front sonar, joins the known phone hotspot, measures Wi-Fi RSSI, drives L298N motors and posts telemetry. |
| `local_gateway_plain.py` on the laptop | Receives telemetry on port `8000` and stores the latest command for the rover to poll. It can use a local rule parser without an AI key. |
| `prototype/index.html` on the laptop | Shows connection, RSSI, front sonar clearance and mission state. It sends operator requests to the backend. |

The phone hotspot is both the search target and the local network. The laptop
and rover must both join it. No password or hotspot name is stored in Git:
those values exist only in the ignored `secrets.h` file.

## Commands

### Frontend

- **Start hunt** — asks the rover to search for the known hotspot while using
  the front sonar as a collision guard.
- **STOP** (top bar, pad centre, Space or Esc) — sends a stop request. It is
  picked up when the current move ends, at most about a second later.

### Manual control and live tuning

The **Manual** card has a direction pad, also on WASD and the arrow keys (one move per
press, repeats allowed), a stop button and a **1 s run** that drives forward
for one second so the speed can be measured.

The sliders change rover settings without re-uploading: drive power, forward
step, pause after each move, manual turn and hunt turn in degrees, turn rate,
obstacle distance, left wheel power and the "found" signal level. The console
posts them to `/api/tuning`; the rover reads them from the gateway's next reply. They are held
by the gateway. If the gateway restarts, the console shows the defaults again
but the rover keeps the values it last received until it reboots or a slider is
moved; press "Reset to defaults" to bring the two back in line.

### Arduino Serial Monitor (115200 baud)

- `c` — collect RSSI samples for ten seconds while the rover is exactly 1 m
  from the phone hotspot.
- `s` — print Wi-Fi connection state, filtered RSSI and current rover mode.

### Backend actions

The rover posts telemetry to `/api/telemetry` and reads the pending command,
tuning and reset token from the reply, so one round trip does both. Actions are
`explore`, `wifi_hunt`, `forward`, `forward_1s`, `spin_1s`, `backward`, `left`,
`right`, `scan`, and `stop`. Each operator command gets a new `command_id`; the
rover acts when the id changes, so the same action can be sent twice. Text the
gateway does not recognise is rejected and never reaches the rover.

## Wi-Fi signal calibration

RSSI is noisy indoors. Before the first hunt, place the rover exactly one
metre from the phone that shares the known hotspot. Open Serial Monitor at
115200 baud and send `c`; keep both devices still for ten seconds. The firmware
filters the readings and prints the measured one-metre RSSI. Put that number
into `WAYFINDER_RSSI_AT_ONE_METRE` in `secrets.h` and set
`WAYFINDER_RSSI_CALIBRATED` to `1` to retain it after reboot.
Send `s` to print the current connection, filtered RSSI, and rover mode.

The telemetry field `rough_wifi_distance_m` is only a rough signal estimate.
Walls, people and phone orientation affect it substantially, so autonomous
movement relies on how the signal changes between stops (see Behaviour), not on
that number as a precise distance.

## Behaviour

`Find WAYFINDER_PLAYER` or `Go to the hotspot` starts the hunt. It is a
run-and-tumble search: the rover takes short forward steps along a heading
while the signal holds up, and turns right by `hunt_turn_deg` when the signal
falls clearly (4 dB) below the best it saw on that heading. One antenna gives
no bearing, so direction only shows up as a change in signal after moving.
After every move it stands still, averages several RSSI samples taken at that
spot, and only then decides. It stops when the average reaches `target_rssi`.

A front obstacle closer than `safe_cm` makes it back up and turn right by about
70 degrees. No echo counts as open space (reported as 350 cm), so a surface
that deflects the ping, such as a wall at a steep angle, can be missed. Only a
sonar that does not respond at all blocks forward motion.

If the rover cannot reach the gateway for six seconds during a hunt, it stops,
because nobody could tell it to.

## Timing rules in the firmware

Wi-Fi calls (`WiFi.status()`, `WiFi.RSSI()`, HTTP) block for tens of
milliseconds to a second, and the motors keep running through them. So while a
move is running the loop only watches the clock and, when driving forward,
pings the sonar. Everything else happens at a standstill. Moves end with a
short motor brake so they stop in the same place every time.

Pings are spaced at least 50 ms apart. Fired faster, the HC-SR04 hears late
echoes of its previous ping and reports false short distances.

## Position and map

There are no encoders or IMU. The rover dead-reckons its pose:

- **Turns** are integrated from the clock at `turn_rate` degrees per second.
  Nothing corrects them, so heading is where the map drifts.
- **Straight moves** are integrated from the clock, then corrected: if there
  was a steady echo ahead before the move and there is one after it, the change
  in the sonar reading is the distance really covered and replaces the guess.
  A wildly different value (someone walked past) is ignored. Each accepted
  measurement also nudges the assumed speed, so steps taken in open space get
  better too. Telemetry reports `speed_cm_s` and `odo_fixes`.

Each report sends the pose and the sonar return measured from the turning
centre (the sonar sits about 9 cm ahead of it; `SONAR_OFFSET_CM` is an
estimate). The gateway turns returns into map points, merges near-duplicates,
and deletes older points that a newer reading sees straight through, which
removes ghosts left by drift and by people walking past. Returns beyond 2 m are
not mapped. The console's reset clears the map and restarts the pose at the
rover's current spot. Read the result as a sketch, not SLAM.

## Calibration

Two one-second tests, both on the console's manual pad:

- **1 s run** (`test`): measure the centimetres covered. This set
  `FORWARD_CM_PER_S` (30 cm at duty 110); sonar-measured steps refine it.
- **1 s spin** (`spin`): estimate the degrees turned and set the `turn_rate`
  slider to that number. This is the calibration that matters most for the
  map. It has not been redone since the motor brake was added.

## Field-test status — 10 October 2026

The assembled rover has successfully connected to the phone hotspot, posted
telemetry to the laptop, measured RSSI and returned valid fixed-sonar readings.
The measured RSSI was about **-60 dBm** at roughly three metres and **-43 dBm**
when the phone was close. The one-metre calibration measured **-53.7 dBm**.

The continuous spinning seen in the first autonomous tests was traced to a
broken D10 -> IN2 jumper, enable pins without PWM, moves overrunning during
HTTP calls and commands the firmware could not parse; all four were fixed and
checked on the floor. The firmware has since been reworked again and that
version has not yet run on the rover, so start with the wheels raised and work
through the checklist in [`TODO.md`](TODO.md).
