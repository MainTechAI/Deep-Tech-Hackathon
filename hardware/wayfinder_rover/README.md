# Wayfinder rover firmware

This is the active firmware for the physical rover. It uses a known Android or
iPhone hotspot as both the target signal and the local network for telemetry.
Bluetooth is not required.

## Wiring

| Module | UNO R4 WiFi pins |
| --- | --- |
| L298N left motor | ENA D8, IN1 D9, IN2 D10 |
| L298N right motor | ENB D13, IN3 D11, IN4 D12 |
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
5. The dashboard should show **Rover online**, Wi-Fi RSSI, and Front range.

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
- **Pause** — sends an immediate stop request.

### Arduino Serial Monitor (115200 baud)

- `c` — collect RSSI samples for ten seconds while the rover is exactly 1 m
  from the phone hotspot.
- `s` — print Wi-Fi connection state, filtered RSSI and current rover mode.

### Backend actions

The rover polls `/api/device/command`; accepted actions are `explore`,
`wifi_hunt`, `forward`, `backward`, `left`, `right`, `scan`, and `stop`.
Frontend language requests are converted to one of those actions by the local
gateway. The Arduino posts telemetry to `/api/telemetry`.

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
movement relies on whether the filtered signal gets stronger after each short
step, not on that number as a precise distance.

## Behaviour

`Find WAYFINDER_PLAYER` or `Go to the hotspot` starts the hunt. The rover takes
short forward steps, keeps a direction when RSSI improves, backs up and turns
when it worsens, and stops near a strong signal. A front obstacle or missing
sonar reading prevents forward motion. With one fixed front sonar, it cannot
know which side is genuinely clear; turns alternate as a conservative escape.

There are no encoders or IMU, so the rover cannot truthfully report global
position or build a room map. The live UI should be read as RSSI history plus
the current forward clearance, not SLAM.

## Field-test status — 10 October 2026

The assembled rover has successfully connected to the phone hotspot, posted
telemetry to the laptop, measured RSSI and returned valid fixed-sonar readings.
The measured RSSI was about **-60 dBm** at roughly three metres and **-43 dBm**
when the phone was close. The one-metre calibration measured **-53.7 dBm**.

Autonomous movement is currently **not safe to use**. During testing, the rover
could rotate continuously and collide with objects. The latest safe action was
`Stop`, which telemetry confirmed. Before another autonomous hunt, validate
motor direction, shorten motion pulses and verify each manual direction with
wheels raised. The complete test record and recovery checklist are in
[`TODO.md`](TODO.md).
