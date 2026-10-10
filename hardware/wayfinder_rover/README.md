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
