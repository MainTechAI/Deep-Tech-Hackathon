# BLE distance finder

Firmware for an Arduino UNO R4 WiFi that scans for one known BLE service UUID,
filters its RSSI, and displays an approximate distance in metres on the built-in
12x8 LED matrix. It is a proximity sensor only: one antenna cannot determine the
direction of the phone.

## Build and upload

Install the **Arduino UNO R4 Boards** package and the **ArduinoBLE** library.
Open `ble_distance_finder.ino`, select **Arduino UNO R4 WiFi**, then upload.
Use Serial Monitor at **115200 baud**.

The current target service UUID is:

```
d4937a10-21fc-4f53-8bc7-4105805d99a0
```

The phone must advertise that exact BLE service UUID. A regular iPhone personal
hotspot does not advertise it, so it cannot be detected by this sketch as-is.

## Serial commands

- `s` — print scanner state, target UUID, and the latest filtered RSSI.
- `c` — hold the phone one metre from the rover for five seconds to calibrate.

After calibration, copy the printed `RSSI_AT_1M` value into
`rssiAtOneMetre` to retain it after reboot.

## Hardware test status

The sketch builds for `arduino:renesas_uno:unor4wifi` (34% flash, 29% RAM).
On the tested board, USB Serial and Wi-Fi communication work and the ESP32-S3
connectivity firmware reports the current version `0.6.0`, but `BLE.begin()`
fails. Reinstalling the same connectivity firmware is not expected to resolve
that issue. This BLE sketch is retained as an experiment; the active rover uses
Wi-Fi hotspot RSSI instead.
