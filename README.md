# Wayfinder

Wayfinder is a local mission console for a small rover. The browser shows the sonar map; the backend receives telemetry and turns scan rays into map points. The `wokwi-esp32` project supplies a ready ESP32 simulation for the complete data path before the physical rover is assembled.

## Project layout

- `prototype/index.html` — the only frontend screen.
- `local_gateway_plain.py` — dependency-free local backend on port 8000.
- `wokwi-esp32/` — Wokwi and PlatformIO project: ESP32, HC-SR04 and scanning servo.
- `docs/components.pdf` — supplied component list.

## Local run

Start the backend from the repository root:

```text
py -3 local_gateway_plain.py
```

In another terminal, serve the frontend:

```text
py -3 -m http.server 5173 --directory prototype
```

Open `http://localhost:5173/index.html`.

## Local room simulation

The room simulation does not require Wokwi. Keep the gateway running, then start this in a second terminal from the repository root:

```text
py -3 room_simulator.py
```

It creates a virtual room with walls and obstacles, a player carrying the `WAYFINDER_PLAYER` hotspot, and a rover that sends the same telemetry contract as the physical device. In the console, click **START HUNT** to make the rover compare its signal readings, avoid obstacles with sonar, and stop at the simulated player.

`BLE check` in the console is explicitly a simulator-only proximity estimate. It is not a claim that the UNO R4 WiFi reads Wi-Fi and Bluetooth simultaneously.

## Wokwi transport test

Open `wokwi-esp32` as a VS Code folder, build with PlatformIO, then select **Wokwi: Start Simulator**. The simulated ESP32 sends sonar telemetry through the Private IoT Gateway to the local backend at port 8000. The mission console updates automatically.
