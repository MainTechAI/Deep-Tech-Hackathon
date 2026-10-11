# Wayfinder

Wayfinder is a local mission console for a small rover. The browser shows the sonar map; the backend receives telemetry and turns scan rays into map points. The `wokwi-esp32` project supplies a ready ESP32 simulation for the complete data path before the physical rover is assembled.

## Project layout

- `prototype/index.html` — the console: one screen with status, map, hunt and manual controls (WASD or arrows to drive, Space to stop), and live tuning sliders. `?api=http://host:port` points it at another gateway.
- `local_gateway_plain.py` — dependency-free local backend on port 8000.
- `wokwi-esp32/` — Wokwi and PlatformIO project: ESP32, HC-SR04 and scanning servo.
- `camera_tracker.py` — optional overhead-camera tracker that gives the rover an accurate pose.
- `docs/components.pdf` — supplied component list.

## Local run

Start the backend from the repository root:

```text
python3 local_gateway_plain.py
```

In another terminal, serve the frontend:

```text
python3 -m http.server 5173 --directory prototype
```

Open `http://localhost:5173/index.html`.

## Local room simulation

The room simulation does not require Wokwi. Keep the gateway running, then start this in a second terminal from the repository root:

```text
python3 room_simulator.py
```

It creates a virtual room with walls and obstacles, a player carrying the `WAYFINDER_PLAYER` hotspot, and a rover that sends the same telemetry contract as the physical device. In the console, click **Start hunt** to use sonar and signal readings together. **Wi-Fi only** runs the signal-following search without sonar geometry, and the map's **Wi-Fi signal** layer shows the RSSI samples by position; the **Sim: walls / Sim: open** switch picks the simulated room.

`BLE check` in the console is explicitly a simulator-only proximity estimate. It is not a claim that the UNO R4 WiFi reads Wi-Fi and Bluetooth simultaneously.

## Overhead camera tracking

Without wheel encoders the rover's own position estimate drifts. A camera
looking down at the floor fixes that: `camera_tracker.py` finds a printed
marker on top of the rover and posts its position and heading to the gateway
ten times a second. While the camera sees the rover, the console and the map
use the camera's pose; when it loses sight, they fall back to the rover's
estimate, which carries on from the last camera pose.

```text
python3 -m pip install -r requirements-camera.txt
python3 camera_tracker.py --width 1.5 --height 1.0
```

1. Print `docs/markers.pdf` at 100 % (no "fit to page") and check the sizes
   with a ruler: 8 cm for the rover marker, 15 cm for the four floor markers.
2. Tape floor markers 1 to 4 on the corners of a rectangle, anticlockwise
   seen from above starting at id 1, and pass its size in metres as `--width`
   (id 1 to id 2) and `--height` (id 1 to id 4). Id 1 is the map origin, id 2
   lies along +x. Which way each floor marker is turned does not matter.
3. Fix the rover marker flat on top of the rover, centred over the axle, with
   the edge labelled FRONT facing forward. Pass its height above the floor as
   `--rover-height-cm`.
4. Mount the camera where it sees all four floor markers. The tracker
   calibrates itself, saves the result to `camera_calibration.json`, and keeps
   working if the rover later covers a floor marker. Press `c` in its window
   to recalibrate after moving the camera.

The tracker has been checked on rendered camera views only (about 1 cm and
2 degrees of error there), not yet with a real camera.

A floor plan can be shown under the map: put an image and a `room.json` in
`prototype/`, for example
`{"image": "room.png", "left": -0.5, "bottom": -0.3, "right": 3.2, "top": 2.4}`,
where the numbers are the floor coordinates in metres of the image's edges.

## Wokwi transport test

Open `wokwi-esp32` as a VS Code folder, build with PlatformIO, then select **Wokwi: Start Simulator**. The simulated ESP32 sends sonar telemetry through the Private IoT Gateway to the local backend at port 8000. The mission console updates automatically.

## Hardware projects

The firmware and hardware experiments from `main` live under `hardware/`:

- `hardware/wayfinder_rover/` — **active firmware**: Wi-Fi hotspot signal hunt, L298N motors and a fixed forward HC-SR04.
- `hardware/ble_distance_finder/` — isolated Bluetooth RSSI experiment; it is not part of the rover mission.
- `hardware/sonar-scanner/` — stationary HC-SR04 + SG90 scanning experiment and viewer.

The Bluetooth experiment estimates proximity only from RSSI. It cannot determine bearing from one antenna, so the rover must still sample while moving or use another directional method to choose a direction.
