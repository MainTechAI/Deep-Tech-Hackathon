# Wayfinder

Wayfinder is a local mission console for a small rover. The browser shows the sonar map; the backend receives telemetry and turns scan rays into map points. The `wokwi-esp32` project supplies a ready ESP32 simulation for the complete data path before the physical rover is assembled.

## Final-day priorities (11 October 2026)

**Current status:** physical phone finding is not reliable. The team reports one successful attempt under lucky circumstances. Wi-Fi telemetry and front sonar work, but uncontrolled spinning and collisions occurred during autonomous tests.

The following is a static review of the committed firmware, gateway, frontend and simulator. These are proposed fixes, not completed or hardware-validated changes. Work through P0 before tuning the search.

| Priority | Weakness in current code | Practical next step |
| --- | --- | --- |
| P0 | **Commands can fail to parse.** Arduino `jsonAction()` expects `"action":"..."`, but the gateway emits `"action": "..."`. | Parse JSON with whitespace support. Verify actual gateway responses on the uploaded firmware, including Stop. |
| P0 | **Motor deadlines depend on a blocking loop.** Sonar reads can take about 99 ms; command HTTP reads can wait 900 ms, plus connection time. Motors can remain powered beyond their intended pulse. | Service motor deadlines independently of blocking work, or keep motors stopped during network operations and make sensing non-blocking. Measure actual pulse lengths and stop latency. |
| P0 | **Pause is not dependable.** Frontend `busy` discards commands during a pending request. With an API key, even Stop passes through the language model. A lost backend connection does not end a hunt while Wi-Fi remains connected. | Let Stop bypass `busy` and the model, give it priority over pending commands, and add a timeout for lost backend contact. |
| P0 | **Full-power, unverified movement.** Enables are simply HIGH; direction flags and turn angles are unverified on the chassis. | Check all four directions with wheels raised, then calibrate much shorter pulses on the floor. Increase front clearance to cover measured stopping distance. |
| P1 | **Signal comparisons are premature.** A 340 ms step is compared using a heavily smoothed RSSI without a stationary sampling window. Any gain below 1.5 dB triggers reversal, including small genuine improvements. | Stop, settle, and collect several spaced readings before and after each trial. Distinguish improving, uncertain and worsening signals. Retry uncertain measurements and stop on stale RSSI. |
| P1 | **Search can oscillate.** Alternating equal right/left turns can revisit the same headings. Timed reversing does not reliably return to the previous position. | Use a bounded sequence of new trial headings, physically moving before judging signal. Cap failed attempts and report no progress. Do not infer bearing from turning alone. |
| P1 | **Avoidance mixes states and treats unknown as an obstacle.** Hunt and avoidance share reverse/turn stages; changing front readings can switch between their turn timings. Invalid sonar also triggers blind reversing. | Give avoidance its own states. Stop and retry invalid readings. Use small, bounded escape moves for confirmed obstacles, accounting for the lack of rear/side sensing. |
| P1 | **Arrival can be declared too early.** One filtered threshold crossing ends the hunt. With the reported calibration, the threshold is -59.7 dBm, close to the observed reading at roughly 3 m. | Tune the threshold at the desired stopping distance and require several fresh stationary readings above it. Report proximity, not a verified phone position. |
| P2 | **Repeated commands and status are misleading.** Identical actions are ignored by `lastPlanAction`; backend `connected` never expires after telemetry stops. | Add command IDs and acknowledgements so repeated clicks work once each. Timestamp telemetry and mark stale devices offline. |

**Validation:** prioritize the physical rover. The simulator evaluates RSSI at unvisited candidate positions and stops using the true target distance, so its success does not validate the firmware. After the P0 fixes, log baseline/after-step RSSI, selected action and actual motion duration across repeated hunts from several starting positions and headings. Record failures as well as successes.

Relevant files: [firmware](hardware/wayfinder_rover/wayfinder_rover.ino), [gateway](local_gateway_plain.py), [frontend](prototype/index.html), [simulator](room_simulator.py), and [hardware test notes](hardware/wayfinder_rover/TODO.md).

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

It creates a virtual room with walls and obstacles, a player carrying the `WAYFINDER_PLAYER` hotspot, and a rover that sends the same telemetry contract as the physical device. In the console, click **START HUNT** to use sonar and signal readings together. The **Map & Sensors** tab has a Wi-Fi-only open-room test: it visualises only RSSI samples from rover positions and can run the signal-following search without sonar geometry.

`BLE check` in the console is explicitly a simulator-only proximity estimate. It is not a claim that the UNO R4 WiFi reads Wi-Fi and Bluetooth simultaneously.

## Wokwi transport test

Open `wokwi-esp32` as a VS Code folder, build with PlatformIO, then select **Wokwi: Start Simulator**. The simulated ESP32 sends sonar telemetry through the Private IoT Gateway to the local backend at port 8000. The mission console updates automatically.

## Hardware projects

The firmware and hardware experiments from `main` live under `hardware/`:

- `hardware/wayfinder_rover/` — **active firmware**: Wi-Fi hotspot signal hunt, L298N motors and a fixed forward HC-SR04.
- `hardware/ble_distance_finder/` — isolated Bluetooth RSSI experiment; it is not part of the rover mission.
- `hardware/sonar-scanner/` — stationary HC-SR04 + SG90 scanning experiment and viewer.

The Bluetooth experiment estimates proximity only from RSSI. It cannot determine bearing from one antenna, so the rover must still sample while moving or use another directional method to choose a direction.
