# Project guide: Deep Tech Hackathon rover

This file explains the whole project to an engineer or agent who has not seen it before: what it
is, what the parts are, how they talk to each other, and where to change things.

It describes the `main` branch at commit `cb68284` (11 October 2026). Where the READMEs in the
repository disagree with the code, this file follows the code and says so.

## 1. What the project is

A hackathon entry for "Project 1: small robot that measures distance to obstacles". The team built
a two-wheel rover and, on the second day, replaced the original design with a camera-guided one.

The repository therefore holds two generations of the system:

| Generation | Folder | Status | Idea |
|---|---|---|---|
| **Vision Nav** | `vision_nav/` | **Current. This is the system to work on.** | A phone above the floor tracks a marker on the rover. A server knows the rover's true position, maps obstacles from the rover's sonar, plans a path and steers the rover along it. |
| Wayfinder | repo root (`local_gateway_plain.py`, `prototype/`, `room_simulator.py`) and `hardware/wayfinder_rover/` | Superseded. Kept for reference. | The rover hunts for a phone by Wi-Fi signal strength with no knowledge of its own position. On `main` this code still has the faults listed in the root `README.md`. |

The two generations share the rover hardware and nothing else. They use different firmware,
different servers, different consoles and different protocols. Do not mix them.

Everything in sections 2 to 11 is about Vision Nav. Section 12 covers the rest of the repository.

## 2. Vision Nav in one picture

```
  PHONE (browser, camera.html)           SERVER (Python, aiohttp)              ROVER (Arduino UNO R4 WiFi)
  ----------------------------           ------------------------              ----------------------------
  camera video                           geometry.py  pixels -> floor cm       L298N + 2 DC gear motors
  ArUco detection in JavaScript   --->   brain.py     pose, missions, control  HC-SR04 sonar, facing forward
  sends marker corners, <= 15/s   WS     planner.py   occupancy grid, A*  ---> front safety, command watchdog
  draws the server's plan on top  <---   app.py       network, 10 Hz loop <--- telemetry at 10 Hz
                                              ^                            TCP
                                              | WebSocket
                                  CONSOLE (browser, index.html): live map, missions, joystick, tuning
```

The division of labour:

- **The phone only detects markers.** It sends a few numbers per frame (the pixel corners of each
  marker it sees), never video.
- **The server does all the thinking.** It turns pixels into floor coordinates, keeps the map,
  plans and decides every wheel command.
- **The rover is a thin, safe actuator.** It runs the wheels as told, reports its sonar distance,
  refuses to drive forward into something close, and stops on its own if commands stop arriving.
- **The console is a view and a remote control.** It holds no state of its own.

Why it is built this way: the rover has no wheel encoders and no gyro, so it cannot know where it
is from its own sensors. The camera measures the rover's real position and heading several times a
second, which makes closed-loop driving and a correct obstacle map possible.

## 3. Coordinates and units

These conventions are used everywhere in Vision Nav.

- **Units:** centimetres and degrees. Wheel speeds are percentages from -100 to 100, positive is
  forward.
- **Arena frame:** the origin is the centre of corner marker #1. +x runs to marker #2, +y runs to
  marker #4. The four corner markers are at (0,0), (W,0), (W,H), (0,H), anticlockwise seen from
  above.
- **Heading:** degrees anticlockwise from +x, wrapped to -180..180. Turning left increases it.
- **Rover pose:** the position of the wheel-axle centre. The marker centre is assumed to be
  `MARKER_FORWARD` cm ahead of it (0 by default) and the sonar `SENSOR_FORWARD` cm ahead of the
  marker (10 by default).
- **Map grid:** square cells of `GRID_CM` = 5 cm. Cell (i, j) covers x from 5i to 5i+5 and y from
  5j to 5j+5.

Marker ids (ArUco 4x4 dictionary):

| Id | Where | Purpose |
|---|---|---|
| 0 | flat on top of the rover, top edge towards the front | rover position and heading |
| 1 to 4 | arena corners, anticlockwise from near-left as the camera sees it | define the floor frame |
| 5 | a card a person holds | target for "Follow me" |

The camera page discards any marker with an id above 5.

## 4. The rover

### 4.1 Hardware

- Arduino UNO R4 WiFi. This is a Renesas RA4M1 (ARM), not an AVR. Its Wi-Fi is a separate ESP32
  module that the main chip talks to over a serial link, so every Wi-Fi call (`WiFi.status()`,
  `WiFi.RSSI()`, any `WiFiClient` method) is a slow round trip.
- L298N motor driver and two DC gear motors on a 2WD chassis with a caster wheel.
- One HC-SR04 ultrasonic sensor, fixed, facing forward. There is nothing looking sideways or back.
- A buzzer, and the board's built-in 12x8 LED matrix.
- Two 18650 cells in series.
- No wheel encoders, no IMU.

| Part | Pins |
|---|---|
| L298N ENA, IN1, IN2 (left motor) | D5 (PWM), D9, D10 |
| L298N ENB, IN3, IN4 (right motor) | D6 (PWM), D11, D12 |
| HC-SR04 TRIG, ECHO | D3, D2 |
| Buzzer | D4 |
| Free | D7, D8, D13 |

Speed control needs the enable pins on PWM-capable pins. On the UNO R4 those are D3, D5, D6, D9,
D10 and D11 only.

Facts measured on this rover during testing (with the older firmware, at PWM duty 110): about
30 cm travelled in one second; the left wheel ran slightly faster than the right; a jumper wire on
D10 was once broken, which made "forward" drive one wheel only.

### 4.2 Firmware

Folder: `vision_nav/firmware/rover/`. It is an Arduino sketch split into modules.

| File | Role |
|---|---|
| `robot.h` | Every pin, constant and tunable in one place. Start here. |
| `rover.ino` | `setup()`, `loop()`, the three modes, serial keys, the `service()` and `waitMs()` helpers |
| `movement.*` | Motor output. Every command passes through the front safety check and the timeout here. |
| `sensor.*` | Sonar reading |
| `link.*` | The TCP connection to the server |
| `web.*` | Wi-Fi setup and the rover's own small HTTP server and joystick page |
| `follow.*` | The stand-alone Wi-Fi-signal "follow" mode (not used by the server) |
| `buzzer.*`, `face.*` | Beeps and LED-matrix pictures |
| `tools/mock_robot.py` | A fake rover that serves the same HTTP API, for testing without hardware |
| `tools/test_api.py` | End-to-end check of the rover's HTTP API |

**Modes.** `MANUAL`, `AUTO`, `FOLLOW`.

- `MANUAL` is the mode the server needs. The rover does only what it is told.
- `AUTO` is stand-alone obstacle avoidance: drive forward at 70 %, and when something is closer
  than 25 cm, back up for 350 ms and spin for 225 ms, alternating right and left.
- `FOLLOW` is a stand-alone hunt for the phone whose hotspot the rover has joined, by signal
  strength: spin through 8 headings measuring RSSI, turn to the strongest, drive a 1.5 s leg,
  repeat. It stops when RSSI reaches -38 dBm. It works only when joined to a network.

The server's drive commands are **ignored unless the rover is in `MANUAL`**. The server's stop
command puts it back into `MANUAL`.

**Main loop.** Each pass does: `service()` (serial keys, the HTTP server, the server link, buzzer
warnings), one sonar reading, `moveUpdate(dist)` to apply safety and timeouts, then one step of the
current mode. Long manoeuvres in `AUTO` and `FOLLOW` use `waitMs()`, which keeps running all of
the above while it waits and returns early if the mode changes.

**Motor output (`movement.cpp`).** A command is a left and right speed in percent plus an optional
lifetime in milliseconds.

- Speeds below 10 % are treated as zero. From 1 to 100 % the PWM duty is mapped from `MIN_PWM`
  (110) to 255, because below about 110 the motors stall.
- Stopping sets both direction pins low and the enable pin to zero, so the motors coast to a halt.
  There is no active brake.
- `INVERT_LEFT`, `INVERT_RIGHT` and `SWAP_SIDES` in `robot.h` correct wiring without rewiring.
- A command with a lifetime is dropped when the time is up. This is the watchdog that stops the
  rover if the server goes quiet.

**Front safety.** Applied to every command whose forward component is positive (left + right > 0):

- Closer than `STOP_CM` (20 cm): the forward part is removed and only the turning part is kept.
  The rover reports itself as blocked.
- Between 20 cm and `SLOW_CM` (50 cm): the speed is scaled from 35 % up to 100 %.
- Turning on the spot and reversing are always allowed. There is no rear sensor.

**Sonar (`sensor.cpp`).** One ping per loop pass using `pulseIn` with a 25 ms timeout. The value
used is the median of the last three readings. A ping with no echo, and also a sensor that does not
answer at all, are both stored as 999, which the rest of the code treats as "clear".

**Wi-Fi (`web.cpp`).** At boot the rover tries for 12 s to join the network named in
`arduino_secrets.h`. If that fails or no network is set, it starts its own access point, `RobotCar`,
at 192.168.4.1. The last number of its IP address is shown on the LED matrix until it first moves.
If the network drops, it stops the motors and retries every 10 s. The server link works only when
the rover has joined a network.

**The rover's own HTTP API** (port 80, for use without the server; documented in `web.h`):
`GET /` joystick page, `/status`, `/m?x=&y=` joystick, `/drive?l=&r=&ms=`, `/stop`,
`/mode?m=auto|follow|manual`, `/beep?ms=&f=`. Every endpoint replies with the same status JSON.

**Serial console** (115200 baud): `w` `a` `s` `d` drive at full speed, `x` stop, `m` toggle
`AUTO`, `f` toggle `FOLLOW`.

**Secrets.** Copy `arduino_secrets.h.example` to `arduino_secrets.h` (git-ignored) and set
`STA_SSID`, `STA_PASS`, `SERVER_HOST`, `SERVER_PORT` and `ROBOT_TOKEN`. An empty `SERVER_HOST`
disables the server link.

**Building.** Board "Arduino UNO R4 WiFi". With the CLI:
`arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi vision_nav/firmware/rover`, and add
`--upload -p /dev/cu.usbmodem…` to flash. The sketch compiles (85 kB, 32 % of flash).

## 5. Rover to server link

A plain TCP connection that the rover opens outwards to `SERVER_HOST:SERVER_PORT` (default port
9000), so it works through NAT and from a phone hotspot. One text line per message.

```
rover  -> server   HELLO <token>
                   T <dist_cm> <left%> <right%> <mode> <rssi> <blocked>     every 100 ms
server -> rover    D <left%> <right%> <ms>     drive; obeyed only in MANUAL; dropped after ms
                   S                           stop and switch to MANUAL
                   B <ms> <freq>               beep
```

- `dist_cm` has one decimal; 999 means nothing in range. `left%` and `right%` are the wheel outputs
  after the safety check, not what was asked for. `blocked` is 1 when forward motion was refused.
- The server checks the token in `HELLO` and closes the connection if it is wrong. If a second
  rover connection arrives, the newest wins.
- The rover clamps `ms` to 1 to 2000.
- On the rover side: it retries the connection every 3 s and stops the motors first, because
  connecting blocks. It checks that the link is still up once a second and reads incoming bytes
  every 20 ms.
- If the link drops, nothing special happens: the last drive command simply expires and the rover
  stops.

## 6. The server

Folder: `vision_nav/server/`. Python 3.12, one dependency (`aiohttp`). One process serves the web
pages, the WebSocket, and the rover's TCP port.

| File | Role |
|---|---|
| `config.py` | Every constant. Values can be overridden by environment variables or a `.env` file next to it. |
| `app.py` | Networking only: HTTP static files, the `/ws` WebSocket, the rover TCP server, the 10 Hz control loop, saving settings |
| `brain.py` | All state and decisions: poses, missions, the controller, calibration. It has no networking and can be driven from a test. |
| `planner.py` | The occupancy grid and A* path planning |
| `geometry.py` | Homography maths in pure Python (no NumPy) |
| `sim.py` | A simulated rover and camera for testing without hardware |

### 6.1 From camera pixels to floor position (`geometry.py`, `Brain.camera_frame`)

1. When all four corner markers are visible, the server solves a homography that maps the pixel
   centres of markers 1 to 4 onto (0,0), (W,0), (W,H), (0,H). This is redone on every frame until
   the calibration is locked, so the phone may be nudged. The camera does not need to look straight
   down.
2. For the rover marker, the centre and the midpoints of its top and bottom edges are mapped to the
   floor. The centre gives the position; the direction from bottom edge to top edge gives the
   heading.
3. `heading_offset` (found by calibration) is added, in case the marker was stuck on rotated.
4. The pose is smoothed: each new measurement moves the stored pose 60 % of the way towards it.
   After more than a second without a sighting, the next measurement is taken as is.

The pose is "fresh" for `POSE_TIMEOUT` = 0.5 s after the last sighting.

**Arena size.** By default the server measures W and H itself from the printed size of the corner
markers (`CORNER_MARKER_CM` = 13). It averages 40 frames, applies the result once, and stops.
Typing a size into the console turns this off. Changing the arena size rebuilds the grid, which
**erases the map**, and discards the current calibration.

**Known error.** The homography is exact only for points on the floor. The rover's marker sits
about 10 cm above it, so an angled camera sees it shifted away from the camera. Raising the corner
markers to the same height removes the error; the code does not correct for it.

### 6.2 The obstacle map (`planner.py` Grid, `Brain.telemetry`)

Each telemetry line from the rover is turned into map evidence, but only while the pose is fresh:

- The sonar's position is the rover pose moved `SENSOR_FORWARD` cm along the heading.
- **Free space:** cells along the ray from the sonar out to the echo minus 5 cm (at most
  `FREE_RANGE` = 80 cm) lose any *unconfirmed* evidence. Confirmed obstacles are never erased this
  way, because the sonar cone is wide and a ray passing beside a box corner would otherwise delete
  it.
- **Hits:** an echo closer than `OBSTACLE_RANGE` (45 cm) adds one point of evidence to the cell it
  lands in. A cell becomes an obstacle at `MAP_HITS` (3) points; evidence is capped at 5.
- Hits are ignored while the rover is turning (wheel outputs differ by more than 30), because the
  cone sweeps across everything.
- Hits within `EDGE_IGNORE` (15 cm) of the arena edge are ignored as things outside the arena,
  unless the echo is closer than 25 cm, which is taken as certainly real.
- Walls drawn by hand in the console are permanent until toggled off.
- "Clear map" is the only way to remove a confirmed obstacle that has been moved away.

### 6.3 Path planning (`planner.py`)

- Obstacles and hand-drawn walls are grown by `ROBOT_RADIUS` (13 cm) so the rover can be treated
  as a point. A fence `FENCE_CM` (20 cm) wide along every arena edge is also blocked, and targets
  are clamped inside it.
- A* runs on the grid with eight directions of movement.
- The cell path is then "string-pulled": corners are dropped whenever a straight line between two
  points is free, leaving a short list of straight segments.
- If the goal is inside a blocked area, the nearest free cell within 8 cells is used. If the rover
  itself is inside a blocked area, it is allowed to drive out of it.
- The plan is refreshed when the map changes, the target moves, or every `REPLAN_S` (0.5 s). The
  current route is kept as long as nothing blocks it, so the rover does not flip between two
  equally good ways round a box.

### 6.4 The control loop (`Brain.tick`, called 10 times a second)

Each tick returns at most one command for the rover. The checks run in this order, and the first
one that applies wins:

1. **Joystick.** If a joystick message arrived in the last 0.6 s, drive with it
   (left = y + x, right = y - x, lifetime 600 ms). Any joystick input cancels the mission.
2. **Calibration** in progress: run its next step.
3. **No mission:** send nothing.
4. **Rover offline** (no telemetry for 1.5 s): send nothing.
5. **Rover not visible.** If it disappeared less than 1.3 s ago, reverse the last motion at 40 %
   to bring it back into view. Otherwise stop.
6. **Pose older than 0.3 s:** stop and wait for the camera, to avoid driving blind.
7. **Work out the target.** For "go to" and "home" it is the clicked point. For "follow" it is a
   point `FOLLOW_DIST` (45 cm) short of the person; within 55 cm the rover only turns to face them.
8. **Arrived** (within `GOAL_TOL` = 8 cm, not used for follow): stop, beep, end the mission.
9. **Plan** if needed. No route means stop with status "no path".
10. **Back off** at -45 % for 300 ms if the rover reports blocked or was just found stuck, but only
    if there is 25 cm of arena behind it.
11. **Follow the path** (below), then run the stuck check.

**Following the path.** With `STEER_GAIN` at its default of 0 the rover drives a polyline: it aims
at the next corner of the route.

- If the heading error is above `TURN_IN_PLACE_DEG` (15 degrees), it stops and turns in pulses
  until it is within `ALIGN_DEG` (6 degrees).
- Otherwise it drives straight at `CRUISE` (55 %), slowing towards `CRUISE_MIN` (35 %) in the last
  40 cm. Each drive command lasts `DRIVE_MS` (400 ms) and is renewed every tick.
- With `STEER_GAIN` above 0 it instead aims at a point `LOOKAHEAD` cm along the path and steers
  gently while driving, with the wheel difference capped at 40 % of the speed.

**Turn pulses.** Camera frames arrive late, so a continuous turn overshoots. A turn is a short
burst followed by a wait:

- Burst length is 3 ms per degree of error, between 40 ms and `TURN_PULSE_MS` (120 ms).
- Then the rover stands still for `TURN_SETTLE_MS` (350 ms) so the camera sees the result.
- With `TURN_PIVOT` = 1 (default) one wheel runs at `TURN_SPEED` (45 %) and the other stays still.
  With 0 the wheels run in opposite directions.
- After each pulse the server checks which way the heading actually moved. Three wrong-way turns in
  a row flip `turn_sign`, which handles motors wired the other way or corner markers laid out
  clockwise.

**Stuck detection.** If the rover has been commanded forward for 1.5 s and the camera has seen it
move less than 3 cm, it has hit something the sonar cannot see. Three obstacle cells are marked in
front of it and it backs off for 0.6 s.

**Wheel trim.** `TRIM` (percent) weakens one wheel on every drive command so the rover runs
straight: positive weakens the left, negative the right.

### 6.5 Calibration (the console's Calibrate button)

A two-step test that needs the rover visible:

1. Spin right for 0.6 s. The heading should fall. If it rises, `turn_sign` is set to -1.
2. Drive forward for 0.8 s. The direction it actually moved, compared with the heading the marker
   reported, gives `heading_offset`. Any change of heading during the run adjusts `TRIM` by 1.5 %
   per degree.

### 6.6 What is kept across restarts

`data/settings.json` (or `$DATA_DIR/settings.json`) holds the live-tuned values, `turn_sign` and
`heading_offset`. It is written whenever they change and read at start-up. The map, the home point
and the camera calibration are not saved.

### 6.7 Access control

- `CONTROL_KEY` (default `dev`): a browser that sends this key in its `hello` message may control
  the rover and may act as the camera. Without it a browser can only watch.
- `ROBOT_TOKEN` (default `dev-robot-token`): the rover must send this in `HELLO`.
- Both pages take the key from `?key=…`, store it in `localStorage`, and remove it from the
  address bar so it does not show on a projector.
- The defaults are for local use. `deploy.sh` generates random values on the server.

## 7. Browser to server protocol

One WebSocket at `/ws`, JSON messages. Both the camera page and the console use it.

**Client to server**

| Message | From | Meaning |
|---|---|---|
| `{"type":"hello","role":"camera"\|"console","key":"…"}` | both | must be first; the reply is `welcome` with `control: true\|false`, then the current map |
| `{"type":"frame","w":…,"h":…,"markers":{"0":[[x,y]×4],…}}` | camera | pixel corners of each visible marker, in the order top-left, top-right, bottom-right, bottom-left. Ignored without the key. |
| `goto {x,y}` | console | drive to a point |
| `follow`, `home`, `stop` | console | start following, return home, stop everything |
| `sethome {x,y}` or `sethome` | console | set home to a point, or to where the rover is now |
| `joy {x,y}` | console | joystick, each -100..100, sent 10 times a second while held |
| `calibrate` | console | run the calibration test |
| `clear` | console | erase the map and the trail |
| `wall {x,y}` | console | toggle a hand-drawn obstacle cell |
| `arena {w,h}`, `arena_auto` | console | set the arena size by hand, or measure it again |
| `lock {on}` | console | freeze or unfreeze the camera calibration |
| `tune {key,value}` | console | change one live setting (clamped to its allowed range) |
| `beep` | console | make the rover beep |

**Server to client**

- `state`, 10 times a second, to everyone. Fields: `arena` (w, h, cell, cal, locked, auto, est),
  `camera` (online, fps), `robot` (online, visible, pose {x, y, th, age}, link, and the last
  telemetry: dist, l, r, mode, rssi, blocked), `person`, `home`, `mission` (type, target, status),
  `path`, `trail` (last 300 points), `map_version`, `events` (last 8 log lines), `tune`.
- `map`, whenever the map changes: `cells` (confirmed obstacles), `walls`, `cell` size, `version`.
- `overlay`, 10 times a second, to cameras only: the arena outline, path, target, home and obstacle
  cells projected back into image pixels, so the phone can draw them over the video.

A client that stops reading is dropped after 0.5 s so it cannot stall the control loop.

## 8. The camera page

File: `vision_nav/web/camera.html`, opened on the phone that is mounted above the arena.

- Uses the browser's camera API, which phones allow only on an HTTPS page. This is why the
  deployed server sits behind HTTPS.
- Marker detection runs in the browser with the bundled `js-aruco2` library (`web/vendor/`),
  dictionary `ARUCO_4X4_1000`.
- Each frame is scaled to a working width chosen on the page (640, 960 or 1280 pixels; 960 by
  default) before detection. At most 15 frames a second are sent.
- It requests a screen wake lock and runs from a timer so that it keeps working when the tab is
  not in front.
- It draws the video with the detected markers outlined and the server's overlay (path, target,
  obstacles) on top, plus status pills: server link, frame rate, corners seen out of 4, rover seen,
  person seen.

## 9. The console

File: `vision_nav/web/index.html`. A single page, no build step, no libraries.

- **Live map:** arena, obstacle cells, hand-drawn walls, rover with heading, trail, planned path,
  target, home, the person marker.
- **Map tools:** "Go here" (click to send the rover), "Draw walls" (click cells to toggle
  obstacles), "Set home".
- **Buttons:** Stop, Follow me, Go home, Calibrate, Clear map, Lock camera, Beep.
- **Arena size:** type W and H, or measure automatically.
- **Tuning panel:** one number field per live setting; a change is sent when the value is
  committed (Enter or leaving the field) and is saved by the server.
- **Driving by hand:** an on-screen joystick, or W A S D and the arrow keys. Space is Stop. Keys
  are ignored while typing in a field.
- Without the key the page shows "View only" and the controls do nothing.

`vision_nav/web/markers.html` is the printable marker sheet with placement instructions;
`vision_nav/print/markers.pdf` is the same thing ready to print (13 cm corner markers, 10 cm rover
marker, A4, print at 100 %).

## 10. Settings

All in `vision_nav/server/config.py`. Any of them can be set as an environment variable or in
`server/.env`. The ones marked "live" can also be changed from the console while running.

| Setting | Default | Live | Meaning |
|---|---|---|---|
| `HTTP_PORT`, `ROBOT_PORT` | 8080, 9000 | | web port and rover TCP port |
| `ROBOT_TOKEN`, `CONTROL_KEY` | dev values | | access control (section 6.7) |
| `CORNER_MARKER_CM` | 13 | | printed side of the corner markers, for measuring the arena |
| `ARENA_W`, `ARENA_H` | 300, 200 | | arena size until measured or typed in |
| `GRID_CM` | 5 | | map cell size |
| `ROBOT_RADIUS` | 13 | yes | clearance added round obstacles |
| `FENCE_CM` | 20 | yes | strip along the arena edge the rover centre stays out of |
| `STOP_CM` | 20 | | must match the firmware's front safety distance |
| `SENSOR_FORWARD`, `MARKER_FORWARD` | 10, 0 | | where the sonar and the marker sit on the rover |
| `CONTROL_HZ` | 10 | | control loop rate |
| `POSE_TIMEOUT` | 0.5 s | | pose is stale after this |
| `MOTION_POSE_AGE` | 0.3 s | | will not drive on a pose older than this |
| `ROBOT_TIMEOUT` | 1.5 s | | rover is offline after this without telemetry |
| `DRIVE_MS` | 400 | | lifetime of each drive command |
| `CRUISE`, `CRUISE_MIN` | 55, 35 | yes | driving speed, and speed near the goal |
| `TURN_SPEED` | 45 | yes | speed during turn pulses |
| `TURN_IN_PLACE_DEG` | 15 | yes | heading error that triggers a stop-and-turn |
| `ALIGN_DEG` | 6 | yes | turn until within this |
| `TURN_PULSE_MS`, `TURN_SETTLE_MS` | 120, 350 | yes | longest turn burst, and the wait after it |
| `TURN_PIVOT` | 1 | yes | 1 turns round one wheel, 0 spins on the spot |
| `STEER_GAIN` | 0.0 | yes | 0 is polyline driving; above 0 steers while driving |
| `STEER_MAX` | 0.4 | | cap on steering while driving |
| `TRIM` | 0.0 | yes | straight-line wheel balance |
| `LOOKAHEAD` | 25 | yes | aim point distance when `STEER_GAIN` is above 0 |
| `GOAL_TOL` | 8 | yes | arrival tolerance |
| `FOLLOW_DIST` | 45 | yes | distance kept from the person |
| `OBSTACLE_RANGE` | 45 | yes | echoes closer than this are mapped |
| `MAP_HITS` | 3 | yes | echoes needed to confirm a cell |
| `EDGE_IGNORE` | 15 | | echoes this near the arena edge are ignored |
| `FREE_RANGE` | 80 | | how far a sonar ray clears unconfirmed evidence |
| `REPLAN_S` | 0.5 | | periodic re-plan interval |

The tuning table in `vision_nav/README.md` is out of date: it gives `TURN_SPEED` 55,
`STEER_GAIN` 0.8, `TURN_IN_PLACE_DEG` 35 and `ROBOT_RADIUS` 16. The values above are the ones in
`config.py`.

Firmware settings (pins, `MIN_PWM`, safety distances, the stand-alone modes) are in
`vision_nav/firmware/rover/robot.h` and need a re-upload to change.

## 11. Running Vision Nav

### 11.1 Without hardware

```bash
cd vision_nav
python3 -m venv .venv && .venv/bin/pip install -r server/requirements.txt
.venv/bin/python server/app.py              # terminal 1
.venv/bin/python server/sim.py --person     # terminal 2
```

Open `http://localhost:8080/?key=dev`. The simulator connects to the server exactly as the real
rover and phone do. It models an angled camera with pixel noise and dropped frames, a 0.35 s camera
delay, a weaker left wheel, a motor dead zone, command latency, a sonar cone, and two boxes the
camera does not know about. Options: `--person`, `--arena WxH`, `--reversed-turn`, `--marker-rot`.

This was run while writing this guide: the server calibrated from the simulated corner markers,
accepted a "go to" command, mapped the hidden box from sonar echoes and re-planned around it. The
target lay behind the box and was not reached within the 60 seconds the run was given.

### 11.2 Deployed

- `./deploy/deploy.sh <ssh-host>` copies the project to `/opt/vision_nav` on the host and starts
  Docker Compose. The default host name is `vpn`.
- Two containers on host networking: the app (web on 127.0.0.1:8080, rover TCP on 0.0.0.0:9000)
  and Caddy, which serves HTTPS on 443 and obtains the certificate through port 80.
- The Caddyfile is written for the domain `coolrover.online`; the firmware's secrets example points
  at the same host.
- On the first run the script creates `server/.env` on the host with a random `ROBOT_TOKEN` and
  `CONTROL_KEY`. The firmware folder is not copied to the server.
- `data/` is a mounted volume, so tuning and calibration survive redeploys.

### 11.3 Setting up the arena

1. Print the markers at 100 % and check one with a ruler.
2. Marker 0 goes flat on top of the rover, arrow to the front.
3. Markers 1 to 4 go on the arena corners, anticlockwise seen from above, starting near-left as the
   camera sees it.
4. Mount the phone high and steady with all four corners in view.
5. On the phone open `https://<host>/camera.html?key=<CONTROL_KEY>` and start the camera.
6. On a laptop open `https://<host>/?key=<CONTROL_KEY>`, press Calibrate, then Lock camera.

## 12. Everything else in the repository

| Path | What it is |
|---|---|
| `local_gateway_plain.py` | Wayfinder backend: a dependency-free HTTP server on port 8000. The rover polls it for commands and posts telemetry as JSON. |
| `prototype/index.html` | Wayfinder console, served with `python3 -m http.server 5173 --directory prototype` |
| `room_simulator.py` | Wayfinder simulator: a fake room, rover and phone posting to the gateway |
| `hardware/wayfinder_rover/` | Wayfinder firmware: joins a phone hotspot and hunts for the phone by Wi-Fi signal strength. Its `README.md` and `TODO.md` record the first field tests. |
| `hardware/movement_reference/` | The first stand-alone obstacle-avoidance sketch. Not safe on the floor: full power, no time limit on manual moves. |
| `hardware/sonar-scanner/` | A stationary experiment: sonar on a servo sweeping 180 degrees, with a Python viewer |
| `hardware/ble_distance_finder/` | A Bluetooth signal-strength experiment, not part of any mission |
| `wokwi-esp32/` | An ESP32 simulation project that posts sonar telemetry to the Wayfinder gateway |
| `Webots-RL/` | A separate reinforcement-learning navigation experiment in the Webots simulator, with scripts for a training cluster. `lumi-upload.zip` at the root belongs to it. Unrelated to the rover code. |
| `docs/components.pdf` | The organisers' parts list |
| `README.md` (root) | Describes Wayfinder and lists its known faults as of 11 October. It does not mention Vision Nav. |
| `CLAUDE.md` (root) | Out of date on `main`: it still describes a stepper-motor build that was never used. |

### Branches

| Branch | State |
|---|---|
| `main` | What this guide describes |
| `artem/vision-nav` | One commit ahead of `main`: avoids staircase routes near the fence and skips path corners closer than 15 cm |
| `rover-pwm-map-tuning` | Not merged. A reworked Wayfinder: PWM speed control, moves that end on time, a dead-reckoned map, a redesigned console with live tuning. A working copy of this branch also has uncommitted additions, including `camera_tracker.py`, an OpenCV overhead-camera tracker for the Wayfinder gateway. Vision Nav covers the same ground in a different way. |
| `3d-models` | Draft STL files for a rover shell and hatch, under `cad/` |
| `akira`, `bluetooth_module_setting`, `maxoven` | Older work, already merged into `main` |

## 13. Things to know before changing anything

**Safety is layered, and the layers are independent.** Keep them that way.

1. The firmware refuses forward motion closer than 20 cm, whatever the server says.
2. Every drive command expires on the rover (400 ms from the server, 2 s at most).
3. The server stops the rover when the camera loses it, when the pose is stale, and on any error
   in a control tick.
4. The server ignores camera frames and commands from browsers without the key.

**Weak points in the current code** (found by reading; the first three were seen as real faults in
the Wayfinder firmware on this same hardware):

- **A dead sonar reads as clear.** In `sensor.cpp` a sensor that never answers gives 999, the same
  as open space. With the sonar unplugged the front safety does nothing.
- **Sonar pings are fired back to back.** The HC-SR04 datasheet asks for about 60 ms between pings.
  Fired faster, it can hear a late echo of the previous ping and report a false short distance.
  Here there is one ping per loop pass with no minimum gap. The median of three and the
  three-echo confirmation on the server hide much of this.
- **Wi-Fi calls block while the motors run.** `linkPoll()` and `webPoll()` are called from the main
  loop during motion. A slow call delays the next safety check and the command timeout.
- **A failed TCP connect may leak a socket.** `tryConnect()` in `link.cpp` does not call
  `client.stop()` when `connect()` fails. On this board a failed connect still takes a socket on
  the Wi-Fi module. If the server is down for a while, the rover may be unable to connect again
  until it is reset. This is inferred from the Wi-Fi library's source, not observed.
- **The rover marker's height is not corrected** (section 6.1). With a low, angled camera the
  position error can be several centimetres and grows away from the camera.
- **Auto-measuring the arena depends on the corner markers being exactly 13 cm.** The marker page
  says the printed size is not critical; that is true only when W and H are typed in by hand.
- **Re-measuring or resizing the arena erases the map.**
- **Latency.** From camera to wheels is roughly 150 to 250 ms according to the Vision Nav README,
  which is why turns are done in pulses and speeds are moderate.
- **One forward sonar.** Obstacles beside or behind the rover are found only by bumping into them
  (stuck detection) or by drawing them on the map.

**Where to make common changes**

| To change | Edit |
|---|---|
| How the rover drives along a path, turning, missions | `server/brain.py` |
| Map rules, clearance, planning | `server/planner.py` |
| Any threshold or speed | `server/config.py` (add it to `TUNABLE` to make it live) |
| A new console command | `handle_console_msg` in `server/app.py`, a method on `Brain`, and a control in `web/index.html` |
| What the rover reports or accepts | `firmware/rover/link.cpp` and `handle_robot` in `server/app.py`, together |
| Pins, minimum PWM, safety distances | `firmware/rover/robot.h` |
| Marker detection | `web/camera.html` |

`brain.py` and `planner.py` have no networking, so new control logic can be tried against `sim.py`
before it goes near the hardware. There are no automated tests in `vision_nav/`.
