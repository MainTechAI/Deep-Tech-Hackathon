# Vision Nav

An obstacle-aware 2WD rover that knows exactly where it is.

A phone mounted above the arena acts as an indoor "GPS": it tracks a marker on the rover, and a
server plans a path around obstacles and steers the rover there, correcting it 10 times a second.
The rover's ultrasonic sensor finds obstacles the camera doesn't know about; they are added to a
shared map and the path is re-planned around them. The same map is then reused, so the rover can
drive back home without searching again.

**What it can do**

- **Go here:** click a point on the live map and the rover drives there around known obstacles.
- **Map obstacles:** every sonar echo is placed on the map at its true position, because the camera
  knows the rover's position and heading at that moment.
- **Return home** along the shortest known path.
- **Follow me:** a person holding the "follow me" marker card is followed at about 45 cm.
- **Stay safe:** the rover itself blocks forward motion closer than 20 cm, every command expires
  after 0.4 s, and it stops if the camera loses it or the link drops.
- **Self-calibrate:** a short spin-and-drive test measures turn direction, speed and how the marker
  is mounted, using the camera.
- **Drive by hand** from any browser: a joystick in the console, or the on-board Wi-Fi joystick page
  that works without the server.

## How it works

```
 Phone camera (browser)               Server (Python, aiohttp)                  Rover (UNO R4 WiFi)
 ──────────────────────               ────────────────────────                  ───────────────────
 ArUco detection on the phone  ──►    homography → pose in cm                    L298N + 2 DC motors
 sends marker corners, 15 fps         occupancy grid from sonar          ──►     HC-SR04 front sonar
 draws the planned path back   ◄──    A* path planning                   ◄──     front safety, watchdog
                                      path following at 10 Hz             TCP    telemetry at 10 Hz
                                            ▲
                       Console in any browser: live map, missions, joystick
```

- **Localisation.** Four corner markers define the arena. A homography maps camera pixels to
  centimetres on the floor, so the camera does not have to look straight down: an angled view from
  the side works. The rover's marker gives its position and heading to within a couple of degrees.
- **Why markers detect on the phone.** Only a few bytes of coordinates per frame cross the
  network instead of video, which keeps latency low on mobile data.
- **Closed loop.** DC motors are imperfect: one wheel is stronger, and speed changes with battery
  level. Because the camera measures the real heading every frame, the controller corrects drift
  continuously instead of relying on timed turns.
- **Mapping.** Sonar hits add evidence to 5 cm grid cells, and free space along each ray removes
  it, so noise and moved obstacles fade out. Obstacles are inflated by the rover's radius before A*
  planning, and the path is smoothed to straight segments.
- **Stuck detection.** If the rover is commanded forward but the camera sees no movement, it has
  hit something the sonar cannot see, such as a box corner beside it. That spot is mapped and the
  rover backs off.

## Repository layout

| Path | What |
|---|---|
| `firmware/rover/` | Arduino sketch for the UNO R4 WiFi. Pins and tuning in `robot.h`, server link in `link.*` |
| `server/` | `app.py` (network), `brain.py` (missions and control), `planner.py` (grid and A*), `geometry.py` (homography), `sim.py` (simulator), `config.py` (all tunables) |
| `web/` | `index.html` console, `camera.html` phone camera, `markers.html` printable markers |
| `deploy/` | Docker Compose (app and Caddy for HTTPS), `deploy.sh` |

## Hardware

UNO R4 WiFi, L298N driver, 2 DC gear motors on the 2WD chassis, HC-SR04, buzzer, 2×18650.

| Part | Pins |
|---|---|
| L298N ENA, IN1, IN2 (left) | D5, D9, D10 |
| L298N ENB, IN3, IN4 (right) | D6, D11, D12 |
| HC-SR04 Trig, Echo | D3, D2 |
| Buzzer | D4 |

Motor direction, minimum PWM, safety distances and turn timing are all in `firmware/rover/robot.h`.

## Running it

### 1. Try it without hardware (simulator)

```bash
cd vision_nav
python3 -m venv .venv && .venv/bin/pip install -r server/requirements.txt
.venv/bin/python server/app.py                # terminal 1
.venv/bin/python server/sim.py --person       # terminal 2: simulated rover + camera + person
```

Open <http://localhost:8080/?key=dev>, click the map, and try **Follow me** and **Go home**. The
simulator models an oblique camera, sonar with a 15° cone, a weaker left wheel, command latency
and boxes the camera doesn't know about.

### 2. Server

```bash
./deploy/deploy.sh <ssh-host>
```

The script copies the project, creates `server/.env` with a random `ROBOT_TOKEN` and `CONTROL_KEY`
on the first run, and starts Docker Compose. Caddy gets a Let's Encrypt certificate (port 80 must
be open) and serves HTTPS on **8443**, because 443 is used by another service on our host. The
rover connects to TCP port **9000**. HTTPS is required: phones only give a web page camera access
on a secure origin.

### 3. Rover firmware

1. Copy `firmware/rover/arduino_secrets.h.example` to `arduino_secrets.h` and fill in:
   - the Wi-Fi network (2.4 GHz; a phone hotspot works);
   - `SERVER_HOST`, `SERVER_PORT` and `ROBOT_TOKEN` from the server's `.env`.
2. Upload with the Arduino IDE (board "Arduino UNO R4 WiFi").
3. Check the result:
   - The LED matrix shows the last number of the rover's IP address.
   - The rover beeps when it connects to the server.
   - Serial Monitor at 115200 shows the details.

Without a server the rover still works on its own: open its IP in a browser for the Wi-Fi
joystick, or use the `AUTO` (obstacle avoidance) mode.

### 4. Arena

1. Print `web/markers.html` at **100 % scale** and check a marker with a ruler.
2. **Marker #0** goes flat on top of the rover, with its arrow towards the front.
3. **Markers #1–#4** go at the arena corners, counter-clockwise as seen from above: #1 near-left,
   #2 near-right, #3 far-right, #4 far-left, as seen from the camera.
   - Measure W (#1→#2) and H (#1→#4) between marker centres and enter them in the console.
   - Raise the corner markers to the height of the rover's marker if you can.
4. Mount the phone high and steady, at least ~45° above the floor, with all four corners in view.
5. On the phone, open `https://<domain>:8443/camera.html?key=<CONTROL_KEY>` and tap
   **Start camera**.
6. In the console, open `https://<domain>:8443/?key=<CONTROL_KEY>` and press **Calibrate**. When
   the arena is calibrated, **Lock camera** freezes the calibration.

Anyone can open the console without the key to watch; only the key gives control.

## Protocols

**Rover ↔ server** (TCP, one line per message):

```
rover  → server   HELLO <token>
                  T <dist_cm> <left%> <right%> <mode> <rssi> <blocked>      10 Hz
server → rover    D <left%> <right%> <ms>    drive; expires after ms (watchdog)
                  S                          stop
                  B <ms> <freq>              beep
```

**Browser ↔ server** (WebSocket `/ws`, JSON):

- Every client first sends `hello` with a `role` and a `key`.
- The camera sends `frame` messages with the marker corners.
- The console sends commands: `goto`, `follow`, `home`, `sethome`, `stop`, `joy`, `calibrate`,
  `clear`, `wall`, `arena`, `lock`, `beep`.
- The server sends `state` at 10 Hz, `map` when the map changes, and `overlay` (the path projected
  into the camera image) to cameras.

The rover's own HTTP API (local network, no server needed) is documented in
`firmware/rover/web.h`. `firmware/rover/tools/test_api.py` checks it end to end.

## Tuning

Everything is in `server/config.py` and can be overridden with environment variables in `.env`.

| Setting | Default | Meaning |
|---|---|---|
| `CRUISE`, `TURN_SPEED` | 55 % | driving and turning speed |
| `STEER_GAIN` | 0.8 | how hard it steers per degree of heading error |
| `TURN_IN_PLACE_DEG` | 35° | above this heading error it turns on the spot first |
| `LOOKAHEAD` | 25 cm | how far ahead on the path it aims |
| `GOAL_TOL` | 8 cm | arrival tolerance |
| `ROBOT_RADIUS` | 16 cm | obstacle clearance |
| `SENSOR_FORWARD`, `MARKER_FORWARD` | 10 / 0 cm | sensor and marker position on the rover |
| `FOLLOW_DIST` | 45 cm | distance kept from the person |

## Limitations

- Localisation needs the camera to see the rover's marker. If it is hidden behind an obstacle or
  a person, the rover stops and waits.
- One forward sonar: obstacles beside or behind the rover are found only by bumping into them
  (stuck detection) or by drawing them on the map.
- An oblique camera sees the rover's marker slightly shifted, because the marker sits above the
  floor. Raising the corner markers to the same height removes the error.
- Latency from camera to rover is about 150–250 ms, so speeds are kept moderate.
