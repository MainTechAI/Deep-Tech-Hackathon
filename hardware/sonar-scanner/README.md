# HC-SR04 local scanner for Arduino UNO R4 WiFi

Standalone, stationary scanner using an SG-90 positional servo. It sweeps back
and forth and maintains an array of angle/distance measurements on the Arduino.
A Python viewer shows these measurements as a local polar map and exports JSON.
This does not drive the wheels, localise the robot, or implement SLAM.

## Wiring

| Part | Connection |
| --- | --- |
| SG-90 signal | D10 |
| SG-90 power | Separate regulated 5 V supply sized for the servo's load/stall current |
| SG-90 ground | Supply ground and Arduino GND |
| HC-SR04 VCC | UNO R4 5 V |
| HC-SR04 GND | Arduino GND |
| HC-SR04 TRIG | D11 |
| HC-SR04 ECHO | **D2**, not D12 in the old repository pin plan |

D2 is reserved for the echo interrupt. Do not also connect a wheel driver there.
The UNO R4 main GPIO is 5 V. This wiring is NOT directly suitable for a 3.3 V
ESP32 Echo input. Never put the series 18650 battery voltage on servo/sensor 5 V
inputs. With USB powering the Arduino, connect external servo ground to Arduino
ground, but do not join the external positive rail to Arduino 5 V.

Keep the rover stationary and its wheel drive disabled. A scanner pause command
does not stop wheel motors. The map origin is the sensor, not the chassis centre.
Mount the sensor near the servo pivot and keep the bracket/wires out of its view.

## Upload

1. Install Arduino IDE and the **Arduino UNO R4 Boards** package.
2. Install **Servo by Arduino** using Library Manager (Renesas/UNO R4 support).
3. Open `DistanceScanner/DistanceScanner.ino`.
4. Select **Arduino UNO R4 WiFi**, select the connected port and upload.
5. Open Serial Monitor at 115200 baud to see JSON records. Close it before
   running the viewer: only one application can own the serial port.

If Arduino CLI is available:

```sh
arduino-cli core install arduino:renesas_uno
arduino-cli lib install Servo
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi DistanceScanner
arduino-cli upload --fqbn arduino:renesas_uno:unor4wifi --port YOUR_PORT DistanceScanner
```

## Servo calibration and full 180-degree sweep

Initial settings are **10 -> 170 -> 10 degrees**, in 10-degree increments.
After checking clearance and mechanical limits, change these constants:

```cpp
constexpr int SCAN_MIN_DEG = 0;
constexpr int SCAN_MAX_DEG = 180;
```

This produces **0 -> 180 -> 0 repeatedly**, sampling each endpoint only once.
It steps, settles, measures, then moves. Do not measure during continuous
rotation: the angle at the time of the echo would be uncertain.

`SERVO_MIN_US=1000` and `SERVO_MAX_US=2000` are deliberately conservative initial
pulse endpoints. They may cover LESS than 180 physical degrees on your SG-90.
With the horn/bracket unloaded, verify physical centre and endpoints and calibrate
these pulse values gradually using your actual servo's specifications. Do not
force a hard stop. The viewer assumes nominal commanded angles: until calibrated,
its geometry is qualitative. Centre the horn so commanded 90 points forward.
If physical left/right is reversed, change `MIRROR_SCAN` to `true`.

## Run the viewer

Python 3.9+ is recommended. Use a Python installation with a desktop GUI backend.

```sh
python -m pip install -r requirements.txt
python viewer.py
```

With no arguments it lists serial ports. Then choose the board:

```sh
python viewer.py --port COM5
# macOS example; use the actual port reported above:
python viewer.py --port /dev/cu.usbmodemXXXX
# Linux example:
python viewer.py --port /dev/ttyACM0
```

Use `python3` in place of `python` if required. Linux may require serial-port
group permissions. Close other serial programs. To inspect the viewer without
hardware: `python viewer.py --demo`. Demo data is explicitly synthetic.

Buttons send `p` (pause), `r` (resume and clear), or `c` (clear measurements).
Export writes `local_scan.json` in the current working directory. Keyboard
commands can also be sent from Serial Monitor. Pause holds the sensor angle.
Resume clears observations because the rover may have been moved while paused.
On connecting midway through a scan, use Resume + clear to start a fresh map.

## What is actually mapped

The Arduino keeps `localMap[BIN_COUNT]`: distance in millimetres, measurement
time, count of valid samples, and separate measured/valid flags for every angle.
Each angle is overwritten on its next visit. This is a rolling scan, not an
instantaneous snapshot. Use the timestamps if consuming it for another task.

At each angle the sketch takes three pings, spaced at least 65 ms apart. It
requires at least two valid returns. With three it uses the median; with two it
uses the nearer return. Missing echoes, stuck-high Echo, and out-of-range readings
overwrite that bin as invalid. No-echo never means free space. Usable return
range defaults to 2-250 cm; near the minimum range, readings can be unreliable.

The sketch waits 160 ms after each 10-degree movement and 800 ms at startup.
Expect roughly 5-7 seconds per one-way sweep, depending on range, timeouts and
sweep limits. This is deliberately a stable mapping baseline, NOT a fast moving
robot's collision monitor. More samples improve consistency but increase latency.
Increase settling if the loaded servo is still moving. If adjusting ping spacing,
respect the sensor manufacturer's recommendation (>60 ms for this HC-SR04).

The viewer shows approximate return locations and illustrative 15-degree arcs.
The arc is a beam illustration, not a calibrated uncertainty interval. It does
not fill areas as guaranteed free. Invalid echoes are crosses on the outer ring
(NOT obstacles at that range). Unobserved angles are blank. Measurements older
than 15 seconds since receipt turn grey; adjust using `--stale-seconds`.

Coordinate convention: bearing 0=forward, +90=right, -90=left.
`x_cm=d_cm*sin(bearing)`, `y_cm=d_cm*cos(bearing)`.
One sonar return cannot precisely locate an object within its beam. Specular
reflections, soft targets and narrow objects may cause misses. Do not interpret
the distance between isolated returns as a verified navigable gap. Clear the
map after moving/rotating the rover. An actual persistent map needs localisation.

## Firmware scheduling / protocol

Echo edges are captured by an interrupt on D2; no `pulseIn()` or millisecond
`delay()` is used. The trigger pulse uses one intentional 10 microsecond wait.
Echo waiting times out after 25 ms. Timers use unsigned subtraction for rollover.
Telemetry is sent in available-buffer chunks and may be dropped if the host
cannot keep up; sensing and the on-board map continue without USB connected.
Do not disable interrupts for long periods if integrating this with other code.

One JSON object per line, 115200 baud. Point example:

```json
{"type":"point","epoch":1,"sweep":0,"angle":90,"bearing":0,"mm":653,"valid":true,"samples":3,"ms":1532}
```

`mm=0,valid=false` means unknown, not a zero-distance obstacle. `ms` is the
Arduino millis counter and wraps. `epoch` changes when the map is cleared.
`sweep` increments at reversals. Reset records provide scan limits and range;
status records provide pause state. A reboot starts a new mapping session.

## Validation and limitations

This deliverable is hardware-untested. Arduino CLI / the actual Renesas toolchain
were not available in the authoring environment. A host C++ harness checks the
sketch's syntax and state logic with mocked Arduino APIs, not board peripherals.
The Python parser/geometry and synthetic rendering were also checked.

Before rover integration, verify a flat board at known distances, confirm
left/right orientation, unplug/disconnect sensor power to verify invalid returns,
and confirm the servo settles at every stop. Avoid moving the chassis during the
scan. No assumptions about actual wheel servo model are made here.

## Resources used / hackathon disclosure

- Arduino Servo library (dependency; core scan/map logic is provided here).
- Arduino interrupt reference: https://docs.arduino.cc/language-reference/en/functions/external-interrupts/attachInterrupt/
- UNO R4 documentation: https://docs.arduino.cc/hardware/uno-r4-wifi/
- HC-SR04 datasheet: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
- pyserial and Matplotlib (viewer dependencies).
- Code generated with ChatGPT during the hackathon; disclose according to event rules.
