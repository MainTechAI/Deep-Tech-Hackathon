"""All tunables in one place. Override any of them with environment variables
(or a .env file next to this one), e.g. ROBOT_TOKEN=abc CONTROL_KEY=xyz."""
import os
from pathlib import Path


def _load_env():
    p = Path(__file__).with_name(".env")
    if p.exists():
        for line in p.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                os.environ.setdefault(k.strip(), v.strip())


_load_env()


def _get(name, default, cast=str):
    v = os.environ.get(name)
    return cast(v) if v is not None else default


# ---- network ----
HTTP_HOST = _get("HTTP_HOST", "0.0.0.0")      # 127.0.0.1 behind a reverse proxy
ROBOT_HOST = _get("ROBOT_HOST", "0.0.0.0")
HTTP_PORT = _get("HTTP_PORT", 8080, int)       # put Caddy/nginx with HTTPS in front
ROBOT_PORT = _get("ROBOT_PORT", 9000, int)     # plain TCP, the robot connects here
ROBOT_TOKEN = _get("ROBOT_TOKEN", "dev-robot-token")
CONTROL_KEY = _get("CONTROL_KEY", "dev")       # browsers need ?key=... to control

# ---- markers (ArUco 4x4 dictionary) ----
ROBOT_MARKER = 0                # on top of the robot, its top edge pointing forward
CORNER_MARKERS = (1, 2, 3, 4)   # arena corners: (0,0) (W,0) (W,H) (0,H), counter-clockwise
FOLLOW_MARKER = 5               # held by the person to follow
# printed size of the black square of corner markers: used to measure the arena automatically
CORNER_MARKER_CM = _get("CORNER_MARKER_CM", 13.0, float)

# ---- arena (cm, between corner marker centres) ----
ARENA_W = _get("ARENA_W", 300.0, float)
ARENA_H = _get("ARENA_H", 200.0, float)
GRID_CM = 5.0                   # map cell size

# ---- robot geometry (cm) ----
FENCE_CM = 20.0                 # the robot centre stays this far inside the arena edge
ROBOT_RADIUS = 13.0             # for obstacle inflation: half the diagonal of a 21 x 16 cm robot
STOP_CM = 20.0                  # the firmware's front safety distance
SENSOR_FORWARD = 10.0           # sonar is this far ahead of the marker centre
MARKER_FORWARD = 0.0            # marker centre ahead of the wheel axle centre

# ---- control ----
CONTROL_HZ = 10
POSE_TIMEOUT = 0.5              # s without camera pose -> robot stops
ROBOT_TIMEOUT = 1.5             # s without telemetry -> robot offline
DRIVE_MS = 400                  # every drive command expires after this (robot watchdog)
CRUISE = 55                     # % forward speed
CRUISE_MIN = 35                 # % near the goal
TURN_SPEED = 45                 # % for turning on the spot
TURN_IN_PLACE_DEG = 25          # heading error above which the robot turns in pulses (else: steers gently)
STEER_MAX = 0.4                 # while driving, wheel difference at most this fraction of speed
STEER_GAIN = 0.0                # steering while driving (0 = straight segments only)
EDGE_TURN_DEG = 10              # stricter heading tolerance near the arena edge
ALIGN_DEG = 12                  # after a turn, keep turning until within this
TURN_PULSE_MS = 120             # on-the-spot turns are pulses (turn, stop, look) of at most this
TURN_PIVOT = 1                  # 1: turn around one wheel (smooth), 0: spin on the spot (sharp)
TURN_SETTLE_MS = 350            # after a pulse: wait this long for the camera to see the result
TRIM = 0.0                      # %: + weakens the left wheel (robot pulls right), - the right
MOTION_POSE_AGE = 0.3           # s: with an older camera pose the robot waits instead of driving blind
LOOKAHEAD = 25.0                # cm along the path
GOAL_TOL = 8.0                  # cm
FOLLOW_DIST = 45.0              # cm to keep from the person
OBSTACLE_RANGE = 45.0           # sonar hits closer than this go on the map
MAP_HITS = 3                    # echoes in one cell before it counts as an obstacle
EDGE_IGNORE = 15.0              # cm: echoes this close to the arena edge are people/walls outside, ignored
FREE_RANGE = 80.0               # sonar ray is free space up to this far
REPLAN_S = 0.5
CORNER_SKIP = 15.0              # cm: path corners closer than this are not aimed at

# Settings the console may change live (name: (min, max))
TUNABLE = {
    "CRUISE": (15, 100), "CRUISE_MIN": (10, 100), "TURN_SPEED": (15, 100),
    "STEER_GAIN": (0.0, 3.0), "TURN_IN_PLACE_DEG": (10, 90), "TURN_PULSE_MS": (40, 600),
    "TURN_SETTLE_MS": (0, 1500), "ALIGN_DEG": (2, 30), "ROBOT_RADIUS": (6, 30), "FENCE_CM": (5, 60), "TURN_PIVOT": (0, 1), "OBSTACLE_RANGE": (10, 150), "MAP_HITS": (1, 5),
    "LOOKAHEAD": (5, 80), "GOAL_TOL": (3, 40), "FOLLOW_DIST": (20, 150), "TRIM": (-30, 30),
}
