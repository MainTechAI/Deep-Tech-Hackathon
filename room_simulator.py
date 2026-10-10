"""Local Wayfinder room simulation. Uses only the Python standard library."""
import json
import math
import time
import urllib.error
import urllib.request

API = "http://127.0.0.1:8000"
TICK_SECONDS = 0.18
ROOM = (-4.5, 4.5, -3.2, 3.2)  # min x, max x, min y, max y in metres
OBSTACLES = [
    (-0.9, -2.9, -0.2, 0.65),
    (0.8, -1.65, 2.6, -0.9),
    (1.15, 0.8, 2.5, 1.8),
    (-3.55, 1.35, -2.45, 2.55),
]
PLAYER = {"x": 3.25, "y": 2.10, "label": "PLAYER / IPHONE HOTSPOT"}


def post(path, payload):
    request = urllib.request.Request(
        API + path,
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=1.5) as response:
        return json.loads(response.read() or b"{}")


def get(path):
    with urllib.request.urlopen(API + path, timeout=1.5) as response:
        return json.loads(response.read() or b"{}")


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_delta(current, target):
    return (target - current + 180) % 360 - 180


def point_inside_obstacle(x, y):
    return any(left <= x <= right and bottom <= y <= top for left, bottom, right, top in OBSTACLES)


def blocked(x, y, ignore_obstacles=False):
    left, right, bottom, top = ROOM
    return x < left or x > right or y < bottom or y > top or (not ignore_obstacles and point_inside_obstacle(x, y))


class RoomSimulator:
    def __init__(self):
        self.reset_token = None
        self.started_at = time.monotonic()
        self.reset_pose()

    def reset_pose(self):
        """Restore the rover to its physical starting pose in the test room."""
        self.x, self.y = -3.45, -2.05
        self.heading = 18.0
        self.servo_angle = -62.0
        self.servo_step = 8.0
        self.mode = "SCANNING"
        self.hunt_active = False
        self.wifi_only = False
        self.manual_until = 0.0
        self.manual_direction = None
        self.last_command = ""
        self.last_rssi = None

    def ray_reading(self, heading, limit=4.0):
        """Return a clearance and whether the HC-SR04 actually hit a surface."""
        radians = math.radians(heading)
        # One-centimetre increments make the rendered return sit on the first
        # surface, rather than visibly inside a wall.
        for distance in [step / 100 for step in range(15, int(limit * 100))]:
            x = self.x + math.cos(radians) * distance
            y = self.y + math.sin(radians) * distance
            if blocked(x, y):
                return round(distance * 100, 1), True
        # 400 cm is the simulated sensor maximum, not an obstacle at 400 cm.
        return round(limit * 100, 1), False

    def ray_distance(self, heading, limit=4.0):
        return self.ray_reading(heading, limit)[0]

    def point_at(self, heading, distance_cm):
        distance = distance_cm / 100
        radians = math.radians(heading)
        return {
            "x": round(self.x + math.cos(radians) * distance, 2),
            "y": round(self.y + math.sin(radians) * distance, 2),
        }

    def sonar(self):
        return {
            "left": self.ray_distance(self.heading + 46),
            "front": self.ray_distance(self.heading),
            "right": self.ray_distance(self.heading - 46),
        }

    def signal(self, x=None, y=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        distance = max(0.25, math.hypot(PLAYER["x"] - x, PLAYER["y"] - y))
        # A small deterministic ripple makes the simulator resemble an indoor RSSI field,
        # while the general trend always improves near the player.
        ripple = math.sin(x * 2.1 + y * 1.3) * 0.18
        wifi = round(-42 - 25 * math.log10(distance) + ripple, 1)
        bluetooth = round(-48 - 21 * math.log10(distance) + ripple * 0.45, 1)
        return wifi, bluetooth, distance

    def plan_from_backend(self):
        try:
            packet = get("/api/device/command")
        except (urllib.error.URLError, TimeoutError, ValueError):
            return
        reset_token = packet.get("reset_token")
        if reset_token != self.reset_token:
            self.reset_token = reset_token
            self.reset_pose()
        plan = packet.get("plan") or {}
        signature = json.dumps(plan, sort_keys=True)
        if not plan or signature == self.last_command:
            return
        self.last_command = signature
        action = str(plan.get("action", "")).lower()
        if action == "stop":
            self.hunt_active = False
            self.manual_direction = None
            self.mode = "STOPPED"
        elif action == "scan":
            self.hunt_active = False
            self.manual_direction = None
            self.mode = "SCANNING"
        elif action == "explore":
            self.hunt_active = True
            self.wifi_only = False
            self.manual_direction = None
            self.mode = "SEARCHING"
        elif action == "wifi_hunt":
            self.hunt_active = True
            self.wifi_only = True
            self.manual_direction = None
            self.mode = "WIFI_SEARCHING"
        elif action in {"forward", "backward", "left", "right"}:
            self.hunt_active = False
            self.manual_direction = action
            self.manual_until = time.monotonic() + 0.65
            self.mode = "MANUAL"

    def move(self, direction, speed):
        radians = math.radians(self.heading)
        sign = -1 if direction == "backward" else 1
        candidate_x = self.x + math.cos(radians) * speed * sign
        candidate_y = self.y + math.sin(radians) * speed * sign
        if not blocked(candidate_x, candidate_y, self.wifi_only):
            self.x, self.y = candidate_x, candidate_y
            return True
        return False

    def hunt_step(self, sonar):
        wifi, _, distance = self.signal()
        if distance < 0.58:
            self.hunt_active = False
            self.mode = "TARGET_NEARBY"
            return
        if not self.wifi_only and sonar["front"] < 42:
            self.mode = "AVOIDING"
            self.heading += 58 if sonar["left"] > sonar["right"] else -58
            return
        # Compare safe short moves in a full circle. This represents the rover's
        # stop → sample → turn → move loop, rather than letting it drift while turning.
        candidates = []
        for trial_heading in range(0, 360, 30):
            if not self.wifi_only and self.ray_distance(trial_heading) < 48:
                continue
            radians = math.radians(trial_heading)
            trial_x = self.x + math.cos(radians) * 0.23
            trial_y = self.y + math.sin(radians) * 0.23
            if not blocked(trial_x, trial_y, self.wifi_only):
                score, _, _ = self.signal(trial_x, trial_y)
                candidates.append((score, trial_heading))
        if candidates:
            _, desired_heading = max(candidates)
            turn = angle_delta(self.heading, desired_heading)
            self.heading = (self.heading + clamp(turn, -16, 16)) % 360
            if abs(turn) > 18:
                self.mode = "WIFI_SCANNING" if self.wifi_only else "SCANNING"
                return
        self.mode = "WIFI_SEARCHING" if self.wifi_only else "SEARCHING"
        self.move("forward", 0.065)

    def manual_step(self, sonar):
        if time.monotonic() >= self.manual_until:
            self.manual_direction = None
            self.mode = "SCANNING"
            return
        if self.manual_direction == "left":
            self.heading += 8
        elif self.manual_direction == "right":
            self.heading -= 8
        elif self.manual_direction in {"forward", "backward"}:
            if self.manual_direction == "forward" and sonar["front"] < 42:
                self.mode = "BLOCKED"
                return
            self.move(self.manual_direction, 0.07)

    def telemetry(self, sonar):
        wifi, bluetooth, distance = self.signal()
        trend = "COLLECTING"
        if self.last_rssi is not None:
            trend = "STRONGER" if wifi > self.last_rssi + 0.35 else "WEAKER" if wifi < self.last_rssi - 0.35 else "STABLE"
        self.last_rssi = wifi
        # Keep the angle and hit from the same instant.  The servo advances only
        # after this reading; otherwise a point would be plotted off the wall.
        scan_angle = self.servo_angle
        scan_heading = self.heading + scan_angle
        scan_distance, has_return = self.ray_reading(scan_heading)
        hit = self.point_at(scan_heading, scan_distance) if has_return else None
        self.servo_angle += self.servo_step
        if self.servo_angle >= 65 or self.servo_angle <= -65:
            self.servo_step *= -1
        return {
            "source": "local-room-simulator",
            "simulation_profile": "wifi-open-room" if self.wifi_only else "sonar-room",
            "mode": self.mode,
            "position": {"x": round(self.x, 2), "y": round(self.y, 2)},
            "heading": round(self.heading % 360, 1),
            "sonar": sonar,
            "scan": {
                "relative_angle": round(scan_angle, 1),
                "distance_cm": scan_distance,
                "hit": hit,
                "valid_return": has_return,
            },
            "rssi_dbm": wifi,
            "bluetooth_rssi_dbm": bluetooth,
            "signal_trend": trend,
            "target_ssid": "WAYFINDER_PLAYER",
            "network": "SIMULATED IPHONE HOTSPOT",
            "target_position": {"x": PLAYER["x"], "y": PLAYER["y"]},
            "target_label": PLAYER["label"],
            "target_nearby": distance < 0.58,
            "estimated_target_distance_m": round(distance, 2),
            "room_bounds": {
                "left": ROOM[0], "right": ROOM[1], "bottom": ROOM[2], "top": ROOM[3]
            },
            "room_obstacles": [
                {"left": left, "bottom": bottom, "right": right, "top": top}
                for left, bottom, right, top in OBSTACLES
            ],
        }

    def run(self):
        try:
            post("/api/reset", {})
        except urllib.error.URLError:
            print("Wayfinder backend is offline. Start local_gateway_plain.py first.")
            return
        print("Wayfinder room simulator is running. Press Ctrl+C to stop it.")
        while True:
            started = time.monotonic()
            self.plan_from_backend()
            sonar = self.sonar()
            if self.hunt_active:
                self.hunt_step(sonar)
            elif self.manual_direction:
                self.manual_step(sonar)
            try:
                post("/api/telemetry", self.telemetry(self.sonar()))
            except (urllib.error.URLError, TimeoutError):
                print("Telemetry retry: backend is temporarily unavailable")
            time.sleep(max(0.02, TICK_SECONDS - (time.monotonic() - started)))


if __name__ == "__main__":
    try:
        RoomSimulator().run()
    except KeyboardInterrupt:
        print("\nWayfinder room simulator stopped.")
