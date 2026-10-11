"""Dependency-free WAYFINDER gateway for locked-down Windows machines."""
import json, math, os, re, threading, time, urllib.request
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from datetime import datetime

STATE = {"connected": False, "mode": "IDLE", "position": None, "heading": None, "sonar": {"left": None, "front": None, "right": None}, "points": [], "events": [], "plan": None, "reset_token": 0}
MAP_POINTS = {}
LOCK = threading.Lock()

def add_event(kind, message):
    item = {"time": datetime.now().strftime("%H:%M:%S"), "kind": kind, "message": message}
    with LOCK: STATE["events"] = [item] + STATE["events"][:39]
    return item

def plan(action, distance_cm=15):
    return {"action":action,"distance_cm":distance_cm,"avoid_obstacles":True,"requires_confirmation":False}

def fallback(text):
    """Map plain text to an action by whole words, so 'latest' is not 'test' and "don't move" is not 'forward'."""
    s = text.lower().strip()
    words = set(re.findall(r"[a-z0-9-]+", s))
    if words & {"stop", "halt", "pause"}: return plan("stop", 0)
    if "wifi only" in s or "wi-fi only" in s or "signal only" in s: return dict(plan("wifi_hunt"), avoid_obstacles=False)
    # The one-second calibration runs must be asked for exactly, never by accident.
    if s in ("spin", "1 s spin"): return plan("spin_1s", 0)
    if s in ("test", "1 s run"): return plan("forward_1s", 0)
    if words & {"find", "hunt", "hotspot", "explore", "search", "come"}: return plan("explore")
    if words & {"back", "backward", "backwards", "reverse"}: return plan("backward")
    if "left" in words: return plan("left")
    if "right" in words: return plan("right")
    if words & {"forward", "forwards", "ahead"}: return plan("forward")
    if words & {"scan", "look"}: return plan("scan", 0)
    return {"action":"unknown"}  # Unrecognised text must not start the rover driving on its own.

def make_plan(text):
    """The rule parser decides first, so Stop and the console buttons never wait on a network call."""
    known = fallback(text)
    key = os.getenv("DEEPSEEK_API_KEY")
    if known["action"] != "unknown" or not key: return known
    body = {"model":os.getenv("DEEPSEEK_MODEL","deepseek-chat"),"messages":[{"role":"system","content":"Return JSON only. Allowed action: scan, explore, wifi_hunt, forward, backward, left, right, stop. Schema: {action:string,distance_cm:number,avoid_obstacles:boolean,requires_confirmation:boolean}."},{"role":"user","content":text}],"response_format":{"type":"json_object"},"temperature":0}
    req = urllib.request.Request("https://api.deepseek.com/chat/completions", data=json.dumps(body).encode(), headers={"Authorization":"Bearer "+key,"Content-Type":"application/json"})
    with urllib.request.urlopen(req, timeout=8) as res: answer = json.loads(json.loads(res.read())["choices"][0]["message"]["content"])
    if isinstance(answer, dict): answer["action"] = str(answer.get("action", "")).lower()
    return answer

TUNING_KEYS = ("drive_duty", "step_ms", "settle_ms", "turn_deg", "hunt_turn_deg", "turn_rate", "safe_cm", "left_trim_pct", "target_rssi")
ACTIONS = {"stop", "scan", "explore", "wifi_hunt", "forward", "backward", "left", "right", "forward_1s", "spin_1s"}
ONLINE_SECONDS = 5.0  # The rover reports a few times a second; silence this long means it is gone.
LAST_SEEN = 0.0
CAMERA_SECONDS = 1.5  # A camera pose older than this is no longer trusted.
CAMERA_POSE = None    # {"x", "y", "heading", "at"} from camera_tracker.py

def camera_pose():
    """The overhead camera's view of the rover, if it has seen it just now."""
    if CAMERA_POSE and time.time() - CAMERA_POSE["at"] < CAMERA_SECONDS: return CAMERA_POSE
    return None

def new_command_id():
    """A fresh id per command lets the rover act on the same action twice in a row."""
    return int(time.time() * 1000) % 2000000000

def device_payload():
    """What a device needs to know: the pending command, live tuning and the reset token."""
    payload = {"plan": STATE.get("plan"), "command_id": STATE.get("command_id", 0), "tuning": STATE.get("tuning", {}), "reset_token": STATE.get("reset_token", 0), "requested_profile": STATE.get("requested_profile", "sonar-room")}
    # With a camera watching, the rover adopts its pose, so its own estimate
    # carries on in the same frame whenever the camera loses sight of it.
    seen = camera_pose()
    if seen: payload.update({"ext_x": seen["x"], "ext_y": seen["y"], "ext_heading": seen["heading"]})
    return payload

def clear_ray(x, y, heading, clear_cm):
    """Drop earlier returns that the sonar now sees straight through.

    Dead reckoning drifts and people walk past, so old points go stale. A new
    reading proves the strip between the rover and its echo is empty. The strip
    is kept narrower than the real sonar beam so only clear cases are removed.
    """
    limit = clear_cm / 100.0
    ux, uy = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    for key in list(MAP_POINTS):
        dx, dy = key[0] - x, key[1] - y
        if 0.05 < dx * ux + dy * uy < limit and abs(dx * uy - dy * ux) < 0.06: del MAP_POINTS[key]

def add_point(x, y):
    """Record a return unless one is already within 4 cm, so a wall is not a smear of duplicates."""
    if any(abs(px - x) < 0.04 and abs(py - y) < 0.04 for px, py in MAP_POINTS): return
    MAP_POINTS[(x, y)] = {"x": x, "y": y}
    while len(MAP_POINTS) > 1500: del MAP_POINTS[next(iter(MAP_POINTS))]

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_): pass
    def send_json(self, value, status=200):
        data = json.dumps(value).encode(); self.send_response(status); self.send_header("Content-Type","application/json"); self.send_header("Access-Control-Allow-Origin","*"); self.send_header("Content-Length",str(len(data))); self.end_headers(); self.wfile.write(data)
    def take_telemetry(self, data):
        """Fold one device report into the state and the map. Called with LOCK held."""
        global LAST_SEEN
        received_points = data.pop("points", [])
        # A device that has not yet heard about the latest reset still reports in
        # the old map's frame; take its vitals but not its pose or its sonar return.
        device_token = data.pop("reset_token", None)
        old_frame = device_token is not None and device_token != STATE.get("reset_token", 0)
        if old_frame: data.pop("position", None); data.pop("heading", None)
        # A command that sat waiting while the rover was away must not run when it comes back.
        if LAST_SEEN and time.time() - LAST_SEEN > ONLINE_SECONDS and (STATE.get("plan") or {}).get("action") not in (None, "stop"):
            STATE["plan"] = plan("stop", 0)
            STATE["command_id"] = new_command_id()
        STATE.update(data)
        STATE["connected"] = True
        LAST_SEEN = time.time()
        scan = data.get("scan") or {}
        position = data.get("position") or {}
        heading = data.get("heading")
        seen = camera_pose()
        if seen and data.get("source") == "uno-r4-wifi":
            # Map this sonar return from where the camera saw the rover, not where the rover guesses it is.
            position, heading = {"x": seen["x"], "y": seen["y"]}, seen["heading"]
        hit = scan.get("hit") or {}
        has_pose = not old_frame and all(isinstance(v, (int, float)) and math.isfinite(v) for v in (position.get("x"), position.get("y"), heading))
        if has_pose and scan.get("clear_cm"):
            clear_ray(float(position["x"]), float(position["y"]), float(heading) + float(scan.get("relative_angle", 0)), float(scan["clear_cm"]))
        if hit.get("x") is not None and hit.get("y") is not None:
            add_point(round(float(hit["x"]), 2), round(float(hit["y"]), 2))
        elif scan.get("valid_return", True) and scan.get("distance_cm") is not None and has_pose:
            bearing = math.radians(float(heading) + float(scan.get("relative_angle", 0)))
            distance = float(scan["distance_cm"]) / 100.0
            add_point(round(float(position["x"]) + distance * math.cos(bearing), 2), round(float(position["y"]) + distance * math.sin(bearing), 2))
        for point in received_points:
            if point.get("x") is not None and point.get("y") is not None:
                MAP_POINTS[(round(point["x"], 2), round(point["y"], 2))] = point
        STATE["points"] = list(MAP_POINTS.values())[-600:]
        # The reply carries the pending command, so a device needs one round trip, not two.
        return dict(device_payload(), ok=True)

    def do_OPTIONS(self): self.send_response(204); self.send_header("Access-Control-Allow-Origin","*"); self.send_header("Access-Control-Allow-Headers","Content-Type"); self.end_headers()
    def do_GET(self):
        if self.path == "/api/health": self.send_json({"ok":True,"deepseek":bool(os.getenv("DEEPSEEK_API_KEY")),"connected":STATE["connected"] and time.time() - LAST_SEEN < ONLINE_SECONDS})
        elif self.path == "/api/state":
            with LOCK:
                state = STATE.copy()
                state["connected"] = state["connected"] and time.time() - LAST_SEEN < ONLINE_SECONDS
                state["telemetry_age_s"] = round(time.time() - LAST_SEEN, 1) if LAST_SEEN else None
                seen = camera_pose()
                state["pose_source"] = "camera" if seen else "estimate"
                if seen: state["position"] = {"x": seen["x"], "y": seen["y"]}; state["heading"] = seen["heading"]
            self.send_json(state)
        elif self.path == "/api/device/command":
            with LOCK: self.send_json(device_payload())
        else: self.send_json({"error":"not found"},404)
    def do_POST(self):
        global LAST_SEEN, CAMERA_POSE
        try:
            length = int(self.headers.get("Content-Length",0)); data = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(data, dict): raise ValueError
        except ValueError:
            self.send_json({"error":"bad json"},400); return
        if self.path == "/api/command":
            text = str(data.get("text",""))
            try: chosen = make_plan(text)
            except Exception: chosen = {"action":"unknown"}  # The language model is optional.
            if not isinstance(chosen, dict) or chosen.get("action") not in ACTIONS:
                add_event("ERROR", "Not understood: " + text)
                self.send_json({"plan":{"action":"unknown"},"accepted":False,"requires_confirmation":False}); return
            add_event("COMMAND", text)
            with LOCK: STATE["plan"] = chosen; STATE["command_id"] = new_command_id()
            self.send_json({"plan":chosen,"accepted":True,"requires_confirmation":False})
        elif self.path == "/api/pose":
            try: pose = {"x": float(data["x"]), "y": float(data["y"]), "heading": float(data["heading"]) % 360.0, "at": time.time()}
            except (KeyError, TypeError, ValueError): self.send_json({"error":"need x, y, heading"},400); return
            with LOCK: CAMERA_POSE = pose
            self.send_json({"ok": True})
        elif self.path == "/api/tuning":
            # Live rover settings from the console sliders; the rover reads them with each command poll.
            with LOCK:
                tuning = dict(STATE.get("tuning") or {})
                for key in TUNING_KEYS:
                    if isinstance(data.get(key), (int, float)): tuning[key] = data[key]
                STATE["tuning"] = tuning
            self.send_json({"ok": True, "tuning": tuning})
        elif self.path == "/api/reset":
            with LOCK:
                MAP_POINTS.clear()
                profile = data.get("profile") if data.get("profile") in {"sonar-room", "wifi-open-room"} else "sonar-room"
                # Start from a clean state so nothing a previous device reported (simulator walls, a target marker) lingers.
                kept = {key: STATE[key] for key in ("events", "tuning") if key in STATE}
                STATE.clear()
                STATE.update({"connected": False, "mode": "IDLE", "position": None, "heading": None, "sonar": {"left": None, "front": None, "right": None}, "points": [], "requested_profile": profile, "reset_token": int(time.time() * 1000)}, **kept)
                # A rover in the middle of a hunt must not carry on into the cleared map.
                STATE["plan"] = plan("stop", 0); STATE["command_id"] = new_command_id()
            add_event("SYSTEM", "Mission state and sonar map reset")
            self.send_json({"ok": True})
        elif self.path == "/api/telemetry":
            with LOCK:
                try: reply = self.take_telemetry(data)
                except (AttributeError, KeyError, TypeError, ValueError): reply = None
            if reply is None: self.send_json({"error":"bad telemetry"},400)
            else: self.send_json(reply)
        else: self.send_json({"error":"not found"},404)

if __name__ == "__main__":
    print("WAYFINDER plain gateway listening on http://0.0.0.0:8000")
    ThreadingHTTPServer(("0.0.0.0",8000), Handler).serve_forever()
