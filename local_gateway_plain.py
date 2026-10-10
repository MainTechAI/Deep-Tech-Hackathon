"""Dependency-free WAYFINDER gateway for locked-down Windows machines."""
import json, os, threading, urllib.request
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from datetime import datetime

STATE = {"connected": False, "mode": "IDLE", "position": None, "heading": None, "sonar": {"left": None, "front": None, "right": None}, "points": [], "events": [], "plan": None, "reset_token": 0}
MAP_POINTS = {}
LOCK = threading.Lock()

def add_event(kind, message):
    item = {"time": datetime.now().strftime("%H:%M:%S"), "kind": kind, "message": message}
    with LOCK: STATE["events"] = [item] + STATE["events"][:9]
    return item

def fallback(text):
    s = text.lower()
    if "stop" in s or "halt" in s: return {"action":"stop","distance_cm":0,"avoid_obstacles":True,"requires_confirmation":False}
    if "wifi only" in s or "wi-fi only" in s or "signal only" in s:
        return {"action":"wifi_hunt","distance_cm":15,"avoid_obstacles":False,"requires_confirmation":False}
    if "test" in s or "calib" in s: return {"action":"forward_1s","distance_cm":0,"avoid_obstacles":True,"requires_confirmation":False}
    if "find" in s or "hunt" in s or "hotspot" in s or "go to" in s or "come" in s: return {"action":"explore","distance_cm":15,"avoid_obstacles":True,"requires_confirmation":False}
    if "scan" in s or "look" in s: return {"action":"scan","distance_cm":0,"avoid_obstacles":True,"requires_confirmation":False}
    if "back" in s or "reverse" in s: action = "backward"
    elif "left" in s: action = "left"
    elif "right" in s: action = "right"
    elif "forward" in s or "move" in s: action = "forward"
    else: action = "explore"
    return {"action":action,"distance_cm":15,"avoid_obstacles":True,"requires_confirmation":action == "explore"}

def make_plan(text):
    key = os.getenv("DEEPSEEK_API_KEY")
    if not key: return fallback(text)
    body = {"model":os.getenv("DEEPSEEK_MODEL","deepseek-chat"),"messages":[{"role":"system","content":"Return JSON only. Allowed action: scan, explore, forward, backward, left, right, stop. Schema: {action:string,distance_cm:number,avoid_obstacles:boolean,requires_confirmation:boolean}."},{"role":"user","content":text}],"response_format":{"type":"json_object"},"temperature":0}
    req = urllib.request.Request("https://api.deepseek.com/chat/completions", data=json.dumps(body).encode(), headers={"Authorization":"Bearer "+key,"Content-Type":"application/json"})
    with urllib.request.urlopen(req, timeout=20) as res: return json.loads(json.loads(res.read())["choices"][0]["message"]["content"])

TUNING_KEYS = ("drive_duty", "step_ms", "settle_ms", "turn_ms", "safe_cm", "left_trim_pct", "target_rssi")

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_): pass
    def send_json(self, value, status=200):
        data = json.dumps(value).encode(); self.send_response(status); self.send_header("Content-Type","application/json"); self.send_header("Access-Control-Allow-Origin","*"); self.send_header("Content-Length",str(len(data))); self.end_headers(); self.wfile.write(data)
    def do_OPTIONS(self): self.send_response(204); self.send_header("Access-Control-Allow-Origin","*"); self.send_header("Access-Control-Allow-Headers","Content-Type"); self.end_headers()
    def do_GET(self):
        if self.path == "/api/health": self.send_json({"ok":True,"deepseek":bool(os.getenv("DEEPSEEK_API_KEY")),"connected":STATE["connected"]})
        elif self.path == "/api/state":
            with LOCK: self.send_json(STATE.copy())
        elif self.path == "/api/device/command":
            self.send_json({"plan": STATE.get("plan"), "command_id": STATE.get("command_id", 0), "tuning": STATE.get("tuning", {}), "reset_token": STATE.get("reset_token", 0), "requested_profile": STATE.get("requested_profile", "sonar-room")})
        else: self.send_json({"error":"not found"},404)
    def do_POST(self):
        length = int(self.headers.get("Content-Length",0)); data = json.loads(self.rfile.read(length) or b"{}")
        if self.path == "/api/command":
            plan = make_plan(data.get("text","")); add_event("COMMAND",data.get("text","")); STATE["plan"] = plan; STATE["command_id"] = int(datetime.now().timestamp() * 1000) % 2000000000; STATE["mode"] = str(plan.get("action", "IDLE")).upper(); self.send_json({"plan":plan,"requires_confirmation":plan.get("requires_confirmation",True)})
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
                STATE.update({"connected": False, "mode": "IDLE", "position": None, "heading": None, "sonar": {"left": None, "front": None, "right": None}, "points": [], "plan": None, "requested_profile": profile, "reset_token": int(datetime.now().timestamp() * 1000)})
            add_event("SYSTEM", "Mission state and sonar map reset")
            self.send_json({"ok": True})
        elif self.path == "/api/telemetry":
            with LOCK:
                received_points = data.pop("points", [])
                STATE.update(data)
                STATE["connected"] = True
                scan = data.get("scan") or {}
                position = data.get("position") or {}
                heading = data.get("heading")
                hit = scan.get("hit") or {}
                if hit.get("x") is not None and hit.get("y") is not None:
                    point = (round(float(hit["x"]), 2), round(float(hit["y"]), 2))
                    MAP_POINTS[point] = {"x": point[0], "y": point[1]}
                elif scan.get("valid_return", True) and scan.get("distance_cm") is not None and position.get("x") is not None and position.get("y") is not None and heading is not None:
                    import math
                    bearing = math.radians(float(heading) + float(scan.get("relative_angle", 0)))
                    distance = float(scan["distance_cm"]) / 100.0
                    point = (round(float(position["x"]) + distance * math.cos(bearing), 2), round(float(position["y"]) + distance * math.sin(bearing), 2))
                    MAP_POINTS[point] = {"x": point[0], "y": point[1]}
                for point in received_points:
                    if point.get("x") is not None and point.get("y") is not None:
                        MAP_POINTS[(round(point["x"], 2), round(point["y"], 2))] = point
                STATE["points"] = list(MAP_POINTS.values())[-600:]
            self.send_json({"ok":True})
        else: self.send_json({"error":"not found"},404)

if __name__ == "__main__":
    print("WAYFINDER plain gateway listening on http://0.0.0.0:8000")
    ThreadingHTTPServer(("0.0.0.0",8000), Handler).serve_forever()
