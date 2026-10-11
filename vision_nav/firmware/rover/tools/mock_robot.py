#!/usr/bin/env python3
"""Fake robot for testing the control page and API without hardware.

Serves the exact page from web.cpp and imitates the robot's HTTP API, including
front safety, command timeouts and AUTO mode. Distance is simulated: driving
forward brings the "wall" closer, reversing moves it away, spinning finds a new wall.

    python3 tools/mock_robot.py [port]      # default 8000, open http://localhost:8000
"""
import json
import random
import re
import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parent.parent
STOP_CM, SLOW_CM, WARN_CM = 20.0, 50.0, 45.0
AUTO_SPEED, AVOID_CM = 70, 25.0
WEB_TIMEOUT_MS = 1000
CM_PER_SEC_AT_100 = 60.0


def load_page():
    src = (ROOT / "web.cpp").read_text(encoding="utf-8")
    m = re.search(r'R"html\((.*?)\)html"', src, re.S)
    return m.group(1).encode("utf-8")


class Robot:
    def __init__(self):
        self.mode = "MANUAL"
        self.dist = 120.0
        self.target = (0, 0)
        self.out = (0, 0)
        self.blocked = False
        self.cmd_start = 0.0
        self.cmd_ms = 0
        self.last = time.time()
        self.start = time.time()

    def drive(self, l, r, ms=0):
        self.target = (max(-100, min(100, l)), max(-100, min(100, r)))
        self.cmd_start, self.cmd_ms = time.time(), ms
        self.apply()

    def apply(self):
        l, r = self.target
        self.blocked = False
        if l + r > 0:
            if self.dist < STOP_CM:
                spin = (l - r) // 2
                l, r = spin, -spin
                self.blocked = True
            elif self.dist < SLOW_CM:
                k = 0.35 + 0.65 * (self.dist - STOP_CM) / (SLOW_CM - STOP_CM)
                l, r = int(l * k), int(r * k)
        shape = lambda v: 0 if abs(v) < 10 else max(-100, min(100, v))
        self.out = (shape(l), shape(r))

    def set_mode(self, m):
        self.mode = m
        self.drive(0, 0)

    def step(self):
        now = time.time()
        dt, self.last = now - self.last, now
        if self.cmd_ms and (now - self.cmd_start) * 1000 >= self.cmd_ms:
            self.target, self.cmd_ms = (0, 0), 0
        l, r = self.out
        fwd = (l + r) / 2
        self.dist -= fwd / 100 * CM_PER_SEC_AT_100 * dt
        if abs(l - r) > 20:                      # spinning: look at a new wall
            if random.random() < dt * 2:
                self.dist = random.uniform(15, 250)
        self.dist = max(3.0, min(300.0, self.dist + random.uniform(-0.5, 0.5)))
        if self.mode == "AUTO":
            if self.dist > AVOID_CM:
                self.target = (AUTO_SPEED, AUTO_SPEED)
            else:                                # skip the maneuver, just "turn away"
                self.dist = random.uniform(40, 250)
        self.apply()

    def status(self):
        d = self.dist if self.dist < 200 else 999
        return {"m": self.mode, "d": round(d, 1), "b": int(self.blocked),
                "l": self.out[0], "r": self.out[1], "net": "sta", "ip": "127.0.0.1",
                "rssi": random.randint(-60, -45), "up": int((time.time() - self.start) * 1000),
                "sc": int(STOP_CM), "wc": int(WARN_CM),
                "fs": "scan" if self.mode == "FOLLOW" else ""}


robot = Robot()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        num = lambda k, d: int(float(q.get(k, d)))
        robot.step()
        p = u.path
        if p == "/":
            body = load_page()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(body)
            return
        if p == "/m":
            if robot.mode == "MANUAL":
                x, y = num("x", 0), num("y", 0)
                robot.drive(y + x, y - x, WEB_TIMEOUT_MS)
        elif p == "/drive":
            if robot.mode == "MANUAL":
                robot.drive(num("l", 0), num("r", 0), max(1, min(10000, num("ms", WEB_TIMEOUT_MS))))
        elif p == "/stop":
            robot.set_mode("MANUAL")
        elif p == "/mode":
            m = q.get("m")
            robot.set_mode("AUTO" if m == "auto" else "FOLLOW" if m == "follow"
                           else "MANUAL" if m == "manual"
                           else ("AUTO" if robot.mode == "MANUAL" else "MANUAL"))
        elif p == "/beep":
            print(f"BEEP {q.get('ms', 150)} ms")
        elif p != "/status":
            self.send_response(404)
            self.end_headers()
            return
        body = json.dumps(robot.status()).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        if not self.path.startswith(("/status", "/m?")):
            super().log_message(fmt, *args)


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    print(f"Mock robot on http://localhost:{port}")
    HTTPServer(("0.0.0.0", port), Handler).serve_forever()
