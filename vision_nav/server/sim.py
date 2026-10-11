"""Simulated rover + simulated overhead camera, so the whole pipeline can be tested
without hardware.

  * robot: connects to the server's TCP port exactly like the firmware does, obeys
    D/S/B commands with the same timeout and front-safety rules, imperfect motors
    (left wheel weaker, dead zone, command latency), sonar ray-cast with noise.
  * camera: projects the ArUco marker corners through an oblique (top-side) virtual
    camera, adds pixel noise and dropped detections, sends frames over the WebSocket.

  python sim.py [--server http://localhost:8080] [--person] [--marker-rot 0]
"""
import argparse
import asyncio
import math
import random
import time

import aiohttp

import config as C
import geometry as G

W, H = C.ARENA_W, C.ARENA_H
OBSTACLES = [            # x0, y0, x1, y1 (cm) - boxes the camera doesn't know about
    (130, 60, 160, 140),
    (220, 0, 235, 90),
]
ROOM = (-60, -60, W + 60, H + 60)   # real walls beyond the arena markers
MAX_SPEED = 30.0         # cm/s at 100 %
WHEEL_BASE = 13.0
LEFT_GAIN, RIGHT_GAIN = 0.88, 1.0
DEAD_ZONE = 12           # % below which a wheel doesn't turn
LATENCY = 0.08
REVERSED_TURN = False
CAM_LATENCY = 0.35       # phone camera pipeline delay: frames show the past    # --reversed-turn: the robot turns the opposite way (mirrored arena)
MARKER = 10.0            # robot marker side, cm
IMG_W, IMG_H = 1280, 720
# floor corners -> image: camera stands behind the near (y=0) edge, looking down at ~45 deg
CAM = G.homography([(0, 0), (W, 0), (W, H), (0, H)],
                   [(170, 660), (1110, 660), (930, 170), (350, 170)])


class Rover:
    def __init__(self):
        self.x, self.y, self.th = 40.0, 40.0, 30.0
        self.cmd = (0, 0)
        self.deadline = 0.0
        self.queue = []          # (apply_at, l, r, ms)
        self.out = (0, 0)
        self.blocked = False
        self.dist = 999.0
        self.mode = "MANUAL"
        self.bumps = 0
        self.history = []        # (t, x, y, th) for the delayed camera

    def seen_pose(self):
        t = time.monotonic() - CAM_LATENCY
        for ht, x, y, th in reversed(self.history):
            if ht <= t:
                return x, y, th
        return self.x, self.y, self.th

    def command(self, l, r, ms):
        self.queue.append((time.monotonic() + LATENCY, l, r, ms))

    def sonar(self):
        """Nearest echo inside a ~15 deg cone, like the HC-SR04."""
        a0 = math.radians(self.th)
        sx = self.x + C.SENSOR_FORWARD * math.cos(a0)
        sy = self.y + C.SENSOR_FORWARD * math.sin(a0)
        best = 999.0
        for da in (-7, 0, 7):
            a = math.radians(self.th + da)
            d = 0.0
            while d < min(best, 300):
                if self._solid(sx + d * math.cos(a), sy + d * math.sin(a)):
                    best = d
                    break
                d += 1.0
        return best if best >= 999 else max(2.0, best + random.uniform(-1, 1))

    def _solid(self, px, py):
        if not (ROOM[0] < px < ROOM[2] and ROOM[1] < py < ROOM[3]):
            return True
        return any(x0 <= px <= x1 and y0 <= py <= y1 for x0, y0, x1, y1 in OBSTACLES)

    def _collides(self, x, y):
        r = C.ROBOT_RADIUS - 3
        return any(self._solid(x + r * math.cos(k * math.pi / 4), y + r * math.sin(k * math.pi / 4))
                   for k in range(8))

    def step(self, dt):
        t = time.monotonic()
        while self.queue and self.queue[0][0] <= t:
            _, l, r, ms = self.queue.pop(0)
            self.cmd = (l, r)
            self.deadline = t + ms / 1000.0
        if t > self.deadline:
            self.cmd = (0, 0)
        self.dist = self.sonar()
        l, r = self.cmd
        # firmware front safety
        self.blocked = l + r > 0 and self.dist < 20
        if self.blocked:
            spin = (l - r) // 2
            l, r = spin, -spin
        self.out = (l, r)

        def wheel(v, gain):
            return 0.0 if abs(v) < DEAD_ZONE else v / 100.0 * MAX_SPEED * gain
        vl, vr = wheel(l, LEFT_GAIN), wheel(r, RIGHT_GAIN)
        v = (vl + vr) / 2
        w = math.degrees((vr - vl) / WHEEL_BASE) * (-1 if REVERSED_TURN else 1)
        nth = G.wrap(self.th + w * dt)
        nx = self.x + v * dt * math.cos(math.radians(nth))
        ny = self.y + v * dt * math.sin(math.radians(nth))
        if self._collides(nx, ny):
            self.bumps += 1
        else:
            self.x, self.y = nx, ny
        self.th = nth
        self.history.append((time.monotonic(), self.x, self.y, self.th))
        self.history = self.history[-200:]


class Person:
    def __init__(self):
        self.t0 = time.monotonic()

    def pos(self):
        t = time.monotonic() - self.t0
        return (W * 0.5 + 90 * math.cos(t * 0.12), H * 0.5 + 60 * math.sin(t * 0.12))


def project(p):
    x, y = G.apply(CAM, p)
    return [x + random.gauss(0, 0.6), y + random.gauss(0, 0.6)]


def square(cx, cy, th, side):
    """Marker corners TL, TR, BR, BL; 'top' edge faces heading th."""
    a = math.radians(th)
    f = (math.cos(a), math.sin(a))
    lft = (-math.sin(a), math.cos(a))
    s = side / 2
    pts = [(s, s), (s, -s), (-s, -s), (-s, s)]      # (forward, left)
    return [(cx + u * f[0] + v * lft[0], cy + u * f[1] + v * lft[1]) for u, v in pts]


async def robot_link(rover, host, port):
    while True:
        try:
            reader, writer = await asyncio.open_connection(host, port)
            writer.write(f"HELLO {C.ROBOT_TOKEN}\n".encode())
            print("[sim robot] connected")

            async def rx():
                while True:
                    line = await reader.readline()
                    if not line:
                        return
                    p = line.decode().split()
                    if p and p[0] == "D":
                        rover.command(int(p[1]), int(p[2]), int(p[3]))
                    elif p and p[0] == "S":
                        rover.command(0, 0, 0)
                    elif p and p[0] == "B":
                        print("[sim robot] BEEP")

            rx_task = asyncio.ensure_future(rx())
            while not rx_task.done():
                l, r = rover.out
                writer.write(f"T {rover.dist:.1f} {l} {r} {rover.mode} -50 {int(rover.blocked)}\n".encode())
                await writer.drain()
                await asyncio.sleep(0.1)
        except (ConnectionError, OSError) as e:
            print("[sim robot] link:", e)
        await asyncio.sleep(2)


async def camera_link(rover, person, url, marker_rot):
    corners_world = {1: (0, 0), 2: (W, 0), 3: (W, H), 4: (0, H)}
    async with aiohttp.ClientSession() as s:
        while True:
            try:
                async with s.ws_connect(url) as ws:
                    await ws.send_json({"type": "hello", "role": "camera", "key": C.CONTROL_KEY})
                    print("[sim camera] connected")

                    async def drain():   # reading keeps the server's heartbeat pings answered
                        async for _ in ws:
                            pass
                    reader = asyncio.ensure_future(drain())
                    while not reader.done():
                        markers = {}
                        for mid, (cx, cy) in corners_world.items():
                            if random.random() > 0.03:
                                markers[str(mid)] = [project(p) for p in square(cx, cy, 90, C.CORNER_MARKER_CM)]
                        sx, sy, sth = rover.seen_pose()
                        in_view = -12 < sx < W + 12 and -12 < sy < H + 12
                        if in_view and random.random() > 0.05:
                            mx = sx + C.MARKER_FORWARD * math.cos(math.radians(sth))
                            my = sy + C.MARKER_FORWARD * math.sin(math.radians(sth))
                            markers["0"] = [project(p) for p in square(mx, my, sth + marker_rot, MARKER)]
                        if person:
                            px, py = person.pos()
                            markers["5"] = [project(p) for p in square(px, py, 90, 15)]
                        await ws.send_json({"type": "frame", "w": IMG_W, "h": IMG_H, "markers": markers})
                        await asyncio.sleep(1 / 12)
                    print("[sim camera] disconnected")
            except (aiohttp.ClientError, ConnectionError, OSError) as e:
                print("[sim camera]", e)
            await asyncio.sleep(2)


async def physics(rover):
    last = time.monotonic()
    shown = 0.0
    while True:
        await asyncio.sleep(0.02)
        t = time.monotonic()
        rover.step(t - last)
        last = t
        if t - shown > 2:
            shown = t
            print(f"[sim] x={rover.x:6.1f} y={rover.y:6.1f} th={rover.th:6.1f} "
                  f"out={rover.out} sonar={rover.dist:5.0f} bumps={rover.bumps}")


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="http://localhost:8080")
    ap.add_argument("--robot-host", default="localhost")
    ap.add_argument("--person", action="store_true", help="simulate a walking person marker")
    ap.add_argument("--arena", default=None, help="real arena size WxH in cm, e.g. 260x180 "
                    "(tests automatic measuring; the server starts with its default)")
    ap.add_argument("--reversed-turn", action="store_true", help="robot turns the other way")
    ap.add_argument("--marker-rot", type=float, default=0,
                    help="marker stuck on rotated by this many degrees (tests calibration)")
    a = ap.parse_args()
    if a.arena:
        global W, H, CAM
        W, H = (float(v) for v in a.arena.lower().split("x"))
        CAM = G.homography([(0, 0), (W, 0), (W, H), (0, H)],
                           [(170, 660), (1110, 660), (930, 170), (350, 170)])
    global REVERSED_TURN
    REVERSED_TURN = a.reversed_turn
    rover = Rover()
    ws_url = a.server.replace("http", "ws", 1).rstrip("/") + "/ws"
    await asyncio.gather(physics(rover), robot_link(rover, a.robot_host, C.ROBOT_PORT),
                         camera_link(rover, Person() if a.person else None, ws_url, a.marker_rot))


if __name__ == "__main__":
    asyncio.run(main())
