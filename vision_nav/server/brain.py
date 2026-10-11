"""World state + mission logic. Pure Python, no networking: app.py feeds it camera
frames and robot telemetry and sends out whatever drive command tick() returns."""
import math
import time

import config as C
import geometry as G
from planner import Grid


def now():
    return time.monotonic()


class Pose:
    def __init__(self):
        self.x = self.y = self.th = 0.0
        self.t = 0.0          # last update, monotonic

    def fresh(self, timeout=C.POSE_TIMEOUT):
        return now() - self.t < timeout

    def update(self, x, y, th, alpha=0.6):
        if not self.fresh(1.0):
            self.x, self.y, self.th = x, y, th
        else:
            self.x += alpha * (x - self.x)
            self.y += alpha * (y - self.y)
            self.th = G.wrap(self.th + alpha * G.wrap(th - self.th))
        self.t = now()

    def dict(self):
        return {"x": round(self.x, 1), "y": round(self.y, 1), "th": round(self.th, 1),
                "age": round(now() - self.t, 2) if self.t else None}


def trimmed(l, r):
    """Straight-line correction for unequal motors (C.TRIM, set by hand or by calibration)."""
    t = C.TRIM
    if t > 0:
        l *= 1 - t / 100
    elif t < 0:
        r *= 1 + t / 100
    return int(round(l)), int(round(r))


class Brain:
    def __init__(self):
        self.grid = Grid(C.ARENA_W, C.ARENA_H)
        self.h_img2floor = None       # homography, image px -> floor cm
        self.h_floor2img = None
        self.cal_t = 0.0
        self.cal_locked = False
        self.img_size = (0, 0)
        self.auto_arena = True        # measure W x H from the corner markers' printed size
        self.size_est = None
        self.size_n = 0
        self.size_done = False        # measured once; re-measuring would clear the map
        self.cam_t = 0.0
        self.cam_fps = 0.0
        self.robot = Pose()           # robot centre (wheel axle)
        self.person = Pose()          # follow marker
        self.tel = {}                 # last telemetry from the robot
        self.tel_t = 0.0
        self.home = None
        self.trail = []
        self.mission = None           # None | dict(type, target, ...)
        self.status = "idle"
        self.path = None
        self.plan_t = 0.0
        self.plan_version = -1
        self.joy = (0, 0, 0.0)        # x, y, time
        self.heading_offset = 0.0     # deg, set by calibration if the marker is rotated
        self.turn_sign = 1            # -1 if the robot spins the other way than expected
        self.calib = None             # running auto-calibration state
        self.progress = (0.0, 0.0, 0.0)   # (t, x, y) checkpoint for stuck detection
        self.backoff_until = 0.0
        self.turn_state = None        # running turn pulse: th0, dir, t_end, until
        self.wrong_turns = 0
        self.events = []              # messages for the console log

    # ------------------------------------------------------------ inputs
    def log(self, msg):
        self.events.append(msg)
        self.events = self.events[-30:]
        print("[brain]", msg)

    def set_arena(self, w, h, auto=False):
        if not auto:
            self.auto_arena = False   # typed in by hand: stop measuring
        self.grid.resize(w, h)
        self.cal_locked = False
        self.h_img2floor = None
        self.path = None
        self.log(f"arena {w:.0f} x {h:.0f} cm")

    def camera_frame(self, w, h, markers):
        """markers: {id: [[x,y]*4]} in image pixels."""
        t = now()
        if self.cam_t:
            dt = t - self.cam_t
            if dt > 0:
                self.cam_fps = 0.8 * self.cam_fps + 0.2 / dt
        self.cam_t = t
        self.img_size = (w, h)

        corners = [markers.get(i) for i in C.CORNER_MARKERS]
        if not self.cal_locked and all(corners) and self.auto_arena and not self.size_done:
            self._measure_arena(corners)
        if not self.cal_locked and all(corners):
            W, H = self.grid.w_cm, self.grid.h_cm
            src = [G.centre(c) for c in corners]
            dst = [(0, 0), (W, 0), (W, H), (0, H)]
            try:
                hm = G.homography(src, dst)
                self.h_img2floor = hm
                self.h_floor2img = G.invert(hm)
                self.cal_t = t
            except ValueError:
                pass
        if self.h_img2floor is None:
            return

        if C.ROBOT_MARKER in markers:
            x, y, th = G.marker_pose(self.h_img2floor, markers[C.ROBOT_MARKER])
            th = G.wrap(th + self.heading_offset)
            # marker centre -> axle centre
            x -= C.MARKER_FORWARD * math.cos(math.radians(th))
            y -= C.MARKER_FORWARD * math.sin(math.radians(th))
            self.robot.update(x, y, th)
            if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 4:
                self.trail.append((round(x, 1), round(y, 1)))
                self.trail = self.trail[-600:]
            if self.home is None:
                self.home = (x, y)
                self.log("home set to the robot's first seen position")
        if C.FOLLOW_MARKER in markers:
            x, y, th = G.marker_pose(self.h_img2floor, markers[C.FOLLOW_MARKER])
            self.person.update(x, y, th, alpha=0.4)

    def _measure_arena(self, corners):
        try:
            est = G.arena_size(corners, C.CORNER_MARKER_CM)
        except ValueError:
            return
        if not est or not (30 < est[0] < 2000 and 30 < est[1] < 2000):
            return
        if self.size_est is None:
            self.size_est = est
        else:   # average over many frames, single-pixel noise matters at this scale
            a = 0.08
            self.size_est = (self.size_est[0] + a * (est[0] - self.size_est[0]),
                             self.size_est[1] + a * (est[1] - self.size_est[1]))
        self.size_n += 1
        w, h = self.size_est
        if self.size_n >= 40:          # ~3 s of frames averaged: apply once and stop
            self.size_done = True
            if abs(w - self.grid.w_cm) > 2 or abs(h - self.grid.h_cm) > 2:
                self.set_arena(round(w), round(h), auto=True)
            self.log(f"arena measured: {w:.0f} x {h:.0f} cm")

    def cmd_arena_auto(self):
        self.auto_arena = True
        self.size_est = None
        self.size_n = 0
        self.size_done = False
        self.cal_locked = False
        self.log(f"measuring the arena (corner markers = {C.CORNER_MARKER_CM:g} cm)")

    def telemetry(self, d):
        self.tel = d
        self.tel_t = now()
        if not self.robot.fresh():
            return
        # Sonar -> map. Points are along the robot's heading from the sensor.
        a = math.radians(self.robot.th)
        sx = self.robot.x + (C.SENSOR_FORWARD + C.MARKER_FORWARD) * math.cos(a)
        sy = self.robot.y + (C.SENSOR_FORWARD + C.MARKER_FORWARD) * math.sin(a)
        dist = d.get("dist", 999)
        free_to = min(dist - 5, C.FREE_RANGE) if dist < 999 else C.FREE_RANGE
        if free_to > 0:
            self.grid.add_free_ray(sx, sy, sx + free_to * math.cos(a), sy + free_to * math.sin(a))
        turning = abs(d.get("l", 0) - d.get("r", 0)) > 30   # the cone sweeps everything around
        if dist < C.OBSTACLE_RANGE and not turning:
            if self.grid.add_hit(sx + dist * math.cos(a), sy + dist * math.sin(a)):
                self.log(f"obstacle mapped at {dist:.0f} cm ahead")

    def robot_online(self):
        return now() - self.tel_t < C.ROBOT_TIMEOUT

    # ------------------------------------------------------------ commands
    def cmd_goto(self, x, y):
        self.mission = {"type": "goto", "target": (x, y)}
        self.status = "planning"
        self.path = None
        self.log(f"go to ({x:.0f}, {y:.0f})")

    def cmd_follow(self):
        self.mission = {"type": "follow", "target": None}
        self.path = None
        self.log("follow the person marker")

    def cmd_home(self):
        if self.home is None:
            self.log("no home yet")
            return
        self.mission = {"type": "home", "target": self.home}
        self.path = None
        self.log("returning home")

    def cmd_set_home(self, x=None, y=None):
        if x is not None:
            self.home = (x, y)
        elif self.robot.fresh():
            self.home = (self.robot.x, self.robot.y)
        else:
            return
        self.log(f"home set to ({self.home[0]:.0f}, {self.home[1]:.0f})")

    def cmd_stop(self):
        self.mission = None
        self.calib = None
        self.path = None
        self.status = "idle"
        self.joy = (0, 0, 0.0)

    def cmd_joy(self, x, y):
        if self.mission or self.calib:
            self.cmd_stop()
        self.joy = (x, y, now())

    def cmd_calibrate(self):
        if not self.robot.fresh():
            self.log("calibration needs the robot visible to the camera")
            return
        self.cmd_stop()
        self.calib = {"step": "spin", "t0": now(), "start": (self.robot.x, self.robot.y, self.robot.th)}
        self.log("calibrating: spin right, then drive forward")

    # ------------------------------------------------------------ control
    def tick(self):
        """Called CONTROL_HZ times a second. Returns a command for the robot:
        ("drive", l, r, ms) | ("stop",) | None (send nothing)."""
        t = now()
        jx, jy, jt = self.joy
        if t - jt < 0.6:
            return ("drive", int(jy + jx), int(jy - jx), 600)

        if self.calib:
            return self._calibrate_step()
        if not self.mission:
            return None
        if not self.robot_online():
            self.status = "robot offline"
            return None
        if not self.robot.fresh():
            self.status = "robot not visible"
            return ("stop",)
        if now() - self.robot.t > C.MOTION_POSE_AGE:
            self.status = "waiting for camera"     # slow camera: stop and look, don't drive blind
            return ("stop",)

        target = self._mission_target()
        if target is None:
            return ("stop",)
        if target == "hold":       # next to the person: just turn to face them
            self.status = "with the person"
            want = math.degrees(math.atan2(self.person.y - self.robot.y, self.person.x - self.robot.x))
            err = G.wrap(want - self.robot.th)
            if abs(err) < 20:
                return ("stop",)
            s = C.TURN_SPEED * self.turn_sign
            return ("drive", -s, s, 250) if err > 0 else ("drive", s, -s, 250)

        dist_goal = math.hypot(target[0] - self.robot.x, target[1] - self.robot.y)
        if dist_goal < C.GOAL_TOL and self.mission["type"] != "follow":
            self.log(f"{self.mission['type']}: arrived")
            self.mission = None
            self.path = None
            self.status = "arrived"
            return ("arrived",)

        # (re)plan when the map changed, the target moved, or periodically
        if (self.path is None or self.plan_version != self.grid.version
                or t - self.plan_t > C.REPLAN_S):
            self.path = self.grid.plan((self.robot.x, self.robot.y), target)
            self.plan_t = t
            self.plan_version = self.grid.version
            if self.path is None:
                self.status = "no path"
                return ("stop",)

        # blocked by the robot's own front safety: back off, the map has the obstacle now
        if t < self.backoff_until or self.tel.get("blocked"):
            self.status = "backing off"
            return ("drive", -45, -45, 300)

        cmd = self._follow_path(dist_goal)
        self._check_stuck(t, cmd)
        return cmd

    def _check_stuck(self, t, cmd):
        """Driving forward but the camera sees no movement: we hit something the sonar
        can't see (e.g. a box corner beside the robot). Map it and back off."""
        driving = bool(cmd) and cmd[0] == "drive" and cmd[1] + cmd[2] > 40
        pt, px, py = self.progress
        if not driving:
            self.progress = (t, self.robot.x, self.robot.y)
            return
        if t - pt < 1.5:
            return
        if math.hypot(self.robot.x - px, self.robot.y - py) < 3:
            for da in (-35, 0, 35):
                a = math.radians(self.robot.th + da)
                r = C.ROBOT_RADIUS + 3
                for _ in range(C.MAP_HITS):
                    self.grid.add_hit(self.robot.x + r * math.cos(a), self.robot.y + r * math.sin(a))
            self.backoff_until = t + 0.6
            self.log("stuck - marked an obstacle in front, backing off")
        self.progress = (t, self.robot.x, self.robot.y)

    def _mission_target(self):
        m = self.mission
        if m["type"] in ("goto", "home"):
            return m["target"]
        # follow: a point FOLLOW_DIST before the person, on the line to the robot
        if not self.person.fresh(1.0):
            self.status = "person not visible"
            return None
        dx, dy = self.robot.x - self.person.x, self.robot.y - self.person.y
        d = math.hypot(dx, dy)
        if d < C.FOLLOW_DIST + 10:
            return "hold"
        return (self.person.x + dx / d * C.FOLLOW_DIST, self.person.y + dy / d * C.FOLLOW_DIST)

    def _lookahead(self):
        """Point LOOKAHEAD cm ahead along the path, starting from the closest segment."""
        p = self.path
        rx, ry = self.robot.x, self.robot.y
        best_i, best_d = 0, 1e9
        for i in range(len(p) - 1):
            ax, ay = p[i]; bx, by = p[i + 1]
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy or 1e-9
            u = max(0.0, min(1.0, ((rx - ax) * vx + (ry - ay) * vy) / L2))
            d = math.hypot(ax + u * vx - rx, ay + u * vy - ry)
            if d < best_d:
                best_d, best_i = d, i
        remaining = C.LOOKAHEAD
        cx, cy = rx, ry
        for i in range(best_i + 1, len(p)):
            bx, by = p[i]
            seg = math.hypot(bx - cx, by - cy)
            if seg >= remaining:
                return (cx + (bx - cx) * remaining / seg, cy + (by - cy) * remaining / seg)
            remaining -= seg
            cx, cy = bx, by
        return p[-1]

    def _follow_path(self, dist_goal):
        lx, ly = self._lookahead()
        want = math.degrees(math.atan2(ly - self.robot.y, lx - self.robot.x))
        err = G.wrap(want - self.robot.th)       # + = target is to the left
        if abs(err) > C.TURN_IN_PLACE_DEG:
            return self._turn_pulse(err)
        self.turn_state = None
        self.status = "driving"
        v = C.CRUISE if dist_goal > 40 else max(C.CRUISE_MIN, C.CRUISE * dist_goal / 40)
        w = C.STEER_GAIN * err * self.turn_sign
        return ("drive", int(v - w), int(v + w), C.DRIVE_MS)

    def _turn_pulse(self, err):
        """Turn on the spot in pulses: turn, stop, wait until the (delayed) camera has seen the
        result, then decide again - no overshoot. Each settled pulse also checks the turn
        direction: if the robot keeps turning the wrong way, the sign is flipped automatically
        (e.g. corner markers laid out clockwise, or motors wired the other way)."""
        self.status = "turning"
        t = now()
        ts = self.turn_state
        if ts and t < ts["until"]:
            return None if t < ts["t_end"] else ("stop",)   # pulse running / settling
        if ts:
            dth = G.wrap(self.robot.th - ts["th0"])
            if abs(dth) > 3:
                if (dth > 0) != (ts["dir"] > 0):
                    self.wrong_turns += 1
                    if self.wrong_turns >= 3:
                        self.turn_sign *= -1
                        self.wrong_turns = 0
                        self.log("robot kept turning the wrong way - turn direction flipped")
                else:
                    self.wrong_turns = 0
        d = 1 if err > 0 else -1                     # +1 = turn left (counter-clockwise)
        ms = int(max(60, min(C.TURN_PULSE_MS, abs(err) * 3)))
        self.turn_state = {"th0": self.robot.th, "dir": d, "t_end": t + ms / 1000,
                           "until": t + (ms + C.TURN_SETTLE_MS) / 1000}
        S = int(C.TURN_SPEED)
        left = (d > 0) == (self.turn_sign > 0)       # physical direction
        if C.TURN_PIVOT >= 0.5:                      # around one wheel: half the speed, no jerk
            return ("drive", 0, S, ms) if left else ("drive", S, 0, ms)
        return ("drive", -S, S, ms) if left else ("drive", S, -S, ms)

    def _calibrate_step(self):
        c = self.calib
        t = now() - c["t0"]
        if not self.robot.fresh():
            self.log("calibration aborted: robot not visible")
            self.calib = None
            return ("stop",)
        if c["step"] == "spin":
            if t < 0.6:
                return ("drive", C.TURN_SPEED, -C.TURN_SPEED, C.DRIVE_MS)   # spin right
            if t < 1.2:
                return ("stop",)
            dth = G.wrap(self.robot.th - c["start"][2])
            if abs(dth) < 5:
                self.log("calibration: robot did not turn - check power / MIN_PWM")
                self.calib = None
                return ("stop",)
            # spinning right should decrease the angle (CCW-positive frame)
            self.turn_sign = 1 if dth < 0 else -1
            self.log(f"calibration: spin right gave {dth:+.0f} deg in 0.6 s"
                     + ("" if self.turn_sign == 1 else " (reversed! turn sign flipped)"))
            c.update(step="drive", t0=now(), start=(self.robot.x, self.robot.y, self.robot.th))
            return ("stop",)
        if c["step"] == "drive":
            if t < 0.8:
                return ("drive", C.CRUISE, C.CRUISE, C.DRIVE_MS)
            if t < 1.4:
                return ("stop",)
            x0, y0, th0 = c["start"]
            dx, dy = self.robot.x - x0, self.robot.y - y0
            d = math.hypot(dx, dy)
            if d < 5:
                self.log("calibration: robot did not move forward")
                self.calib = None
                return ("stop",)
            move_dir = math.degrees(math.atan2(dy, dx))
            off = G.wrap(move_dir - th0)
            # the marker may be stuck on rotated: correct the heading by the measured offset
            self.heading_offset = G.wrap(self.heading_offset + off)
            drift = G.wrap(self.robot.th - th0) * self.turn_sign   # + = pulled left
            # ~1.5 % of trim per degree of drift in 0.8 s; drifting right (-) -> weaken the left wheel
            if abs(drift) >= 2:
                C.TRIM = max(-30.0, min(30.0, round(C.TRIM - 1.5 * drift, 1)))
            self.log(f"calibration: {d:.0f} cm in 0.8 s ({d / 0.8:.0f} cm/s), marker offset "
                     f"{off:+.0f} deg (corrected), drift {drift:+.0f} deg -> straight trim {C.TRIM:+.1f} %"
                     + (" (press Calibrate again to refine)" if abs(drift) >= 4 else ""))
            self.calib = None
            return ("stop",)
        return ("stop",)

    # ------------------------------------------------------------ output
    def state(self):
        m = self.mission
        cal_ok = self.h_img2floor is not None
        return {
            "type": "state",
            "arena": {"w": self.grid.w_cm, "h": self.grid.h_cm, "cell": self.grid.cell,
                      "cal": cal_ok, "locked": self.cal_locked, "auto": self.auto_arena,
                      "est": [round(v) for v in self.size_est] if self.size_est else None},
            "camera": {"online": now() - self.cam_t < 1.0, "fps": round(self.cam_fps, 1)},
            "robot": {"online": self.robot_online(), "visible": self.robot.fresh(),
                      "pose": self.robot.dict(), **self.tel},
            "person": {"visible": self.person.fresh(1.0), "pose": self.person.dict()},
            "home": self.home,
            "mission": {"type": m["type"] if m else ("calibrate" if self.calib else None),
                        "target": (m or {}).get("target") if m and m["type"] != "follow"
                        else None, "status": self.status},
            "path": self.path,
            "trail": self.trail[-300:],
            "map_version": self.grid.version,
            "events": self.events[-8:],
            "tune": {k: getattr(C, k) for k in C.TUNABLE},
        }

    def map_cells(self):
        return {"type": "map", "version": self.grid.version, "cell": self.grid.cell,
                "cells": sorted(self.grid.occupied()), "walls": sorted(self.grid.walls)}

    def overlay(self):
        """Path / target / obstacles projected back into the camera image (for the AR view)."""
        if not self.h_floor2img:
            return None
        P = lambda p: [round(v, 1) for v in G.apply(self.h_floor2img, p)]
        W, H = self.grid.w_cm, self.grid.h_cm
        m = self.mission
        tgt = (m or {}).get("target") if m else None
        return {
            "type": "overlay",
            "arena": [P(p) for p in [(0, 0), (W, 0), (W, H), (0, H)]],
            "path": [P(p) for p in self.path] if self.path else [],
            "target": P(tgt) if tgt else None,
            "home": P(self.home) if self.home else None,
            "cells": [P(self.grid.centre_of(i, j)) for (i, j) in self.grid.occupied()][:400],
            "status": self.status,
        }
