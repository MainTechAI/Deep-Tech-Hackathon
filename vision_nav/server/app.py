"""Vision-nav server.

  robot  --TCP :9000-->  server  <--WebSocket /ws--  browsers (camera phone, consoles)

Robot protocol (one line per message):
  robot -> server   HELLO <token>
                    T <dist_cm> <left%> <right%> <mode> <rssi> <blocked>
  server -> robot   D <left%> <right%> <ms>      drive, expires after ms
                    S                            stop (MANUAL mode)
                    B <ms> <freq>                beep
WebSocket (JSON):
  -> {"type":"hello","role":"camera"|"console","key":"..."}
  camera -> {"type":"frame","w":..,"h":..,"markers":{"0":[[x,y]*4],...}}
  console -> goto{x,y} follow home sethome stop joy{x,y} calibrate clear wall{x,y}
             arena{w,h} lock{on} beep
  <- state (10 Hz), map (when it changes), overlay (camera, 10 Hz)

Run:  python app.py      then open http://localhost:8080/?key=<CONTROL_KEY>
"""
import asyncio
import json
import os
import time
import traceback
from pathlib import Path

from aiohttp import WSMsgType, web

import config as C
from brain import Brain, trimmed

WEB = Path(__file__).resolve().parent.parent / "web"

brain = Brain()
DATA = Path(os.environ.get("DATA_DIR", str(WEB.parent / "data")))
SETTINGS = DATA / "settings.json"
_saved = None


def snapshot():
    return {"tune": {k: getattr(C, k) for k in C.TUNABLE},
            "turn_sign": brain.turn_sign, "heading_offset": brain.heading_offset}


def load_settings():
    """Tuning and calibration survive restarts and redeploys."""
    global _saved
    try:
        s = json.loads(SETTINGS.read_text())
    except (OSError, ValueError):
        return
    for k, v in s.get("tune", {}).items():
        if k in C.TUNABLE:
            setattr(C, k, v)
    brain.turn_sign = s.get("turn_sign", 1)
    brain.heading_offset = s.get("heading_offset", 0.0)
    _saved = snapshot()
    print(f"[app] settings loaded from {SETTINGS}")


def save_settings():
    global _saved
    snap = snapshot()
    if snap != _saved:
        try:
            DATA.mkdir(parents=True, exist_ok=True)
            SETTINGS.write_text(json.dumps(snap, indent=1))
            _saved = snap
        except OSError as e:
            print("[app] cannot save settings:", e)
clients = {}             # ws -> {"role":..., "control": bool}
robot = {"writer": None, "peer": None, "rtt": None}


# ---------------------------------------------------------------- robot link
async def robot_send(line):
    w = robot["writer"]
    if w is None:
        return
    try:
        w.write((line + "\n").encode())
        await asyncio.wait_for(w.drain(), 0.5)
    except (ConnectionError, RuntimeError, asyncio.TimeoutError):
        robot["writer"] = None
        w.close()


async def handle_robot(reader, writer):
    peer = writer.get_extra_info("peername")
    try:
        hello = await asyncio.wait_for(reader.readline(), 5)
    except asyncio.TimeoutError:
        writer.close()
        return
    parts = hello.decode(errors="replace").split()
    if len(parts) < 2 or parts[0] != "HELLO" or parts[1] != C.ROBOT_TOKEN:
        got = parts[1] if len(parts) > 1 else ""
        print(f"[robot] rejected {peer}: first word {parts[0] if parts else '(empty)'!r}, "
              f"token length {len(got)} starts {got[:3]!r} (expected length {len(C.ROBOT_TOKEN)} "
              f"starts {C.ROBOT_TOKEN[:3]!r})")
        writer.close()
        return
    if robot["writer"]:
        robot["writer"].close()          # newest connection wins (robot reconnected)
    robot.update(writer=writer, peer=peer)
    brain.log(f"robot connected from {peer[0]}")
    try:
        while True:
            line = await asyncio.wait_for(reader.readline(), C.ROBOT_TIMEOUT * 4)
            if not line:
                break
            p = line.decode(errors="replace").split()
            if p and p[0] == "T" and len(p) >= 7:
                try:
                    brain.telemetry({"dist": float(p[1]), "l": int(p[2]), "r": int(p[3]),
                                     "mode": p[4], "rssi": int(p[5]), "blocked": p[6] == "1"})
                except ValueError:
                    pass
    except (asyncio.TimeoutError, ConnectionError):
        pass
    finally:
        if robot["writer"] is writer:
            robot["writer"] = None
            brain.log("robot disconnected")
        writer.close()


# ---------------------------------------------------------------- websockets
async def _send(ws, data):
    try:
        await asyncio.wait_for(ws.send_str(data), 0.5)
    except Exception:
        # a client that stops reading must never stall the control loop
        clients.pop(ws, None)
        asyncio.ensure_future(ws.close())


async def broadcast(msg, role=None):
    data = json.dumps(msg)
    targets = [ws for ws, info in list(clients.items()) if not role or info["role"] == role]
    if targets:
        await asyncio.gather(*(_send(ws, data) for ws in targets))


async def handle_console_msg(m):
    t = m.get("type")
    if t == "goto":
        brain.cmd_goto(float(m["x"]), float(m["y"]))
    elif t == "follow":
        brain.cmd_follow()
    elif t == "home":
        brain.cmd_home()
    elif t == "sethome":
        if "x" in m:
            brain.cmd_set_home(float(m["x"]), float(m["y"]))
        else:
            brain.cmd_set_home()
    elif t == "stop":
        brain.cmd_stop()
        await robot_send("S")
    elif t == "joy":
        brain.cmd_joy(max(-100, min(100, int(m["x"]))), max(-100, min(100, int(m["y"]))))
    elif t == "calibrate":
        brain.cmd_calibrate()
    elif t == "clear":
        brain.grid.clear()
        brain.trail.clear()
        brain.log("map cleared")
    elif t == "wall":
        brain.grid.toggle_wall(float(m["x"]), float(m["y"]))
    elif t == "arena":
        brain.set_arena(float(m["w"]), float(m["h"]))
    elif t == "tune":
        k = m.get("key")
        if k in C.TUNABLE:
            lo, hi = C.TUNABLE[k]
            v = max(lo, min(hi, float(m["value"])))
            setattr(C, k, int(v) if isinstance(getattr(C, k), int) else v)
            brain.log(f"{k} = {getattr(C, k)}")
    elif t == "arena_auto":
        brain.cmd_arena_auto()
    elif t == "lock":
        brain.cal_locked = bool(m.get("on")) and brain.h_img2floor is not None
        brain.log("camera calibration " + ("locked" if brain.cal_locked else "unlocked"))
    elif t == "beep":
        await robot_send("B 150 2000")


async def ws_handler(request):
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    clients[ws] = {"role": "console", "control": False}
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                m = json.loads(msg.data)
            except ValueError:
                continue
            info = clients[ws]
            t = m.get("type")
            if t == "hello":
                info["role"] = m.get("role", "console")
                info["control"] = m.get("key") == C.CONTROL_KEY
                await ws.send_str(json.dumps({"type": "welcome", "control": info["control"],
                                              "role": info["role"]}))
                await ws.send_str(json.dumps(brain.map_cells()))
            elif t == "frame":
                if info["control"]:          # only trusted cameras may move the robot
                    markers = {int(k): v for k, v in m.get("markers", {}).items()}
                    brain.camera_frame(m.get("w", 0), m.get("h", 0), markers)
            elif info["control"]:
                await handle_console_msg(m)
    finally:
        clients.pop(ws, None)
    return ws


# ---------------------------------------------------------------- loops
async def control_loop():
    period = 1.0 / C.CONTROL_HZ
    while True:
        t0 = time.monotonic()
        try:
            await control_step()
            took = time.monotonic() - t0
            if took > 0.1:
                print(f"[app] slow control step: {took * 1000:.0f} ms")
        except Exception:                 # never let one bad tick kill the control loop
            traceback.print_exc()
            brain.cmd_stop()
            await robot_send("D 0 0 100")
        await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))


_sent_map_version = -1


async def control_step():
    global _sent_map_version
    cmd = brain.tick()
    if cmd:
        if cmd[0] == "drive":
            l, r = trimmed(cmd[1], cmd[2])
            await robot_send(f"D {l} {r} {cmd[3]}")
        elif cmd[0] == "stop":
            await robot_send("D 0 0 100")
        elif cmd[0] == "arrived":
            await robot_send("D 0 0 100")
            await robot_send("B 300 2500")
    save_settings()
    st = brain.state()
    st["robot"]["link"] = robot["writer"] is not None
    await broadcast(st)
    if brain.grid.version != _sent_map_version:
        _sent_map_version = brain.grid.version
        await broadcast(brain.map_cells())
    ov = brain.overlay()
    if ov:
        await broadcast(ov, role="camera")


async def on_startup(app):
    load_settings()
    app["robot_server"] = await asyncio.start_server(handle_robot, C.ROBOT_HOST, C.ROBOT_PORT)
    app["loop_task"] = asyncio.ensure_future(control_loop())
    print(f"HTTP on :{C.HTTP_PORT}, robot TCP on :{C.ROBOT_PORT}")


async def on_cleanup(app):
    app["loop_task"].cancel()
    app["robot_server"].close()


async def static_file(request):
    name = request.match_info["path"] or "index.html"
    p = (WEB / name).resolve()
    if WEB not in p.parents or not p.is_file():
        raise web.HTTPNotFound()
    return web.FileResponse(p)


@web.middleware
async def no_cache(request, handler):
    resp = await handler(request)
    resp.headers["Cache-Control"] = "no-cache"
    return resp


def make_app():
    app = web.Application(middlewares=[no_cache])
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/{path:.*}", static_file)
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


if __name__ == "__main__":
    web.run_app(make_app(), host=C.HTTP_HOST, port=C.HTTP_PORT, print=None)
