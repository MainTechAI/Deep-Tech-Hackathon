#!/usr/bin/env python3
"""Checks the robot's HTTP API end to end.

    python3 tools/test_api.py 172.20.10.5          # safe checks, motors stay off
    python3 tools/test_api.py 172.20.10.5 --move   # also spins the wheels briefly
    python3 tools/test_api.py localhost:8000 --move  # against tools/mock_robot.py

With --move, put the robot on a stand (wheels in the air) or on the floor with space around.
"""
import json
import sys
import time
from urllib.request import urlopen

FIELDS = {"m", "d", "b", "l", "r", "net", "ip", "rssi", "up", "sc", "wc"}
failures = 0


def get(base, path, raw=False):
    t = time.time()
    with urlopen(base + path, timeout=3) as r:
        body = r.read()
    ms = (time.time() - t) * 1000
    return (body, ms) if raw else (json.loads(body), ms)


def check(name, ok, info=""):
    global failures
    print(("  PASS  " if ok else "  FAIL  ") + name + (f"  ({info})" if info else ""))
    if not ok:
        failures += 1


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        sys.exit(__doc__)
    host = args[0]
    base = host if host.startswith("http") else "http://" + host
    move = "--move" in sys.argv

    print(f"Testing {base}")
    s, ms = get(base, "/status")
    check("status has all fields", FIELDS <= s.keys(), f"missing {FIELDS - s.keys()}" if not FIELDS <= s.keys() else "")
    print(f"        mode={s['m']} dist={s['d']} net={s['net']} ip={s['ip']} rssi={s['rssi']} uptime={s['up'] // 1000}s")

    page, ms = get(base, "/", raw=True)
    check("control page served", b"RobotCar" in page and page.rstrip().endswith(b"</html>"),
          f"{len(page)} bytes, {ms:.0f} ms")

    s, _ = get(base, "/stop")
    check("/stop -> MANUAL and motors off", s["m"] == "MANUAL" and s["l"] == 0 and s["r"] == 0)

    s, _ = get(base, "/beep?ms=100")
    check("/beep answers", s["m"] == "MANUAL")

    s, _ = get(base, "/mode?m=auto")
    check("/mode?m=auto", s["m"] == "AUTO")
    s, _ = get(base, "/m?x=0&y=100")
    check("joystick ignored in AUTO", s["m"] == "AUTO")
    s, _ = get(base, "/mode?m=manual")
    check("/mode?m=manual -> motors off", s["m"] == "MANUAL" and s["l"] == 0 and s["r"] == 0)

    try:
        get(base, "/nope")
        check("unknown path -> 404", False)
    except Exception as e:
        check("unknown path -> 404", "404" in str(e))

    times = []
    for _ in range(20):
        _, ms = get(base, "/status")
        times.append(ms)
    times.sort()
    check("latency", times[-1] < 500, f"median {times[10]:.0f} ms, max {times[-1]:.0f} ms")

    if move:
        print("Motion tests (wheels will turn):")
        s, _ = get(base, "/drive?l=-60&r=60&ms=400")
        check("spin left", s["l"] < 0 < s["r"], f"L {s['l']} R {s['r']}")
        time.sleep(0.6)
        s, _ = get(base, "/status")
        check("/drive stops by itself after ms", s["l"] == 0 and s["r"] == 0, f"L {s['l']} R {s['r']}")

        s, _ = get(base, "/m?x=60&y=0")
        check("joystick spin right", s["l"] > 0 > s["r"], f"L {s['l']} R {s['r']}")
        time.sleep(1.5)
        s, _ = get(base, "/status")
        check("joystick timeout stops motors", s["l"] == 0 and s["r"] == 0, f"L {s['l']} R {s['r']}")

        s, _ = get(base, "/m?x=0&y=60")
        if s["d"] < s["sc"]:
            check("forward blocked near obstacle", s["b"] == 1 and s["l"] == 0 and s["r"] == 0,
                  f"dist {s['d']}")
        else:
            check("forward drives", s["l"] > 0 and s["r"] > 0, f"L {s['l']} R {s['r']}, dist {s['d']}")
        s, _ = get(base, "/m?x=0&y=-60")
        check("reverse always allowed", s["l"] < 0 and s["r"] < 0, f"L {s['l']} R {s['r']}")
        s, _ = get(base, "/stop")
        check("stop", s["l"] == 0 and s["r"] == 0)

    print("ALL PASSED" if failures == 0 else f"{failures} FAILED")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
