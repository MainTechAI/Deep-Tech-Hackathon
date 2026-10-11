"""Occupancy grid of the arena + A* path planning around known obstacles."""
import heapq
import math

import config as C


class Grid:
    def __init__(self, w_cm, h_cm, cell=C.GRID_CM):
        self.cell = cell
        self.resize(w_cm, h_cm)

    def resize(self, w_cm, h_cm):
        self.w_cm, self.h_cm = w_cm, h_cm
        self.nx = int(math.ceil(w_cm / self.cell))
        self.ny = int(math.ceil(h_cm / self.cell))
        self.hits = {}         # (i, j) -> evidence 0..5, sonar
        self.walls = set()     # (i, j) drawn by hand, permanent
        self.version = 0
        self._inflated = None

    # ---- coordinates ----
    def cell_of(self, x, y):
        return int(x // self.cell), int(y // self.cell)

    def centre_of(self, i, j):
        return (i + 0.5) * self.cell, (j + 0.5) * self.cell

    def inside(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    # ---- map updates ----
    def occupied(self):
        return self.walls | {k for k, v in self.hits.items() if v >= C.MAP_HITS}

    def _changed(self):
        self.version += 1
        self._inflated = None

    def add_hit(self, x, y, force=False):
        """force: a sure obstacle (very close echo, robot stuck) - also counted at the arena edge."""
        k = self.cell_of(x, y)
        if not self.inside(*k):
            return False
        e = C.EDGE_IGNORE     # the fence keeps the robot off the edge anyway; echoes there are outside things
        if not force and (x < e or y < e or x > self.w_cm - e or y > self.h_cm - e):
            return False
        before = self.hits.get(k, 0)
        self.hits[k] = min(5, before + 1)
        if before < C.MAP_HITS <= self.hits[k]:
            self._changed()
            return True
        return False

    def add_free_ray(self, x0, y0, x1, y1):
        """Sonar saw nothing along this segment: forget single (unconfirmed) echoes on it.
        Confirmed obstacles stay - the sonar cone is wide, so a ray passing right next to a box
        corner would otherwise erase it and the planned route would flip back and forth.
        Moved obstacles are removed with Clear map."""
        n = int(math.hypot(x1 - x0, y1 - y0) / (self.cell / 2)) + 1
        for s in range(n):
            t = s / n
            k = self.cell_of(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            if 0 < self.hits.get(k, 0) < C.MAP_HITS:
                del self.hits[k]

    def toggle_wall(self, x, y):
        k = self.cell_of(x, y)
        if not self.inside(*k):
            return
        if k in self.walls or self.hits.get(k, 0) >= C.MAP_HITS:
            self.walls.discard(k)
            self.hits.pop(k, None)
        else:
            self.walls.add(k)
        self._changed()

    def clear(self):
        self.hits.clear()
        self.walls.clear()
        self._changed()

    # ---- planning ----
    def inflated(self):
        """Cells the robot centre must not enter: obstacles grown by the robot radius."""
        if self._inflated is None:
            # cells whose centre is within ROBOT_RADIUS of the obstacle cell's centre
            rc = C.ROBOT_RADIUS / self.cell
            r = int(math.ceil(rc))
            blocked = set()
            offs = [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1)
                    if di * di + dj * dj <= rc * rc]
            for (i, j) in self.occupied():
                for di, dj in offs:
                    blocked.add((i + di, j + dj))
            # virtual fence: keep the robot centre FENCE_CM inside the arena edge
            m = self.fence_cells()
            for i in range(self.nx):
                for j in list(range(m)) + list(range(self.ny - m, self.ny)):
                    blocked.add((i, j))
            for j in range(self.ny):
                for i in list(range(m)) + list(range(self.nx - m, self.nx)):
                    blocked.add((i, j))
            self._inflated = blocked
        return self._inflated

    def fence_cells(self):
        return max(1, int(round(C.FENCE_CM / self.cell)))

    def free(self, k):
        return self.inside(*k) and k not in self.inflated()

    def line_free(self, a, b):
        (x0, y0), (x1, y1) = self.centre_of(*a), self.centre_of(*b)
        n = int(math.hypot(x1 - x0, y1 - y0) / (self.cell / 2)) + 1
        for s in range(n + 1):
            t = s / n
            if not self.free(self.cell_of(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)):
                return False
        return True

    def path_free(self, pts):
        """Is a route (cm points) still clear of inflated obstacles? The first 15 cm are not
        checked: the robot itself may already be inside an inflated zone."""
        skip = 15.0
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
            n = int(L / (self.cell / 2)) + 1
            for s in range(n + 1):
                d = L * s / n
                if skip > 0 and d < skip:
                    continue
                if not self.free(self.cell_of(x0 + (x1 - x0) * s / n, y0 + (y1 - y0) * s / n)):
                    return False
            skip -= L
        return True

    def nearest_free(self, k, max_r=8):
        if self.free(k):
            return k
        for r in range(1, max_r + 1):
            best = None
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    c = (k[0] + di, k[1] + dj)
                    if max(abs(di), abs(dj)) == r and self.free(c):
                        d = di * di + dj * dj
                        if best is None or d < best[0]:
                            best = (d, c)
            if best:
                return best[1]
        return None

    def plan(self, start_xy, goal_xy):
        """List of (x, y) cm waypoints from start to goal, or None if unreachable.
        The start may be inside an inflated zone (robot already close to something)."""
        s = self.cell_of(*start_xy)
        # robot outside the arena but still seen: start from the nearest edge cell, drive back in
        s = (min(max(s[0], 0), self.nx - 1), min(max(s[1], 0), self.ny - 1))
        g = self.nearest_free(self.cell_of(*goal_xy))
        if g is None:
            return None
        blocked = self.inflated()
        occupied = self.occupied()
        # allow leaving the inflated area around the start
        escape = set()
        if s in blocked:
            er = max(4, self.fence_cells() + 2)
            for di in range(-er, er + 1):
                for dj in range(-er, er + 1):
                    escape.add((s[0] + di, s[1] + dj))

        def ok(k):
            return self.inside(*k) and (k not in blocked or (k in escape and k not in occupied))

        moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        h = lambda k: math.hypot(k[0] - g[0], k[1] - g[1])
        openq = [(h(s), 0.0, s)]
        came = {s: None}
        cost = {s: 0.0}
        while openq:
            _, c, k = heapq.heappop(openq)
            if k == g:
                break
            if c > cost.get(k, 1e9):
                continue
            for di, dj, w in moves:
                n = (k[0] + di, k[1] + dj)
                if not ok(n):
                    continue
                nc = c + w
                if nc < cost.get(n, 1e9):
                    cost[n] = nc
                    came[n] = k
                    heapq.heappush(openq, (nc + h(n), nc, n))
        if g not in came:
            return None
        cells = []
        k = g
        while k is not None:
            cells.append(k)
            k = came[k]
        cells.reverse()
        # string pulling: keep only the corners we actually need
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self.line_free(cells[i], cells[j]):
                j -= 1
            out.append(cells[j])
            i = j
        pts = [self.centre_of(*c) for c in out]
        pts[0] = tuple(start_xy)
        if self.cell_of(*goal_xy) == g:
            pts[-1] = tuple(goal_xy)
        return pts
