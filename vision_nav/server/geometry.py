"""Camera image <-> arena floor (cm) via a homography from the 4 corner markers."""
import math


def solve(a, b):
    """Gaussian elimination with partial pivoting, a: n x n, b: n."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            raise ValueError("singular")
        m[col], m[piv] = m[piv], m[col]
        for r in range(n):
            if r != col:
                f = m[r][col] / m[col][col]
                for c in range(col, n + 1):
                    m[r][c] -= f * m[col][c]
    return [m[i][n] / m[i][i] for i in range(n)]


def homography(src, dst):
    """3x3 matrix H with dst ~ H * src, from 4 point pairs."""
    a, b = [], []
    for (x, y), (u, v) in zip(src, dst):
        a.append([x, y, 1, 0, 0, 0, -u * x, -u * y]); b.append(u)
        a.append([0, 0, 0, x, y, 1, -v * x, -v * y]); b.append(v)
    h = solve(a, b)
    return [h[0:3], h[3:6], h[6:8] + [1.0]]


def apply(h, p):
    x, y = p
    w = h[2][0] * x + h[2][1] * y + h[2][2]
    return ((h[0][0] * x + h[0][1] * y + h[0][2]) / w,
            (h[1][0] * x + h[1][1] * y + h[1][2]) / w)


def invert(h):
    a, b, c = h[0]; d, e, f = h[1]; g, i, j = h[2]
    det = a * (e * j - f * i) - b * (d * j - f * g) + c * (d * i - e * g)
    inv = [[e * j - f * i, c * i - b * j, b * f - c * e],
           [f * g - d * j, a * j - c * g, c * d - a * f],
           [d * i - e * g, b * g - a * i, a * e - b * d]]
    return [[v / det for v in row] for row in inv]


def centre(corners):
    return (sum(p[0] for p in corners) / 4, sum(p[1] for p in corners) / 4)


def marker_pose(h, corners):
    """Floor position and heading (deg, CCW from +x) of a marker.
    corners: image points in marker order top-left, top-right, bottom-right, bottom-left;
    'forward' = from the bottom edge towards the top edge."""
    c = apply(h, centre(corners))
    top = apply(h, ((corners[0][0] + corners[1][0]) / 2, (corners[0][1] + corners[1][1]) / 2))
    bot = apply(h, ((corners[3][0] + corners[2][0]) / 2, (corners[3][1] + corners[2][1]) / 2))
    th = math.degrees(math.atan2(top[1] - bot[1], top[0] - bot[0]))
    return c[0], c[1], th


def arena_size(corner_markers, side):
    """Measure the arena (W, H) in cm from the 4 corner markers, using their known printed size.
    corner_markers: image corners of markers #1..#4 in arena order.
    The centres define a homography to the unit square; there, every marker edge e must satisfy
    (W*ex)^2 + (H*ey)^2 = side^2. 16 edges -> least squares for W^2 and H^2."""
    hn = homography([centre(c) for c in corner_markers], [(0, 0), (1, 0), (1, 1), (0, 1)])
    a11 = a12 = a22 = b1 = b2 = 0.0
    s2 = side * side
    for c in corner_markers:
        q = [apply(hn, p) for p in c]
        for i in range(4):
            u = (q[(i + 1) % 4][0] - q[i][0]) ** 2
            v = (q[(i + 1) % 4][1] - q[i][1]) ** 2
            a11 += u * u; a12 += u * v; a22 += v * v
            b1 += u * s2; b2 += v * s2
    det = a11 * a22 - a12 * a12
    if abs(det) < 1e-24:
        return None
    w2 = (b1 * a22 - b2 * a12) / det
    h2 = (a11 * b2 - a12 * b1) / det
    if w2 <= 0 or h2 <= 0:
        return None
    return math.sqrt(w2), math.sqrt(h2)


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0
