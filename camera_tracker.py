"""Overhead-camera tracker for the Wayfinder rover.

A camera looks down at the floor. Four printed ArUco markers taped to the floor
at known positions define the floor's coordinate frame; a fifth marker on top of
the rover gives its position and heading. The pose is posted to the gateway,
which then uses it in place of the rover's own dead-reckoned estimate.

    python3 camera_tracker.py --print-markers            # writes markers.pdf, print at 100 %
    python3 camera_tracker.py --width 1.5 --height 1.0   # floor markers on a 1.5 x 1.0 m rectangle

Floor layout, seen from above, with each marker's centre on a rectangle corner:

    id 4 (0, H) -------- id 3 (W, H)
       |                     |
    id 1 (0, 0) -------- id 2 (W, 0)

The rover marker (id 0) sits flat on top of the rover, centred over the axle,
with its top edge towards the front. Requires: opencv-python, numpy, pillow.
"""
import argparse, json, math, sys, time, urllib.request
from pathlib import Path

import cv2
import numpy as np

ROVER_ID = 0
FLOOR_IDS = (1, 2, 3, 4)
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
CALIBRATION_FILE = Path(__file__).with_name("camera_calibration.json")


def print_markers(path, rover_cm, floor_cm):
    """Write an A4 PDF with every marker at its true size, one per page."""
    from PIL import Image, ImageDraw
    dpi = 300
    page_w, page_h = round(8.27 * dpi), round(11.69 * dpi)
    pages = []
    for marker_id in (ROVER_ID,) + FLOOR_IDS:
        size_cm = rover_cm if marker_id == ROVER_ID else floor_cm
        side = round(size_cm / 2.54 * dpi)
        marker = Image.fromarray(cv2.aruco.generateImageMarker(DICTIONARY, marker_id, side))
        page = Image.new("L", (page_w, page_h), 255)
        left, top = (page_w - side) // 2, (page_h - side) // 2
        page.paste(marker, (left, top))
        draw = ImageDraw.Draw(page)
        label = f"id {marker_id} - " + ("ROVER, this edge to the FRONT" if marker_id == ROVER_ID else "FLOOR corner")
        draw.text((left, top - 90), label, fill=0, font_size=60)
        draw.text((left, top + side + 30), f"Printed side must measure {size_cm:g} cm. Keep the white border.", fill=0, font_size=44)
        pages.append(page)
    pages[0].save(path, save_all=True, append_images=pages[1:], resolution=dpi)


def floor_positions(width, height):
    return {1: (0.0, 0.0), 2: (width, 0.0), 3: (width, height), 4: (0.0, height)}


class Floor:
    """Maps image pixels to floor metres, and corrects for the rover marker's height."""

    def __init__(self, homography, camera_xyz):
        self.homography = np.asarray(homography, dtype=np.float64)
        self.camera = None if camera_xyz is None else np.asarray(camera_xyz, dtype=np.float64)

    def to_floor(self, pixel, height_m=0.0):
        point = cv2.perspectiveTransform(np.array([[pixel]], dtype=np.float64), self.homography)[0, 0]
        # The homography is exact only for points on the floor. Something raised
        # above it appears pushed away from the spot under the camera; pull it
        # back along the line to that spot in proportion to its height.
        if height_m and self.camera is not None and self.camera[2] > height_m:
            point = self.camera[:2] + (point - self.camera[:2]) * (self.camera[2] - height_m) / self.camera[2]
        return point

    def save(self):
        CALIBRATION_FILE.write_text(json.dumps({"homography": self.homography.tolist(), "camera": None if self.camera is None else self.camera.tolist()}))

    @staticmethod
    def load():
        if not CALIBRATION_FILE.exists(): return None
        data = json.loads(CALIBRATION_FILE.read_text())
        return Floor(data["homography"], data.get("camera"))


def centre_of(quad):
    """Where the diagonals cross: the true centre of a square seen in perspective."""
    a, b, c, d = quad
    denominator = (a[0] - c[0]) * (b[1] - d[1]) - (a[1] - c[1]) * (b[0] - d[0])
    if abs(denominator) < 1e-9: return quad.mean(axis=0)
    t = ((a[0] - b[0]) * (b[1] - d[1]) - (a[1] - b[1]) * (b[0] - d[0])) / denominator
    return a + t * (c - a)


def calibrate(corners_by_id, positions, frame_size, hfov_deg):
    """Build the floor mapping from the centres of the four floor markers.

    Only the centres are used, so it does not matter which way each floor
    marker is turned.
    """
    image_points = np.array([centre_of(corners_by_id[marker_id]) for marker_id in FLOOR_IDS], dtype=np.float64)
    floor_points = np.array([positions[marker_id] for marker_id in FLOOR_IDS], dtype=np.float64)
    homography = cv2.getPerspectiveTransform(image_points.astype(np.float32), floor_points.astype(np.float32))

    # Camera position, for the height correction. The lens is not calibrated:
    # the focal length is taken from a nominal field of view, which is accurate
    # enough because the correction itself is only a few percent.
    width, height = frame_size
    focal = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    intrinsics = np.array([[focal, 0, width / 2.0], [0, focal, height / 2.0], [0, 0, 1]], dtype=np.float64)
    object_points = np.hstack([floor_points, np.zeros((4, 1))])
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, intrinsics, None, flags=cv2.SOLVEPNP_IPPE)
    camera = None
    if ok:
        rotation, _ = cv2.Rodrigues(rvec)
        camera = (-rotation.T @ tvec).ravel()
        if camera[2] < 0: camera = None  # Markers laid out clockwise: the frame is mirrored.
    return Floor(homography, camera)


def rover_pose(corners, floor, height_m, heading_offset):
    """Position of the marker centre and the direction its top edge faces, in floor coordinates."""
    centre = floor.to_floor(centre_of(corners), height_m)
    front = floor.to_floor((corners[0] + corners[1]) / 2.0, height_m)
    heading = math.degrees(math.atan2(front[1] - centre[1], front[0] - centre[0])) + heading_offset
    return float(centre[0]), float(centre[1]), heading % 360.0


def detect(detector, frame):
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None: return {}
    return {int(marker_id): quad.reshape(4, 2).astype(np.float64) for quad, marker_id in zip(corners, ids.ravel())}


def post_pose(api, x, y, heading):
    body = json.dumps({"x": round(x, 3), "y": round(y, 3), "heading": round(heading, 1), "source": "camera"}).encode()
    request = urllib.request.Request(api + "/api/pose", data=body, headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(request, timeout=0.5).read()
        return True
    except OSError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--print-markers", action="store_true", help="write markers.pdf and exit")
    parser.add_argument("--rover-marker-cm", type=float, default=8.0)
    parser.add_argument("--floor-marker-cm", type=float, default=15.0)
    parser.add_argument("--camera", type=int, default=0, help="camera index; try 1 or 2 if 0 is the wrong one")
    parser.add_argument("--width", type=float, default=1.0, help="floor rectangle, id 1 to id 2, metres")
    parser.add_argument("--height", type=float, default=1.0, help="floor rectangle, id 1 to id 4, metres")
    parser.add_argument("--rover-height-cm", type=float, default=10.0, help="height of the rover marker above the floor")
    parser.add_argument("--heading-offset", type=float, default=0.0, help="degrees added if the marker is not square to the rover")
    parser.add_argument("--hfov", type=float, default=65.0, help="camera horizontal field of view, degrees")
    parser.add_argument("--api", default="http://127.0.0.1:8000")
    parser.add_argument("--rate", type=float, default=10.0, help="poses posted per second")
    parser.add_argument("--no-window", action="store_true")
    args = parser.parse_args()

    if args.print_markers:
        print_markers("markers.pdf", args.rover_marker_cm, args.floor_marker_cm)
        print("Wrote markers.pdf. Print at 100 % (no 'fit to page') and check the sizes with a ruler.")
        return

    capture = cv2.VideoCapture(args.camera)
    if not capture.isOpened(): sys.exit(f"Cannot open camera {args.camera}. Try another --camera index, and allow camera access for the terminal.")
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920); capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    detector = cv2.aruco.ArucoDetector(DICTIONARY, cv2.aruco.DetectorParameters())
    positions = floor_positions(args.width, args.height)
    floor = Floor.load()
    print("Loaded saved floor calibration." if floor else "No calibration yet: show all four floor markers (ids 1-4) to the camera.")
    print("Keys in the window: c = recalibrate from the floor markers, q = quit.")

    last_post, last_report, gateway_ok = 0.0, 0.0, None
    while True:
        ok, frame = capture.read()
        if not ok: sys.exit("Camera stopped delivering frames.")
        found = detect(detector, frame)
        key = -1 if args.no_window else cv2.waitKey(1) & 0xFF
        if key == ord("q"): break
        floor_visible = all(marker_id in found for marker_id in FLOOR_IDS)
        if floor_visible and (floor is None or key == ord("c")):
            floor = calibrate(found, positions, (frame.shape[1], frame.shape[0]), args.hfov)
            floor.save()
            height = "unknown (check the marker order runs anticlockwise from id 1)" if floor.camera is None else f"{floor.camera[2]:.2f} m"
            print(f"Floor calibrated and saved. Camera height: {height}")

        pose = None
        if floor is not None and ROVER_ID in found:
            pose = rover_pose(found[ROVER_ID], floor, args.rover_height_cm / 100.0, args.heading_offset)
            now = time.monotonic()
            if now - last_post >= 1.0 / args.rate:
                last_post = now
                gateway_ok = post_pose(args.api, *pose)

        now = time.monotonic()
        if now - last_report >= 2.0:
            last_report = now
            where = "rover not visible" if pose is None else f"rover x={pose[0]:.2f} y={pose[1]:.2f} m heading={pose[2]:.0f}"
            note = "" if gateway_ok is not False else "  [gateway unreachable]"
            print(("no calibration, " if floor is None else "") + where + note)

        if not args.no_window:
            for marker_id, quad in found.items():
                colour = (0, 200, 0) if marker_id == ROVER_ID else (255, 160, 0)
                cv2.polylines(frame, [quad.astype(np.int32)], True, colour, 3)
                cv2.putText(frame, str(marker_id), tuple(quad[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, colour, 2)
            status = "NO CALIBRATION - show floor markers 1-4" if floor is None else ("rover not visible" if pose is None else f"x {pose[0]:.2f}  y {pose[1]:.2f}  heading {pose[2]:.0f}")
            cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255) if pose is None else (0, 200, 0), 2)
            cv2.imshow("Wayfinder camera tracker", cv2.resize(frame, None, fx=0.6, fy=0.6))
    capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
