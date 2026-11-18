"""Pure vision / geometry; no ROS, no referee truth inputs. SI units throughout."""
import math
import cv2
import numpy as np

BAR_RADIUS = 0.0475  # bar centre to target centre, metres, from supplied sim SDF
AXIS = np.array([0.0, -0.123, 0.928])  # visible LED plane (blue)


def rotation(yaw, pitch):
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.]]) @ np.array(
        [[1., 0, 0], [0, cp, -sp], [0, sp, cp]])


def camera_pose(yaw, pitch):
    r = rotation(yaw, pitch)
    pivot = np.array([0., -2.07, 0.495])
    c = pivot + r @ np.array([0., 0.036, 0.075])
    # OpenCV optical: right, down, forward -> model: +X, -Z, +Y
    rcw = r @ np.array([[1., 0, 0], [0, 0, 1.], [0, -1., 0]])
    return c, rcw


def plane_position(uv, k, d, yaw, pitch):
    c, rcw = camera_pose(yaw, pitch)
    xy = cv2.undistortPoints(np.array(uv, np.float64).reshape(1, 1, 2), k, d)[0, 0]
    ray = rcw @ np.array([xy[0], xy[1], 1.])
    if abs(ray[1]) < 1e-5:
        return None
    scale = (AXIS[1] - c[1]) / ray[1]
    return c + scale * ray if scale > 0 else None


def detect(image, k, vmin=100, smin=110, blue_min=90, blue_max=135):
    """Blue bars -> small elongated contours -> neighbouring groups of 2 or 4.

    Initial values need checking against the actual renderer. Never classify
    an arbitrary blue blob as a target. Return the mask for visual diagnosis.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (blue_min, smin, vmin), (blue_max, 255, 255))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centres = []
    fx = float(k[0, 0])
    # Sim range 1.6-2.2m; a bar is 41 x 20 mm. Scale pixel limits by fx.
    for contour in contours:
        area = cv2.contourArea(contour)
        (_, _), (w, h), _ = cv2.minAreaRect(contour)
        lo, hi = min(w, h), max(w, h)
        if lo < 2 or not (1.3 < hi / lo < 4.5):
            continue
        if not (0.006 * fx < hi < 0.038 * fx and 12 < area < 0.0015 * fx * fx):
            continue
        mm = cv2.moments(contour)
        if mm['m00']:
            centres.append([mm['m10'] / mm['m00'], mm['m01'] / mm['m00']])
    points = np.array(centres, np.float64).reshape(-1, 2)
    groups, unseen = [], set(range(len(points)))
    # Need to connect both adjacent (67mm) AND opposite active bars (95mm).
    # Different target centres are >400mm apart in the model.
    near = 0.075 * fx
    while unseen:
        stack = [unseen.pop()]
        ids = []
        while stack:
            i = stack.pop()
            ids.append(i)
            ns = [j for j in unseen if np.linalg.norm(points[j] - points[i]) < near]
            unseen.difference_update(ns)
            stack.extend(ns)
        p = points[ids]
        if len(p) not in (2, 4):
            continue
        centre = p.mean(axis=0)
        radii = np.linalg.norm(p - centre, axis=1)
        if radii.min() < 0.014 * fx or radii.max() > 0.038 * fx:
            continue
        if len(p) == 4:
            if radii.max() / radii.min() > 1.6:
                continue
            angles = np.sort(np.arctan2(p[:, 1] - centre[1], p[:, 0] - centre[0]))
            gaps = np.diff(np.r_[angles, angles[0] + 2 * np.pi])
            if np.max(np.abs(gaps - np.pi / 2)) > 0.5:
                continue
        groups.append(dict(points=p, centre=centre,
                           state='PENDING' if len(p) == 4 else 'ACTIVE'))
    return mask, groups


def pnp_pose(points, k, d):
    """4 bar centres have square symmetry: orientation is only modulo 90deg.
    tvec is the target centre in OpenCV camera coordinates (metres).
    """
    p = np.asarray(points, np.float64)
    centre = p.mean(axis=0)
    order = np.argsort(np.arctan2(p[:, 1] - centre[1], p[:, 0] - centre[0]))
    image_points = p[order]
    a = BAR_RADIUS
    object_points = np.array([[a, 0, 0], [0, a, 0], [-a, 0, 0], [0, -a, 0]], np.float64)
    ok, rv, tv = cv2.solvePnP(object_points, image_points, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or tv[2, 0] <= 0:
        return None
    projected, _ = cv2.projectPoints(object_points, rv, tv, k, d)
    error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
    if error > 3.0 or not 0.5 < tv[2, 0] < 5.0:
        return None
    return rv.ravel(), tv.ravel(), error


def aim(target, speed=12., initial=(0., 0.)):
    """Iterate muzzle position and the low-arc gravity-compensated solution."""
    y, p = initial
    target = np.asarray(target, float)
    for _ in range(5):
        r = rotation(y, p)
        muzzle = np.array([0., -2.07, 0.495]) + r @ np.array([0., 0.120, 0.])
        dx, dy, dz = target - muzzle
        dist = math.hypot(dx, dy)
        disc = speed ** 4 - 9.81 * (9.81 * dist * dist + 2 * dz * speed * speed)
        if disc < 0 or dist < 0.01:
            return None
        y = math.atan2(-dx, dy)
        p = math.atan((speed * speed - math.sqrt(disc)) / (9.81 * dist))
    if not (-math.pi / 2 < y < math.pi / 2 and math.radians(-10) < p < math.pi / 4):
        return None
    return y, p, dist / (speed * math.cos(p))


class Motion:
    """Estimate phase speed/acceleration from IMAGE positions, never /rune/state."""
    def __init__(self):
        self.samples = []

    def reset(self):
        self.samples.clear()

    def update(self, t, position):
        angle = math.atan2(position[2] - AXIS[2], position[0] - AXIS[0])
        if self.samples:
            dt = t - self.samples[-1][0]
            diff = math.atan2(math.sin(angle - self.samples[-1][1]), math.cos(angle - self.samples[-1][1]))
            if dt <= 0 or dt > 0.25 or abs(diff) > 0.20 + 3 * dt:
                self.reset()
            else:
                angle = self.samples[-1][1] + diff
        self.samples.append((t, angle))
        self.samples = [(s, a) for s, a in self.samples if t - s < 0.45]

    def predict(self, t, horizon):
        if len(self.samples) < 5 or self.samples[-1][0] - self.samples[0][0] < 0.12:
            return None
        ts, aa = np.array(self.samples).T
        degree = 2 if ts[-1] - ts[0] >= 0.25 and len(ts) >= 8 else 1
        coeff = np.polyfit(ts - t, aa, degree)
        velocity = float(np.polyval(np.polyder(coeff), 0))
        if abs(velocity) > 3.5:
            return None
        phase = float(np.polyval(coeff, horizon))
        radius = 0.35
        return AXIS + np.array([radius * math.cos(phase), 0., radius * math.sin(phase)])
