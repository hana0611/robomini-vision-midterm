"""Offline synthetic checks. Passing these is NOT a simulated match pass."""
import ast
import math
from pathlib import Path
import cv2
import numpy as np
from vision_core import camera_pose, detect, pnp_pose, plane_position, aim, rotation, Motion, AXIS


def project(world, c, rcw, k):
    points = (rcw.T @ (np.asarray(world) - c).T).T
    image, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), k, np.zeros(5))
    return image.reshape(-1, 2)


def synth(theta, yaw, pitch, active=False):
    k = np.array([[1108.5, 0, 640], [0, 1108.5, 360], [0, 0, 1.]], float)
    c, rcw = camera_pose(yaw, pitch)
    radial = np.array([math.cos(theta), 0, math.sin(theta)])
    tangent = np.array([-math.sin(theta), 0, math.cos(theta)])
    centre = AXIS + .35 * radial
    image = np.full((720, 1280, 3), 20, np.uint8)
    axes = [radial, -radial, tangent, -tangent]
    for i, axis in enumerate(axes):
        if active and i < 2:
            continue
        other = np.array([-axis[2], 0., axis[0]])
        b = centre + .0475 * axis
        corners = [b + x * .0205 * axis + z * .010 * other
                   for x, z in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
        uv = project(corners, c, rcw, k)
        cv2.fillConvexPoly(image, np.rint(uv).astype(np.int32), (255, 65, 0))
    return image, k, centre, c, rcw


def run():
    detection_cases = 0
    max_position_error = max_pnp_error = 0.
    for theta in np.linspace(-math.pi, math.pi, 16, endpoint=False):
        for active in (False, True):
            image, k, centre, c, rcw = synth(theta, -.05, .22, active)
            _, groups = detect(image, k)
            want = 'ACTIVE' if active else 'PENDING'
            assert len(groups) == 1 and groups[0]['state'] == want, (theta, want, groups)
            detection_cases += 1
            if not active:
                pose = pnp_pose(groups[0]['points'], k, np.zeros(5))
                assert pose is not None, theta
                _, tv, err = pose
                world_pnp = c + rcw @ tv
                pnp_error = np.linalg.norm(world_pnp - centre)
                max_pnp_error = max(max_pnp_error, pnp_error)
                assert pnp_error < .06, (theta, pnp_error)
                pos = plane_position(groups[0]['centre'], k, np.zeros(5), -.05, .22)
                position_error = np.linalg.norm(pos - centre)
                max_position_error = max(max_position_error, position_error)
                assert position_error < .005, (theta, position_error)
    # Simultaneous active + pending must yield both labels.
    im1, k, *_ = synth(-2., 0., .22, False)
    im2, *_ = synth(.8, 0., .22, True)
    _, groups = detect(np.maximum(im1, im2), k)
    assert sorted(g['state'] for g in groups) == ['ACTIVE', 'PENDING']
    _, groups = detect(np.full_like(im1, 20), k)
    assert not groups
    # A large blue background must not become a target.
    bg = np.full_like(im1, 20)
    cv2.rectangle(bg, (0, 550), (1279, 719), (255, 65, 0), -1)
    assert not detect(bg, k)[1]
    max_ballistic_error = 0.
    for theta in np.linspace(-math.pi, math.pi, 20, endpoint=False):
        target = AXIS + [.35 * math.cos(theta), 0., .35 * math.sin(theta)]
        result = aim(target)
        assert result is not None
        yaw, pitch, tof = result
        r = rotation(yaw, pitch)
        muzzle = np.array([0., -2.07, .495]) + r @ [0., .120, 0.]
        hit = muzzle + 12. * tof * (r @ [0., 1., 0.]) + [0., 0., -.5 * 9.81 * tof ** 2]
        error = np.linalg.norm(hit - target)
        max_ballistic_error = max(max_ballistic_error, error)
        assert error < .0001, error
    for acceleration in (0., .8):
        m = Motion()
        for t in np.arange(0, .60, 1 / 60):
            a = .3 - 1.05 * t + .5 * acceleration * t*t
            m.update(t, AXIS + [.35 * math.cos(a), 0., .35 * math.sin(a)])
        end = t + .2
        a = .3 - 1.05 * end + .5 * acceleration * end*end
        pred = m.predict(t, .2)
        actual = AXIS + [.35 * math.cos(a), 0., .35 * math.sin(a)]
        assert np.linalg.norm(pred - actual) < 1e-5
        m.update(t + 1 / 60, AXIS + [.35, 0., 0.])
    for f in Path(__file__).parent.glob('*.py'):
        ast.parse(f.read_text(encoding='utf-8'))
    node_source = (Path(__file__).parent / 'vision_node.py').read_text(encoding='utf-8')
    assert '/rune/state' not in node_source and '/rune/hit' not in node_source and '/score' not in node_source
    print(f'PASS: {detection_cases} projected 2/4-bar cases, simultaneous states, blank image, background rejection.')
    print(f'PASS: synthetic PnP max centre error={max_pnp_error:.6f}m; plane centre max error={max_position_error:.6f}m.')
    print(f'PASS: 20 gravity trajectory checks, max error={max_ballistic_error:.9f}m; constant/accelerating phase prediction.')
    print('PASS: Python AST syntax; vision does not consume referee state/hit/score topics.')
    print('NOT TESTED: Ubuntu packages, ROS2 runtime, Gazebo visuals, real match hit counts, official grading.')


if __name__ == '__main__':
    run()
