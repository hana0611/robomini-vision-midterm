#!/usr/bin/env python3
"""Blue-side beginner baseline. Run with system Python after sourcing Jazzy.
This is a starting implementation, not a claim of passed on-device evaluation.
"""
import argparse
import csv
import math
from collections import deque
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from robomini_interfaces.msg import GimbalToVision, VisionToGimbal
from vision_core import detect, pnp_pose, plane_position, aim, Motion


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Vision(Node):
    def __init__(self, cfg):
        super().__init__('beginner_vision')
        self.cfg = cfg
        self.bridge = CvBridge()
        self.k = self.d = self.gimbal = self.image = None
        self.feedback = deque(maxlen=400)
        self.motion = Motion()
        self.last_frame = -1.
        self.last_fire = -1e9
        self.last_seen = -1e9
        self.pub = self.create_publisher(VisionToGimbal, '/vision_to_gimbal', 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(GimbalToVision, '/gimbal_to_vision', self.on_gimbal, 10)
        self.create_timer(0.02, self.tick)
        self.outdir = Path(cfg.output)
        self.outdir.mkdir(parents=True, exist_ok=True)
        self.log = open(self.outdir / 'vision.csv', 'w', newline='')
        self.writer = csv.writer(self.log)
        self.writer.writerow(['sim_s', 'tx_m', 'ty_m', 'tz_m', 'rx_rad', 'ry_rad', 'rz_rad',
                              'reproj_px', 'world_x', 'world_y', 'world_z', 'yaw_abs_rad',
                              'pitch_abs_rad', 'delta_yaw_rad', 'delta_pitch_rad', 'fire'])
        self.get_logger().info(f'BLUE side only; fire={cfg.fire}; press S to save image, Q to stop.')
        self.get_logger().info('Waiting for /clock, Image, CameraInfo and GimbalToVision...')
        cv2.namedWindow('vision', cv2.WINDOW_NORMAL)
        cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('vision', 1100, 620)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_info(self, m):
        self.k = np.array(m.k, np.float64).reshape(3, 3)
        self.d = np.array(m.d, np.float64) if len(m.d) else np.zeros(5)

    def on_image(self, m):
        self.image = m  # keep newest frame only; do not build an image backlog

    def on_gimbal(self, m):
        self.gimbal = m
        self.feedback.append((self.now(), m.yaw, m.pitch))

    def at_time(self, t):
        if not self.feedback:
            return None
        a = list(self.feedback)
        nearest = min(a, key=lambda x: abs(x[0] - t))
        if abs(nearest[0] - t) > 0.10:
            return None
        # Feedback has no header; receipt sim time is an approximation.
        for left, right in zip(a, a[1:]):
            if left[0] <= t <= right[0] and right[0] > left[0]:
                w = (t - left[0]) / (right[0] - left[0])
                return left[1] * (1-w) + right[1] * w, left[2] * (1-w) + right[2] * w
        return nearest[1], nearest[2]

    def command(self, yaw=None, pitch=None, fire=False):
        m = VisionToGimbal()
        if self.gimbal is None:
            return
        m.mode = 2 if fire else 1
        m.yaw = float(self.gimbal.yaw if yaw is None else yaw)
        m.pitch = float(self.gimbal.pitch if pitch is None else pitch)
        self.pub.publish(m)

    def tick(self):
        now = self.now()
        if self.image is None or self.k is None or self.gimbal is None:
            self.command()
            return
        stamp = seconds(self.image.header.stamp)
        age = now - stamp
        feedback_age = now - self.feedback[-1][0]
        if stamp == self.last_frame:
            # A stale/no-new image must never keep sending fire commands.
            if age > .2 or feedback_age > .2:
                self.command()
                self.motion.reset()
            return
        self.last_frame = stamp
        actual = self.at_time(stamp)
        if age < -.05 or age > .2 or actual is None or feedback_age > .2:
            self.command()
            self.motion.reset()
            return
        frame = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        mask, groups = detect(frame, self.k, self.cfg.vmin, self.cfg.smin,
                              self.cfg.blue_min, self.cfg.blue_max)
        pending = [g for g in groups if g['state'] == 'PENDING']
        for g in groups:
            p = g['centre'].astype(int)
            colour = (0, 255, 255) if g['state'] == 'PENDING' else (0, 255, 0)
            cv2.circle(frame, tuple(p), 36, colour, 2)
            cv2.putText(frame, g['state'], tuple(p + [25, 0]), cv2.FONT_HERSHEY_SIMPLEX, .6, colour, 2)
            for b in g['points'].astype(int):
                cv2.circle(frame, tuple(b), 4, (255, 255, 255), -1)
        lines = [f'BLUE ONLY | fire enabled: {self.cfg.fire} | frame age: {age:.3f}s']
        solution = None
        if len(pending) == 1:
            g = pending[0]
            pose = pnp_pose(g['points'], self.k, self.d)
            pos = plane_position(g['centre'], self.k, self.d, *actual)
            if pose is not None and pos is not None and abs(math.hypot(pos[0], pos[2] - .928) - .35) < .10:
                self.last_seen = now
                rv, tv, error = pose
                lines += [f'PnP t(m): {tv[0]:.3f}, {tv[1]:.3f}, {tv[2]:.3f} | reproj {error:.2f}px',
                          f'PnP r(rad): {rv[0]:.3f}, {rv[1]:.3f}, {rv[2]:.3f} (90deg symmetry)']
                self.motion.update(stamp, pos)
                speed = max(1., float(self.gimbal.bullet_speed))
                first = aim(pos, speed, (self.gimbal.yaw, self.gimbal.pitch))
                if first is not None:
                    horizon = first[2] + max(age, 0.) + self.cfg.delay
                    future = self.motion.predict(stamp, horizon)
                    if future is not None:
                        solution = aim(future, speed, (self.gimbal.yaw, self.gimbal.pitch))
                if solution is not None:
                    yaw, pitch, _ = solution
                    dy, dp = yaw - self.gimbal.yaw, pitch - self.gimbal.pitch
                    fire = (self.cfg.fire and abs(dy) < .025 and abs(dp) < .025
                            and now - self.last_fire >= .36)
                    self.command(yaw, pitch, fire)
                    if fire:
                        self.last_fire = now
                    lines += [f'ABS yaw/pitch: {math.degrees(yaw):.2f}, {math.degrees(pitch):.2f} deg',
                              f'TURN delta: {math.degrees(dy):.2f}, {math.degrees(dp):.2f} deg | FIRE {fire}']
                    self.writer.writerow([stamp, *tv, *rv, error, *pos, yaw, pitch, dy, dp, int(fire)])
                    self.log.flush()
                    # projected future hit point is distinct from current target
                    from vision_core import camera_pose
                    c, rcw = camera_pose(*actual)
                    optical = rcw.T @ (future - c)
                    if optical[2] > 0:
                        point, _ = cv2.projectPoints(optical.reshape(1, 3), np.zeros(3), np.zeros(3), self.k, self.d)
                        cv2.drawMarker(frame, tuple(point.reshape(2).astype(int)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            else:
                self.motion.reset()
                lines += ['Rejected: pose or world circle check failed; NO FIRE']
        else:
            self.motion.reset()
            lines += [f'Expected ONE pending, found {len(pending)}; NO FIRE']
        if solution is None:
            # Hold last view when target disappears. Home upward only at startup.
            if now - self.last_seen > .5:
                self.command(0., .22)
            else:
                self.command()
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (15, 28 + i * 28), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 4)
            cv2.putText(frame, line, (15, 28 + i * 28), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
        cv2.imshow('vision', frame)
        cv2.imshow('mask', mask)
        key = cv2.waitKey(1) & 0xff
        if key in (ord('s'), ord('S')):
            cv2.imwrite(str(self.outdir / f'vision_{stamp:.3f}.png'), frame)
            cv2.imwrite(str(self.outdir / f'mask_{stamp:.3f}.png'), mask)
        if key in (ord('q'), ord('Q')):
            raise KeyboardInterrupt

    def close(self):
        self.command()
        self.log.close()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fire', action='store_true', help='enable simulated firing')
    parser.add_argument('--vmin', type=int, default=100)
    parser.add_argument('--smin', type=int, default=110)
    parser.add_argument('--blue-min', type=int, default=90)
    parser.add_argument('--blue-max', type=int, default=135)
    parser.add_argument('--delay', type=float, default=.06, help='extra control delay, sim seconds')
    parser.add_argument('--output', default='evidence/vision')
    cfg, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Vision(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
