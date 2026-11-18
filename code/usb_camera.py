#!/usr/bin/env python3
"""Optional real USB camera evidence; does not control the gimbal."""
import argparse
from pathlib import Path
import cv2

p = argparse.ArgumentParser()
p.add_argument('--index', type=int, default=0)
p.add_argument('--output', default='evidence/usb_camera')
a = p.parse_args()
out = Path(a.output)
out.mkdir(parents=True, exist_ok=True)
cap = cv2.VideoCapture(a.index, cv2.CAP_V4L2)
if not cap.isOpened():
    raise SystemExit(f'Cannot open /dev/video{a.index}; try --index 1 or inspect v4l2-ctl --list-devices')
count = 0
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError('Camera read failed')
        cv2.putText(frame, 'LIVE REAL CAMERA | S save | Q quit', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
        cv2.imshow('real_camera', frame)
        k = cv2.waitKey(1) & 255
        if k in (ord('s'), ord('S')):
            cv2.imwrite(str(out / f'camera_{count:03d}.png'), frame)
            count += 1
        if k in (ord('q'), ord('Q')):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
