#!/usr/bin/env python3
"""Independent observer only. Never sends target data or commands to vision."""
import argparse
import csv
from pathlib import Path
import rclpy
from rclpy.node import Node
from robomini_interfaces.msg import ScoreBoard, RuneHit


class Watch(Node):
    def __init__(self, output):
        super().__init__('assessment_observer')
        Path(output).mkdir(parents=True, exist_ok=True)
        self.f = open(Path(output) / 'score.csv', 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['sim_s', 'fixed_hits', 'random_fixed_hits', 'small_hits', 'big_hits', 'sim_score', 'penalty', 'shots'])
        self.last = None
        self.create_subscription(ScoreBoard, '/score', self.on_score, 10)
        self.create_subscription(RuneHit, '/rune/hit', self.on_hit, 10)

    def on_hit(self, m):
        print(f'HIT target={m.target} valid={m.valid} score={m.score}', flush=True)

    def on_score(self, m):
        key = (*m.mode_hits, m.penalty, m.balls_fired)
        if key == self.last:
            return
        self.last = key
        self.w.writerow([self.get_clock().now().nanoseconds * 1e-9, *m.mode_hits, m.total, m.penalty, m.balls_fired])
        self.f.flush()
        completed = sum(v >= 5 for v in m.mode_hits)
        print(f'Valid hits fixed/random/small/big={list(m.mode_hits)}; '
              f'completed={completed}/4; sim_score={m.total}; penalty={m.penalty}; shots={m.balls_fired}', flush=True)
        print(f'If all 3 basic tasks receive full marks: estimated assessment={30+10*completed}/70. '
              'This is a checklist estimate, not an official grade.', flush=True)


p = argparse.ArgumentParser()
p.add_argument('--output', default='evidence/score')
cfg, ros_args = p.parse_known_args()
rclpy.init(args=ros_args)
n = Watch(cfg.output)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    n.f.close()
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
