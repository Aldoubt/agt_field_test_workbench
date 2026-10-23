"""Pure, unit-testable signed distance and yaw control math (no ROS imports)."""
import math
from dataclasses import dataclass


def wrap_pi(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def forward_distance(start, now):
    """Signed progress along the INITIAL heading, not path arc length."""
    x0, y0, yaw0 = start
    x, y, _ = now
    return (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)


@dataclass
class Limits:
    max_linear: float = 0.12
    max_angular: float = 0.3
    max_acceleration: float = 0.18
    distance_tolerance: float = 0.025
    angle_tolerance: float = math.radians(2.5)
    heading_gain: float = 1.5
    min_linear: float = 0.025


class ClosedLoopMotion:
    """Single target, velocity-limited P-control; external safety checks are mandatory."""
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.kind = None
        self.target = 0.0
        self.start = None
        self.last_yaw = None
        self.yaw_progress = 0.0
        self.v = 0.0
        self.w = 0.0

    def begin(self, kind, target, start):
        if kind not in ('distance', 'rotation'):
            raise ValueError('invalid motion kind')
        if not all(math.isfinite(v) for v in (*start, target)) or abs(target) < 1e-5:
            raise ValueError('non-finite or empty target')
        self.kind, self.target, self.start = kind, target, tuple(start)
        self.last_yaw, self.yaw_progress = start[2], 0.0
        self.v = self.w = 0.0

    def reset(self):
        self.kind = None
        self.start = None
        self.v = self.w = 0.0

    def tick(self, now, dt):
        if self.kind is None:
            return 0.0, 0.0, 0.0, True
        if not all(math.isfinite(v) for v in now):
            raise ValueError('non-finite odometry')
        dt = max(0.001, min(0.1, dt))
        lim = self.limits
        if self.kind == 'distance':
            progress = forward_distance(self.start, now)
            remain = self.target - progress
            if abs(remain) <= lim.distance_tolerance and abs(self.v) <= 0.06:
                self.reset()
                return 0.0, 0.0, progress, True
            speed = min(lim.max_linear, math.sqrt(2.0 * lim.max_acceleration * abs(remain)))
            speed = max(lim.min_linear, speed) * math.copysign(1.0, remain)
            dv = lim.max_acceleration * dt
            self.v += max(-dv, min(dv, speed - self.v))
            err_yaw = wrap_pi(self.start[2] - now[2])
            self.w = max(-lim.max_angular, min(lim.max_angular, lim.heading_gain * err_yaw))
            return self.v, self.w, progress, False
        self.yaw_progress += wrap_pi(now[2] - self.last_yaw)
        self.last_yaw = now[2]
        remain = self.target - self.yaw_progress
        if abs(remain) <= lim.angle_tolerance:
            result = self.yaw_progress
            self.reset()
            return 0.0, 0.0, result, True
        self.w = max(-lim.max_angular, min(lim.max_angular, 1.1 * remain))
        return 0.0, self.w, self.yaw_progress, False
