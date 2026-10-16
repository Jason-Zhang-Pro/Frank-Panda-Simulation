"""Rest-to-rest quintic timing, matching control_node.TimedTrajectory exactly.

Every path waypoint is retained (no corner cutting). Analytical peak speed
and acceleration determine each segment duration. This is deliberately a
stop-at-waypoints teaching trajectory, not time-optimal blending.
"""
import math
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .collision_checker import JOINT_NAMES


class TrajectoryPlanner:
    def __init__(self, velocity_limits, acceleration_limits, sample_period=0.02):
        self.velocity = np.asarray(velocity_limits, dtype=float)
        self.acceleration = np.asarray(acceleration_limits, dtype=float)
        if (self.velocity.shape != (7,) or self.acceleration.shape != (7,)
                or not np.all(np.isfinite([self.velocity, self.acceleration]))
                or np.any(self.velocity <= 0) or np.any(self.acceleration <= 0)
                or not math.isfinite(sample_period) or sample_period <= 0):
            raise ValueError('Invalid trajectory limits or sampling period')
        self.sample_period = sample_period

    def build(self, path, checker, resolution=0.01):
        points = np.asarray(path, dtype=float)
        if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] != 7 or not np.all(np.isfinite(points)):
            raise ValueError('Expected finite N x 7 path')
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('Invalid collision resolution')
        # Remove duplicates, but retain a two-point hold for a zero-length path.
        unique = [points[0]]
        for q in points[1:]:
            if np.linalg.norm(q-unique[-1]) > 1e-10:
                unique.append(q)
        if len(unique) == 1:
            unique.append(unique[0].copy())
        points = np.asarray(unique)
        delta = np.abs(np.diff(points, axis=0))
        durations = np.maximum.reduce([
            np.max(1.875*delta/self.velocity, axis=1),
            np.max(np.sqrt((10*math.sqrt(3)/3)*delta/self.acceleration), axis=1),
            np.full(len(delta), .3),
        ]) * 1.02
        times = np.rint(np.concatenate(([0.], np.cumsum(durations)))*1e9).astype(np.int64)
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES.copy()
        for q, ns in zip(points, times):
            point = JointTrajectoryPoint()
            point.positions = q.tolist()
            point.time_from_start.sec = int(ns // 1_000_000_000)
            point.time_from_start.nanosec = int(ns % 1_000_000_000)
            msg.points.append(point)
        samples = []
        for a, b, duration in zip(points, points[1:], np.diff(times)*1e-9):
            # max blend derivative is 1.875: bound displacement per sample.
            count = max(1, math.ceil(duration/self.sample_period),
                        math.ceil(1.875*np.linalg.norm(b-a)/resolution))
            for i in range(count):
                u = i/count
                q = a + (10*u**3-15*u**4+6*u**5)*(b-a)
                if not checker.is_state_valid(q):
                    raise ValueError('Trajectory collision: '+checker.last_reason)
                samples.append(q.tolist())
        if not checker.is_state_valid(points[-1]):
            raise ValueError('Trajectory goal collision: '+checker.last_reason)
        samples.append(points[-1].tolist())
        return msg, samples
