#!/usr/bin/env python3
"""Panda Gazebo teaching controller: tau = Kp(q_ref-q) + Kd(dq_ref-dq) + g(q).

Subscribes to /planned_trajectory (trajectory_msgs/JointTrajectory).
Positions-only, all seven joints, first time zero, header stamp zero.
Each segment uses quintic rest-to-rest interpolation in feedback simulation time.
Holds first measured posture until a trajectory arrives; holds endpoint afterward.
Rejects new trajectories while busy. No collision checking or acceleration limits.
Reads the running /robot_description, including hand/finger masses.
Requires ROS 2 Humble and NumPy; no Pinocchio/Conda installation required.
Assumptions: seven Panda revolute joints, two actuated prismatic fingers, URDF root
aligned with Gazebo world, world gravity (0, 0, -9.81), no hidden gravity
compensation. This calculates gravity ONLY, not inertia/Coriolis/friction.

Start the existing paused launch, run this script, request activation of both
controllers, then play Gazebo. Pause Gazebo BEFORE stopping this process.
The forwarding controller can retain its last command after process failure.
"""
import argparse
import math
import time
import xml.etree.ElementTree as ET

import numpy as np

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]
Q_TARGET = np.array([0., -0.785398, 0., -2.356194, 0., 1.570796, 0.785398])
KP = np.array([80., 100., 80., 100., 30., 20., 10.])
KD = np.array([2, 10., 2.5, 12., 1.5, 3., 0.5])
TORQUE_LIMIT = np.array([40., 70., 35., 35., 10., 10., 8.])
GRAVITY = np.array([0., 0., -9.81])
GRIPPER_OPEN_KP = 100.0
GRIPPER_CLOSE_KP = 200.0
GRIPPER_FORCE_LIMIT = 5.0  # N per finger; matches URDF prismatic effort limit.


def vector(text):
    values = np.array([float(v) for v in text.split()])
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError(f"Invalid URDF vector: {text}")
    return values


def rotation(axis, angle):
    """Rodrigues rotation about a unit axis."""
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = vector(element.get("xyz", "0 0 0"))
        roll, pitch, yaw = vector(element.get("rpy", "0 0 0"))
        transform[:3, :3] = (rotation([0, 0, 1], yaw)
                             @ rotation([0, 1, 0], pitch)
                             @ rotation([1, 0, 0], roll))
    return transform


class GravityModel:
    """URDF forward kinematics + sum of link gravitational moments.

    Compensation = -sum(J_com.T @ mass * gravity) = gradient of potential.
    Finger positions affect their COM; arm gravity includes both finger masses.
    Inertial-frame rotation is unnecessary for gravity: only mass/COM matter.
    """
    def __init__(self, xml, joint_names=JOINTS, gravity=GRAVITY):
        root = ET.fromstring(xml)
        self.names = list(joint_names)
        self.gravity = np.array(gravity, dtype=float)
        self.links = {}
        self.children = {}
        self.joints = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            mass, com = 0., np.zeros(3)
            if inertial is not None:
                mass = float(inertial.find("mass").get("value"))
                com = origin(inertial.find("origin"))[:3, 3]
                if not math.isfinite(mass) or mass < 0:
                    raise ValueError("Invalid link mass")
            self.links[link.get("name")] = (mass, com)
        child_links = set()
        movable = set()
        for joint in root.findall("joint"):
            name, kind = joint.get("name"), joint.get("type")
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in self.links or child not in self.links or child in child_links:
                raise ValueError("Invalid URDF link tree")
            child_links.add(child)
            if kind != "fixed":
                if name in FINGERS and kind == "prismatic":
                    pass
                elif kind in ("revolute", "continuous") and name in self.names:
                    movable.add(name)
                else:
                    raise ValueError(f"Unsupported moving joint: {name}")
                if self.links[child][0] <= 0:
                    raise ValueError(f"Missing positive mass for {child}")
            axis_element = joint.find("axis")
            axis = vector(axis_element.get("xyz", "1 0 0") if axis_element is not None else "1 0 0")
            if np.linalg.norm(axis) == 0:
                raise ValueError("Zero joint axis")
            axis /= np.linalg.norm(axis)
            record = (name, kind, child, origin(joint.find("origin")), axis)
            self.children.setdefault(parent, []).append(record)
            self.joints[name] = record
        roots = set(self.links) - child_links
        if len(roots) != 1 or movable != set(self.names):
            raise ValueError("Expected one rooted tree and exactly the requested moving joints")
        self.root = roots.pop()
        self.index = {name: i for i, name in enumerate(self.names)}
        self.total_mass = sum(m for m, _ in self.links.values())

    def evaluate(self, q, finger_positions=(0.04, 0.04)):
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.names),) or not np.all(np.isfinite(q)):
            raise ValueError("Invalid joint state")
        compensation = np.zeros(len(self.names))
        potential = 0.
        visited = set()

        def visit(link, transform, ancestors):
            nonlocal potential
            if link in visited:
                raise ValueError("Cycle in URDF")
            visited.add(link)
            mass, com = self.links[link]
            position = transform[:3, :3] @ com + transform[:3, 3]
            force = mass * self.gravity
            potential -= force @ position
            for index, point, axis_world in ancestors:
                compensation[index] -= axis_world @ np.cross(position - point, force)
            for name, kind, child, offset, axis in self.children.get(link, []):
                joint_frame = transform @ offset
                child_ancestors = ancestors
                if kind == "prismatic":
                    motion = np.eye(4)
                    motion[:3, 3] = axis * finger_positions[FINGERS.index(name)]
                    joint_frame = joint_frame @ motion
                elif kind != "fixed":
                    index = self.index[name]
                    child_ancestors = ancestors + [(index, joint_frame[:3, 3].copy(), joint_frame[:3, :3] @ axis)]
                    motion = np.eye(4)
                    motion[:3, :3] = rotation(axis, q[index])
                    joint_frame = joint_frame @ motion
                visit(child, joint_frame, child_ancestors)

        visit(self.root, np.eye(4), [])
        if len(visited) != len(self.links):
            raise ValueError("Disconnected URDF")
        return compensation, potential


class TimedTrajectory:
    """Positions-only trajectory; quintic, rest-to-rest between waypoints."""

    def __init__(self, message, current_q, lower, upper, speed):
        if len(message.joint_names) != 7 or set(message.joint_names) != set(JOINTS):
            raise ValueError("必须包含七个 Panda 关节，且名称不能重复")
        if message.header.stamp.sec or message.header.stamp.nanosec:
            raise ValueError("本接口要求 header.stamp 为零")
        if len(message.points) < 2:
            raise ValueError("至少需要两个轨迹点")
        order = [message.joint_names.index(name) for name in JOINTS]
        times, positions = [], []
        for point in message.points:
            if len(point.positions) != 7:
                raise ValueError("每个点必须有七个关节位置")
            if point.velocities or point.accelerations or point.effort:
                raise ValueError("只接收 positions 和 time_from_start，导数由控制端插值生成")
            sec, nsec = point.time_from_start.sec, point.time_from_start.nanosec
            if sec < 0 or not 0 <= nsec < 1_000_000_000:
                raise ValueError("无效的 time_from_start")
            times.append(sec + nsec * 1e-9)
            positions.append([point.positions[i] for i in order])
        self.times = np.asarray(times, dtype=float)
        self.positions = np.asarray(positions, dtype=float)
        if not np.all(np.isfinite(self.positions)):
            raise ValueError("位置含 NaN 或无穷大")
        if self.times[0] != 0 or np.any(np.diff(self.times) <= 0):
            raise ValueError("第一点时间必须为零，随后严格递增")
        if np.any(self.positions < lower) or np.any(self.positions > upper):
            raise ValueError("轨迹超出 URDF 关节位置限位")
        if np.max(np.abs(self.positions[0] - current_q)) > 0.05:
            raise ValueError("轨迹起点与实测位置相差超过 0.05 rad")
        # Quintic smoothstep has peak normalized speed 1.875.
        peak_speed = 1.875 * np.abs(np.diff(self.positions, axis=0)) / np.diff(self.times)[:, None]
        if np.any(peak_speed > speed):
            raise ValueError("插值速度超过 URDF 上限，请增加轨迹时间")
        self.start = None

    def sample(self, stamp):
        if self.start is None:
            self.start = stamp
        elapsed = stamp - self.start
        if elapsed >= self.times[-1]:
            return self.positions[-1].copy(), np.zeros(7), True
        i = int(np.searchsorted(self.times, elapsed, side="right") - 1)
        i = max(0, min(i, len(self.times) - 2))
        duration = self.times[i + 1] - self.times[i]
        u = np.clip((elapsed - self.times[i]) / duration, 0., 1.)
        blend = 10*u**3 - 15*u**4 + 6*u**5
        rate = (30*u**2 - 60*u**3 + 30*u**4) / duration
        delta = self.positions[i + 1] - self.positions[i]
        return self.positions[i] + blend*delta, rate*delta, False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gravity-scale", type=float, choices=[0., 1.], default=1.,
                        help="1: PD+gravity (default), 0: matched pure-PD comparison")
    args, ros_args = parser.parse_known_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from trajectory_msgs.msg import JointTrajectory
    from std_msgs.msg import Float64, Float64MultiArray, String

    class PandaPDGravity(Node):
        def __init__(self):
            super().__init__("panda_pd_gravity_control")
            self.gripper_goal = 0.04
            self.gripper_reference = 0.04
            self.gripper_stamp = None
            self.gripper_pub = self.create_publisher(Float64MultiArray, "/panda_gripper_controller/commands", 1)
            self.gripper_sub = self.create_subscription(Float64, "/gripper_goal", self.on_gripper_goal, 10)
            self.model = None
            self.model_failed = False
            self.last_feedback = None
            self.last_stamp = None
            self.last_feedback_stamp = None
            self.latest_sim_time = None
            self.last_log = 0.
            self.current_q = None
            self.hold_q = None
            self.trajectory = None
            self.lower = self.upper = self.speed = None
            self.first = True
            self.warned = False
            self.publisher = self.create_publisher(Float64MultiArray, "/panda_effort_controller/commands", 1)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.description_sub = self.create_subscription(String, "/robot_description", self.on_description, qos)
            self.state_sub = self.create_subscription(JointState, "/joint_states", self.on_state, qos_profile_sensor_data)
            self.status_pub = self.create_publisher(String, "/trajectory_status", 10)
            self.trajectory_sub = self.create_subscription(
                JointTrajectory, "/planned_trajectory", self.on_trajectory, 10)
            self.clock_sub = self.create_subscription(
                Clock, "/clock", self.on_clock, qos_profile_sensor_data)
            self.watchdog = self.create_timer(0.05, self.check_feedback)
            self.get_logger().info("Waiting for live /robot_description and /joint_states. Run ONLY this torque publisher.")

        def on_gripper_goal(self, message):
            if math.isfinite(message.data) and 0 <= message.data <= 0.04:
                self.gripper_goal = message.data
                self.get_logger().info(f"夹爪目标总开口 {2*message.data:.3f} m")
            else:
                self.get_logger().warning("夹爪半开口必须在 [0, 0.04] m")

        def on_trajectory(self, message):
            try:
                if self.model is None or self.current_q is None:
                    raise ValueError("等待机器人模型和关节状态")
                if self.last_feedback is None or time.monotonic() - self.last_feedback > 0.2:
                    raise ValueError("关节状态过期，请恢复仿真后重新发送")
                if self.trajectory is not None:
                    raise ValueError("已有轨迹正在执行")
                trajectory = TimedTrajectory(
                    message, self.current_q, self.lower, self.upper, self.speed)
                self.trajectory = trajectory
                self.status_pub.publish(String(data="accepted"))
                self.get_logger().info(
                    f"已接收 {len(trajectory.times)} 个轨迹点，"
                    f"总时间 {trajectory.times[-1]:.2f} 秒")
            except (ValueError, TypeError, OverflowError) as exc:
                self.get_logger().warning(f"拒绝轨迹：{exc}")
                self.status_pub.publish(String(data="rejected:"+str(exc)))

        def publish(self, torque):
            message = Float64MultiArray()
            message.data = np.asarray(torque, dtype=float).tolist()
            self.publisher.publish(message)

        def on_description(self, message):
            if self.model is not None or self.model_failed:
                return
            try:
                model = GravityModel(message.data)
                for link in ("panda_hand", "panda_leftfinger", "panda_rightfinger"):
                    if model.links.get(link, (0., None))[0] <= 0:
                        raise ValueError(f"Expected gripper model with mass for {link}")
                model.evaluate(Q_TARGET)  # Check the entire tree before publishing.
                elements = {j.get("name"): j for j in ET.fromstring(message.data).findall("joint")}
                limits = [elements[name].find("limit") for name in JOINTS]
                lower = np.array([float(limit.get("lower")) for limit in limits])
                upper = np.array([float(limit.get("upper")) for limit in limits])
                speed = np.array([float(limit.get("velocity")) for limit in limits])
                if (not np.all(np.isfinite([lower, upper, speed]))
                        or np.any(lower >= upper) or np.any(speed <= 0)):
                    raise ValueError("Invalid URDF joint limits")
                self.lower, self.upper, self.speed = lower, upper, speed
                self.model = model
                self.get_logger().info(f"Model loaded: actuated gripper included; mass={model.total_mass:.3f} kg; gravity scale={args.gravity_scale}")
            except Exception as exc:
                self.model_failed = True
                self.get_logger().error(f"Model rejected: {exc}. Pause Gazebo and fix the model.")

        def on_state(self, message):
            if self.model is None:
                return
            try:
                indices = {name: i for i, name in enumerate(message.name)}
                q = np.array([message.position[indices[n]] for n in JOINTS])
                dq = np.array([message.velocity[indices[n]] for n in JOINTS])
                fingers = np.array([message.position[indices[n]] for n in FINGERS])
                finger_speed = np.array([message.velocity[indices[n]] for n in FINGERS])
                if not np.all(np.isfinite([fingers, finger_speed])):
                    raise ValueError("Invalid finger feedback")
                if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)):
                    raise ValueError("Non-finite feedback")
            except (KeyError, IndexError, ValueError):
                self.publish(np.zeros(7))
                return
            now = time.monotonic()
            self.last_feedback = now
            self.warned = False
            # At most 100 Hz of simulation time, driven by fresh joint samples.
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self.last_feedback_stamp = stamp
            dt = 0.0 if self.gripper_stamp is None else max(0., min(.02, stamp-self.gripper_stamp))
            if self.gripper_stamp is not None and stamp < self.gripper_stamp:
                self.gripper_reference = float(np.mean(fingers))
            self.gripper_stamp = stamp
            self.gripper_reference += np.clip(self.gripper_goal-self.gripper_reference, -.02*dt, .02*dt)
            gripper_kp = GRIPPER_CLOSE_KP if self.gripper_goal == 0.0 else GRIPPER_OPEN_KP
            finger_torque = np.clip(gripper_kp*(self.gripper_reference-fingers)-1.*finger_speed,
                                    -GRIPPER_FORCE_LIMIT, GRIPPER_FORCE_LIMIT)
            self.gripper_pub.publish(Float64MultiArray(data=finger_torque.tolist()))
            if self.last_stamp is not None and 0 <= stamp - self.last_stamp < 0.01:
                return
            if self.last_stamp is not None and stamp < self.last_stamp:
                self.trajectory = None
                self.hold_q = q.copy()
                self.status_pub.publish(String(data="rejected:仿真时间回退，轨迹已取消"))
                self.get_logger().warning("仿真时间回退，取消旧轨迹并保持当前位置")
            self.last_stamp = stamp
            self.current_q = q.copy()
            if self.hold_q is None:
                self.hold_q = q.copy()
            q_ref, dq_ref = self.hold_q.copy(), np.zeros(7)
            if self.trajectory is not None:
                q_ref, dq_ref, finished = self.trajectory.sample(stamp)
                if finished:
                    self.hold_q = q_ref.copy()
                    self.trajectory = None
                    self.status_pub.publish(String(data="reference_complete"))
                    self.get_logger().info("轨迹参考播放结束，保持终点；实际误差见控制日志")
            gravity, _ = self.model.evaluate(q, fingers)
            error = q_ref - q
            pd = KP * error + KD * (dq_ref - dq)
            raw = pd + args.gravity_scale * gravity
            torque = np.clip(raw, -TORQUE_LIMIT, TORQUE_LIMIT)
            self.publish(torque)
            if self.first or now - self.last_log >= 1.:
                self.get_logger().info(
                    f"error_deg={np.round(np.degrees(error), 2).tolist()} | "
                    f"g_Nm={np.round(gravity, 2).tolist()} | "
                    f"tau_Nm={np.round(torque, 2).tolist()} | "
                    f"saturated={bool(np.any(np.abs(raw) > TORQUE_LIMIT))}")
                self.first = False
                self.last_log = now

        def on_clock(self, message):
            self.latest_sim_time = message.clock.sec + message.clock.nanosec * 1e-9

        def check_feedback(self):
            if self.last_feedback is not None and time.monotonic() - self.last_feedback > 0.2:
                # Paused simulation: retain the last command for the next step.
                # If simulation time advances without joint feedback, stop commands.
                if (self.latest_sim_time is not None
                        and self.last_feedback_stamp is not None
                        and self.latest_sim_time <= self.last_feedback_stamp + 0.02):
                    return
                self.publish(np.zeros(7))
                self.gripper_pub.publish(Float64MultiArray(data=[0., 0.]))
                if not self.warned:
                    self.get_logger().warning("Feedback absent >0.2 wall seconds (also happens when paused): zero torque. Zero does NOT support gravity.")
                    self.warned = True

    rclpy.init(args=ros_args)
    node = PandaPDGravity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
