"""Goals -> IK -> RRT -> collision-checked quintic timing -> controller."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import math
import subprocess
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from .collision_checker import CollisionChecker, JOINT_NAMES
from .path_planner import JointRRT, JointRRTConnect, RRTConfig
from .trajectory_planner import TrajectoryPlanner
from .visualization import draw_path
from .grasp_task import GraspTask


class PandaPlanner(Node):
    def __init__(self):
        super().__init__('panda_planner')
        share = Path(get_package_share_directory('panda_planning'))
        for name, value in {
            'world_urdf': str(share/'worlds/world.urdf'), 'clearance': .008,
            'home_joint_positions': [0., -.785398, 0., -2.356194, 0., 1.570796, .785398],
            'velocity_scale': .2, 'acceleration_limit': .6,
            'planning_timeout': 15., 'visualize': True, 'rrt_seed': -1,
            'planner_algorithm': 'rrt_connect',
            'shortcut_enabled': True, 'shortcut_attempts': 150, 'shortcut_timeout': 2.,
        }.items():
            self.declare_parameter(name, value)
        self.current_q = None
        self.feedback_time = 0.
        self.xml = None
        self.sim_time = 0.
        self.busy = False
        self.future = None
        self.accepted = False
        self.reference_done = False
        self.goal_q = None
        self.sent_at = 0.
        self.completed_at = None
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.checker = None
        self.checker_xml = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/planner_status', latched)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/planned_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_state, qos_profile_sensor_data)
        self.create_subscription(String, '/robot_description', self.on_description, latched)
        self.create_subscription(JointState, '/joint_goal', self.on_joint_goal, 10)
        self.create_subscription(PoseStamped, '/planning_goal', self.on_pose_goal, 10)
        self.create_subscription(String, '/trajectory_status', self.on_control_status, 10)
        self.create_service(Trigger, '/plan_to_target', self.on_target)
        self.create_timer(.05, self.poll)
        self.grasp_task = GraspTask(self)
        self.status('idle: 等待目标；不会自动运动')

    def status(self, text):
        if hasattr(self, 'grasp_task'):
            self.grasp_task.on_planner_status(text)
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_description(self, msg):
        self.xml = msg.data

    def on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(n in values for n in JOINT_NAMES):
            return
        q = np.array([values[n] for n in JOINT_NAMES])
        if np.all(np.isfinite(q)):
            self.current_q = q
            self.feedback_time = time.monotonic()
            self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

    def on_control_status(self, msg):
        if not self.busy or self.future is not None:
            return
        if msg.data == 'accepted':
            self.accepted = True
            self.status('executing: 控制节点已接收轨迹')
        elif msg.data == 'reference_complete':
            self.reference_done = True
            self.completed_at = self.sim_time
        elif msg.data.startswith('rejected:'):
            self.busy = False
            self.status('failed: 控制节点'+msg.data)

    def request(self, kind, goal, internal=False):
        if (self.grasp_task.active or self.grasp_task.payload is not None) and not internal:
            raise ValueError("夹取流程进行中")
        if self.busy:
            raise ValueError('正在规划或执行，请等待完成')
        if self.xml is None or self.current_q is None:
            raise ValueError('尚未收到机器人模型和关节状态')
        if time.monotonic()-self.feedback_time > .5:
            raise ValueError('关节反馈过期，请先播放仿真')
        if self.trajectory_pub.get_subscription_count() != 1:
            raise ValueError('需要且只能有一个轨迹控制订阅者')
        if self.count_publishers('/panda_effort_controller/commands') != 1:
            raise ValueError('需要且只能有一个力矩发布节点')
        self.busy = True
        self.accepted = self.reference_done = False
        self.completed_at = None
        self.status('planning: 正在进行碰撞检测、路径搜索和轨迹生成')
        self.future = self.pool.submit(self.compute, self.xml, self.current_q.copy(), kind, goal)

    def on_joint_goal(self, msg):
        try:
            if len(msg.name) != 7 or set(msg.name) != set(JOINT_NAMES) or len(msg.position) != 7:
                raise ValueError('joint_goal 必须包含七个不重复的关节名和七个角度')
            values = dict(zip(msg.name, msg.position))
            self.request('joint', [values[n] for n in JOINT_NAMES])
        except ValueError as exc:
            self.status('rejected: '+str(exc))

    def on_pose_goal(self, msg):
        try:
            if msg.header.frame_id not in ('world', 'base_link', 'panda_link0'):
                raise ValueError('目标坐标系必须为 world/base_link/panda_link0，且基座在世界原点')
            if msg.header.stamp.sec or msg.header.stamp.nanosec:
                raise ValueError('静态世界目标要求时间戳为零')
            position, orientation = msg.pose.position, msg.pose.orientation
            self.request('pose', ([position.x, position.y, position.z],
                                  [orientation.x, orientation.y, orientation.z, orientation.w]))
        except ValueError as exc:
            self.status('rejected: '+str(exc))

    def on_target(self, request, response):
        try:
            # Target pose is read from the actual static URDF instead of hardcoded.
            import xml.etree.ElementTree as ET
            root = ET.parse(str(self.get_parameter('world_urdf').value)).getroot()
            joint = next(j for j in root.findall('joint')
                         if j.find('child').get('link') == 'target_cube')
            if joint.find('parent').get('link') != 'obstacle_world_root':
                raise ValueError('目标必须直接连接场景根节点')
            position = [float(x) for x in joint.find('origin').get('xyz').split()]
            if self.grasp_task.target_state is not None:
                position = self.grasp_task.target_state[0].copy()
            position[2] += .13  # TCP approach position, no contact with target.
            self.request('pose', (position, None))
            response.success, response.message = True, '目标上方路径规划已提交；查看 /planner_status'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def compute(self, xml, start, kind, goal):
        if self.checker is None or xml != self.checker_xml:
            if self.checker is not None:
                self.checker.close()
            self.checker = CollisionChecker(xml, self.get_parameter('world_urdf').value,
                                             self.get_parameter('clearance').value)
            self.checker_xml = xml
        c = self.checker
        c.allow_finger_target = kind in ("grasp_pose", "grasp_descent")
        task = self.grasp_task
        if task.payload is not None:
            captured, position, orientation, width = task.payload
            c.finger_width = width
            c.attach_payload(captured, position, orientation)
            c.support_contacts = ({'target_pedestal'} if task.stage == 'lifting' else
                                  {'place_pedestal'} if task.stage == 'lowering' else set())
        else:
            c.finger_width = .04
            if task.target_state is not None:
                c.target_pose(*task.target_state)
            c.support_contacts = set()
        if not c.is_state_valid(start):
            raise ValueError('起点碰撞：'+c.last_reason)
        target = goal if kind == 'joint' else c.inverse_kinematics(goal[0], start, goal[1])
        algorithm = self.get_parameter('planner_algorithm').value
        planners = {'rrt': JointRRT, 'rrt_connect': JointRRTConnect}
        if algorithm not in planners:
            raise ValueError('planner_algorithm 必须为 rrt 或 rrt_connect')
        planner = planners[algorithm](list(zip(c.lower, c.upper)), c.is_state_valid,
                           RRTConfig(timeout=float(self.get_parameter('planning_timeout').value),
                                     max_iterations=12000, seed=(None if int(self.get_parameter('rrt_seed').value) < 0
                                                                else int(self.get_parameter('rrt_seed').value))))
        if kind == "grasp_descent":
            initial, _ = c.tcp_pose(start)
            path = [start.tolist()]
            count = max(1, int(np.ceil(np.linalg.norm(np.array(goal[0])-initial)/.01)))
            for i in range(1, count+1):
                xyz = initial+(np.array(goal[0])-initial)*(i/count)
                q = c.local_ik(xyz, path[-1], goal[1])
                if np.max(np.abs(np.array(q)-path[-1])) > .25:
                    raise ValueError("竖直下降发生逆解跳变，拒绝执行")
                path.append(q)
            target = path[-1]
        else:
            self.get_logger().info('路径搜索器：'+algorithm)
            path = planner.plan(start, target)
        if path is None:
            reason = getattr(planner, 'last_status', 'not_found')
            detail = {'timeout':'搜索超时', 'iteration_limit':'达到最大搜索次数',
                      'not_found':'搜索超时或未找到路径'}.get(reason, reason)
            raise ValueError(algorithm+': '+detail)
        if kind != 'grasp_descent' and self.get_parameter('shortcut_enabled').value:
            before = len(path)
            path = planner.shortcut(path,
                                    attempts=int(self.get_parameter('shortcut_attempts').value),
                                    timeout=float(self.get_parameter('shortcut_timeout').value))
            self.get_logger().info(f'shortcut: {before} → {len(path)} 个路径点，碰撞复检通过')
        scale = float(self.get_parameter('velocity_scale').value)
        if not math.isfinite(scale) or not 0 < scale <= 1:
            raise ValueError('velocity_scale 必须在 (0,1] 内')
        timing = TrajectoryPlanner(c.velocity*scale,
                                   [float(self.get_parameter('acceleration_limit').value)]*7)
        trajectory, samples = timing.build(path, c)
        marker = False
        if self.get_parameter('visualize').value:
            xyz = [c.tcp_pose(q)[0].tolist() for q in samples]
            try:
                marker = draw_path(xyz)
            except (OSError, subprocess.SubprocessError):
                pass
        return trajectory, target, marker

    def poll(self):
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            try:
                trajectory, target, marker = future.result()
                if time.monotonic()-self.feedback_time > .5:
                    raise ValueError('规划期间关节反馈中断')
                if np.max(np.abs(np.array(trajectory.points[0].positions)-self.current_q)) > .05:
                    raise ValueError('规划期间机械臂移动，请重新规划')
                self.goal_q = np.array(target)
                self.sent_at = time.monotonic()
                self.trajectory_pub.publish(trajectory)
                self.status('ready: 碰撞与速度/加速度检查通过，已发送轨迹；Gazebo路径显示='+str(marker))
            except Exception as exc:
                self.busy = False
                self.status('failed: '+str(exc))
        if self.busy and self.future is None:
            if not self.accepted and time.monotonic()-self.sent_at > 3:
                self.busy = False
                self.status('failed: 控制节点未确认接收轨迹，请检查 /trajectory_status')
            elif self.reference_done:
                error = float(np.max(np.abs(self.goal_q-self.current_q)))
                if error < .03:
                    self.busy = False
                    self.status(f'succeeded: 已到达，最大关节误差 {error:.4f} rad')
                elif self.sim_time-self.completed_at > 3:
                    self.busy = False
                    self.status(f'failed: 轨迹播放结束但跟踪误差 {error:.4f} rad 仍过大')

    def close(self):
        self.pool.shutdown(wait=True)
        if self.checker is not None:
            self.checker.close()


def main(args=None):
    rclpy.init(args=args)
    node = PandaPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
